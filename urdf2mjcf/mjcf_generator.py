"""
URDF -> MJCF (MuJoCo) Converter (refactored)

Public interface / 对外接口:
    mjcf_generator(urdf_path, mjcf_path, json_config_path=None)

EN:
- Parse <ros2_control ... MujocoSystem> to generate actuators/sensors by default.
- Append extra actuators/sensors/cameras from JSON if provided.
- Fallback: if no ros2_control and no JSON definitions, add default actuators (motor) for non-mimic joints.
- No sites are created unless sensors/cameras require them.

CN:
- 解析 <ros2_control ... MujocoSystem> 以默认生成执行器/传感器
- 若 JSON 提供额外 actuator/sensor/camera，则追加
- 回退策略：若无 ros2_control 且 JSON 也未配置，则为非 mimic 关节添加默认执行器（motor）
- 只有当传感器/相机需要时才创建 site
"""

from __future__ import annotations

import json
import logging
import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union
from xml.dom import minidom
import numpy as np

from .urdf_parser import UrdfParser


# IMPORTANT:
# EN: Do NOT call logging.basicConfig() here. CLI configures logging globally.
# CN: 不要在此模块内配置 logging.basicConfig()，由 CLI 统一配置日志风格。
logger = logging.getLogger("urdf2mjcf.mjcf_generator")
logger_json = logging.getLogger("urdf2mjcf.mjcf_generator.JsonParser")
logger_urdf = logging.getLogger("urdf2mjcf.mjcf_generator.UrdfParser")
logger_mjcf = logging.getLogger("urdf2mjcf.mjcf_generator.MjcfBuilder")


# =============================================================================
# XML utils
# XML 工具
# =============================================================================
def _pretty_write_xml(root: ET.Element, out_path: Union[str, Path]) -> None:
    """
    EN: Write pretty-formatted XML to file.
    CN: 将 XML 以较美观的缩进格式写入文件。
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rough = ET.tostring(root, encoding="utf-8")
    reparsed = minidom.parseString(rough)
    pretty = reparsed.toprettyxml(indent="  ")
    pretty = re.sub(r"\n\s*\n", "\n", pretty)
    out_path.write_text(pretty, encoding="utf-8")


def _safe_float(x: Optional[str], default: float = 0.0) -> float:
    """
    EN: Convert str to float safely.
    CN: 安全地把字符串转换为 float。
    """
    if x is None:
        return default
    try:
        return float(x)
    except Exception:
        return default


def _resolve_mesh_path(urdf_file_path: Path, mesh_path: str, mjcf_output_path: Path) -> str:
    """
    EN: Resolve mesh file path from URDF relative path to MJCF relative path.
    CN: 将mesh文件路径从URDF中的相对路径转换为相对于输出MJCF文件的路径。
    
    :param urdf_file_path: URDF文件的绝对路径
    :param mesh_path: URDF中定义的mesh文件路径（可能是相对路径或绝对路径）
    :param mjcf_output_path: 输出MJCF文件的路径（可以是相对或绝对路径）
    :return: 相对于MJCF文件的mesh路径
    """
    mesh_path_obj = Path(mesh_path)
    
    if mesh_path_obj.is_absolute():
        abs_mesh_path = mesh_path_obj.resolve()
    else:
        urdf_dir = urdf_file_path.parent.resolve()
        abs_mesh_path = (urdf_dir / mesh_path_obj).resolve()
    
    mjcf_dir = mjcf_output_path.resolve().parent
    
    try:
        relative_path = abs_mesh_path.relative_to(mjcf_dir)
        return str(relative_path)
    except ValueError:
        pass
    
    for up_levels in range(1, 4):
        parent_dir = mjcf_dir
        for _ in range(up_levels):
            parent_dir = parent_dir.parent
            if parent_dir == parent_dir.parent:
                break
        
        try:
            relative_path = abs_mesh_path.relative_to(parent_dir)
            prefix = "../" * up_levels
            return prefix + str(relative_path)
        except ValueError:
            continue
    
    logger.warning(f"Mesh path '{abs_mesh_path}' is not within 3 levels of MJCF directory '{mjcf_dir}'. Using absolute path.")
    return str(abs_mesh_path)


def _str2vec(x: str) -> tuple:
    """
    EN: Normalize string to numeric triple.
    CN: 将字符串规范化为数值元组格式。
    """
    if x is None:
        return (0.0, 0.0, 0.0)
    vals = x.split()
    return tuple(float(v) for v in vals)


def _vec2str(vec: Optional[list|tuple]) -> Optional[str]:
    """
    EN: Convert numeric list or tuple to space-separated string.
    CN: 将数值列表或元组转换为空格分隔的字符串。
    
    :param vec: A list or tuple containing numeric values
    :return: Space-separated string representation
    """
    if vec is None:
        return None
    return " ".join(str(float(v)) for v in vec)


def _diagonalize_inertia(
    ixx: float, iyy: float, izz: float,
    ixy: float, ixz: float, iyz: float,
    qw: float, qx: float, qy: float, qz: float
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """
    将惯量矩阵对角化，并计算对应的新四元数。
    
    参数:
        ixx, iyy, izz: 原始惯量矩阵的对角元素
        ixy, ixz, iyz: 原始惯量矩阵的非对角元素
        qw, qx, qy, qz: 原始四元数（表示从参考坐标系到当前惯性坐标系的旋转）
        
    返回:
        (new_ixx, new_iyy, new_izz): 对角化后的主转动惯量
        (new_qw, new_qx, new_qy, new_qz): 对应的新四元数
    """
    def q_normalize(q: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(q)
        if n == 0 or not np.isfinite(n):
            raise ValueError("Invalid quaternion: norm is zero or non-finite.")
        return q / n

    def q_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        # Hamilton product, both in (w,x,y,z)
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ], dtype=float)

    def rotmat_to_quat(R: np.ndarray) -> np.ndarray:
        # Robust conversion, returns (w,x,y,z)
        R = np.asarray(R, dtype=float)
        t = np.trace(R)
        if t > 0.0:
            s = np.sqrt(t + 1.0) * 2.0
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
            if i == 0:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif i == 1:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        q = np.array([w, x, y, z], dtype=float)
        # Make sign deterministic (optional but nice): keep w >= 0
        q = q_normalize(q)
        if q[0] < 0:
            q = -q
        return q

    # 1) Build symmetric inertia tensor in the given frame F
    I = np.array([
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz]
    ], dtype=float)

    # Basic sanity
    if not np.all(np.isfinite(I)):
        raise ValueError("Inertia tensor has NaN/Inf.")
    if np.max(np.abs(I - I.T)) > 1e-10:
        raise ValueError("Inertia tensor must be symmetric.")

    # 2) Eigen-decomposition (for symmetric matrix => orthonormal eigenvectors)
    #    Columns of V are eigenvectors; I = V diag(w) V^T
    evals, V = np.linalg.eigh(I)

    # Sort eigenvalues (and eigenvectors) ascending (common convention)
    order = np.argsort(evals)
    evals = evals[order]
    V = V[:, order]

    # 3) Ensure right-handed rotation (det = +1). If det < 0, flip one axis.
    if np.linalg.det(V) < 0:
        V[:, -1] *= -1.0

    # V maps principal frame P axes expressed in F (columns) => R_FP = V
    R_FP = V.T

    # 4) Convert R_FP to quaternion q_FP
    q_FP = rotmat_to_quat(R_FP)

    # 5) Compose with provided inertia-frame quaternion: q_BP = q_BF ⊗ q_FP
    q_BF = q_normalize(np.array([qw, qx, qy, qz], dtype=float))
    q_BP = q_normalize(q_mul(q_BF, q_FP))
    if q_BP[0] < 0:
        q_BP = -q_BP

    diag = (float(evals[0]), float(evals[1]), float(evals[2]))
    quat = (float(q_BP[0]), float(q_BP[1]), float(q_BP[2]), float(q_BP[3]))
    return diag, quat


def _rpy_to_quaternion(rpy: tuple, degrees=False) -> tuple:
    """
    将 RPY 角（绕固定轴ZYX旋转）转换为四元数（wxyz顺序）
    
    参数:
    rpy: tuple|list - (roll, pitch, yaw) 弧度或度
    degrees: bool - 如果为True，输入是度；否则是弧度
    
    返回:
    tuple - 四元数 (w, x, y, z)
    """
    # 解包 RPY
    roll, pitch, yaw = rpy
    
    # 如果输入是度，转换为弧度
    if degrees:
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
    
    # 计算半角
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    
    # URDF的RPY是绕固定轴旋转：Z(yaw) -> Y(pitch) -> X(roll)
    # 对应的四元数乘法顺序是相反的： q_roll * q_pitch * q_yaw
    
    # 计算每个轴的四元数
    # 绕X轴旋转（roll）
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-10:
        return (1.0, 0.0, 0.0, 0.0)
    return (qw / norm, qx / norm, qy / norm, qz / norm)


# =============================================================================
# Config (JSON)
# JSON 配置
# =============================================================================

@dataclass
class MjcfConfig:
    """
    EN: Parsed configuration container.
    CN: 解析后的配置容器。
    """

    config: Dict[str, Any] = field(default_factory=dict)

    compiler: Dict[str, Any] = field(default_factory=dict)
    option: Dict[str, Any] = field(default_factory=dict)
    visual: Dict[str, Any] = field(default_factory=dict)
    default: Dict[str, Any] = field(default_factory=dict)

    assets: List[Dict[str, Any]] = field(default_factory=list)
    worldbody: Dict[str, Any] = field(default_factory=dict)
    contacts: List[Dict[str, Any]] = field(default_factory=list)
    actuators: List[Dict[str, Any]] = field(default_factory=list)
    equalities: List[Dict[str, Any]] = field(default_factory=list)
    sites: List[Dict[str, Any]] = field(default_factory=list)
    sensors: List[Dict[str, Any]] = field(default_factory=list)
    lights: List[Dict[str, Any]] = field(default_factory=list)


    cameras: List[Dict[str, Any]] = field(default_factory=list)
    textures: List[Dict[str, Any]] = field(default_factory=list)


    @staticmethod
    def _load_labeled_config_list(src_config: Dict[str, Any]) -> List[Dict[str, Any]]:
        """加载带标签的配置项"""
        target_list:List[Dict[str, Any]] = []
        if isinstance(src_config, dict):
            for config_label, config_list in src_config.items():
                if isinstance(config_list, list):
                    for config_item in config_list:
                        if isinstance(config_item, dict):
                            item = config_item.copy() # config_item: {"group": 0, "ctrlrange":"-150 150", ...}
                            item["label"] = config_label # config_label: "motor", "position", ...
                            target_list.append(item) # item: {"label": "motor", "group": 0, "ctrlrange":"-150 150", ...}
        return target_list

    @staticmethod
    def _cfg_get(d: Dict[str, Any], path: str, default: Any = None) -> Any:
        """EN: Nested dict getter. CN: 字典嵌套路径读取。"""
        cur: Any = d
        for key in path.split("."):
            if not isinstance(cur, dict) or key not in cur:
                return default
            cur = cur[key]
        return cur

    @classmethod
    def load(cls, json_path: Optional[Union[str, Path]]) -> MjcfConfig:
        """
        EN: Load config from JSON (supports nested roots: mjcf/mujoco/urdf_to_mjcf/converter).
        CN: 从 JSON 加载配置（支持多种嵌套根节点）。
        """
        
        defaults_world = {
                "add_default_floor": True, 
                "add_default_skybox": True, 
                "add_default_light": True, 
                "add_freejoint": True
            }

        if json_path is None:
            logger_json.debug("No JSON config provided. Using built-in defaults.")
            return MjcfConfig(worldbody=defaults_world.copy())

        p = Path(json_path)
        if not p.exists():
            raise FileNotFoundError(f"JSON config file not found: {p}")

        raw = json.loads(p.read_text(encoding="utf-8"))

        # 定义 mjcf_generator json 配置的根节点
        mjcf_config_key = "mjcf_generator"
        if not isinstance(raw, dict):
            logger_json.error(f"JSON config root is not a dict: {p}")
            root = {}
        elif mjcf_config_key not in raw:
            logger_json.error(f"JSON config missing '{mjcf_config_key}' dict: {p}")
            root = {}
        elif not isinstance(raw[mjcf_config_key], dict):
            logger_json.error(f"JSON config missing '{mjcf_config_key}' dict: {p}")
            root = {}
        else:
            root = raw[mjcf_config_key]
            logger_json.info(f"Loading JSON config form [{mjcf_config_key}] in [{p}]")


        config_json = cls._cfg_get(root, "config", {}) or {}
        cls.config = config_json

        compiler_json = cls._cfg_get(root, "compiler", {}) or {}
        cls.compiler = compiler_json
        option_json = cls._cfg_get(root, "option", {}) or {}
        cls.option = option_json

        visuals_json = cls._cfg_get(root, "visual", {}) or {}
        cls.visual = visuals_json

        defaults_json = cls._cfg_get(root, "defaults", {}) or {}
        cls.default = defaults_json

        assets_json = cls._cfg_get(root, "asset", {}) or {}
        cls.assets = cls._load_labeled_config_list(assets_json)

        worldbody_json = cls._cfg_get(root, "worldbody", {}) or {}
        cls.worldbody = worldbody_json

        contacts_json = cls._cfg_get(root, "contact", {}) or {}
        cls.contacts = cls._load_labeled_config_list(contacts_json)

        actuators_json = cls._cfg_get(root, "actuator", {}) or {}
        cls.actuators = cls._load_labeled_config_list(actuators_json)

        equalities_json = cls._cfg_get(root, "equality", {}) or {}
        cls.equalities = cls._load_labeled_config_list(equalities_json)

        sites_json = cls._cfg_get(root, "site", {}) or {}
        cls.sites = cls._load_labeled_config_list(sites_json)

        sensors_json = cls._cfg_get(root, "sensor", {}) or {}
        cls.sensors = cls._load_labeled_config_list(sensors_json)

        lights_json = cls._cfg_get(root, "light", {}) or {}
        cls.lights = cls._load_labeled_config_list(lights_json)

        cameras_json = cls._cfg_get(root, "camera", {}) or {}
        cls.cameras = cls._load_labeled_config_list(cameras_json)

        textures_json = cls._cfg_get(root, "texture", {}) or {}
        cls.textures = cls._load_labeled_config_list(textures_json)

        logger_json.info(
            "Loaded JSON config: actuators=%d, sensors=%d, cameras=%d, textures=%d, contacts=%d, equalitys=%d",
            len(cls.actuators),
            len(cls.sensors),
            len(cls.cameras),
            len(cls.textures),
            len(cls.contacts),
            len(cls.equalities),
        )
        return cls

# =============================================================================
# MJCF Builder
# MJCF 构建器
# =============================================================================
class MjcfBuilder:
    """
    EN: MJCF builder and asset/worldbody generator.
    CN: MJCF 构建器：生成 asset/worldbody/actuator/sensor 等块。
    """

    def __init__(self, model_name: str, json_cfg: MjcfConfig, urdf_model: UrdfParser, mjcf_path: Path):
        self.json_cfg = json_cfg
        self.urdf_model = urdf_model
        self.mjcf_path = mjcf_path

        self.root = ET.Element("mujoco", attrib={"model": model_name})
        self.compiler = ET.SubElement(self.root, "compiler")
        self.option = ET.SubElement(self.root, "option")
        self.visual = ET.SubElement(self.root, "visual")
        self.default = ET.SubElement(self.root, "default")
        self.asset = ET.SubElement(self.root, "asset")
        self.worldbody = ET.SubElement(self.root, "worldbody")
        self.contact = ET.SubElement(self.root, "contact")
        self.actuator = ET.SubElement(self.root, "actuator")
        self.equality = ET.SubElement(self.root, "equality")
        self.sensor = ET.SubElement(self.root, "sensor")


    # -------------------------
    # Core blocks
    # 核心块
    # -------------------------


    def add_default_compiler(self) -> int:
        """
        添加默认 compiler 设置
        
        """
        default_attrib = {"angle": "radian", "balanceinertia": "true"}
        for k, v in (default_attrib or {}).items():
            self.compiler.attrib[k] = str(v).lower() if isinstance(v, bool) else str(v)
        return 1
    
    def add_default_option(self) -> int:
        """
        添加默认 option 设置
        
        """
        default_attrib = {"gravity": "0 0 -9.81"}
        for k, v in (default_attrib or {}).items():
            self.option.attrib[k] = str(v).lower() if isinstance(v, bool) else str(v)
        return 1

    def add_default_visual(self) -> int:
        """
        添加默认 visual 配置
        
        """
        default_visual = """
            <visual>
                <global offwidth="640" offheight="480"/>
                <quality shadowsize="4096"/>
            </visual>
        """
        visual_elem = ET.fromstring(default_visual)
        for sub_elem in visual_elem:
            self.visual.append(sub_elem)
        return 1
    
    def add_default_default(self) -> int:
        """
        添加默认 default 配置
        
        """
        default_default = """
            <default>
                <default class="robot">
                    <default class="visual">
                        <geom contype="0" conaffinity="0" group="2"/>
                    </default>
                    <default class="collision">
                        <geom contype="1" conaffinity="1" group="3" condim="6"/>
                    </default>
                </default>
            </default>
        """
        default_elem = ET.fromstring(default_default)
        for sub_elem in default_elem:
            self.default.append(sub_elem)
        return 1


    def add_default_worldbody_floor(self) -> int:
        """
        添加默认 floor 配置
        
        """
        default_floor_geom = """
            <geom name="floor" type="plane" size="0 0 0.05" material="floor"/>
        """
        default_floor_material = """
            <material name="floor" texture="floor" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
        """
        default_floor_texture = """
            <texture name="floor" type="2d" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
        """
        self.worldbody.append(ET.fromstring(default_floor_geom))
        self.asset.append(ET.fromstring(default_floor_material))
        self.asset.append(ET.fromstring(default_floor_texture))
        return 1


    def add_default_worldbody_skybox(self) -> int:
        """
        添加默认 skybox 配置
        
        """
        default_skybox = """
            <texture name="skybox" type="skybox" builtin="gradient" rgb1=".3 .5 .7" rgb2="0 0 0" width="1024" height="1024"/>
        """
        self.asset.append(ET.fromstring(default_skybox))
        return 1


    def add_default_worldbody_light(self) -> int:
        """
        添加默认 light 配置
        
        """
        default_light = """
            <light name="default_light" pos="0 0 10" dir="0 0 -1" directional="true"/>
        """
        self.worldbody.append(ET.fromstring(default_light))
        return 1


    def add_default_worldbody_freejoint(self) -> int:
        """
        添加默认 freejoint 配置
        
        """
        default_freejoint = """
            <freejoint/>
        """
        worldbody_bodies = self.worldbody.findall("body")
        for worldbody_body in worldbody_bodies:
            worldbody_body.append(ET.fromstring(default_freejoint))
        return 1


    def add_json_compiler(self) -> int:
        """
        添加 json 文件中的 compiler 配置
        
        """
        add_num = 0
        json_attrib = self.json_cfg.compiler or {}
        for k, v in (json_attrib or {}).items():
            self.compiler.attrib[k] = str(v).lower() if isinstance(v, bool) else str(v)
            add_num += 1
        logger_mjcf.debug(f"Added compiler from json: {len(json_attrib)} attributes")
        return add_num

    def add_json_option(self) -> int:
        """
        添加 json 文件中的 option 配置
        
        """
        add_num = 0
        json_attrib = self.json_cfg.option or {}
        for k, v in (json_attrib or {}).items():
            self.option.attrib[k] = str(v).lower() if isinstance(v, bool) else str(v)
            add_num += 1
        logger_mjcf.debug(f"Added option from json: {len(json_attrib)} attributes")
        return add_num

    def add_json_visual(self) -> int:
        """
        添加 json 文件中的 visual 配置
        
        """
        add_num = 0
        json_visual = self.json_cfg.visual or {}
        for label, attribs in json_visual.items():
            sub_elem = ET.SubElement(self.visual, label)
            for k, v in (attribs or {}).items():
                sub_elem.attrib[k] = str(v).lower() if isinstance(v, bool) else str(v)
            add_num += 1
            logger_mjcf.debug(f"Added visual from json. label: {label}, num: {len(attribs)}")
        return add_num


    def add_json_default(self) -> int:
        """
        添加 json 文件中的 default 配置
        
        """
        json_default = self.json_cfg.default or {}
        pass

    @staticmethod
    def __add_json_list_configs_label(parent: ET.Element[str], configs_list: List[Dict[str, Any]]) -> int:
        """
        添加 json 文件中的列表配置
        
        parent: 父级元素
        configs_list: 列表配置
        """
        add_num = 0
        for config in configs_list: # config: {"label": "material", "name": "test_material_1", "rgba": [1, 0, 0, 1]}
            label = config.get("label")
            attribs = {k: v for k, v in config.items() if k != "label"}
            ET.SubElement(parent, label, attrib={k: str(v) for k, v in attribs.items()})
            logger_mjcf.debug(f"Added {label} from json. name: {config.get('name')}, num: {len(attribs)}")
            add_num += 1
        return add_num

    def add_json_asset(self) -> int:
        """
        添加 json 文件中的 asset 配置
        
        """
        json_assets = self.json_cfg.assets or []
        add_num = self.__add_json_list_configs_label(self.asset, json_assets)
        logger_mjcf.debug(f"Added asset from json: {len(json_assets)} assets")
        return add_num
        

    def add_json_contact(self) -> int:
        """
        添加 json 文件中的 contact 配置
        
        """
        json_contact = self.json_cfg.contacts or []
        add_num = self.__add_json_list_configs_label(self.contact, json_contact)
        logger_mjcf.debug(f"Added contact from json: {len(json_contact)} contacts")
        return add_num

    def add_json_actuator(self) -> int:
        """
        添加 json 文件中的 actuator 配置
        
        """
        json_actuator = self.json_cfg.actuators or []
        add_num = self.__add_json_list_configs_label(self.actuator, json_actuator)
        logger_mjcf.debug(f"Added actuator from json: {len(json_actuator)} actuators")
        return add_num

    def add_json_sensor(self) -> int:
        """
        添加 json 文件中的 sensor 配置
        
        """
        json_sensor = self.json_cfg.sensors or []
        add_num = self.__add_json_list_configs_label(self.sensor, json_sensor)
        logger_mjcf.debug(f"Added sensor from json: {len(json_sensor)} sensors")
        return add_num

    def add_json_equality(self) -> int:
        """
        添加 json 文件中的 equality 配置
        
        """
        json_equality = self.json_cfg.equalities or []
        add_num = self.__add_json_list_configs_label(self.equality, json_equality)
        logger_mjcf.debug(f"Added equality from json: {len(json_equality)} equalities")
        return add_num
        
    @staticmethod
    def __find_body_by_name(parent: ET.Element[str], name: str) -> Optional[ET.Element[str]]:
        """
        根据名称递归查找 body 元素

        :param parent: 父元素
        :param name: body 名称
        
        """
        # 检查父元素是否为 body 或 worldbody
        if parent.tag not in ("body", "worldbody"):
            return None
        
        # 检查当前元素是否匹配
        if parent.tag == "body" and parent.attrib.get("name") == name:
            return parent

        # 遍历所有子元素
        for child in parent:
            if child.tag == "body" and child.attrib.get("name") == name:
                return child
            # 递归搜索子元素
            result = MjcfBuilder.__find_body_by_name(child, name)
            if result is not None:
                return result
        
        return None
    
    @staticmethod
    def __add_json_list_configs_body(worldbody: ET.Element[str], configs_list: List[Dict[str, Any]], label: str) -> int:
        """
        添加 json 文件中的列表配置

        :param configs_list: 列表配置
        
        """
        add_num = 0
        for config in configs_list: # config: {"label": "ur_ft_frame", "name": "ur_ft_frame_SITE", "pos":"0 0 0", "quat": "1 0 0 0" }
            parent_body_name = config.get("label")
            if parent_body_name == "worldbody":
                parent = worldbody
            else:
                parent = MjcfBuilder.__find_body_by_name(worldbody, parent_body_name)
            # label = config.get("label")
            attribs = {k: v for k, v in config.items() if k != "label"}
            if parent is None:
                logger_mjcf.error(f"Can not find body: {parent_body_name} when add {label}")
                continue
            ET.SubElement(parent, label, attribs)
            logger_mjcf.debug(f"Added {label} to {parent_body_name} from json. name: {attribs.get('name')}")
            add_num += 1
        return add_num

    def add_json_site(self) -> int:
        """
        添加 json 文件中的 site 配置
        
        """
        json_site = self.json_cfg.sites or []
        add_num = self.__add_json_list_configs_body(self.worldbody, json_site, "site")
        logger_mjcf.debug(f"Added site from json: {len(json_site)} sites")
        return add_num

    def add_json_light(self) -> int:
        """
        添加 json 文件中的 light 配置
        
        """
        json_light = self.json_cfg.lights or []
        add_num = self.__add_json_list_configs_body(self.worldbody, json_light, "light")
        logger_mjcf.debug(f"Added light from json: {len(json_light)} lights")
        return add_num

    def add_json_camera(self) -> int:
        """
        添加 json 文件中的 camera 配置
        
        """
        json_camera = self.json_cfg.cameras or []
        add_num = self.__add_json_list_configs_body(self.worldbody, json_camera, "camera")
        logger_mjcf.debug(f"Added camera from json: {len(json_camera)} cameras")
        return add_num

    def add_json_texture(self) -> int:
        """
        添加 json 文件中的 texture 配置
        
        """
        add_num = 0
        json_texture = self.json_cfg.textures or []
        for config in json_texture:
            parent_body_name = config.get("label")
            texture_name = config.get("name")
            texture_file = config.get("file")
            texture_geom_pos = config.get("pos")
            texture_geom_quat = config.get("quat")
            texture_geom_euler = config.get("euler")
            texture_geom_size = config.get("size")

            if texture_name is None:
                logger_mjcf.error(f"The texture 'name' is None when add texture. body name: {parent_body_name}")
                continue
            if texture_file is None:
                logger_mjcf.error(f"The texture 'file' is None when add texture. texture name: {texture_name}")
                continue
            if texture_geom_size is None:
                logger_mjcf.error(f"The texture 'size' is None when add texture. texture name: {texture_name}")
                continue
            if texture_geom_pos is None:
                logger_mjcf.warning(f"The texture 'pos' is None when add texture. texture name: {texture_name}")
                continue
            if texture_geom_quat is None and texture_geom_euler is None:
                logger_mjcf.warning(f"The texture 'quat' and 'euler' is None when add texture. texture name: {texture_name}")
                continue
            if texture_geom_quat is not None and texture_geom_euler is not None:
                logger_mjcf.error(f"The texture has both 'quat' and 'euler' when add texture. texture name: {texture_name}")
                continue
            
            ET.SubElement(self.asset, "texture", attrib={"name": texture_name, "file": texture_file, "type": "2d"})
            ET.SubElement(self.asset, "material", attrib={"name": texture_name, "texture": texture_name})
            parent_body = MjcfBuilder.__find_body_by_name(self.worldbody, parent_body_name)
            texture_body_attribs = {
                "class": "visual",
                "name": texture_name,
                "type": "box",
                "size": _vec2str(x/2.0 for x in _str2vec(texture_geom_size))+" 1e-5",
                "material": texture_name,
            }
            if texture_geom_pos is not None:
                texture_body_attribs["pos"] = texture_geom_pos
            if texture_geom_quat is not None:
                texture_body_attribs["quat"] = texture_geom_quat
            elif texture_geom_euler is not None:
                texture_body_attribs["euler"] = texture_geom_euler
            MjcfBuilder._add_geom(body=parent_body, attribs=texture_body_attribs)
            add_num += 1
            logger_mjcf.debug(f"Added texture to {parent_body_name} from json. texture name: {texture_name}")
        return add_num


    def add_json_customize(self) -> int:
        """
        添加 json 文件中的 customize 配置
        
        """
        pass


    def add_json_worldbody_floor(self) -> int:
        """
        添加 json 文件中的 floor 配置
        
        """
        json_floor = self.json_cfg.worldbody.get("floor")
        if not isinstance(json_floor, dict):
            logger_mjcf.error(f"The floor config is invalid when add floor.")
            return 0
        json_floor_geom_attribs = json_floor.get("geom")
        json_floor_material_attribs = json_floor.get("material")
        json_floor_texture_attribs = json_floor.get("texture")
        if json_floor_geom_attribs is None or not isinstance(json_floor_geom_attribs, dict):
            logger_mjcf.error(f"The floor 'geom' config is invalid when add floor.")
            return 0
        if json_floor_material_attribs is None or not isinstance(json_floor_material_attribs, dict):
            logger_mjcf.error(f"The floor 'material' config is invalid when add floor.")
            return 0
        if json_floor_texture_attribs is None or not isinstance(json_floor_texture_attribs, dict):
            logger_mjcf.error(f"The floor 'texture' config is invalid when add floor.")
            return 0
        ET.SubElement(self.worldbody, "geom", attrib=json_floor_geom_attribs)
        ET.SubElement(self.asset, "material", attrib=json_floor_material_attribs)
        ET.SubElement(self.asset, "texture", attrib=json_floor_texture_attribs)
        logger_mjcf.debug(f"Added floor from json. floor geom name: {json_floor_geom_attribs.get('name')}")
        return 1


    def add_json_worldbody_skybox(self) -> int:
        """
        添加 json 文件中的 skybox 配置
        
        """
        json_skybox_texture_attribs = self.json_cfg.worldbody.get("skybox_texture")
        if not isinstance(json_skybox_texture_attribs, dict):
            return 0
        ET.SubElement(self.asset, "texture", attrib=json_skybox_texture_attribs)
        logger_mjcf.debug(f"Added skybox from json. skybox texture name: {json_skybox_texture_attribs.get('name')}")
        return 1
        

    def add_json_worldbody_freejoint(self) -> int:
        """
        添加 json 文件中的 freejoint 配置
        
        """
        add_num = 0
        json_freejoint_bodies = self.json_cfg.worldbody.get("freejoint_body")
        if not isinstance(json_freejoint_bodies, list):
            logger_mjcf.error(f"The freejoint config is invalid when add freejoint.")
            return 0
        worldbody_bodies = self.worldbody.findall("body")
        for worldbody_body in worldbody_bodies:
            if worldbody_body.attrib.get("name") in json_freejoint_bodies:
                ET.SubElement(worldbody_body, "freejoint", attrib={"name": worldbody_body.attrib.get("name")+"_freejoint"})
                logger_mjcf.debug(f"Added freejoint to {worldbody_body.attrib.get('name')} from json.")
                add_num += 1
        return add_num

    def add_urdf_material(self) -> int:
        """
        添加 urdf 文件中的 material 信息
        """
        add_num = 0
        for material in self.urdf_model.materials:
            attraib = {
                "name": material.m_name or "unkonwn_material",
                "rgba": material.color_rgba or "1 1 1 1",
            }
            ET.SubElement(self.asset, "material", attraib)
            add_num += 1
            logger_mjcf.debug(f"Added material from urdf: material name{material}")
        return add_num

    def add_urdf_contact(self) -> int:

        add_num = 0
        for joint in self.urdf_model.joints:
            if joint.j_type not in  ["revolute", "continuous", "prismatic"]:
                continue
            p_link_name = joint.parent
            c_link_name = joint.child
            attraibs = {"body1": p_link_name, "body2": c_link_name}
            ET.SubElement(self.contact, "exclude", attrib=attraibs)
            add_num += 1
        return add_num

    def add_urdf_actuator_default(self) -> None:
        """
        添加 urdf 文件中的 joint 为 actuatir 
        
        """
        pass

    def add_urdf_actuator_mj_actuator(self) -> None:
        """
        添加 urdf 文件中的 ros2_control 为 actuatir 
        
        """
        pass

    def add_urdf_equality(self) -> None:
        """
        添加 urdf 文件中的 mimic(equality) 配置
        
        """
        add_num = 0
        for joint in self.urdf_model.joints:
            if joint.j_type not in  ["revolute", "continuous", "prismatic"]:
                continue
            if joint.mimic is None:
                continue
            joint_name = joint.j_name
            mimic_link_name = joint.mimic.joint
            mimic_multiplier = joint.mimic.multiplier
            mimic_offset = joint.mimic.offset

            polycoef = [mimic_offset, mimic_multiplier, 0, 0, 0]
            attraibs = {"joint1": mimic_link_name, "joint2": joint_name, "polycoef": _vec2str(polycoef)}
            ET.SubElement(self.equality, "joint", attrib=attraibs)
            add_num += 1
        return add_num




    # -------------------------
    # Primitives
    # 基础节点
    # -------------------------

    @staticmethod
    def _add_body(parent: ET.Element[str], attribs: Dict[str, Any]) -> Optional[ET.Element[str]]:
        """
        添加 body 节点
        
        :param parent: 父节点
        :param attribs: 属性
        :return: 添加的节点
        """
        # 检查名称
        if "name" in attribs:
            if attribs["name"] == "world":
                logger_mjcf.warning("The name of body is 'world', which is reserved. It will be deleted")
            exist_bodys = parent.findall("body")
            if any(body.attrib.get("name") == attribs["name"] for body in exist_bodys):
                logger_mjcf.warning("Body name '%s' already exists in '%s', Ignoring...", attribs["name"], parent.attrib.get("name", "unknown"))
                return None
        elem_attribs = {}
        # 按照指定顺序添加关键属性
        for k in ["name", "pos", "quat", "euler"]:
            if k in attribs:
                elem_attribs[k] = str(attribs[k])
        # 添加其余属性
        for k, v in attribs.items():
            if k not in ["name", "pos", "quat", "euler"]:
                elem_attribs[k] = str(v)
        sub_elem = ET.SubElement(parent, "body", attrib=elem_attribs)
        logger_mjcf.debug("Created body: name='%s', parent='%s'", attribs.get("name", "unknown"), parent.attrib.get("name", "unknown"))
        return sub_elem
    
    @staticmethod
    def _add_inertial(body: ET.Element[str], attribs: Dict[str, Any]) ->  Optional[ET.Element[str]]:
        """
        添加 inertial 节点
        
        :param body: 父节点
        :param attribs: 节点属性
        :return: 添加的节点
        """
        # 检查
        if any(body.findall("inertial")):
            logger_mjcf.warning("Inertial already exists in '%s', Ignoring...", body.attrib.get("name", "unknown"))
            return None
        elem_attribs = {}
        # 按照指定顺序添加关键属性
        for k in ["mass", "diaginertia", "pos", "quat", "euler"]:
            if k in attribs:
                elem_attribs[k] = str(attribs[k])
        # 添加其余属性
        for k, v in attribs.items():
            if k not in ["mass", "diaginertia", "pos", "quat", "euler"]:
                elem_attribs[k] = str(v)
        logger_mjcf.debug("Created inertial: body='%s'", body.attrib.get("name", "unknown"))
        sub_elem = ET.SubElement(body, "inertial", attrib=elem_attribs)
        return sub_elem
    
    @staticmethod
    def _add_joint(body: ET.Element[str], attribs: Dict[str, Any]) ->  Optional[ET.Element[str]]:
        """
        添加 joint 节点
        
        :param body: 父节点
        :param attribs: 节点属性
        :return: 添加的节点
        """
        # 检查
        if "name" in attribs:
            exist_joints = body.findall("joint")
            if any(joint.attrib.get("name") == attribs["name"] for joint in exist_joints):
                logger_mjcf.warning("Joint name '%s' already exists in '%s', Ignoring...", attribs["name"], body.attrib.get("name", "unknown"))
                return None
        elem_attribs = {}
        # 按照指定顺序添加关键属性
        for k in ["name", "type", "pos", "axis", "ref", "range"]:
            if k in attribs:
                elem_attribs[k] = str(attribs[k])
        # 添加其余属性
        for k, v in attribs.items():
            if k not in ["name", "type", "axis", "ref", "range"]:
                elem_attribs[k] = str(v)
        logger_mjcf.debug("Created joint: name='%s', parent='%s'", attribs.get("name", "unknown"), body.attrib.get("name", "unknown"))
        sub_elem = ET.SubElement(body, "joint", attrib=elem_attribs)
        return sub_elem

    @staticmethod
    def _add_geom(body: ET.Element[str], attribs: Dict[str, Any]) ->  Optional[ET.Element[str]]:
        """
        添加 geom 节点
        
        :param body: 父节点
        :param attribs: 节点属性
        :return: 添加的节点
        """
        # 检查
        if "name" in attribs:
            exist_geoms = body.findall("geom")
            if any(geom.attrib.get("name") == attribs["name"] for geom in exist_geoms):
                logger_mjcf.warning("Geom name '%s' already exists in '%s', Ignoring...", attribs["name"], body.attrib.get("name", "unknown"))
                return None
        elem_attribs = {}
        # 按照指定顺序添加关键属性
        for k in ["class", "name", "type", "mesh", "material", "rgba", "pos", "quat", "euler"]:
            if k in attribs:
                elem_attribs[k] = str(attribs[k])
        # 添加其余属性
        for k, v in attribs.items():
            if k not in ["class", "name", "type", "mesh", "material", "rgba", "pos", "quat", "euler"]:
                elem_attribs[k] = str(v)
        logger_mjcf.debug("Created geom: name='%s', body='%s'", attribs.get("name", "unknown"), body.attrib.get("name", "unknown"))
        sub_elem = ET.SubElement(body, "geom", attrib=elem_attribs)
        return sub_elem
    
    @staticmethod
    def _add_mesh(asset: ET.Element[str], attribs: Dict[str, Any]) ->  Optional[ET.Element[str]]:
        """
        添加mesh

        :param asset: 父 asset 元素
        :param attribs: 属性
        :return: 添加的 mesh 元素
        """
        # 检查mesh
        if "name" in attribs:
            exist_meshes = asset.findall("mesh")
            if any(mesh.attrib.get("name") == attribs["name"] for mesh in exist_meshes):
                logger_mjcf.warning("Mesh with name '%s' already exists, Ignoring...", attribs["name"])
                return None
        elem_attribs = {}
        # 按照指定顺序添加关键属性
        for k in ["name", "file", "scale", "material"]:
            if k in attribs:
                elem_attribs[k] = str(attribs[k])
        # 添加其他属性
        for k, v in attribs.items():
            if k not in ["name", "file", "scale", "material"]:
                elem_attribs[k] = str(v)
        logger_mjcf.debug("Created mesh: name=%s, file=%s", attribs.get("name", "unknown"), attribs.get("file", "unknown"))
        sub_elem = ET.SubElement(asset, "mesh", attrib=elem_attribs)
        return sub_elem
        

    @staticmethod
    def _build_body(parent: ET.Element[str], link: UrdfParser.UrdfLink, joint:Optional[UrdfParser.UrdfJoint],  asset:ET.Element[str], urdf_file_path: Path = None, mjcf_output_path: Path = None) -> Optional[ET.Element[str]]:
        """
        构建 body 节点
        
        :param parent: 父节点
        :param link: 要构建的 body 在 urdf 中的 link
        :param joint: 以 link 为子节点的 joint
        :param asset: 资源节点，用于添加 mesh
        :param urdf_file_path: URDF文件路径，用于解析相对路径的mesh
        :param mjcf_output_path: MJCF输出路径，用于生成相对路径
        :return: 添加的 body 节点
        """
        # 构建 body
        body_attribs = {
            "name": link.l_name,
            "pos": joint.origin.xyz if joint is not None else "0 0 0",
            "quat": _vec2str(_rpy_to_quaternion(_str2vec(joint.origin.rpy))) if joint is not None else "0 0 0",
        }
        body = MjcfBuilder._add_body(parent, body_attribs)
        if body is None:
            return None

        # 构建 inertial
        if link.inertial is not None:
            inertial_quat = _rpy_to_quaternion(_str2vec(link.inertial.origin.rpy))
            if abs(link.inertial.inertia.ixy) > 1e-12 or abs(link.inertial.inertia.ixz) > 1e-12 or abs(link.inertial.inertia.iyz) > 1e-12:
                diaginertia, new_quat = _diagonalize_inertia(
                    ixx=link.inertial.inertia.ixx,
                    ixy=link.inertial.inertia.ixy,
                    ixz=link.inertial.inertia.ixz,
                    iyy=link.inertial.inertia.iyy, 
                    iyz=link.inertial.inertia.iyz,
                    izz=link.inertial.inertia.izz,
                    qw=inertial_quat[0],
                    qx=inertial_quat[1],
                    qy=inertial_quat[2],
                    qz=inertial_quat[3]
                )
            else:
                diaginertia = (link.inertial.inertia.ixx, link.inertial.inertia.iyy, link.inertial.inertia.izz)
                new_quat = inertial_quat
            inertial_attribs = {
                "mass": link.inertial.mass,
                "pos": link.inertial.origin.xyz,
                "quat": _vec2str(new_quat),
                "diaginertia": _vec2str(diaginertia),
            }
            MjcfBuilder._add_inertial(body, inertial_attribs)

        # 构建 joint
        if joint is not None:
            if joint.j_type in ["revolute", "continuous", "prismatic"]:
                joint_attribs = {
                    "name": joint.j_name,
                    "axis": joint.axis,
                    # "pos": "0 0 0", #joint.origin.xyz,
                    "ref": ""
                }
                if joint.limit is not None:
                    joint_attribs["range"] = _vec2str([joint.limit.lower, joint.limit.upper])
                if joint.j_type == "revolute":
                    joint_attribs["type"] = "hinge"
                elif joint.j_type == "continuous":
                    joint_attribs["type"] = "hinge"
                elif joint.j_type == "prismatic":
                    joint_attribs["type"] = "slide"
                MjcfBuilder._add_joint(body, joint_attribs)

        # 构建 collision geom
        if link.collisions is not None:
            geom_index = 0
            for collision in link.collisions:
                collision_geom_attribs = {
                    "class": "collision",
                    "name": "COLLISION_" + link.l_name + "_" + str(geom_index),
                    "type": collision.geometry.g_type,
                    "pos": collision.origin.xyz,
                    "quat": _vec2str(_rpy_to_quaternion(_str2vec(collision.origin.rpy))),
                }
                if collision.material is not None:
                    if collision.material.color_rgba is not None:
                        collision_geom_attribs["rgba"] = collision.material.color_rgba
                    elif collision.material.m_name is not None:
                        collision_geom_attribs["material"] = collision.material.m_name
                if collision.geometry.g_type == "mesh":
                    mesh_name = "MESH_" + collision.geometry.filename.split("/")[-1].split(".")[0] # 文件名不带扩展名
                    if urdf_file_path and mjcf_output_path:
                        mesh_file_path = _resolve_mesh_path(urdf_file_path, collision.geometry.filename, mjcf_output_path)
                    else:
                        mesh_file_path = collision.geometry.filename
                    MjcfBuilder._add_mesh(asset=asset, attribs={"name": mesh_name, "file": mesh_file_path})
                    collision_geom_attribs["mesh"] = mesh_name
                elif collision.geometry.g_type == "box":
                    collision_geom_attribs["size"] = _vec2str(x/2.0 for x in _str2vec(collision.geometry.size))
                elif collision.geometry.g_type == "sphere":
                    collision_geom_attribs["size"] = collision.geometry.radius
                elif collision.geometry.g_type == "cylinder":
                    collision_geom_attribs["size"] = collision.geometry.radius + " " + str(float(collision.geometry.length)/2)
                MjcfBuilder._add_geom(body=body, attribs=collision_geom_attribs)
                geom_index += 1

        # 构建 visual geom
        if link.visuals is not None:
            geom_index = 0
            for visual in link.visuals:
                visual_geom_attribs = {
                    "class": "visual",
                    "name": "VISUAL_" + link.l_name + "_" + str(geom_index),
                    "type": visual.geometry.g_type,
                    "pos": visual.origin.xyz,
                    "quat": _vec2str(_rpy_to_quaternion(_str2vec(visual.origin.rpy))),
                }
                if visual.material is not None:
                    if visual.material.color_rgba is not None:
                        visual_geom_attribs["rgba"] = visual.material.color_rgba
                    elif visual.material.m_name is not None:
                        visual_geom_attribs["material"] = visual.material.m_name
                if visual.geometry.g_type == "mesh":
                    mesh_name = "MESH_" + visual.geometry.filename.split("/")[-1].split(".")[0] # 文件名不带扩展名
                    if urdf_file_path and mjcf_output_path:
                        mesh_file_path = _resolve_mesh_path(urdf_file_path, visual.geometry.filename, mjcf_output_path)
                    else:
                        mesh_file_path = visual.geometry.filename
                    MjcfBuilder._add_mesh(asset=asset, attribs={"name": mesh_name, "file": mesh_file_path})
                    visual_geom_attribs["mesh"] = mesh_name
                elif visual.geometry.g_type == "box":
                    visual_geom_attribs["size"] = _vec2str(x/2.0 for x in _str2vec(visual.geometry.size))
                elif visual.geometry.g_type == "sphere":
                    visual_geom_attribs["size"] = visual.geometry.radius
                elif visual.geometry.g_type == "cylinder":
                    visual_geom_attribs["size"] = visual.geometry.radius + " " + str(float(visual.geometry.length)/2)
                MjcfBuilder._add_geom(body=body, attribs=visual_geom_attribs)
                geom_index += 1
        return body


    def build_worldbody_from_urdf(self) -> ET.Element[str]:
        """
        Builds the worldbody from the URDF model
        """
        root_name = self.urdf_model.root_link
        # get the UrdfLink named child_name
        urdf_link = list(filter(lambda link: link.l_name == root_name, self.urdf_model.links))[0]
        # get the UrdfJoints which are parented to child_name
        urdf_joint = list(filter(lambda joint: joint.child == root_name, self.urdf_model.joints))
        if len(urdf_joint) > 0:
            urdf_joint = urdf_joint[0]
        else:
            urdf_joint = None
        # build the root body to worldbody
        root_body: ET.Element[str] = MjcfBuilder._build_body(self.worldbody, urdf_link, urdf_joint, asset=self.asset, 
                                                            urdf_file_path=self.urdf_model.urdf_file_path, 
                                                            mjcf_output_path=self.mjcf_path)
        
        def rec_build_bodies(parent_elem: ET.Element[str],):
            """
            Recursively build the mjcf bodies
            """
            parent_name = parent_elem.attrib["name"]
            if parent_name in self.urdf_model.parent_map:
                for child_name in self.urdf_model.parent_map[parent_name]:
                    # get the UrdfLink named child_name
                    urdf_link = list(filter(lambda link: link.l_name == child_name, self.urdf_model.links))[0]
                    # get the UrdfJoints which are parented to child_name
                    urdf_joint = list(filter(lambda joint: joint.child == child_name, self.urdf_model.joints))
                    if len(urdf_joint) > 0:
                        urdf_joint = urdf_joint[0]
                    else:
                        urdf_joint = None

                    body = MjcfBuilder._build_body(parent_elem, urdf_link, urdf_joint, asset=self.asset, 
                                                   urdf_file_path=self.urdf_model.urdf_file_path, 
                                                   mjcf_output_path=self.mjcf_path)

                    rec_build_bodies(body)

        # recursively build bodies
        rec_build_bodies(root_body)

        # delete root body which is named "world"
        if root_body.attrib.get('name') == "world":
            logger_mjcf.warning("Root body name is 'world', deleting it...")
            sub_elems = root_body.findall('body')
            self.worldbody.remove(root_body)
            logger_mjcf.warning("Root body name is 'world', deleting it... Done")
            for sub_elem in sub_elems:
                self.worldbody.append(sub_elem)
            logger_mjcf.warning(f"'worldbody' now contains {len(sub_elems)} sub-elements")

        return root_body

    def save_mjcf(self, filename):
        """
        Save the MJCF model to a file.

        Parameters
        filename : str
            The path to the file.
        """
        ET.ElementTree(self.root).write(filename, pretty_print=True)


# =============================================================================
# Public interface
# 对外接口
# =============================================================================
def mjcf_generator(
    urdf_model: UrdfParser,
    mjcf_path: Union[str, Path],
    json_config_path: Optional[Union[str, Path]] = None,
) -> None:
    """
    EN: Convert URDF to MJCF.
    CN: 将 URDF 转换为 MJCF。
    """

    mjcf_path = Path(mjcf_path)

    logger.info("MJCF generation started: output_mjcf='%s'", str(mjcf_path))
    cfg = MjcfConfig.load(json_config_path)
    model = urdf_model

    builder = MjcfBuilder(model_name=model.robot_name, json_cfg=cfg, urdf_model=model, mjcf_path=mjcf_path)

    builder.build_worldbody_from_urdf()

    # Actuator rules: URDF ros2_control + JSON + default fallback
    config_cfg = cfg.config or {}
    worldbody_cfg = cfg.worldbody or {}

    if bool(config_cfg.get("add_default_compiler", True)):
        is_add_default_compiler = builder.add_default_compiler()
    if bool(config_cfg.get("add_default_option", True)):
        is_add_default_option = builder.add_default_option()
    if bool(config_cfg.get("add_default_visual", True)):
        is_add_default_visual = builder.add_default_visual()
    if bool(config_cfg.get("add_default_default", True)):
        is_add_default_default = builder.add_default_default()
    if bool(config_cfg.get("add_default_contact", True)):
        is_add_default_contact = builder.add_urdf_contact()
    if bool(worldbody_cfg.get("add_default_floor", True)):
        is_add_default_worldbody_floor = builder.add_default_worldbody_floor()
    if bool(worldbody_cfg.get("add_default_skybox", True)):
        is_add_default_worldbody_skybox = builder.add_default_worldbody_skybox()
    if bool(worldbody_cfg.get("add_default_light", True)):
        is_add_default_worldbody_light = builder.add_default_worldbody_light()
    if bool(worldbody_cfg.get("add_default_freejoint", True)):
        is_add_default_worldbody_freejoint = builder.add_default_worldbody_freejoint()

    if bool(config_cfg.get("add_json_compiler", False)):
        is_add_json_compiler = builder.add_json_compiler()
    if bool(config_cfg.get("add_json_option", False)):
        is_add_json_option = builder.add_json_option()
    if bool(config_cfg.get("add_json_visual", False)):
        is_add_json_visual = builder.add_json_visual()
    if bool(config_cfg.get("add_json_default", False)):
        is_add_json_default = builder.add_json_default()
    if bool(config_cfg.get("add_json_asset", False)):
        is_add_json_asset = builder.add_json_asset()
    if bool(config_cfg.get("add_json_contact", False)):
        is_add_json_contact = builder.add_json_contact()
    if bool(config_cfg.get("add_json_equality", False)):
        is_add_json_equality = builder.add_json_equality()
    if bool(config_cfg.get("add_json_actuator", False)):
        is_add_json_actuator = builder.add_json_actuator()
    if bool(config_cfg.get("add_json_site", False)):
        is_add_json_visual = builder.add_json_site()
    if bool(config_cfg.get("add_json_sensor", False)):
        is_add_json_sensor = builder.add_json_sensor()
    if bool(config_cfg.get("add_json_light", False)):
        is_add_json_light = builder.add_json_light()
    if bool(config_cfg.get("add_json_camera", False)):
        is_add_json_camera = builder.add_json_camera()
    if bool(config_cfg.get("add_json_texture", False)):
        is_add_json_texture = builder.add_json_texture()
    if bool(config_cfg.get("add_json_customize", False)):
        is_add_json_customize = builder.add_json_customize()
    if bool(worldbody_cfg.get("add_json_floor", False)):
        is_add_json_worldbody_floor = builder.add_json_worldbody_floor()
    if bool(worldbody_cfg.get("add_json_skybox", False)):
        is_add_json_worldbody_skybox = builder.add_json_worldbody_skybox()
    if bool(worldbody_cfg.get("add_json_freejoint", False)):
        is_add_json_worldbody_freejoint = builder.add_json_worldbody_freejoint()

    if bool(config_cfg.get("add_urdf_material", True)):
        is_add_urdf_material = builder.add_urdf_material()
    if bool(config_cfg.get("add_default_actuator", False)):
        is_add_urdf_actuator_default = builder.add_urdf_actuator_default()
    if bool(config_cfg.get("add_plugin_actuator", False)):
        is_add_urdf_actuator_mj_actuator = builder.add_urdf_actuator_mj_actuator()
    if bool(config_cfg.get("add_urdf_equality", True)):
        is_add_urdf_equality = builder.add_urdf_equality()


    # logger.info(
    #     "Actuators: urdf=%s, json=%s, default=%s | Sensors: urdf=%s, json=%s | Cameras: json=%s | Textures: json=%s | Contact: json=%s",
    #     "yes" if added_act_urdf else "no",
    #     "yes" if added_act_json else "no",
    #     "yes" if added_act_def else "no",
    #     "yes" if added_sens_urdf else "no",
    #     "yes" if added_sens_json else "no",
    #     "yes" if added_cam_json else "no",
    #     "yes" if added_tex_json else "no",
    #     "yes" if added_contact_json else "no",
    # )

    _pretty_write_xml(builder.root, mjcf_path)
    logger.info("Saved MJCF: %s", str(mjcf_path))


if __name__ == "__main__":
    # EN: Do not configure logging here; the CLI is the recommended entry.
    # CN: 这里不配置 logging；推荐从 CLI 调用。
    urdf_path="./examples/urdf/UR5e_fixed_0.urdf"
    urdf_model = UrdfParser.parse_urdf(urdf_path)
    mjcf_generator(urdf_model=urdf_model, mjcf_path="./examples/mjcf/UR5e0.xml", json_config_path="./examples/config/config.json")
    pass