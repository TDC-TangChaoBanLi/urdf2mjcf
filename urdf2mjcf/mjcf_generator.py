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

# IMPORTANT:
# EN: Do NOT call logging.basicConfig() here. CLI configures logging globally.
# CN: 不要在此模块内配置 logging.basicConfig()，由 CLI 统一配置日志风格。
logger = logging.getLogger("urdf2mjcf.mjcf_generator")


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


def _vec3_str(x: str) -> str:
    """
    EN: Normalize "x y z" string to numeric triple.
    CN: 将 "x y z" 字符串规范化为数值三元组格式。
    """
    vals = x.split()
    if len(vals) != 3:
        return "0 0 0"
    return " ".join(str(float(v)) for v in vals)


def _quat_str(q: List[float]) -> str:
    """EN: quat list -> string. CN: 四元数列表转字符串。"""
    return f"{q[0]} {q[1]} {q[2]} {q[3]}"


def _quat_normalize(q: List[float]) -> List[float]:
    """EN: Normalize quaternion. CN: 归一化四元数。"""
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    return [v / n for v in q]


def _quat_multiply(q1: List[float], q2: List[float]) -> List[float]:
    """EN: Quaternion multiply (q1 * q2). CN: 四元数乘法 (q1 * q2)。"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return _quat_normalize([w, x, y, z])


def _rpy_to_quat(rpy: str) -> List[float]:
    """
    EN: Convert roll-pitch-yaw (rpy) string to quaternion.
    CN: 将 rpy 字符串转换为四元数。
    """
    try:
        r, p, y = map(float, rpy.split())
    except Exception:
        r, p, y = 0.0, 0.0, 0.0
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return _quat_normalize([qw, qx, qy, qz])


def _rot_to_quat(R: np.ndarray) -> List[float]:
    """EN: Rotation matrix -> quaternion. CN: 旋转矩阵转四元数。"""
    trace = float(np.trace(R))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + float(R[0, 0]) - float(R[1, 1]) - float(R[2, 2])) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + float(R[1, 1]) - float(R[0, 0]) - float(R[2, 2])) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + float(R[2, 2]) - float(R[0, 0]) - float(R[1, 1])) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return _quat_normalize([qw, qx, qy, qz])


def _diagonalize_inertia(
    ixx: float, iyy: float, izz: float, ixy: float, ixz: float, iyz: float
) -> Tuple[Tuple[float, float, float], List[float]]:
    """
    EN: Diagonalize inertia matrix to principal moments and rotation quaternion.
    CN: 将惯性矩阵对角化，得到主惯量与旋转四元数。
    """
    I = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float)
    w, V = np.linalg.eigh(I)
    idx = np.argsort(w)
    w = w[idx]
    V = V[:, idx]
    if np.linalg.det(V) < 0:
        V[:, -1] *= -1.0
    R_old_to_principal = V.T
    q_old_to_principal = _rot_to_quat(R_old_to_principal)
    return (float(w[0]), float(w[1]), float(w[2])), q_old_to_principal


def _strip_file_uri(path: str) -> str:
    """EN: Remove file:// prefix if present. CN: 去掉 file:// 前缀。"""
    if path.startswith("file://"):
        return path[len("file://") :]
    return path


def _mesh_token_from_filename(mesh_filename: str) -> str:
    """
    EN: Turn filename into a safe token for MJCF naming.
    CN: 将 mesh 文件名转换为安全 token，用于 MJCF 资产命名。
    """
    mesh_filename = _strip_file_uri(mesh_filename)
    base = Path(mesh_filename).name
    base = base.replace(".", "_")
    base = re.sub(r"[^0-9A-Za-z_]+", "_", base)
    return base


def _resolve_mesh_path_preserve_symlink_semantics(
    mesh_filename: str, urdf_path: Path, mjcf_path: Path, package_map: Dict[str, str]
) -> str:
    """
    EN:
    Resolve mesh path for MJCF, but preserve symlink semantics:
    - Do not call resolve()/realpath() on filesystem symlinks
    - Keep absolute paths as-is
    - Relative paths are resolved relative to URDF dir, then written as relative to MJCF dir if possible

    CN:
    解析 mesh 路径用于 MJCF，但保持“符号链接语义”：
    - 不对符号链接做 resolve()/realpath()
    - 绝对路径原样保留
    - 相对路径相对 URDF 目录解析，再尽量转为相对 MJCF 输出目录的路径
    """
    mesh_filename = _strip_file_uri(mesh_filename)

    if mesh_filename.startswith("package://"):
        m = re.match(r"package://([^/]+)/(.+)$", mesh_filename)
        if m:
            pkg, rel = m.group(1), m.group(2)
            if pkg in package_map:
                full = Path(package_map[pkg]) / rel
                return full.as_posix()
            logger.warning(
                "package:// URI not resolved (missing package_map for '%s'): %s",
                pkg,
                mesh_filename,
            )
        return mesh_filename

    p = Path(mesh_filename)
    if p.is_absolute():
        return p.as_posix()

    urdf_dir = urdf_path.parent
    full = Path(os.path.abspath(os.path.join(str(urdf_dir), str(p))))  # no resolve
    try:
        rel = os.path.relpath(str(full), str(mjcf_path.parent))
        return Path(rel).as_posix()
    except Exception:
        return full.as_posix()


# =============================================================================
# Config (JSON)
# JSON 配置
# =============================================================================
def _cfg_get(d: Dict[str, Any], path: str, default: Any = None) -> Any:
    """EN: Nested dict getter. CN: 字典嵌套路径读取。"""
    cur: Any = d
    for key in path.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


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
    defaults: Dict[str, Any] = field(default_factory=dict)
    asset: Dict[str, Any] = field(default_factory=dict)
    worldbody: Dict[str, Any] = field(default_factory=dict)
    contact: Dict[str, Any] = field(default_factory=dict)
    equality: Dict[str, Any] = field(default_factory=dict)

    actuators: List[Dict[str, Any]] = field(default_factory=list)
    sensors: List[Dict[str, Any]] = field(default_factory=list)
    cameras: List[Dict[str, Any]] = field(default_factory=list)

    package_map: Dict[str, str] = field(default_factory=dict)

    @staticmethod
    def load(json_path: Optional[Union[str, Path]]) -> "MjcfConfig":
        """
        EN: Load config from JSON (supports nested roots: mjcf/mujoco/urdf_to_mjcf/converter).
        CN: 从 JSON 加载配置（支持多种嵌套根节点）。
        """
        defaults_world = {"add_floor": True, "add_skybox": True, "add_freejoint": True, "add_light": True}

        if json_path is None:
            logger.debug("No JSON config provided. Using built-in defaults.")
            return MjcfConfig(worldbody=defaults_world.copy())

        p = Path(json_path)
        if not p.exists():
            raise FileNotFoundError(f"JSON config file not found: {p}")

        raw = json.loads(p.read_text(encoding="utf-8"))

        root = raw
        for k in ("mjcf", "mujoco", "urdf_to_mjcf", "converter"):
            if isinstance(root, dict) and k in root and isinstance(root[k], dict):
                root = root[k]

        config = _cfg_get(root, "config", {}) or {}
        if isinstance(config, dict):
            for k, v in config.items():
                if k in defaults_world:
                    defaults_world[k] = v
        else:
            config = defaults_world.copy()

        worldbody = _cfg_get(root, "worldbody", {}) or {}
        if isinstance(worldbody, dict):
            for k, v in defaults_world.items():
                worldbody.setdefault(k, v)
        else:
            worldbody = defaults_world.copy()

        actuator = _cfg_get(root, "actuator", {}) or {}
        sensor = _cfg_get(root, "sensor", {}) or {}
        camera = _cfg_get(root, "camera", {}) or {}

        # Flatten actuator list
        actuators: List[Dict[str, Any]] = []
        if isinstance(actuator, dict):
            for act_type, act_list in actuator.items():
                if isinstance(act_list, list):
                    for act in act_list:
                        if isinstance(act, dict):
                            a = act.copy()
                            a["type"] = act_type
                            actuators.append(a)

        # Flatten sensor list
        sensors: List[Dict[str, Any]] = []
        if isinstance(sensor, dict):
            for sens_type, sens_list in sensor.items():
                if isinstance(sens_list, list):
                    for sens in sens_list:
                        if isinstance(sens, dict):
                            s = sens.copy()
                            s["type"] = sens_type
                            sensors.append(s)

        # Flatten cameras list: each cam gets "body"
        cameras: List[Dict[str, Any]] = []
        if isinstance(camera, dict):
            for body_name, cam_list in camera.items():
                if isinstance(cam_list, list):
                    for cam in cam_list:
                        if isinstance(cam, dict):
                            c = cam.copy()
                            c["body"] = body_name
                            cameras.append(c)

        cfg = MjcfConfig(
            config=config,
            compiler=_cfg_get(root, "compiler", {}) or {},
            option=_cfg_get(root, "option", {}) or {},
            visual=_cfg_get(root, "visual", {}) or {},
            defaults=_cfg_get(root, "default", _cfg_get(root, "defaults", {})) or {},
            asset=_cfg_get(root, "asset", {}) or {},
            worldbody=worldbody,
            contact=_cfg_get(root, "contact", {}) or {},
            equality=_cfg_get(root, "equality", {}) or {},
            actuators=actuators,
            sensors=sensors,
            cameras=cameras,
            package_map=_cfg_get(root, "package_map", {}) or {},
        )

        logger.info(
            "Loaded JSON config: file='%s', actuators=%d, sensors=%d, cameras=%d",
            str(p),
            len(cfg.actuators),
            len(cfg.sensors),
            len(cfg.cameras),
        )
        return cfg


# =============================================================================
# URDF IR (internal representation)
# URDF 中间表示
# =============================================================================
@dataclass
class UrdfMaterial:
    name: str
    rgba: Optional[str] = None


@dataclass
class UrdfGeometry:
    kind: str  # mesh|box|cylinder|sphere
    mesh_filename: Optional[str] = None
    scale: Optional[str] = None
    size: Optional[str] = None


@dataclass
class UrdfGeom:
    name: str
    origin_pos: str
    origin_quat: str
    geometry: UrdfGeometry
    material_name: Optional[str] = None


@dataclass
class UrdfInertial:
    pos: str
    quat: str
    mass: float
    diaginertia: Tuple[float, float, float]


@dataclass
class UrdfJoint:
    name: str
    jtype: str
    parent: str
    child: str
    origin_pos: str
    origin_quat: str
    axis: str = "0 0 1"
    limit_lower: Optional[float] = None
    limit_upper: Optional[float] = None
    mimic_joint: Optional[str] = None
    mimic_multiplier: float = 1.0
    mimic_offset: float = 0.0


@dataclass
class UrdfLink:
    name: str
    inertial: Optional[UrdfInertial] = None
    visuals: List[UrdfGeom] = field(default_factory=list)
    collisions: List[UrdfGeom] = field(default_factory=list)


# ---------- ros2_control parsing (MujocoSystem) ----------
@dataclass
class Ros2ControlJoint:
    name: str
    command_interfaces: List[str] = field(default_factory=list)
    state_interfaces: List[str] = field(default_factory=list)
    params: Dict[str, str] = field(default_factory=dict)


@dataclass
class Ros2ControlSensor:
    name: str
    state_interfaces: List[str] = field(default_factory=list)
    params: Dict[str, str] = field(default_factory=dict)


@dataclass
class Ros2ControlInfo:
    present: bool = False
    plugin: str = ""
    joints: Dict[str, Ros2ControlJoint] = field(default_factory=dict)
    sensors: Dict[str, Ros2ControlSensor] = field(default_factory=dict)


@dataclass
class UrdfModel:
    robot_name: str
    materials: Dict[str, UrdfMaterial]
    links: Dict[str, UrdfLink]
    joints: Dict[str, UrdfJoint]
    root_link: str
    parent_map: Dict[str, List[str]]
    child_to_joint: Dict[str, str]
    ros2_control: Ros2ControlInfo


class UrdfParser:
    """
    EN: Parse URDF XML into UrdfModel.
    CN: 解析 URDF XML 生成 UrdfModel。
    """

    def __init__(self, urdf_path: Union[str, Path], cfg: MjcfConfig):
        self.urdf_path = Path(urdf_path)
        self.cfg = cfg
        if not self.urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {self.urdf_path}")

        self.tree = ET.parse(self.urdf_path)
        self.robot = self.tree.getroot()
        if self.robot is None or self.robot.tag != "robot":
            raise ValueError("Invalid URDF: root element is not <robot>")

    def parse(self) -> UrdfModel:
        robot_name = self.robot.attrib.get("name", "converted_robot")
        materials = self._parse_materials()
        links = self._parse_links(materials)
        joints, parent_map, child_to_joint = self._parse_joints()
        root_link = self._find_root_link(links, child_to_joint)
        ros2 = self._parse_ros2_control_mujoco_system()

        logger.info(
            "Parsed URDF: robot='%s', links=%d, joints=%d, root='%s', ros2_control=%s",
            robot_name,
            len(links),
            len(joints),
            root_link,
            "yes" if ros2.present else "no",
        )

        return UrdfModel(
            robot_name=robot_name,
            materials=materials,
            links=links,
            joints=joints,
            root_link=root_link,
            parent_map=parent_map,
            child_to_joint=child_to_joint,
            ros2_control=ros2,
        )

    def _parse_materials(self) -> Dict[str, UrdfMaterial]:
        mats: Dict[str, UrdfMaterial] = {}
        for m in self.robot.findall("material"):
            name = m.attrib.get("name")
            if not name:
                continue
            rgba = None
            color = m.find("color")
            if color is not None:
                rgba = color.attrib.get("rgba")
            mats[name] = UrdfMaterial(name=name, rgba=rgba)

        for link in self.robot.findall("link"):
            for visual in link.findall("visual"):
                vm = visual.find("material")
                if vm is None:
                    continue
                name = vm.attrib.get("name")
                if not name:
                    continue
                rgba = None
                color = vm.find("color")
                if color is not None:
                    rgba = color.attrib.get("rgba")
                if name not in mats:
                    mats[name] = UrdfMaterial(name=name, rgba=rgba)
                else:
                    if mats[name].rgba is None and rgba is not None:
                        mats[name].rgba = rgba
        return mats

    def _parse_links(self, materials: Dict[str, UrdfMaterial]) -> Dict[str, UrdfLink]:
        links: Dict[str, UrdfLink] = {}
        for link_elem in self.robot.findall("link"):
            lname = link_elem.attrib.get("name")
            if not lname:
                continue
            link = UrdfLink(name=lname)
            link.inertial = self._parse_inertial(link_elem, lname)
            link.visuals = self._parse_visuals(link_elem, lname, materials)
            link.collisions = self._parse_collisions(link_elem, lname)
            links[lname] = link
        return links

    def _parse_inertial(self, link_elem: ET.Element, link_name: str) -> Optional[UrdfInertial]:
        inertial = link_elem.find("inertial")
        if inertial is None:
            return None

        origin = inertial.find("origin")
        pos = origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"
        rpy = origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"
        q_link_to_inertial = _rpy_to_quat(rpy)

        mass_elem = inertial.find("mass")
        mass = _safe_float(mass_elem.attrib.get("value") if mass_elem is not None else None, 0.0)

        inertia = inertial.find("inertia")
        if inertia is None:
            return UrdfInertial(pos=_vec3_str(pos), quat=_quat_str(q_link_to_inertial), mass=mass, diaginertia=(0.0, 0.0, 0.0))

        ixx = _safe_float(inertia.attrib.get("ixx"), 0.0)
        iyy = _safe_float(inertia.attrib.get("iyy"), 0.0)
        izz = _safe_float(inertia.attrib.get("izz"), 0.0)
        ixy = _safe_float(inertia.attrib.get("ixy"), 0.0)
        ixz = _safe_float(inertia.attrib.get("ixz"), 0.0)
        iyz = _safe_float(inertia.attrib.get("iyz"), 0.0)

        has_off = (abs(ixy) > 1e-12) or (abs(ixz) > 1e-12) or (abs(iyz) > 1e-12)
        if not has_off:
            return UrdfInertial(pos=_vec3_str(pos), quat=_quat_str(q_link_to_inertial), mass=mass, diaginertia=(ixx, iyy, izz))

        principal, q_old_to_principal = _diagonalize_inertia(ixx, iyy, izz, ixy, ixz, iyz)
        q_link_to_principal = _quat_multiply(q_link_to_inertial, q_old_to_principal)

        logger.info("Diagonalized inertia: link='%s', principal=%s", link_name, principal)
        return UrdfInertial(pos=_vec3_str(pos), quat=_quat_str(q_link_to_principal), mass=mass, diaginertia=principal)

    def _parse_geometry(self, geom_elem: ET.Element, default_box_half: str = "0.1 0.1 0.1") -> UrdfGeometry:
        box = geom_elem.find("box")
        if box is not None:
            size_full = box.attrib.get("size", "0 0 0").split()
            if len(size_full) == 3:
                half = [float(size_full[i]) / 2.0 for i in range(3)]
                return UrdfGeometry(kind="box", size=f"{half[0]} {half[1]} {half[2]}")
            return UrdfGeometry(kind="box", size=default_box_half)

        cyl = geom_elem.find("cylinder")
        if cyl is not None:
            r = _safe_float(cyl.attrib.get("radius"), 0.1)
            L = _safe_float(cyl.attrib.get("length"), 1.0)
            return UrdfGeometry(kind="cylinder", size=f"{r} {L/2.0}")

        sph = geom_elem.find("sphere")
        if sph is not None:
            r = _safe_float(sph.attrib.get("radius"), 0.1)
            return UrdfGeometry(kind="sphere", size=f"{r}")

        mesh = geom_elem.find("mesh")
        if mesh is not None:
            fn = mesh.attrib.get("filename")
            sc = mesh.attrib.get("scale")
            return UrdfGeometry(kind="mesh", mesh_filename=fn, scale=sc)

        return UrdfGeometry(kind="box", size=default_box_half)

    def _parse_visuals(self, link_elem: ET.Element, link_name: str, materials: Dict[str, UrdfMaterial]) -> List[UrdfGeom]:
        out: List[UrdfGeom] = []
        visuals = link_elem.findall("visual")
        for i, v in enumerate(visuals):
            origin = v.find("origin")
            pos = origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            rpy = origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            q = _rpy_to_quat(rpy)

            geom = v.find("geometry")
            if geom is None:
                continue
            g = self._parse_geometry(geom)

            mat_name = None
            vm = v.find("material")
            if vm is not None:
                mat_name = vm.attrib.get("name")

            name = f"visual_{link_name}_{i}" if len(visuals) > 1 else f"visual_{link_name}"
            out.append(
                UrdfGeom(
                    name=name,
                    origin_pos=_vec3_str(pos),
                    origin_quat=_quat_str(q),
                    geometry=g,
                    material_name=mat_name,
                )
            )
        return out

    def _parse_collisions(self, link_elem: ET.Element, link_name: str) -> List[UrdfGeom]:
        out: List[UrdfGeom] = []
        cols = link_elem.findall("collision")
        for i, c in enumerate(cols):
            origin = c.find("origin")
            pos = origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            rpy = origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            q = _rpy_to_quat(rpy)

            geom = c.find("geometry")
            if geom is None:
                continue
            g = self._parse_geometry(geom)

            name = f"collision_{link_name}_{i}" if len(cols) > 1 else f"collision_{link_name}"
            out.append(
                UrdfGeom(
                    name=name,
                    origin_pos=_vec3_str(pos),
                    origin_quat=_quat_str(q),
                    geometry=g,
                    material_name=None,
                )
            )
        return out

    def _parse_joints(self) -> Tuple[Dict[str, UrdfJoint], Dict[str, List[str]], Dict[str, str]]:
        joints: Dict[str, UrdfJoint] = {}
        parent_map: Dict[str, List[str]] = {}
        child_to_joint: Dict[str, str] = {}

        for j in self.robot.findall("joint"):
            jname = j.attrib.get("name")
            jtype = j.attrib.get("type", "fixed")
            if not jname:
                continue
            parent = j.find("parent")
            child = j.find("child")
            if parent is None or child is None:
                continue
            p = parent.attrib.get("link")
            c = child.attrib.get("link")
            if not p or not c:
                continue

            origin = j.find("origin")
            pos = origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            rpy = origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            q = _rpy_to_quat(rpy)

            axis = "0 0 1"
            axis_elem = j.find("axis")
            if axis_elem is not None and axis_elem.attrib.get("xyz"):
                axis = _vec3_str(axis_elem.attrib["xyz"])

            lower = upper = None
            limit = j.find("limit")
            if limit is not None and "lower" in limit.attrib and "upper" in limit.attrib:
                lower = _safe_float(limit.attrib.get("lower"), None)  # type: ignore
                upper = _safe_float(limit.attrib.get("upper"), None)  # type: ignore

            uj = UrdfJoint(
                name=jname,
                jtype=jtype,
                parent=p,
                child=c,
                origin_pos=_vec3_str(pos),
                origin_quat=_quat_str(q),
                axis=axis,
                limit_lower=lower,
                limit_upper=upper,
            )

            mimic = j.find("mimic")
            if mimic is not None and mimic.attrib.get("joint"):
                uj.mimic_joint = mimic.attrib.get("joint")
                uj.mimic_multiplier = _safe_float(mimic.attrib.get("multiplier"), 1.0)
                uj.mimic_offset = _safe_float(mimic.attrib.get("offset"), 0.0)

            joints[jname] = uj
            parent_map.setdefault(p, []).append(c)
            child_to_joint[c] = jname

        return joints, parent_map, child_to_joint

    def _find_root_link(self, links: Dict[str, UrdfLink], child_to_joint: Dict[str, str]) -> str:
        all_links = set(links.keys())
        child_links = set(child_to_joint.keys())
        roots = list(all_links - child_links)
        if not roots:
            raise ValueError("Cannot determine URDF root link (no unique root found).")
        if len(roots) > 1:
            logger.warning("Multiple root links detected. Using the first one: %s", roots[0])
        return roots[0]

    def _parse_ros2_control_mujoco_system(self) -> Ros2ControlInfo:
        """
        EN:
        Parse all <ros2_control> blocks whose plugin is 'mujoco_ros2_control/MujocoSystem'.
        Merge joints/sensors across blocks.

        CN:
        解析所有 plugin 为 'mujoco_ros2_control/MujocoSystem' 的 <ros2_control> 块，
        并合并其中 joints/sensors 信息。
        """
        info = Ros2ControlInfo(present=False)

        rc_elements: List[ET.Element] = []
        for e in self.robot.findall("ros2_control"):
            plugin_elem = e.find("./hardware/plugin")
            plugin_text = (plugin_elem.text or "").strip() if plugin_elem is not None else ""
            if plugin_text == "mujoco_ros2_control/MujocoSystem":
                rc_elements.append(e)

        if not rc_elements:
            return info

        info.present = True
        plugin_elem = rc_elements[0].find("./hardware/plugin")
        if plugin_elem is not None and (plugin_elem.text or "").strip():
            info.plugin = (plugin_elem.text or "").strip()

        for rc in rc_elements:
            # joints
            for je in rc.findall("./joint"):
                jn = je.attrib.get("name")
                if not jn:
                    continue
                rj = Ros2ControlJoint(name=jn)

                for ci in je.findall("./command_interface"):
                    n = ci.attrib.get("name")
                    if n:
                        rj.command_interfaces.append(n)

                for si in je.findall("./state_interface"):
                    n = si.attrib.get("name")
                    if n:
                        rj.state_interfaces.append(n)

                for pe in je.findall("./param"):
                    pk = pe.attrib.get("name")
                    pv = (pe.text or "").strip()
                    if pk:
                        rj.params[pk] = pv

                if jn in info.joints:
                    ex = info.joints[jn]
                    for ci in rj.command_interfaces:
                        if ci not in ex.command_interfaces:
                            ex.command_interfaces.append(ci)
                    for si in rj.state_interfaces:
                        if si not in ex.state_interfaces:
                            ex.state_interfaces.append(si)
                    ex.params.update(rj.params)
                else:
                    info.joints[jn] = rj

            # sensors
            for se in rc.findall("./sensor"):
                sn = se.attrib.get("name")
                if not sn:
                    continue
                rs = Ros2ControlSensor(name=sn)

                for si in se.findall("./state_interface"):
                    n = si.attrib.get("name")
                    if n:
                        rs.state_interfaces.append(n)

                for pe in se.findall("./param"):
                    pk = pe.attrib.get("name")
                    pv = (pe.text or "").strip()
                    if pk:
                        rs.params[pk] = pv

                if sn in info.sensors:
                    exs = info.sensors[sn]
                    for si in rs.state_interfaces:
                        if si not in exs.state_interfaces:
                            exs.state_interfaces.append(si)
                    exs.params.update(rs.params)
                else:
                    info.sensors[sn] = rs

        logger.info(
            "Parsed ros2_control MujocoSystem: joints=%d, sensors=%d, plugin='%s'",
            len(info.joints),
            len(info.sensors),
            info.plugin,
        )
        return info


# =============================================================================
# MJCF Builder
# MJCF 构建器
# =============================================================================
@dataclass
class MeshAssetInfo:
    mesh_name: str
    material_name: str
    material_elem: ET.Element


class MjcfBuilder:
    """
    EN: MJCF builder and asset/worldbody generator.
    CN: MJCF 构建器：生成 asset/worldbody/actuator/sensor 等块。
    """

    def __init__(self, model_name: str, cfg: MjcfConfig, urdf_path: Path, mjcf_path: Path):
        self.cfg = cfg
        self.urdf_path = urdf_path
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

        self._mesh_assets: Dict[Tuple[str, str], MeshAssetInfo] = {}
        self._mesh_kind_counter: Dict[str, int] = {"VISUAL": 0, "COLLISION": 0}

    # -------------------------
    # Core blocks
    # 核心块
    # -------------------------
    def apply_core_blocks(self) -> None:
        """EN: Apply compiler/option/visual/default/asset/contact extras. CN: 应用基础配置块。"""
        self._apply_compiler()
        self._apply_option()
        self._apply_visual()
        self._apply_default()
        self._apply_asset_extras()
        self._apply_contact_extras()

    def _apply_compiler(self) -> None:
        attrib = {"angle": "radian", "balanceinertia": "true"}
        for k, v in (self.cfg.compiler or {}).items():
            attrib[k] = str(v).lower() if isinstance(v, bool) else str(v)
        self.compiler.attrib = attrib

    def _apply_option(self) -> None:
        attrib: Dict[str, str] = {"gravity": "0 0 -9.81"}
        for k, v in (self.cfg.option or {}).items():
            attrib[k] = str(v).lower() if isinstance(v, bool) else str(v)
        self.option.attrib = attrib

    def _apply_visual(self) -> None:
        self.visual.clear()
        vcfg = self.cfg.visual or {}
        if not vcfg:
            return
        for block_name, block_value in vcfg.items():
            if not isinstance(block_value, dict):
                g = self.visual.find("global")
                if g is None:
                    g = ET.SubElement(self.visual, "global")
                g.attrib[block_name] = str(block_value)
                continue
            blk = ET.SubElement(self.visual, block_name)
            for k, v in block_value.items():
                blk.attrib[k] = str(v).lower() if isinstance(v, bool) else str(v)

    def _apply_default(self) -> None:
        self.default.clear()
        robot_class = (self.cfg.defaults or {}).get("robot_class", "robot")
        robot_default = ET.SubElement(self.default, "default", attrib={"class": robot_class})

        visual_default = ET.SubElement(robot_default, "default", attrib={"class": "visual"})
        ET.SubElement(visual_default, "geom", attrib={"contype": "0", "conaffinity": "0", "group": "2"})

        collision_default = ET.SubElement(robot_default, "default", attrib={"class": "collision"})
        ET.SubElement(
            collision_default,
            "geom",
            attrib={
                "condim": "4",
                "contype": "1",
                "group": "3",
                "conaffinity": "1",
                "priority": "0",
                "solref": "0.02 1",
                "solimp": "0.9 0.95 0.001",
                "friction": "1 0.005 0.0001",
            },
        )

        if self.default.find("./default[@class='floor']") is None:
            fdef = ET.SubElement(self.default, "default", attrib={"class": "floor"})
            ET.SubElement(fdef, "geom", attrib={"type": "plane", "size": "0 0 0.05", "material": "groundplane"})

    def _apply_asset_extras(self) -> None:
        acfg = self.cfg.asset or {}
        for tex in acfg.get("textures", []) if isinstance(acfg.get("textures", []), list) else []:
            if isinstance(tex, dict):
                ET.SubElement(self.asset, "texture", attrib={k: str(v) for k, v in tex.items()})
        for mat in acfg.get("materials", []) if isinstance(acfg.get("materials", []), list) else []:
            if isinstance(mat, dict):
                ET.SubElement(self.asset, "material", attrib={k: str(v) for k, v in mat.items()})
        for mesh in acfg.get("meshes", []) if isinstance(acfg.get("meshes", []), list) else []:
            if isinstance(mesh, dict):
                ET.SubElement(self.asset, "mesh", attrib={k: str(v) for k, v in mesh.items()})

        if self.asset.find("./material[@name='default_material']") is None:
            ET.SubElement(self.asset, "material", attrib={"name": "default_material", "rgba": "0.7 0.7 0.7 1"})

    def _apply_contact_extras(self) -> None:
        ccfg = self.cfg.contact or {}
        excludes = ccfg.get("excludes", [])
        if isinstance(excludes, list):
            for ex in excludes:
                if isinstance(ex, dict) and "body1" in ex and "body2" in ex:
                    ET.SubElement(self.contact, "exclude", attrib={"body1": str(ex["body1"]), "body2": str(ex["body2"])})

    # -------------------------
    # Primitives
    # 基础节点
    # -------------------------
    def add_body(self, parent: ET.Element, name: str, pos: str, quat: str) -> ET.Element:
        return ET.SubElement(parent, "body", attrib={"name": name, "pos": pos, "quat": quat})

    def ensure_site(self, body: ET.Element, name: str, pos: str = "0 0 0", quat: str = "1 0 0 0") -> ET.Element:
        """
        EN: Ensure a site exists under body.
        CN: 确保 body 下存在指定 site（不存在则创建）。
        """
        existing = body.find(f"./site[@name='{name}']")
        if existing is not None:
            return existing
        ret = ET.SubElement(body, "site", attrib={"name": name, "pos": pos, "quat": quat})
        logger.info("Created site: name='%s', body='%s'", name, body.attrib.get("name", "unknown"))
        return ret

    def add_inertial(self, body: ET.Element, inertial: UrdfInertial) -> ET.Element:
        attrs = {
            "pos": inertial.pos,
            "quat": inertial.quat,
            "mass": str(inertial.mass),
            "diaginertia": f"{inertial.diaginertia[0]} {inertial.diaginertia[1]} {inertial.diaginertia[2]}",
        }
        return ET.SubElement(body, "inertial", attrib=attrs)

    def add_joint(self, body: ET.Element, mj_joint_name: str, mj_type: str, axis: str, range_pair: Optional[Tuple[float, float]]) -> ET.Element:
        attrs: Dict[str, str] = {"name": mj_joint_name, "type": mj_type, "axis": axis, "ref": "0.0"}
        if range_pair is not None:
            attrs["range"] = f"{range_pair[0]} {range_pair[1]}"
        return ET.SubElement(body, "joint", attrib=attrs)

    def add_geom(
        self,
        body: ET.Element,
        class_name: str,
        name: str,
        pos: str,
        quat: str,
        geom: UrdfGeometry,
        material: Optional[str] = None,
        mesh_name: Optional[str] = None,
    ) -> ET.Element:
        attrs: Dict[str, str] = {"class": class_name, "name": name, "pos": pos, "quat": quat}
        if geom.kind == "mesh":
            attrs["type"] = "mesh"
            if mesh_name:
                attrs["mesh"] = mesh_name
            if geom.scale:
                attrs["scale"] = geom.scale
        else:
            attrs["type"] = geom.kind
            if geom.size:
                attrs["size"] = geom.size
            if geom.scale:
                attrs["scale"] = geom.scale

        if material and class_name == "visual":
            attrs["material"] = material

        return ET.SubElement(body, "geom", attrib=attrs)

    # -------------------------
    # Materials/assets naming
    # 材质与资产命名
    # -------------------------
    def add_urdf_materials(self, urdf_materials: Dict[str, UrdfMaterial]) -> None:
        """
        EN: Add URDF-defined materials into MJCF <asset>.
        CN: 将 URDF 中定义的 material 写入 MJCF 的 <asset>。
        """
        added = 0
        for name, m in urdf_materials.items():
            if not name:
                continue
            if m.rgba and self.asset.find(f"./material[@name='{name}']") is None:
                ET.SubElement(self.asset, "material", attrib={"name": name, "rgba": m.rgba})
                added += 1
        logger.debug("Added URDF materials into MJCF: %d", added)

    def ensure_mesh_and_material(self, kind_upper: str, mesh_filename: str) -> MeshAssetInfo:
        """
        EN: Ensure a <mesh> and a corresponding <material> exist in <asset>.
        CN: 确保 <asset> 中存在对应的 <mesh> 和 <material>。
        """
        key = (kind_upper, mesh_filename)
        if key in self._mesh_assets:
            return self._mesh_assets[key]

        self._mesh_kind_counter[kind_upper] += 1

        token = _mesh_token_from_filename(mesh_filename)
        mesh_name = f"{kind_upper}_{token}_MESH"
        material_name = f"{kind_upper}_{token}_MATERIAL"

        resolved_path = _resolve_mesh_path_preserve_symlink_semantics(mesh_filename, self.urdf_path, self.mjcf_path, self.cfg.package_map)
        ET.SubElement(self.asset, "mesh", attrib={"name": mesh_name, "file": resolved_path})

        mat_elem = ET.SubElement(self.asset, "material", attrib={"name": material_name, "rgba": "0.7 0.7 0.7 1"})
        info = MeshAssetInfo(mesh_name=mesh_name, material_name=material_name, material_elem=mat_elem)
        self._mesh_assets[key] = info

        logger.debug("Registered mesh asset: kind=%s, mesh='%s', file='%s'", kind_upper, mesh_name, resolved_path)
        return info

    # -------------------------
    # World decorations
    # 世界装饰（地面/天空盒/灯光/freejoint）
    # -------------------------
    def add_world_decorations(self) -> None:
        wcfg = self.cfg.worldbody or {}
        if bool(wcfg.get("add_floor", True)):
            self._add_floor()
        if bool(wcfg.get("add_skybox", True)):
            self._add_skybox()
        if bool(wcfg.get("add_light", True)):
            self._add_light()

    def add_freejoint_to_root(self, root_body: ET.Element) -> None:
        wcfg = self.cfg.worldbody or {}
        if bool(wcfg.get("add_freejoint", True)):
            ET.SubElement(root_body, "freejoint", attrib={"name": f"{root_body.attrib.get('name','root')}_free"})

    def _add_floor(self) -> None:
        if self.asset.find("./texture[@name='groundplane']") is None:
            ET.SubElement(
                self.asset,
                "texture",
                attrib={
                    "type": "2d",
                    "name": "groundplane",
                    "builtin": "checker",
                    "mark": "edge",
                    "rgb1": "0.2 0.3 0.4",
                    "rgb2": "0.1 0.2 0.3",
                    "markrgb": "0.8 0.8 0.8",
                    "width": "300",
                    "height": "300",
                },
            )
        if self.asset.find("./material[@name='groundplane']") is None:
            ET.SubElement(
                self.asset,
                "material",
                attrib={
                    "name": "groundplane",
                    "texture": "groundplane",
                    "texuniform": "true",
                    "texrepeat": "5 5",
                    "reflectance": "0.2",
                },
            )
        if self.worldbody.find("./geom[@name='floor']") is None:
            ET.SubElement(self.worldbody, "geom", attrib={"name": "floor", "class": "floor", "size": "0 0 0.05"})

    def _add_skybox(self) -> None:
        if self.asset.find("./texture[@name='skybox']") is None:
            ET.SubElement(
                self.asset,
                "texture",
                attrib={
                    "name": "skybox",
                    "type": "skybox",
                    "builtin": "gradient",
                    "rgb1": ".3 .5 .7",
                    "rgb2": "0 0 0",
                    "width": "1024",
                    "height": "1024",
                },
            )
        g = self.visual.find("global")
        if g is None:
            g = ET.SubElement(self.visual, "global")
        g.attrib.setdefault("offwidth", "640")
        g.attrib.setdefault("offheight", "480")

    def _add_light(self) -> None:
        if self.worldbody.find("./light[@name='default_light']") is None:
            ET.SubElement(
                self.worldbody,
                "light",
                attrib={
                    "name": "default_light",
                    "pos": "0 0 2",
                    "dir": "0 0 -1",
                    "directional": "true",
                },
            )

    # -------------------------
    # Contact / Equality
    # 接触排除 / Mimic 约束
    # -------------------------
    def add_contact_excludes_from_tree(self, model: UrdfModel) -> None:
        for j in model.joints.values():
            lp = model.links.get(j.parent)
            lc = model.links.get(j.child)
            if lp and lc and lp.collisions and lc.collisions:
                ET.SubElement(self.contact, "exclude", attrib={"body1": j.parent, "body2": j.child})

    def add_mimic_equalities(self, model: UrdfModel) -> None:
        for j in model.joints.values():
            if j.mimic_joint:
                polycoef = f"{j.mimic_offset} {j.mimic_multiplier} 0 0 0"
                ET.SubElement(
                    self.equality,
                    "joint",
                    attrib={
                        "joint1": j.mimic_joint,
                        "joint2": j.name,
                        "polycoef": polycoef,
                        "solimp": str((self.cfg.equality or {}).get("solimp", "0.95 0.99 0.001")),
                        "solref": str((self.cfg.equality or {}).get("solref", "0.005 1")),
                    },
                )

    # =============================================================================
    # Actuator / Sensor / Camera logic
    # 执行器 / 传感器 / 相机逻辑
    # =============================================================================
    @staticmethod
    def _ros2_ci_to_actuator_tag(ci: str) -> str:
        """EN: Map ros2_control command_interface to MJCF actuator tag. CN: 接口名到 actuator 标签映射。"""
        c = ci.strip().lower()
        if c.startswith("position"):
            return "position"
        if c.startswith("velocity"):
            return "velocity"
        if c.startswith("effort") or c.startswith("motor"):
            return "motor"
        return c

    @staticmethod
    def _infer_sensor_tags_from_state_ifaces(state_ifaces: List[str]) -> List[str]:
        """EN: Infer sensor tags from state interfaces. CN: 从 state_interface 推断传感器类型。"""
        s = [x.strip().lower() for x in state_ifaces]
        tags: List[str] = []
        if any(x.startswith("force.") or x == "force" for x in s):
            tags.append("force")
        if any(x.startswith("torque.") or x == "torque" for x in s):
            tags.append("torque")
        if any("framequat" in x or x.endswith("quat") or "orientation" in x for x in s):
            tags.append("framequat")
        if any("gyro" in x or "angular_velocity" in x or "angvel" in x for x in s):
            tags.append("gyro")
        if any("accelerometer" in x or "accel" in x or "linear_acceleration" in x for x in s):
            tags.append("accelerometer")
        return tags

    def add_actuators_urdf(self, model: UrdfModel) -> bool:
        """
        EN: Add actuators derived from ros2_control MujocoSystem.
        CN: 从 URDF 的 ros2_control MujocoSystem 生成 actuator。
        """
        added_any = False
        if model.ros2_control.present and model.ros2_control.joints:
            for jn, rj in model.ros2_control.joints.items():
                for ci in rj.command_interfaces:
                    tag = self._ros2_ci_to_actuator_tag(ci)
                    attrs: Dict[str, str] = {"joint": jn}
                    attrs["name"] = f"{jn}_{tag.upper()}"

                    if ci.strip().lower() != tag:
                        attrs["interface"] = ci

                    for pk, pv in (rj.params or {}).items():
                        attrs[pk] = str(pv)

                    ET.SubElement(self.actuator, tag, attrib=attrs)
                    logger.info("Added actuator (URDF): tag='%s', name='%s', joint='%s'", tag, attrs["name"], jn)
                    added_any = True
        return added_any

    def add_actuators_from_json(self, mj_joint_names: List[str]) -> bool:
        """
        EN: Add actuators from JSON (flattened list in cfg.actuators).
        CN: 从 JSON 添加执行器（cfg.actuators 为扁平列表）。
        """
        added_any = False
        for a in self.cfg.actuators:
            if a.get("joint") not in mj_joint_names:
                logger.warning(
                    "Actuator skipped (joint not found): name='%s', joint='%s'",
                    a.get("name", "unnamed"),
                    a.get("joint", "unknown"),
                )
                continue
            act_type = str(a.get("type", "motor"))
            attrs = {k: str(v) for k, v in a.items() if k not in ("type",)}
            ET.SubElement(self.actuator, act_type, attrib=attrs)
            logger.info("Added actuator (JSON): type='%s', name='%s', joint='%s'", act_type, a.get("name"), a.get("joint"))
            added_any = True
        return added_any

    def add_actuators_default(self, model: UrdfModel, mj_joint_names: List[str]) -> bool:
        """
        EN: Add default actuators for non-mimic joints.
        CN: 为非 mimic 关节添加默认执行器。
        """
        added_any = False
        for jn in mj_joint_names:
            uj = model.joints.get(jn)
            if uj is None:
                logger.warning("Default actuator skipped (joint missing): joint='%s'", jn)
                continue
            if uj.mimic_joint:
                logger.debug("Default actuator skipped (mimic joint): joint='%s'", jn)
                continue
            ET.SubElement(self.actuator, "motor", attrib={"name": f"{jn}_MOTOR", "joint": jn})
            logger.info("Added actuator (default): type='motor', joint='%s'", jn)
            added_any = True
        return added_any

    def add_sensors_from_urdf(self, model: UrdfModel, body_map: Dict[str, ET.Element]) -> bool:
        """
        EN: Add sensors derived from ros2_control MujocoSystem sensors.
        CN: 从 URDF 的 ros2_control MujocoSystem 生成传感器。
        """
        added_any = False
        if model.ros2_control.present and model.ros2_control.sensors:
            for sn, rs in model.ros2_control.sensors.items():
                frame_id = (rs.params or {}).get("frame_id", "")
                target_body = body_map.get(frame_id)
                if target_body is None:
                    logger.warning("Sensor skipped (body not found): sensor='%s', frame_id='%s'", sn, frame_id)
                    continue

                site_name = f"{sn}_SITE"
                self.ensure_site(target_body, site_name)

                tags = self._infer_sensor_tags_from_state_ifaces(rs.state_interfaces)
                for t in tags:
                    nm = f"{sn}_{t}"
                    ET.SubElement(self.sensor, t, attrib={"name": nm, "site": site_name})
                    logger.info("Added sensor (URDF): type='%s', name='%s', body='%s'", t, nm, frame_id)
                    added_any = True
        return added_any

    def add_sensors_from_json(self, body_map: Dict[str, ET.Element]) -> bool:
        """
        EN:
        JSON sensors are flattened into cfg.sensors with a "type" key.
        Rule:
        - Use "body" to locate where to create/ensure the site.
        - Write sensor XML with all attributes except "body" and "type".
        - Ensure "site" attribute exists.

        CN:
        JSON sensor 已扁平化到 cfg.sensors（包含 type 字段）。
        规则：
        - 用 body 定位挂载位置并创建/确保 site
        - 写入除 body/type 外的所有属性
        - 必须有 site 属性
        """
        added_any = False
        for sens in self.cfg.sensors:
            tag = str(sens.get("type", "force"))
            body_name = str(sens.get("body", ""))
            site_name = f"{body_name}_SITE"

            target_body = body_map.get(body_name)
            if target_body is None:
                logger.warning("Sensor skipped (body not found): name='%s', body='%s'", sens.get("name", "unnamed"), body_name)
                continue

            self.ensure_site(target_body, site_name)

            attrs = {k: str(v) for k, v in sens.items() if k not in ("body", "type")}
            attrs.setdefault("site", site_name)
            ET.SubElement(self.sensor, tag, attrib=attrs)
            logger.info("Added sensor (JSON): type='%s', name='%s', body='%s'", tag, sens.get("name", "unnamed"), body_name)
            added_any = True
        return added_any

    def add_cameras_from_json(self, body_map: Dict[str, ET.Element]) -> bool:
        """
        EN: Add cameras from JSON. Each camera has a 'body' field indicating parent body.
        CN: 从 JSON 添加相机，每个 camera 的 'body' 指示其父 body。
        """
        if not self.cfg.cameras:
            return False

        camera_dict: Dict[str, List[Dict[str, Any]]] = {}
        for cam in self.cfg.cameras:
            body_name = cam.get("body")
            if not body_name:
                continue
            camera_dict.setdefault(str(body_name), [])
            cam_attrs = {k: v for k, v in cam.items() if k != "body"}
            camera_dict[str(body_name)].append(cam_attrs)

        added_any = False
        for body_name, cam_specs in camera_dict.items():
            parent = body_map.get(str(body_name))
            if parent is None:
                logger.warning("Camera parent body not found: '%s'", body_name)

            for cam_spec in cam_specs:
                ET.SubElement(parent, "camera", attrib={k: str(v) for k, v in cam_spec.items()})
                logger.info("Added camera (JSON): name='%s', body='%s'", cam_spec.get("name", "unnamed"), body_name)
                added_any = True

        return added_any


# =============================================================================
# Build worldbody recursively (no auto site creation)
# 从 URDF 构建 worldbody（默认不创建 site）
# =============================================================================
def _build_worldbody_from_urdf(builder: MjcfBuilder, model: UrdfModel) -> Tuple[ET.Element, Dict[str, ET.Element], List[str]]:
    """
    EN:
    Returns:
      - root_body (ET.Element)
      - body_map: link_name -> body element
      - mj_joint_names: URDF joint names created in MJCF

    CN:
    返回：
      - root_body（根 body）
      - body_map：link_name -> body 元素
      - mj_joint_names：在 MJCF 中创建的关节名列表
    """
    body_map: Dict[str, ET.Element] = {}
    mj_joint_names: List[str] = []

    def rec_build(link_name: str, parent_elem: ET.Element) -> ET.Element:
        if link_name == model.root_link:
            pos = "0 0 0"
            quat = "1 0 0 0"
        else:
            jname = model.child_to_joint.get(link_name)
            if jname:
                j = model.joints[jname]
                pos = j.origin_pos
                quat = j.origin_quat
            else:
                pos = "0 0 0"
                quat = "1 0 0 0"

        body = builder.add_body(parent_elem, name=link_name, pos=pos, quat=quat)
        body_map[link_name] = body

        link = model.links.get(link_name)

        # inertial
        if link and link.inertial is not None:
            builder.add_inertial(body, link.inertial)

        # joint (non-root)
        if link_name != model.root_link:
            jname = model.child_to_joint.get(link_name)
            if jname:
                uj = model.joints[jname]
                if uj.jtype in ("revolute", "continuous", "prismatic"):
                    mj_type = "hinge" if uj.jtype in ("revolute", "continuous") else "slide"
                    r = None
                    if uj.limit_lower is not None and uj.limit_upper is not None:
                        r = (float(uj.limit_lower), float(uj.limit_upper))
                    builder.add_joint(body, mj_joint_name=uj.name, mj_type=mj_type, axis=uj.axis, range_pair=r)
                    mj_joint_names.append(uj.name)

        # geoms
        if link:
            for vg in link.visuals:
                if vg.geometry.kind == "mesh" and vg.geometry.mesh_filename:
                    info = builder.ensure_mesh_and_material("VISUAL", vg.geometry.mesh_filename)
                    if vg.material_name and vg.material_name in model.materials and model.materials[vg.material_name].rgba:
                        info.material_elem.attrib["rgba"] = model.materials[vg.material_name].rgba
                    builder.add_geom(body, "visual", vg.name, vg.origin_pos, vg.origin_quat, vg.geometry, material=info.material_name, mesh_name=info.mesh_name)
                else:
                    mat = vg.material_name if (vg.material_name and vg.material_name in model.materials and model.materials[vg.material_name].rgba) else "default_material"
                    builder.add_geom(body, "visual", vg.name, vg.origin_pos, vg.origin_quat, vg.geometry, material=mat, mesh_name=None)

            for cg in link.collisions:
                if cg.geometry.kind == "mesh" and cg.geometry.mesh_filename:
                    info = builder.ensure_mesh_and_material("COLLISION", cg.geometry.mesh_filename)
                    builder.add_geom(body, "collision", cg.name, cg.origin_pos, cg.origin_quat, cg.geometry, material=None, mesh_name=info.mesh_name)
                else:
                    builder.add_geom(body, "collision", cg.name, cg.origin_pos, cg.origin_quat, cg.geometry, material=None, mesh_name=None)

        for child in model.parent_map.get(link_name, []):
            rec_build(child, body)

        return body

    root_body = rec_build(model.root_link, builder.worldbody)
    logger.info("Built worldbody: bodies=%d, mj_joints=%d", len(body_map), len(mj_joint_names))
    return root_body, body_map, mj_joint_names


# =============================================================================
# Public interface
# 对外接口
# =============================================================================
def mjcf_generator(
    urdf_path: Union[str, Path],
    mjcf_path: Union[str, Path],
    json_config_path: Optional[Union[str, Path]] = None,
) -> None:
    """
    EN: Convert URDF to MJCF.
    CN: 将 URDF 转换为 MJCF。
    """
    urdf_path = Path(urdf_path)
    mjcf_path = Path(mjcf_path)

    logger.info("MJCF generation started: urdf='%s', mjcf='%s'", str(urdf_path), str(mjcf_path))
    cfg = MjcfConfig.load(json_config_path)
    model = UrdfParser(urdf_path, cfg).parse()

    builder = MjcfBuilder(model_name=model.robot_name, cfg=cfg, urdf_path=urdf_path, mjcf_path=mjcf_path)
    builder.apply_core_blocks()

    # Keep URDF materials
    builder.add_urdf_materials(model.materials)

    # Build worldbody (no sites by default)
    root_body, body_map, mj_joint_names = _build_worldbody_from_urdf(builder, model)

    # Default: freejoint + decorations (unless JSON overrides)
    builder.add_freejoint_to_root(root_body)
    builder.add_world_decorations()

    # Always: contact excludes + mimic constraints
    builder.add_contact_excludes_from_tree(model)
    builder.add_mimic_equalities(model)

    # Actuator rules: URDF ros2_control + JSON + default fallback
    config_cfg = cfg.config or {}
    added_act_urdf = False
    added_act_json = False
    added_act_def = False

    if bool(config_cfg.get("add_ros2_mujoco_actuator", True)):
        added_act_urdf = builder.add_actuators_urdf(model)
    if bool(config_cfg.get("add_json_actuator", True)):
        added_act_json = builder.add_actuators_from_json(mj_joint_names)
    if bool(config_cfg.get("add_default_actuator", True)) or config_cfg is None:
        added_act_def = builder.add_actuators_default(model, mj_joint_names)

    # Sensor rules: URDF sensors + JSON sensors (create sites only when needed)
    added_sens_urdf = False
    added_sens_json = False
    if bool(config_cfg.get("add_ros2_mujoco_sensor", True)):
        added_sens_urdf = builder.add_sensors_from_urdf(model, body_map)
    if bool(config_cfg.get("add_json_sensor", True)):
        added_sens_json = builder.add_sensors_from_json(body_map)

    # Camera rules: JSON only
    added_cam_json = False
    if bool(config_cfg.get("add_json_camera", False)):
        added_cam_json = builder.add_cameras_from_json(body_map)

    logger.info(
        "Actuators: urdf=%s, json=%s, default=%s | Sensors: urdf=%s, json=%s | Cameras: json=%s",
        "yes" if added_act_urdf else "no",
        "yes" if added_act_json else "no",
        "yes" if added_act_def else "no",
        "yes" if added_sens_urdf else "no",
        "yes" if added_sens_json else "no",
        "yes" if added_cam_json else "no",
    )

    _pretty_write_xml(builder.root, mjcf_path)
    logger.info("Saved MJCF: %s", str(mjcf_path))


if __name__ == "__main__":
    # EN: Do not configure logging here; the CLI is the recommended entry.
    # CN: 这里不配置 logging；推荐从 CLI 调用。
    pass
