"""
OBJ / STL 网格处理模块 - MJCF XML 用

功能：
- 提取 body 中 visual / collision geom 对应的 mesh 文件
- STL 兼容性修复：ASCII STL -> 二进制 STL（MuJoCo 仅支持二进制 STL）
- OBJ 分解（按材质），提取 MTL -> rgba / 贴图(map_Kd) -> PNG
- 凸包分解（CoACD），生成 STL mesh
- 在 XML 中添加 mesh / material / texture / geom，并替换原几何体

MuJoCo 格式约束（官方实现）：
- STL：仅二进制（plugin/stl_decoder 校验 nfaces*50+84 == 文件大小）
- OBJ：只读几何/法线/UV；mtllib/usemtl 的颜色不会被解析
  -> 材质必须显式写入 MJCF <material>/<texture>
- texture 仅支持 PNG / KTX
"""

import json
import logging
import os
import re
import shutil
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union

logger = logging.getLogger(__name__)

# MuJoCo 默认材质名（当 URDF visual 未显式指定颜色时由 mjcf_generator 生成）
_DEFAULT_MATERIAL_RGBA = "1 1 1 1"

# 已注册的 mesh 解码扩展名（asset/mesh 可引用）；其余扩展名不做 STL 修复
_MESH_STL_SUFFIXES = (".stl",)


def _sanitize_xml_name(name: str) -> str:
    """将任意字符串清洗为可安全用于 MJCF name 属性的字符串。"""
    cleaned = re.sub(r"[^0-9A-Za-z_.\-]", "_", name)
    cleaned = re.sub(r"_+", "_", cleaned).strip("_")
    return cleaned or "mesh"


def _is_ascii_stl(path: Union[str, Path]) -> bool:
    """
    判断文件是否为 ASCII STL。

    规则：读取头 84 字节；若以 b"solid" 开头且无法通过
    二进制 STL 头部面数校验（nfaces*50 + 84 == 文件大小），
    则判定为 ASCII。
    """
    path = Path(path)
    try:
        size = path.stat().st_size
        if size < 84:
            return False
        with path.open("rb") as f:
            head = f.read(84)
        if head[:5] != b"solid":
            return False
        nfaces = struct.unpack("<I", head[80:84])[0]
        if nfaces > 0 and nfaces * 50 + 84 == size:
            return False  # 头碰巧是 solid 开头的二进制（极罕见）
        return True
    except (OSError, struct.error):
        return False


def _is_default_material_rgba(rgba: Optional[str]) -> bool:
    """判断 rgba 是否为缺省值（mjcf_generator 在无颜色时写入 1 1 1 1）。"""
    if rgba is None:
        return True
    try:
        vals = [float(x) for x in rgba.split()]
        return len(vals) >= 3 and vals[:3] == [1.0, 1.0, 1.0]
    except ValueError:
        return False


# ====================== CoACD 配置 ======================

@dataclass
class CoacdConfig:
    """
    CoACD 参数配置，对应 JSON 文件字段。
    额外增加 log_level 控制 CoACD 日志等级。
    """
    threshold: float = 0.5
    max_convex_hull: int = 10
    preprocess_mode: str = "auto"
    preprocess_resolution: int = 50
    resolution: int = 2000
    mcts_nodes: int = 20
    mcts_iterations: int = 150
    mcts_max_depth: int = 3
    pca: bool = False
    merge: bool = True
    decimate: bool = False
    max_ch_vertex: int = 256
    extrude: bool = False
    extrude_margin: float = 0.01
    apx_mode: str = "ch"
    seed: int = 0
    log_level: str = "error"  # 额外字段：CoACD 日志等级

    @classmethod
    def load_from_json(cls, path: Optional[Union[str, Path]]) -> "CoacdConfig":
        """
        从 JSON 中加载 CoACD 配置。

        JSON 可能包含其他内容：
        - 若存在键 "coacd" 且为 dict，则只使用 data["coacd"] 作为 CoACD 配置；
        - 否则，将顶层键视为 CoACD 配置。
        """
        cfg = cls()
        if path is None:
            return cfg

        path = Path(path)
        if not path.exists():
            logger.info(f"CoACD config file not found, using defaults: {path}")
            return cfg

        try:
            with path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            logger.warning(f"Failed to read CoACD config, using defaults: {path}: {e}")
            return cfg

        # 若 JSON 中包含 "coacd" 字段，则只取其中的配置
        if isinstance(data, dict) and "coacd" in data and isinstance(data["coacd"], dict):
            src = data["coacd"]
        else:
            src = data

        if not isinstance(src, dict):
            logger.warning(f"CoACD config format invalid, using defaults: {path}")
            return cfg

        for k, v in src.items():
            if hasattr(cfg, k):
                setattr(cfg, k, v)

        logger.info(f"Loaded CoACD config: {path}")
        return cfg


class ObjMeshProcessor:
    """
    处理 MJCF XML 中的 OBJ / STL 网格。

    对外接口：
        1. ObjMeshProcessor(xml_path).process(...) - 类实例方法
        2. process_obj_meshes(...) - 独立函数（推荐）
    """

    def __init__(
        self,
        xml_path: Union[str, Path, None] = None,
        is_copy_meshes: bool = False,
        is_symlink_copy: bool = False,
    ):

        self.tree: Optional[ET.ElementTree[ET.Element[str]]] = None
        self.root: Optional[ET.Element[str]] = None
        self.xml_path: Path = Path(xml_path) if xml_path is not None else Path()

        # 是否把生成的贴图/二进制 STL 等伴随文件复制（或软链接）到 XML 同目录的 meshes 下
        self.is_copy_meshes: bool = is_copy_meshes
        self.is_symlink_copy: bool = is_symlink_copy

        # OBJ 分解时记录：原 obj -> {子 obj -> 材质名}
        self._obj_split_materials: Dict[Path, Dict[Path, str]] = {}

        # mesh 文件 -> 兼容性修复后的实际文件（ASCII STL -> 二进制 STL）
        self._fixed_mesh_cache: Dict[Path, Path] = {}

        # 材质名 -> (material, texture) 记录（用于 asset 引用一致性）
        self._added_materials: Dict[str, ET.Element] = {}
        self._added_textures: Dict[str, ET.Element] = {}

        # CoACD 配置
        self.coacd_config: CoacdConfig = CoacdConfig()

    # ======================================================================
    # 读取 / 保存 XML
    # ======================================================================

    def load_xml(self) -> None:
        """读取 xml 文件。"""
        self.tree = ET.parse(self.xml_path)
        self.root = self.tree.getroot()

    def save_xml(self, output_path: Union[str, Path]) -> None:
        """将 xml 文件保存（美化输出）。"""
        from xml.dom import minidom

        if self.root is None:
            raise RuntimeError("XML 尚未加载，请先调用 load_xml()")

        rough_string = ET.tostring(self.root, encoding="utf-8")
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")
        # 去掉空行
        pretty_xml = re.sub(r"\n\s*\n", "\n", pretty_xml)

        output_path = Path(output_path)
        output_path.write_text(pretty_xml, encoding="utf-8")
        logger.info(f"XML saved: {output_path}")

    # ======================================================================
    # 基础工具：asset / mesh 映射
    # ======================================================================

    def _require_root(self) -> ET.Element:
        if self.root is None:
            raise RuntimeError("XML 尚未加载，请先调用 load_xml()")
        return self.root

    def _get_assets(self) -> ET.Element:
        root = self._require_root()
        assets = root.find("asset")
        if assets is None:
            assets = ET.SubElement(root, "asset")
        return assets

    def _get_mesh_name_to_file(self) -> Dict[str, Path]:
        """获取 asset 中 mesh name -> 文件路径 映射（不 resolve，保持相对位置语义）。"""
        assets = self._get_assets()
        mapping: Dict[str, Path] = {}
        for m in assets.findall("mesh"):
            name = m.get("name")
            file_attr = m.get("file")
            if not name or not file_attr:
                continue
            mapping[name] = self.xml_path.parent / file_attr
        return mapping

    def _get_material_name_to_rgba(self) -> Dict[str, str]:
        """获取 asset 中 material name -> rgba 映射。"""
        assets = self._get_assets()
        mapping: Dict[str, str] = {}
        for m in assets.findall("material"):
            name = m.get("name")
            rgba = m.get("rgba")
            if name and rgba:
                mapping[name] = rgba
        return mapping

    def _get_texture_name_to_file(self) -> Dict[str, Path]:
        """获取 asset 中 texture name -> 文件路径 映射（相对 xml 目录）。"""
        assets = self._get_assets()
        mapping: Dict[str, Path] = {}
        for t in assets.findall("texture"):
            name = t.get("name")
            file_attr = t.get("file")
            if name and file_attr:
                mapping[name] = self.xml_path.parent / file_attr
        return mapping

    # ======================================================================
    # 贴图 / 材质引用辅助
    # ======================================================================

    def _ensure_texture_for_material(
        self, material_xml_name: str, tex_src: Path, rgba: str
    ) -> Optional[str]:
        """
        为材质准备贴图：
        - 若已有同名 material 且带 texture，直接复用该 texture
        - 否则将贴图转 PNG 落地并添加 texture asset
        返回 texture 名称（失败返回 None）。
        """
        assets = self._get_assets()
        # 若 material 已存在且有 texture，则复用其 texture
        for m in assets.findall("material"):
            if m.get("name") == material_xml_name and m.get("texture"):
                return m.get("texture")

        texture_xml_name = f"{material_xml_name}_TEX"
        # 若同名 texture 已存在（可能此前已添加），直接返回
        for t in assets.findall("texture"):
            if t.get("name") == texture_xml_name:
                return texture_xml_name

        ready = self.ensure_texture_ready(tex_src, Path(self.xml_path).parent)
        if ready is None:
            return None

        self.add_texture_asset(
            texture_xml_name,
            ready,
            type_="2d",
        )
        return texture_xml_name

    def _ensure_material_has_texture(
        self, material_xml_name: str, orig_rgba: Optional[str], tex_src: Optional[Path]
    ) -> None:
        """
        保留原显式颜色材质时，若原 material 尚无 texture 且 MTL 有 map_Kd，
        则补加 texture（rgba 保留，作为 tint 与贴图相乘）。
        """
        if tex_src is None or not tex_src.is_file():
            return
        assets = self._get_assets()
        for m in assets.findall("material"):
            if m.get("name") == material_xml_name and m.get("texture"):
                return  # 已有 texture，无需补
        for m in assets.findall("material"):
            if m.get("name") != material_xml_name:
                continue
            rgba = m.get("rgba") or orig_rgba or "1 1 1 1"
            texture_xml_name = self._ensure_texture_for_material(
                material_xml_name, tex_src, rgba
            )
            if texture_xml_name:
                m.set("texture", texture_xml_name)
                logger.info(
                    f"[texture] Added texture '{texture_xml_name}' to existing material "
                    f"'{material_xml_name}'"
                )
            return

    # ======================================================================
    # 添加 mesh / material / texture asset
    # ======================================================================

    def add_mesh_asset(
        self, mesh_name: str, mesh_file: Path, **extra_attrib: str
    ) -> ET.Element:
        """
        用 mesh 文件以及提供的 mesh 元素 name，为 xml 添加 mesh 元素。
        file 属性使用相对于 xml 的相对路径。
        """
        assets = self._get_assets()
        for m in assets.findall("mesh"):
            if m.get("name") == mesh_name:
                logger.info(
                    f"mesh asset already exists: name='{mesh_name}', file='{m.get('file')}'"
                )
                return m

        rel_path = os.path.relpath(mesh_file, self.xml_path.parent)
        attrib: Dict[str, str] = {"name": mesh_name, "file": rel_path}
        attrib.update(extra_attrib)
        elem = ET.SubElement(assets, "mesh", attrib=attrib)
        logger.info(f"Added mesh asset: name='{mesh_name}'")
        return elem

    def add_texture_asset(
        self,
        texture_name: str,
        texture_file: Union[str, Path],
        type_: str = "2d",
        **extra_attrib: str,
    ) -> ET.Element:
        """
        用贴图文件以及提供的 texture 元素 name，为 xml 添加 texture 元素。
        file 属性使用相对于 xml 的相对路径。
        """
        assets = self._get_assets()
        for t in assets.findall("texture"):
            if t.get("name") == texture_name:
                logger.info(f"texture asset already exists: name='{texture_name}'")
                return t

        attrib: Dict[str, str] = {"name": texture_name, "file": str(texture_file), "type": type_}
        attrib.update(extra_attrib)
        elem = ET.SubElement(assets, "texture", attrib=attrib)
        self._added_textures[texture_name] = elem
        logger.info(f"Added texture asset: name='{texture_name}'")
        return elem

    def add_material_asset(
        self, material_name: str, rgba: str, texture_name: Optional[str] = None
    ) -> ET.Element:
        """
        用 rgba 的值以及提供的 material 元素 name，为 xml 添加 material 元素。
        若提供 texture_name，则同时写入 texture 属性（2D 贴图材质）。
        """
        assets = self._get_assets()
        for m in assets.findall("material"):
            if m.get("name") == material_name:
                logger.info(
                    f"material asset already exists: name='{material_name}', "
                    f"rgba='{m.get('rgba')}'"
                )
                return m

        attrib: Dict[str, str] = {"name": material_name, "rgba": rgba}
        if texture_name:
            attrib["texture"] = texture_name
        elem = ET.SubElement(assets, "material", attrib=attrib)
        self._added_materials[material_name] = elem
        logger.info(f"Added material asset: name='{material_name}'")
        return elem

    # ======================================================================
    # 伴随文件落地：复制 / 软链接
    # ======================================================================

    def _ensure_file_available(self, src_path: Path, dest_dir: Path) -> Path:
        """
        将 src_path 复制（或软链接）到 dest_dir 下并返回目标路径。

        若未开启 is_copy_meshes 或目标已存在则直接返回 src_path。
        文件名冲突时追加 hash 前缀，避免覆盖同名不同内容文件。
        """
        src_path = Path(src_path)
        if not src_path.is_file():
            return src_path
        if not self.is_copy_meshes:
            return src_path
        dest_dir = Path(dest_dir)
        dest_dir.mkdir(parents=True, exist_ok=True)

        dest = dest_dir / src_path.name
        if os.path.lexists(dest):
            # 已存在：若为有效文件且内容一致直接复用，否则用 hash 区分
            try:
                if dest.is_file() and not dest.is_symlink():
                    same_size = dest.stat().st_size == src_path.stat().st_size
                    if same_size and dest.read_bytes() == src_path.read_bytes():
                        return dest
            except OSError:
                pass
            try:
                import hashlib
                digest = hashlib.md5(src_path.read_bytes()).hexdigest()[:8]
            except OSError:
                digest = "x"
            dest = dest_dir / f"{src_path.stem}_{digest}{src_path.suffix}"

        if self.is_symlink_copy:
            if not dest.exists() and not dest.is_symlink():
                os.symlink(os.path.abspath(src_path), dest)
                logger.info(f"[file] symlinked: {dest} <- {src_path}")
        else:
            shutil.copy2(src_path, dest)
            logger.info(f"[file] copied: {src_path} -> {dest}")
        return dest

    # ======================================================================
    # STL 兼容：ASCII -> 二进制
    # ======================================================================

    def convert_ascii_stl_to_binary(self, stl_path: Path) -> Path:
        """
        若 stl_path 是 ASCII STL，则转换为同目录二进制 STL 并返回新路径；
        否则原样返回。

        转换使用 trimesh 读写，产物命名 "{stem}_binary.stl"。
        """
        stl_path = Path(stl_path)
        if not stl_path.is_file():
            return stl_path
        if stl_path.suffix.lower() not in _MESH_STL_SUFFIXES:
            return stl_path
        if not _is_ascii_stl(stl_path):
            return stl_path

        # 命中缓存
        if stl_path in self._fixed_mesh_cache:
            return self._fixed_mesh_cache[stl_path]

        logger.warning(f"[stl-fix] ASCII STL detected, converting to binary STL: {stl_path}")
        try:
            import trimesh
            loaded = trimesh.load(stl_path, force="mesh", process=False)
            if loaded is None:
                raise ValueError("trimesh returned None")
            binary_path = stl_path.with_name(f"{stl_path.stem}_binary.stl")
            loaded.export(binary_path, file_type="stl")
        except Exception as e:
            logger.error(f"[stl-fix] Failed to convert ASCII STL '{stl_path}': {e}. "
                         f"Original file will be kept and may fail to load in MuJoCo.")
            return stl_path

        self._fixed_mesh_cache[stl_path] = binary_path
        logger.info(f"[stl-fix] Converted: {binary_path}")
        return binary_path

    # ======================================================================
    # 贴图兼容：转 PNG 并落地到目标目录
    # ======================================================================

    @staticmethod
    def _convert_texture_to_png(src_path: Path, dest_path: Path) -> Optional[Path]:
        """
        将图片文件转换为 PNG 并写入 dest_path（Pillow），失败返回 None。

        若源已是 PNG，则直接复制。
        """
        try:
            from PIL import Image
        except ImportError:
            logger.warning(
                f"[texture] Pillow is required to convert texture '{src_path}' to PNG. "
                f"Install with: pip install Pillow"
            )
            return None

        try:
            if src_path.suffix.lower() == ".png":
                shutil.copy2(src_path, dest_path)
                return dest_path
            with Image.open(src_path) as opened:
                opened.load()
                # 统一转 RGB（贴图仅需颜色）；RGBA 保留 alpha
                if opened.mode in ("RGBA", "LA") or (
                    opened.mode == "P" and "transparency" in opened.info
                ):
                    converted = opened.convert("RGBA")
                else:
                    converted = opened.convert("RGB")
            converted.save(dest_path, format="PNG")
            return dest_path
        except Exception as e:
            logger.warning(
                f"[texture] Failed to convert texture '{src_path}' to PNG: {e}"
            )
            return None

    def ensure_texture_ready(self, src_path: Path, base_dir: Path) -> Optional[Path]:
        """
        确保贴图以 PNG 形式存在于 base_dir（或 XML 同目录 meshes 下），
        返回可直接写入 <texture file=...> 的相对路径（相对 XML）。

        - 已开启 is_copy_meshes：把 PNG 落到 XML 同目录的 "meshes" 子目录下
        - 未开启：尽量就地转换/使用原路径
        """
        src_path = Path(src_path)
        if not src_path.is_file():
            logger.warning(f"[texture] Texture file not found: {src_path}")
            return None

        if not self.is_copy_meshes:
            # 就地：若是 PNG 直接用；否则在源旁生成 PNG
            if src_path.suffix.lower() == ".png":
                ready_path = src_path
            else:
                png_path = src_path.with_suffix(".png")
                if png_path.exists():
                    ready_path = png_path
                else:
                    converted = self._convert_texture_to_png(src_path, png_path)
                    if converted is None:
                        return None
                    ready_path = converted
            # 返回相对 XML 的路径（不可相对时回退绝对路径）
            return self._to_xml_relative_or_absolute(ready_path)

        # copy 模式：统一落到 XML 同目录 "meshes"（与 mesh 一致）
        xml_dir = Path(self.xml_path).parent
        meshes_dir = xml_dir / "meshes"
        meshes_dir.mkdir(parents=True, exist_ok=True)

        if src_path.suffix.lower() == ".png":
            dest = self._ensure_file_available(src_path, meshes_dir)
        else:
            # 先转 PNG 到临时同目录，再 copy 落地
            png_src = src_path.with_suffix(".png")
            conv = png_src if png_src.exists() else None
            if conv is None:
                conv = self._convert_texture_to_png(src_path, png_src)
            if conv is None:
                return None
            dest = self._ensure_file_available(conv, meshes_dir)

        # 返回相对 XML 的路径
        return self._to_xml_relative_or_absolute(dest)

    def _to_xml_relative_or_absolute(self, path: Path) -> Path:
        """
        返回 path 相对 XML 所在目录的路径；若无法相对（跨盘等）则返回绝对路径。
        """
        try:
            rel = os.path.relpath(path, Path(self.xml_path).parent)
            return Path(rel)
        except ValueError:
            return Path(os.path.abspath(path))

    # ======================================================================
    # 材质 / 贴图解析（MTL）
    # ======================================================================

    @staticmethod
    def _parse_mtl_rgba_map(mtl_file: Path) -> Dict[str, str]:
        """
        解析 MTL 文件，返回 {材质名: "r g b a"}。
        只解析 Kd / d / tr，忽略 map_Kd 等贴图语句。
        """
        result: Dict[str, str] = {}
        if not mtl_file.is_file():
            return result

        current_name: Optional[str] = None
        kd: Optional[Tuple[float, float, float]] = None
        alpha = 1.0

        with mtl_file.open("r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith("#"):
                    continue
                lower = s.lower()
                # 排除贴图语句 map_*
                if lower.startswith("map_"):
                    continue

                if lower.startswith("newmtl"):
                    if current_name and kd:
                        r, g, b = kd
                        result[current_name] = f"{r} {g} {b} {alpha}"
                    parts = s.split(maxsplit=1)
                    current_name = parts[1].strip() if len(parts) > 1 else None
                    kd = None
                    alpha = 1.0
                elif lower.startswith("kd") and current_name:
                    parts = s.split()
                    if len(parts) >= 4:
                        try:
                            kd = tuple(float(x) for x in parts[1:4])  # type: ignore[assignment]
                        except ValueError:
                            kd = (1.0, 1.0, 1.0)
                elif lower.startswith("d") and current_name:
                    parts = s.split()
                    if len(parts) >= 2:
                        try:
                            alpha = float(parts[1])
                        except ValueError:
                            alpha = 1.0
                elif lower.startswith("tr") and current_name:
                    parts = s.split()
                    if len(parts) >= 2:
                        try:
                            alpha = 1.0 - float(parts[1])
                        except ValueError:
                            alpha = 1.0

        if current_name and kd:
            r, g, b = kd
            result[current_name] = f"{r} {g} {b} {alpha}"
        return result

    @staticmethod
    def _parse_mtl_map_kd(mtl_file: Path) -> Dict[str, Optional[Path]]:
        """
        解析 MTL 文件中的 map_Kd 语句，返回 {材质名: 贴图文件绝对路径|None}。
        贴图文件路径相对 MTL 所在目录。
        """
        result: Dict[str, Optional[Path]] = {}
        if not mtl_file.is_file():
            return result

        current_name: Optional[str] = None
        mtl_dir = mtl_file.parent

        with mtl_file.open("r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith("#"):
                    continue
                lower = s.lower()
                if lower.startswith("newmtl"):
                    parts = s.split(maxsplit=1)
                    current_name = parts[1].strip() if len(parts) > 1 else None
                    if current_name is not None:
                        result.setdefault(current_name, None)
                    continue
                if current_name is None:
                    continue
                if lower.startswith("map_kd"):
                    # map_Kd [options] texturefile —— 去掉 -xxx 选项
                    tokens = s.split()
                    tex = None
                    for tok in tokens[1:]:
                        if tok.startswith("-"):
                            continue
                        tex = tok
                        break
                    if tex is not None:
                        tex_path = (mtl_dir / tex).resolve()
                        result[current_name] = tex_path
        return result

    def _get_mtl_info(
        self, obj_file: Path
    ) -> Tuple[Path, Dict[str, str], Dict[str, Optional[Path]]]:
        """
        给定 obj 文件，解析其 mtllib 引用的 MTL 文件，返回：
            (mtl_path, {材质: rgba}, {材质: 贴图路径|None})
        若 obj 无 mtllib 或 MTL 不存在，返回 (obj 旁同名 mtl, {}, {})。
        """
        mtl_name = self._parse_mtl_name_from_obj(obj_file)
        if not mtl_name:
            logger.info(f"[mtl] OBJ {obj_file} has no MTL reference")
            return obj_file.with_suffix(".mtl"), {}, {}
        mtl_file = (obj_file.parent / mtl_name).resolve()
        if not mtl_file.is_file():
            logger.warning(f"[mtl] MTL file not found: {mtl_file}")
            return mtl_file, {}, {}
        rgba_map = self._parse_mtl_rgba_map(mtl_file)
        map_kd_map = self._parse_mtl_map_kd(mtl_file)
        logger.info(
            f"[mtl] Parsed MTL: {mtl_file} -> {len(rgba_map)} colors, "
            f"{sum(1 for v in map_kd_map.values() if v)} textures"
        )
        return mtl_file, rgba_map, map_kd_map

    # ======================================================================
    # 几何修复调度：替换 mesh 文件引用（ASCII STL）
    # ======================================================================

    def fix_mesh_compatibility(self) -> None:
        """
        遍历 asset 中所有 mesh 元素，若为 ASCII STL 则：
        - 转换成同目录二进制 STL（或按 copy 策略落地）
        - 更新 asset/mesh 的 file 指向新文件
        """
        assets = self._get_assets()
        if self.xml_path is None:
            return
        xml_dir = Path(self.xml_path).parent

        for m in assets.findall("mesh"):
            file_attr = m.get("file")
            name = m.get("name")
            if not file_attr or not name:
                continue
            mesh_path = xml_dir / file_attr
            if mesh_path.suffix.lower() not in _MESH_STL_SUFFIXES:
                continue

            fixed = self.convert_ascii_stl_to_binary(mesh_path)
            if fixed == mesh_path:
                continue

            if self.is_copy_meshes:
                # 二进制产物也统一落地到 meshes 目录
                fixed = self._ensure_file_available(fixed, xml_dir / "meshes")

            rel = os.path.relpath(fixed, xml_dir)
            m.set("file", rel)
            logger.info(f"[stl-fix] mesh '{name}' now points to binary STL: {rel}")
            # 注意：几何 body geom 的 mesh 引用名不变，asset file 已更新


    # ======================================================================
    # 网格凸包分解（CoACD）
    # ======================================================================

    def convex_decompose_mesh_file(self, mesh_file: Path) -> List[Path]:
        """
        对提供的 mesh 文件进行凸包分解，返回分解后得到的文件列表，
        分解后的文件保存到 mesh_file 所在目录下：
            "[原文件名不带后缀]_convex_[num].stl"

        注意：不对 mesh_file 做 resolve()，因此若 mesh_file 是软链接，
              输出会出现在「软链接所在目录」下。
        """
        import trimesh
        try:
            import coacd
        except ImportError:
            logger.error(
                "convex_decompose_mesh_file requires coacd, please install: pip install coacd"
            )
            return []

        if not mesh_file.exists():
            logger.warning(f"[convex] Mesh file not found: {mesh_file}")
            return []

        logger.info(f"[convex] Starting convex decomposition: {mesh_file}")
        try:
            mesh = trimesh.load(mesh_file, force="mesh", process=False)
        except Exception as e:
            logger.warning(f"[convex] Failed to load mesh: {mesh_file}: {e}")
            return []

        m = coacd.Mesh(mesh.vertices, mesh.faces)

        # 设置 CoACD 日志等级 & 参数
        try:
            coacd.set_log_level(self.coacd_config.log_level)
        except Exception:
            # 老版本 coacd 可能没有 set_log_level，忽略
            pass

        parts = coacd.run_coacd(
            mesh=m,
            threshold=self.coacd_config.threshold,
            max_convex_hull=self.coacd_config.max_convex_hull,
            preprocess_mode=self.coacd_config.preprocess_mode,
            preprocess_resolution=self.coacd_config.preprocess_resolution,
            resolution=self.coacd_config.resolution,
            mcts_nodes=self.coacd_config.mcts_nodes,
            mcts_iterations=self.coacd_config.mcts_iterations,
            mcts_max_depth=self.coacd_config.mcts_max_depth,
            pca=self.coacd_config.pca,
            merge=self.coacd_config.merge,
            decimate=self.coacd_config.decimate,
            max_ch_vertex=self.coacd_config.max_ch_vertex,
            extrude=self.coacd_config.extrude,
            extrude_margin=self.coacd_config.extrude_margin,
            apx_mode=self.coacd_config.apx_mode,
            seed=self.coacd_config.seed,
        )

        out_paths: List[Path] = []
        for idx, (vs, fs) in enumerate(parts):
            import trimesh as _tm
            part_mesh = _tm.Trimesh(vs, fs)
            out_path = mesh_file.parent / f"{mesh_file.stem}_convex_{idx}.stl"
            part_mesh.export(out_path.as_posix())
            out_paths.append(out_path)
            logger.info(f"[convex]       generated convex file: {out_path}")

        return out_paths

    # ======================================================================
    # geom 生成与替换
    # ======================================================================

    def create_geom(
        self,
        pos: Optional[str],
        quat: Optional[str],
        geom_class: str,
        name: str,
        mesh: str,
        material: str,
    ) -> ET.Element:
        """
        根据指定的 pos / quat / class / name / mesh / material 生成 geom 元素。
        """
        attrs = {
            "class": geom_class,
            "name": name,
            "type": "mesh",
            "mesh": mesh,
            "material": material,
        }
        if pos:
            attrs["pos"] = pos
        if quat:
            attrs["quat"] = quat
        return ET.Element("geom", attrib=attrs)

    def replace_body_geoms(
        self, body: ET.Element, geom_class: str, new_geoms: List[ET.Element]
    ) -> None:
        """
        根据指定的 geom 元素列表，替换指定 body 中 class=geom_class 的 geom。
        """
        old_geoms = [
            g for g in body.findall("geom") if g.get("class") == geom_class
        ]
        for g in old_geoms:
            body.remove(g)
        for g in new_geoms:
            body.append(g)
        logger.info(
            f"[geom-replace] body='{body.get('name','')}', class='{geom_class}' "
            f"replaced {len(new_geoms)} geoms"
        )

    # ======================================================================
    # 凸包分解并应用到 visual/collision geom（逐个 geom 保持自身 pos/quat）
    # ======================================================================

    def convex_decompose_and_apply(self, target: str) -> None:
        """
        对指定类型的 geom（visual 或 collision）对应的 mesh 进行凸包分解，
        并使用分解后的 mesh 替换原 geom。

        - 对 XML 中每个 body：
            * 找出其中 class=target 的所有 geom；
            * 对每个 geom：
                - 根据其 mesh 名找到网格文件，做凸包分解（带缓存）；
                - 为每个凸包 mesh 生成 mesh asset；
                - 新生成的 geom：
                    class 与原 geom 相同 (target)
                    pos / quat 与原 geom 一一对应
                    material = "default_material"
        """
        root = self._require_root()
        if target not in ("visual", "collision"):
            logger.warning(f"[convex] Invalid target: {target}")
            return

        mesh_map = self._get_mesh_name_to_file()
        # 缓存：mesh_file -> [convex_file...]
        convex_cache: Dict[Path, List[Path]] = {}

        for body in root.findall(".//body"):
            geoms = [
                g for g in body.findall("geom")
                if (g.get("class") == target and g.get("mesh") is not None)
            ]
            if not geoms:
                continue

            body_name = body.get("name", "body")
            new_geoms_for_body: List[ET.Element] = []

            for geom_idx, geom in enumerate(geoms):
                mesh_name = geom.get("mesh")
                if not mesh_name:
                    # 没 mesh，保持原 geom
                    new_geoms_for_body.append(geom)
                    continue

                mesh_file = mesh_map.get(mesh_name)
                if not mesh_file or not mesh_file.exists():
                    logger.warning(
                        f"[convex] Mesh file not found: mesh='{mesh_name}' "
                        f"for geom='{geom.get('name','')}'"
                    )
                    new_geoms_for_body.append(geom)
                    continue

                # 凸包分解（带缓存）
                if mesh_file not in convex_cache:
                    convex_cache[mesh_file] = self.convex_decompose_mesh_file(mesh_file)
                convex_files = convex_cache[mesh_file]

                if not convex_files:
                    # 分解失败/无结果，保留原 geom
                    new_geoms_for_body.append(geom)
                    continue

                pos = geom.get("pos")
                quat = geom.get("quat")

                for part_idx, convex_file in enumerate(convex_files):
                    stem = convex_file.stem
                    mesh_xml_name = f"COLLISION_{stem}_MESH"
                    # 添加/复用 mesh asset
                    self.add_mesh_asset(mesh_xml_name, convex_file)

                    new_name = f"COLLISION_{body_name}_{geom_idx}_{part_idx}"
                    new_geom = self.create_geom(
                        pos=pos,
                        quat=quat,
                        geom_class="collision",
                        name=new_name,
                        mesh=mesh_xml_name,
                        material="default_material",
                    )
                    new_geoms_for_body.append(new_geom)

            # 用新生成的 geom 替换原有该 class 的 geom
            self.replace_body_geoms(body, "collision", new_geoms_for_body)

    # ======================================================================
    # OBJ 按材质分解 + 提取 MTL RGBA
    # ======================================================================

    @staticmethod
    def _parse_mtl_name_from_obj(obj_file: Path) -> Optional[str]:
        pattern = re.compile(
            r"^mtllib\s+(.+?\.mtl)(?:\s*#.*)?\s*$", re.IGNORECASE
        )
        with obj_file.open("r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                m = pattern.match(line.strip())
                if m:
                    return m.group(1)
        return None

    @staticmethod
    def _parse_mtl_rgba_file(mtl_file: Path) -> Dict[str, str]:
        """
        解析 MTL 文件，返回 {材质名: "r g b a"}。

        兼容别名：委托给 _parse_mtl_rgba_map。
        """
        return ObjMeshProcessor._parse_mtl_rgba_map(mtl_file)

    def split_obj_by_materials(self, obj_file: Path) -> List[Path]:
        """
        对提供的 obj 文件按照材质进行分解，返回分解后的文件列表。

        - 分解后的文件保存到原文件所在目录，命名：
            "[原文件名不带后缀]_obj_[num].obj"
        - 分解后的 obj 文件中引用的材质文件名与原 obj 文件一致 (mtllib 相同)。
        - 内部记录：self._obj_split_materials[obj_file][子 obj] = 材质名
        """
        obj_file = obj_file  # 不 resolve，保持原路径（软链接安全）
        if not obj_file.exists():
            logger.warning(f"[obj-split] OBJ file not found: {obj_file}")
            return []

        # 若之前已分解过，直接复用已有记录
        if obj_file in self._obj_split_materials:
            return list(self._obj_split_materials[obj_file].keys())

        mtl_name = self._parse_mtl_name_from_obj(obj_file)
        if not mtl_name:
            logger.info(
                f"[obj-split] OBJ {obj_file} has no MTL reference; skipping material split"
            )
            return []

        logger.info(f"[obj-split] Splitting OBJ: {obj_file} ")

        with obj_file.open("r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()

        header_lines: List[str] = []
        faces_by_material: Dict[str, List[str]] = {}
        current_mat = "default"

        for raw in lines:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if not parts:
                continue
            cmd = parts[0]

            if cmd in ("v", "vt", "vn"):
                header_lines.append(raw.rstrip("\n"))
            elif cmd == "usemtl":
                if len(parts) > 1:
                    current_mat = parts[1]
                else:
                    current_mat = "default"
                faces_by_material.setdefault(current_mat, [])
            elif cmd in ("f", "l", "p"):
                faces_by_material.setdefault(current_mat, []).append(raw.rstrip("\n"))

        out_paths: List[Path] = []
        mat_map: Dict[Path, str] = {}

        for idx, (mat, face_lines) in enumerate(faces_by_material.items()):
            if not face_lines:
                continue
            out_path = obj_file.with_name(f"{obj_file.stem}_obj_{idx}.obj")
            with out_path.open("w", encoding="utf-8") as f:
                f.write(f"mtllib {mtl_name}\n")
                f.write(f"usemtl {mat}\n")
                for h in header_lines:
                    f.write(h + "\n")
                for face in face_lines:
                    f.write(face + "\n")

            out_paths.append(out_path)
            mat_map[out_path] = mat
            logger.info(
                f"[obj-split]     produced OBJ: {out_path} (material={mat})"
            )

        self._obj_split_materials[obj_file] = mat_map
        return out_paths

    def get_obj_mtl_rgba(self, obj_file: Path) -> Dict[str, str]:
        """
        对提供的 obj 文件，提取其对应的 mtl 材质文件中的信息，转换为 rgba 格式。
        返回: {材质名: "r g b a"}
        """
        obj_file = obj_file
        mtl_name = self._parse_mtl_name_from_obj(obj_file)
        if not mtl_name:
            logger.info(f"[mtl-rgba] OBJ {obj_file} has no MTL reference")
            return {}
        mtl_file = obj_file.parent / mtl_name
        logger.info(f"[mtl-rgba] Parsing MTL: {mtl_file}")
        rgba_map = self._parse_mtl_rgba_file(mtl_file)
        logger.info(
            f"[mtl-rgba]     parsed {len(rgba_map)} materials: "
            f"{list(rgba_map.keys())}"
        )
        return rgba_map

    # ======================================================================
    # OBJ 转换并替换 visual geom（逐个 visual geom，保持各自 pos/quat）
    # ======================================================================

    def process_visual_objs(self) -> None:
        """
        对每个 body 的 visual 中的 OBJ 类型 mesh 文件：
        - 按材质分解 OBJ（split_obj_by_materials）
        - 从 MTL 提取 rgba 与 map_Kd 贴图
        - 为每个子 OBJ 生成 mesh/material asset；有贴图时生成 texture asset
        - 对每个 visual geom 单独处理：
            * 新 geom 的 pos/quat == 原 geom 的 pos/quat
            * 若 geom 使用的 mesh 非 obj 或分解失败，则保留原 geom
        - 颜色优先级：
            * 原 geom 已引用 material（且非默认 1 1 1 1 颜色）时保留原材质名，
              仅补充贴图（若原 material 无 texture）
            * 否则使用 MTL 的 rgba / 贴图生成新 material
        """
        root = self._require_root()
        mesh_map = self._get_mesh_name_to_file()
        material_map = self._get_material_name_to_rgba()

        for body in root.findall(".//body"):
            visual_geoms = [
                g for g in body.findall("geom") if g.get("class") == "visual"
            ]
            if not visual_geoms:
                continue

            body_name = body.get("name", "body")
            new_visuals_for_body: List[ET.Element] = []

            # 每个 body 可能引用多个不同的 OBJ mesh；我们按 geom 逐个处理
            for geom_idx, geom in enumerate(visual_geoms):
                mesh_name = geom.get("mesh")
                if not mesh_name:
                    # 无 mesh，原样保留
                    new_visuals_for_body.append(geom)
                    continue

                mesh_file = mesh_map.get(mesh_name)
                if not mesh_file or not mesh_file.exists():
                    # 文件不存在，原样保留
                    new_visuals_for_body.append(geom)
                    continue

                if mesh_file.suffix.lower() != ".obj":
                    # 非 obj（STL 等）：无需按材质分解，原样保留
                    new_visuals_for_body.append(geom)
                    continue

                # 对该 OBJ 文件进行按材质分解（带缓存）
                split_files = self.split_obj_by_materials(mesh_file)
                if not split_files:
                    # 分解失败/无结果（无 MTL 引用等情况），保留原 geom
                    new_visuals_for_body.append(geom)
                    continue

                # 解析 MTL（rgba + map_Kd）
                mtl_file, rgba_map, map_kd_map = self._get_mtl_info(mesh_file)
                mat_map_for_obj = self._obj_split_materials.get(mesh_file, {})

                # 原 geom 颜色优先级判断
                orig_material_name = geom.get("material")
                if orig_material_name:
                    orig_material_rgba = material_map.get(orig_material_name)
                else:
                    orig_material_rgba = None
                has_explicit_color = not _is_default_material_rgba(orig_material_rgba)

                pos = geom.get("pos")
                quat = geom.get("quat")
                orig_name = geom.get("name") or f"VISUAL_{body_name}_{geom_idx}"

                for part_idx, split_file in enumerate(split_files):
                    mat_name_raw = mat_map_for_obj.get(split_file, None)
                    mtl_rgba = rgba_map.get(mat_name_raw) if mat_name_raw else None
                    mtl_tex = map_kd_map.get(mat_name_raw) if mat_name_raw else None

                    # 决定最终材质 / 颜色 / 贴图
                    if has_explicit_color:
                        # URDF/原材质显式颜色优先；仅当原材质未贴图时补贴图
                        if orig_material_name:
                            material_xml_name = orig_material_name
                            self._ensure_material_has_texture(
                                material_xml_name, orig_material_rgba, mtl_tex
                            )
                        else:
                            material_xml_name = f"VISUAL_{_sanitize_xml_name(orig_name)}_MATERIAL"
                            self.add_material_asset(
                                material_xml_name, orig_material_rgba or "1 1 1 1"
                            )
                    else:
                        rgba = mtl_rgba or "1 1 1 1"
                        stem = split_file.stem
                        material_xml_name = f"VISUAL_{stem}_MATERIAL"
                        # 贴图：确保 PNG 可用并添加 texture asset
                        texture_xml_name = None
                        if mtl_tex is not None and mtl_tex.is_file():
                            texture_xml_name = self._ensure_texture_for_material(
                                material_xml_name, mtl_tex, rgba
                            )
                        # 添加/复用 material（带 texture 或纯 rgba）
                        self.add_material_asset(material_xml_name, rgba, texture_xml_name)

                    mesh_xml_name = f"VISUAL_{split_file.stem}_MESH"
                    # 拆分后的子 OBJ 常为薄壳面片，体积趋近 0；
                    # 使用 inertia="shell" 避免 MuJoCo "mesh volume is too small" 编译错误
                    self.add_mesh_asset(mesh_xml_name, split_file, inertia="shell")

                    # 新 geom 的 pos/quat 对应当前这个原 visual geom
                    new_name = f"{orig_name}_{part_idx}"
                    new_geom = self.create_geom(
                        pos=pos,
                        quat=quat,
                        geom_class="visual",
                        name=new_name,
                        mesh=mesh_xml_name,
                        material=material_xml_name,
                    )
                    new_visuals_for_body.append(new_geom)

            # 用新的 visual geom 列表替换 body 中所有 class="visual" 的 geom
            self.replace_body_geoms(body, "visual", new_visuals_for_body)


    # ======================================================================
    # 资产清理：删除未使用的 mesh / material
    # ======================================================================

    def prune_unused_assets(self) -> None:
        """
        扫描所有 geom，收集实际使用的 mesh / material / texture 名称，
        删除 asset 中未被使用的 mesh / material / texture 元素。

        这样可以自动删除：
        - 被 OBJ 分解 / 凸包分解完全替代的旧 mesh 资产
        - 不再被任何 geom 引用的旧 material 资产
        - 不再被任何 material 引用的贴图资产
        """
        root = self._require_root()
        assets = self._get_assets()

        used_meshes = set()
        used_materials = set()

        for geom in root.findall(".//geom"):
            m = geom.get("mesh")
            mat = geom.get("material")
            if m:
                used_meshes.add(m)
            if mat:
                used_materials.add(mat)

        # material 的 texture 引用（贴图被 material 使用则保留）
        material_to_texture: Dict[str, str] = {}
        for mat_elem in assets.findall("material"):
            mat_name = mat_elem.get("name")
            tex = mat_elem.get("texture")
            if mat_name and tex:
                material_to_texture[mat_name] = tex

        used_textures = {
            tex for name, tex in material_to_texture.items() if name in used_materials
        }

        for child in list(assets):
            if child.tag == "mesh":
                name = child.get("name")
                if name and name not in used_meshes:
                    assets.remove(child)
                    logger.info(f"[prune] Removed unused mesh asset: {name}")
            elif child.tag == "material":
                name = child.get("name")
                if name and name not in used_materials:
                    assets.remove(child)
                    logger.info(f"[prune] Removed unused material asset: {name}")
            elif child.tag == "texture":
                name = child.get("name")
                tex_type = child.get("type", "2d")
                # 仅清理未被引用的 2d 贴图；skybox/cube 被全局渲染使用，保留
                if tex_type == "2d" and name and name not in used_textures:
                    assets.remove(child)
                    logger.info(f"[prune] Removed unused texture asset: {name}")
            # 其他 asset 类型保持不动

    # ======================================================================
    # 其它：重排 body 子元素
    # ======================================================================

    def _reorder_body_children(self) -> None:
        """
        将每个 <body> 中子元素顺序调整为：
            所有其他元素在前，其次是 <geom> ，最后是 <body>。
        """
        root = self._require_root()
        for body in root.findall(".//body"):
            children = list(body)
            geoms = [c for c in children if c.tag == "geom"]
            bodies = [c for c in children if c.tag == "body"]
            others = [c for c in children if c.tag not in ("geom", "body")]

            for c in children:
                body.remove(c)
            for c in others + geoms + bodies:
                body.append(c)


# ======================================================================
# 独立处理函数
# ======================================================================

def mesh_decomposer(
    xml_path: Union[str, Path],
    decompose_target: Optional[str] = None,
    config_path: Optional[Union[str, Path]] = None,
    is_copy_meshes: bool = False,
    is_symlink_copy: bool = False,
) -> str:
    """
    处理 MJCF XML 中的 OBJ / STL 网格的完整流程。

    处理流程：
    1. 加载 XML
    2. STL 兼容性修复（ASCII STL -> 二进制 STL）
    3. visual 中 OBJ mesh 分解 + 材质/贴图处理 + geom 替换
    4. 可选：对 visual 或 collision 的 mesh 做凸包分解并替换对应 geom
    5. 清理未用 mesh / material / texture 资产
    6. 调整 body 子元素顺序
    7. 保存 XML（原地覆盖）

    Args:
        xml_path: XML 文件路径
        decompose_target: 是否做凸包分解，及针对的 geom 类型：
            - None: 不做凸包分解
            - "visual": 对所有 class="visual" 的 mesh 凸包分解，并替换 visual geom
            - "collision": 对所有 class="collision" 的 mesh 凸包分解，并替换 collision geom
        config_path: CoACD 配置 JSON 路径；
            若为 None，则尝试 "<xml同名>.coacd.json"；
            JSON 中若存在 "coacd" 字段，则只取该字段为 CoACD 配置。
        is_copy_meshes: 是否将生成的贴图/二进制 STL 等伴随文件复制到
            XML 同目录 meshes 下（与 mesh_converter 的 --copy-meshes 语义一致）。
        is_symlink_copy: 复制方式使用软链接而非拷贝（需 is_copy_meshes=True）。

    Returns:
        处理后的 XML 文件路径
    """

    processor = ObjMeshProcessor(
        xml_path=Path(xml_path),
        is_copy_meshes=is_copy_meshes,
        is_symlink_copy=is_symlink_copy,
    )
    if not processor.xml_path.exists():
        raise FileNotFoundError(f"XML 文件不存在: {processor.xml_path}")

    # 加载 CoACD 配置
    cfg_path: Optional[Path]
    if config_path is not None:
        cfg_path = Path(config_path)
    else:
        default_cfg = processor.xml_path.with_suffix(".coacd.json")
        cfg_path = default_cfg if default_cfg.exists() else None
    processor.coacd_config = CoacdConfig.load_from_json(cfg_path)

    processor.load_xml()

    # ASCII STL -> 二进制 STL（asset/mesh file 更新）
    processor.fix_mesh_compatibility()

    # visual 中 OBJ 分解 + 材质/贴图应用
    processor.process_visual_objs()

    # 凸包分解（可选，对 visual 或 collision 的 mesh）
    if decompose_target in ("visual", "collision"):
        processor.convex_decompose_and_apply(target=decompose_target)

    # 清理未使用资产（包括原有被完全替代的 mesh/material/texture）
    processor.prune_unused_assets()

    # 调整 body 子元素顺序
    processor._reorder_body_children()

    processor.save_xml(processor.xml_path)
    return str(processor.xml_path)
