"""
OBJ / STL 网格处理模块 - MJCF XML 用

功能：
- 提取 body 中 visual / collision geom 对应的 mesh 文件
- OBJ 分解（按材质），提取 MTL -> rgba
- 凸包分解（CoACD），生成 STL mesh
- 在 XML 中添加 mesh / material / geom，并替换原几何体
"""

import json
import logging
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Union

logger = logging.getLogger(__name__)


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

    def __init__(self):

        self.tree: Optional[ET.ElementTree] = None
        self.root: Optional[ET.Element] = None

        # OBJ 分解时记录：原 obj -> {子 obj -> 材质名}
        self._obj_split_materials: Dict[Path, Dict[Path, str]] = {}

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

    # ======================================================================
    # 添加 mesh / material asset
    # ======================================================================

    def add_mesh_asset(self, mesh_name: str, mesh_file: Path) -> ET.Element:
        """
        用 mesh 文件以及提供的 mesh 元素 name，为 xml 添加 mesh 元素。
        file 属性使用相对于 xml 的相对路径。
        """
        assets = self._get_assets()
        for m in assets.findall("mesh"):
            if m.get("name") == mesh_name:
                logger.info(f"mesh asset already exists: name='{mesh_name}', file='{m.get('file')}'")
                return m

        rel_path = os.path.relpath(mesh_file, self.xml_path.parent)
        elem = ET.SubElement(
            assets, "mesh", attrib={"name": mesh_name, "file": rel_path}
        )
        logger.info(f"Added mesh asset: name='{mesh_name}'")
        return elem

    def add_material_asset(self, material_name: str, rgba: str) -> ET.Element:
        """
        用 rgba 的值以及提供的 material 元素 name，为 xml 添加 material 元素。
        """
        assets = self._get_assets()
        for m in assets.findall("material"):
            if m.get("name") == material_name:
                logger.info(f"material asset already exists: name='{material_name}', rgba='{m.get('rgba')}'")
                return m

        elem = ET.SubElement(
            assets, "material", attrib={"name": material_name, "rgba": rgba}
        )
        logger.info(f"Added material asset: name='{material_name}'")
        return elem

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
        try:
            import trimesh
            import coacd
        except ImportError:
            logger.warning(
                "convex_decompose_mesh_file requires trimesh and coacd, please install: pip install trimesh coacd"
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
            f"[geom-replace] body='{body.get('name','')}', class='{geom_class}' replaced {len(new_geoms)} geoms"
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
            geoms = [g for g in body.findall("geom") if (g.get("class") == target and g.get("mesh") is not None)]
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
                        f"[convex] Mesh file not found: mesh='{mesh_name}' for geom='{geom.get('name','')}'"
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
        """
        result: Dict[str, str] = {}
        if not mtl_file.exists():
            return result

        current_name: Optional[str] = None
        kd = None
        alpha = 1.0

        with mtl_file.open("r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith("#"):
                    continue
                lower = s.lower()

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
                            kd = tuple(float(x) for x in parts[1:4])
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
        - 从 MTL 提取 rgba
        - 为每个子 OBJ 生成 mesh/material asset
        - 对每个 visual geom 单独处理：
            * 新 geom 的 pos/quat == 原 geom 的 pos/quat
            * 若 geom 使用的 mesh 非 obj 或分解失败，则保留原 geom
        """
        root = self._require_root()
        mesh_map = self._get_mesh_name_to_file()

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
                if not mesh_file or not mesh_file.exists() or mesh_file.suffix.lower() != ".obj":
                    # 非 obj 或文件不存在，原样保留
                    new_visuals_for_body.append(geom)
                    continue

                # 对该 OBJ 文件进行按材质分解（带缓存）
                split_files = self.split_obj_by_materials(mesh_file)
                if not split_files:
                    # 分解失败/无结果，保留原 geom
                    new_visuals_for_body.append(geom)
                    continue

                # 解析 MTL -> rgba（缓存由调用者控制，这里按文件重用为主）
                rgba_map = self.get_obj_mtl_rgba(mesh_file)
                mat_map_for_obj = self._obj_split_materials.get(mesh_file, {})

                pos = geom.get("pos")
                quat = geom.get("quat")
                orig_name = geom.get("name") or f"VISUAL_{body_name}_{geom_idx}"

                for part_idx, split_file in enumerate(split_files):
                    mat_name_raw = mat_map_for_obj.get(split_file, None)
                    if mat_name_raw and mat_name_raw in rgba_map:
                        rgba = rgba_map[mat_name_raw]
                    else:
                        rgba = "1 1 1 1"

                    stem = split_file.stem
                    mesh_xml_name = f"VISUAL_{stem}_MESH"
                    material_xml_name = f"VISUAL_{stem}_MATERIAL"

                    # 生成 material / mesh asset
                    logger.info(
                        f"[obj-split] Creating Mesh/Material: "
                        f"{mesh_xml_name} / {material_xml_name}"
                    )
                    self.add_material_asset(material_xml_name, rgba)
                    self.add_mesh_asset(mesh_xml_name, split_file)

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
        扫描所有 geom，收集实际使用的 mesh / material 名称，
        删除 asset 中未被使用的 mesh / material 元素。

        这样可以自动删除：
        - 被 OBJ 分解 / 凸包分解完全替代的旧 mesh 资产
        - 不再被任何 geom 引用的旧 material 资产
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
            # 其他 asset 类型（如 texture 等）保持不动

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
) -> str:
    """
    处理 MJCF XML 中的 OBJ / STL 网格的完整流程。
    
    处理流程：
    1. 加载 XML
    2. visual 中 OBJ mesh 分解 + 材质处理 + geom 替换
    3. 可选：对 visual 或 collision 的 mesh 做凸包分解并替换对应 geom
    4. 清理未用 mesh / material 资产
    5. 调整 body 子元素顺序
    6. 保存 XML（原地覆盖）

    Args:
        xml_path: XML 文件路径
        decompose_target: 是否做凸包分解，及针对的 geom 类型：
            - None: 不做凸包分解
            - "visual": 对所有 class="visual" 的 mesh 凸包分解，并替换 visual geom
            - "collision": 对所有 class="collision" 的 mesh 凸包分解，并替换 collision geom
        config_path: CoACD 配置 JSON 路径；
            若为 None，则尝试 "<xml同名>.coacd.json"；
            JSON 中若存在 "coacd" 字段，则只取该字段为 CoACD 配置。

    Returns:
        处理后的 XML 文件路径
    """
    from xml.dom import minidom
    import xml.etree.ElementTree as ET
    
    processor = ObjMeshProcessor()
    processor.xml_path = Path(xml_path)
    if not processor.xml_path.exists():
        raise FileNotFoundError(f"XML 文件不存在: {processor.xml_path}")

    # 加载 CoACD 配置
    if config_path is not None:
        cfg_path = Path(config_path)
    else:
        default_cfg = processor.xml_path.with_suffix(".coacd.json")
        cfg_path = default_cfg if default_cfg.exists() else None
    processor.coacd_config = CoacdConfig.load_from_json(cfg_path)

    processor.load_xml()

    # visual 中 OBJ 分解 + 材质应用
    processor.process_visual_objs()

    # 凸包分解（可选，对 visual 或 collision 的 mesh）
    if decompose_target in ("visual", "collision"):
        processor.convex_decompose_and_apply(target=decompose_target)

    # 清理未使用资产（包括原有被完全替代的 mesh/material）
    processor.prune_unused_assets()

    # 调整 body 子元素顺序
    processor._reorder_body_children()

    processor.save_xml(processor.xml_path)
    return str(processor.xml_path)
