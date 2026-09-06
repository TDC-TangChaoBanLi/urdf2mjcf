#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OBJ 多材质分解模块（model 级 / 写 MJCF 前）。

EN:
- Split a multi-material OBJ into one OBJ per material (usemtl groups).
- Extract rgba / map_Kd texture references from the OBJ's MTL and apply them
  to the model's material (UrdfMaterial), so the information survives even
  when the MTL is deleted later.
- Work directly on the in-memory UrdfParser model; register created files in
  the ResourceRegistry.

CN:
- 将多材质 OBJ 按材质（usemtl 分组）拆分为每个材质一个 OBJ。
- 从 OBJ 的 MTL 提取 rgba / map_Kd 贴图引用并写回 model 的 material
  （UrdfMaterial），使后续删除 MTL 也不丢失材质信息。
- 直接在内存 UrdfParser model 上工作；创建的文件登记到 ResourceRegistry。
"""

from __future__ import annotations

import logging
import re
from pathlib import Path
from typing import Dict, List, Optional, Union

from . import mesh_utils
from .resource_registry import ResourceRegistry
from .urdf_parser import UrdfParser

logger = logging.getLogger("urdf2mjcf.mesh_decomposer")


def _sanitize_xml_name(name: str) -> str:
    """将任意字符串清洗为可安全用于 MJCF name 属性的字符串。"""
    cleaned = re.sub(r"[^0-9A-Za-z_.\-]", "_", name)
    cleaned = re.sub(r"_+", "_", cleaned).strip("_")
    return cleaned or "mesh"


def _is_default_material_rgba(rgba: Optional[str]) -> bool:
    """判断 rgba 是否为缺省值（无颜色时按 1 1 1 1 处理）。"""
    if rgba is None:
        return True
    try:
        vals = [float(x) for x in rgba.split()]
        return len(vals) >= 3 and vals[:3] == [1.0, 1.0, 1.0]
    except ValueError:
        return False


def split_obj_file(obj_file: Path) -> List[Path]:
    """
    将 OBJ 文件按材质（usemtl）拆分成多个子 OBJ，返回子 OBJ 文件列表。

    - 子 OBJ 与源 OBJ 同目录，命名 "{源stem}_obj_{idx}.obj"
    - 子 OBJ 保留 mtllib 引用（材质名与源一致）
    - 若 OBJ 无 mtllib 或无法拆分返回 []
    """
    obj_file = obj_file
    if not obj_file.exists():
        logger.warning(f"[obj-split] OBJ file not found: {obj_file}")
        return []

    mtl_name = mesh_utils.parse_mtl_name_from_obj(obj_file)
    if not mtl_name:
        logger.info(f"[obj-split] OBJ {obj_file} has no MTL reference; skipping material split")
        return []

    logger.info(f"[obj-split] Splitting OBJ: {obj_file}")

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
            current_mat = parts[1] if len(parts) > 1 else "default"
            faces_by_material.setdefault(current_mat, [])
        elif cmd in ("f", "l", "p"):
            faces_by_material.setdefault(current_mat, []).append(raw.rstrip("\n"))

    out_paths: List[Path] = []
    for idx, (mat, face_lines) in enumerate(faces_by_material.items()):
        if not face_lines:
            continue
        out_path = obj_file.with_name(f"{obj_file.stem}_obj_{idx}.obj")
        with out_path.open("w", encoding="utf-8") as f:
            # NOTE: MuJoCo 只读 OBJ 几何，不解析 mtllib/usemtl 颜色。
            # 材质（rgba/贴图）已由调用方从 MTL 提取并写回 model，
            # 因此子 OBJ 不再引用 mtllib —— 源 MTL 删除后子 OBJ 依然自足。
            f.write(f"usemtl {mat}\n")
            for h in header_lines:
                f.write(h + "\n")
            for face in face_lines:
                f.write(face + "\n")
        out_paths.append(out_path)
        logger.info(f"[obj-split]     produced OBJ: {out_path} (material={mat})")

    return out_paths


def decompose_obj_meshes(
    urdf_model: UrdfParser,
    meshes_dir: Optional[Union[str, Path]] = None,
    registry: Optional[ResourceRegistry] = None,
    is_symlink_copy: bool = False,
) -> int:
    """
    处理 URDF model 中所有引用多材质 OBJ 的 visual：

    - 对每个 OBJ 按材质分解（split_obj_file），产物写入源所在目录
    - 若指定 meshes_dir：分解前先确保源 OBJ（及伴生 MTL/贴图）在 meshes_dir
      有可写副本，分解产物也落地到 meshes_dir
    - 从 MTL 提取 rgba / map_Kd：写回被替换 visual 的 material
      （UrdfMaterial：color_rgba + texture_file），保证删 MTL 后材质仍存在
    - 用分解后的多个 visual 替换原 visual
    - 创建的分解文件登记到 registry（供最终清理）

    :return: 成功处理的 OBJ visual 数量
    """
    if registry is None:
        registry = ResourceRegistry()

    handled: Dict[str, List[Path]] = {}  # 源 obj(abs) -> split files
    processed = 0

    def split_for_mesh(obj_path: Path) -> List[Path]:
        """对 obj 分解；必要时先落地副本到 meshes_dir。"""
        obj_path = obj_path
        key = str(obj_path.resolve())
        if key in handled:
            return handled[key]

        work_obj = obj_path
        if meshes_dir is not None:
            work_obj = mesh_utils.ensure_file_available(
                obj_path,
                meshes_dir,
                is_symlink_copy=is_symlink_copy,
                register=registry,
                kind="copy",
                force_land=True,
            )

        split_files = split_obj_file(work_obj)
        # 登记分解产物
        for sp in split_files:
            registry.mark_split(sp, origin=work_obj)
        handled[key] = split_files
        return split_files

    for link in urdf_model.links:
        if not link.visuals:
            continue
        new_visuals: List[UrdfParser.UrdfVisual] = []
        for visual in link.visuals:
            geom = visual.geometry
            if geom.g_type != "mesh" or not geom.filename:
                new_visuals.append(visual)
                continue
            mesh_path = Path(geom.filename)
            if not mesh_path.is_absolute():
                cand = mesh_path.resolve()
                if cand.is_file():
                    mesh_path = cand
            if mesh_path.suffix.lower() != ".obj" or not mesh_path.is_file():
                new_visuals.append(visual)
                continue

            split_files = split_for_mesh(mesh_path)
            if not split_files:
                new_visuals.append(visual)
                continue

            # 从源 OBJ 的 MTL 提取材质信息（rgba + map_Kd）
            mtl_file, rgba_map, map_kd_map = mesh_utils.get_mtl_info(mesh_path)

            for split_file in split_files:
                # 从子 OBJ 的 usemtl 名取材质；此处用 MTL 中第一个有信息的材质兜底
                usemtl = _first_usemtl(split_file)
                rgba = rgba_map.get(usemtl) if usemtl else None
                tex = map_kd_map.get(usemtl) if usemtl else None

                # 材质名唯一化：不同 OBJ/子 OBJ 的 newmtl/usemtl 可能同名
                # （如 trimesh 从 DAE 导出的子 OBJ 都叫 material_0），
                # 若直接使用会导致 MJCF 中不同颜色的材质互相覆盖。
                mat_name = f"{split_file.stem}_{usemtl}" if usemtl else f"{split_file.stem}_default"

                # 贴图落地到 meshes_dir（保证 MJCF 可引用）；失败则保留 None
                tex_landed: Optional[str] = None
                if tex is not None and tex.is_file():
                    landed = mesh_utils.ensure_texture_ready(
                        tex,
                        meshes_dir,
                        is_symlink_copy=is_symlink_copy,
                        register=registry,
                    ) if meshes_dir is not None else tex
                    if landed is not None:
                        tex_landed = str(Path(landed).resolve())
                    else:
                        logger.warning(f"[obj-split] texture not usable, ignored: {tex}")

                if visual.material is not None:
                    # 保留 URDF/转换阶段显式材质（其名已保证唯一或由用户指定）；
                    # 若无 rgba/texture 再补 MTL 信息
                    mat = visual.material
                    if mat.color_rgba is None and rgba is not None:
                        mat.color_rgba = rgba
                    if mat.texture_file is None and tex_landed is not None:
                        mat.texture_file = tex_landed
                else:
                    mat = UrdfParser.UrdfMaterial(
                        m_name=mat_name,
                        color_rgba=rgba,
                        texture_file=tex_landed,
                    )

                new_geom = UrdfParser.UrdfGeometry(
                    g_type="mesh", filename=str(split_file.resolve()), scale=geom.scale
                )
                new_visuals.append(
                    UrdfParser.UrdfVisual(visual.origin, new_geom, mat)
                )
            processed += 1

        link.visuals = new_visuals

    logger.info(f"[obj-split] Processed {processed} OBJ visual mesh(es).")
    return processed


def _first_usemtl(obj_file: Path) -> Optional[str]:
    """读取子 OBJ 的第一个 usemtl 名。"""
    try:
        with obj_file.open("r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                s = line.strip()
                if s.lower().startswith("usemtl "):
                    parts = s.split()
                    return parts[1] if len(parts) > 1 else None
    except OSError:
        pass
    return None


def mesh_decomposer(
    urdf_model: UrdfParser,
    meshes_dir: Optional[Union[str, Path]] = None,
    registry: Optional[ResourceRegistry] = None,
    is_symlink_copy: bool = False,
) -> int:
    """
    EN: Public entry - decompose multi-material OBJ meshes referenced by the model.
    CN: 公开入口——分解 model 中引用的多材质 OBJ。

    :return: EN number of OBJ visuals processed CN 处理的 OBJ visual 数量
    """
    return decompose_obj_meshes(
        urdf_model=urdf_model,
        meshes_dir=meshes_dir,
        registry=registry,
        is_symlink_copy=is_symlink_copy,
    )
