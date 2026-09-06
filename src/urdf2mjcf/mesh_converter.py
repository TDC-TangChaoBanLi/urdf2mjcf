#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mesh conversion and processing module (model-level / pre-MJCF).

EN:
- Resolve URDF mesh paths (package://, file://, relative/absolute)
- Copy / symlink mesh files into the target meshes directory (optionally)
- Convert non-stl/obj meshes: visual -> OBJ (try to preserve materials),
  collision -> STL
- Fix ASCII STL -> binary STL (MuJoCo only supports binary STL)
- Land URDF <material><texture> texture files into the meshes directory
- Register every created file in the ResourceRegistry for later cleanup

This module works on the in-memory UrdfParser model and does NOT write any
intermediate URDF file.

CN:
- 解析 URDF mesh 路径（package://、file://、相对/绝对）
- 将 mesh 复制/软链接到目标 meshes 目录（可选）
- 非 stl/obj 格式转换：visual -> OBJ（尽可能保留材质），collision -> STL
- ASCII STL -> 二进制 STL 修复（MuJoCo 仅支持二进制 STL）
- 将 URDF <material><texture> 贴图落地到 meshes 目录
- 所有创建的文件登记到 ResourceRegistry 供后续清理

本模块在内存中的 UrdfParser 模型上工作，不再产出中间 URDF 文件。
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import pathlib
import re
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Union

import trimesh

from . import mesh_utils
from .resource_registry import ResourceRegistry
from .urdf_parser import UrdfParser

# IMPORTANT:
# EN: Do NOT call logging.basicConfig() here. CLI configures logging globally.
# CN: 不要在此模块内配置 logging.basicConfig()，由 CLI 统一配置日志风格。
logger = logging.getLogger("urdf2mjcf.mesh_converter")


class MeshConverter:
    def __init__(self,
                urdf_model: UrdfParser,
                meshes_dir: Union[str, Path],
                json_path: Optional[Union[str, Path]] = None,
                is_copy_meshes: bool = True,
                is_symlink_copy: bool = False,
                registry: Optional[ResourceRegistry] = None,
        ):

        self.urdf_model: UrdfParser = urdf_model
        self.json_cfg: dict = {}
        self.package_map: Dict[str, str] = {}
        self.is_copy_meshes: bool = is_copy_meshes
        self.meshes_dir: Path = Path(meshes_dir)
        self.meshes_dir.mkdir(parents=True, exist_ok=True)
        self.is_symlink_copy: bool = is_symlink_copy
        self.registry: ResourceRegistry = registry if registry is not None else ResourceRegistry()

        # load json config (mesh_converter section -> package_map)
        if json_path is None:
            self.json_cfg = {}
        else:
            json_path = Path(json_path)
            if not json_path.is_file():
                logger.error(f"{json_path} does not exist.")
            raw = json.loads(json_path.read_text(encoding="utf-8"))
            mesh_converter_key = "mesh_converter"
            if mesh_converter_key in raw:
                self.json_cfg = raw[mesh_converter_key]
            else:
                logger.warning(f"{json_path} does not contain '{mesh_converter_key}' key.")
            if "package_map" in self.json_cfg:
                self.package_map = self.json_cfg["package_map"]
            else:
                logger.warning(f"{json_path}.{mesh_converter_key} does not contain 'package_map' key.")

    # ------------------------------------------------------------------
    # Path resolution
    # ------------------------------------------------------------------

    @staticmethod
    def _find_package_path(pkg_name: str, pkg_map: Dict[str, str]) -> Optional[str]:
        # 1) rospkg
        try :
            import rospkg
            _ROSPKG = rospkg.RosPack()
        except ImportError:
            _ROSPKG = None
        if _ROSPKG is not None:
            try:
                return _ROSPKG.get_path(pkg_name)
            except Exception:
                pass

        # 2) CMAKE_PREFIX_PATH
        cmake_prefix_paths = os.environ.get("CMAKE_PREFIX_PATH", "")
        if cmake_prefix_paths:
            for path_str in cmake_prefix_paths.split(":"):
                if not path_str:
                    continue
                p = pathlib.Path(path_str)

                # Common install layout: <prefix>/share/<pkg_name>
                share_pkg_path = p / "share" / pkg_name
                if share_pkg_path.exists():
                    return str(share_pkg_path)

                # Sometimes the prefix itself is the pkg directory
                if p.name == pkg_name and p.exists():
                    return str(p)

        # 3) /opt/ros/<distro>/share/<pkg_name>
        opt_ros = pathlib.Path("/opt/ros")
        if opt_ros.exists():
            for ros_distro_dir in opt_ros.iterdir():
                if not ros_distro_dir.is_dir():
                    continue
                share_dir = ros_distro_dir / "share" / pkg_name
                if share_dir.exists():
                    return str(share_dir)

        # 4) package_map
        if pkg_name in pkg_map:
            if Path(pkg_map[pkg_name]).exists():
                return pkg_map[pkg_name]
        return None

    @staticmethod
    def _resolve_mesh_path(path: str, pkg_map: Dict[str,str]) -> Optional[str]:
        p = path.strip()
        if p.startswith("package://"): # package:// 格式
            p = p[len("package://"):] # 去掉 package://
            package = p.split("/")[0]
            pkg_path = MeshConverter._find_package_path(package, pkg_map)
            if pkg_path is None:
                logger.error(f"Can not find mesh file: '{path}', in package: '{package}'")
                return None
            p =  pkg_path + "/" + path[len("package://")+len(package)+1:]
            if Path(p).is_file():
                return p
            else:
                logger.error(f"Can not find mesh file: '{path}', resolve to: '{p}'")
                return None

        elif p.startswith("file://"): # file:// 格式
            p = p[len("file://"):] # 去掉 file://
            if Path(p).is_file():
                return p
            else:
                logger.error(f"Can not find mesh file: '{path}', resolve to: '{p}'")
                return None

        else:
            if Path(p).is_file():
                return p
            else:
                logger.error(f"Can not find mesh file: '{path}', resolve to: '{p}'")
                return None


    @staticmethod
    def _convert_mesh(source_path: Union[str,Path], dest_dir: Union[str,Path], dest_name: Optional[str], dest_format: str="stl") -> List[Path]:
        """
        模型格式转换与分解

        :param source_path: 原模型文件路径
        :param dest_dir: 输出模型文件目录
        :param dest_name: 输出模型文件名称
        :param dest_format: 输出模型文件格式 (stl, obj) 输出 stl 文件将输出为单个文件， 输出 obj 文件将输出为分解后的多个文件
        :return: 输出模型文件路径列表
        """
        source_path = Path(source_path)
        dest_dir = Path(dest_dir)
        dest_name = Path(dest_name).stem

        output_paths = []

        # 加载模型
        loaded = trimesh.load(source_path, force="scene")
        scene = loaded if isinstance(loaded, trimesh.Scene) else trimesh.Scene(loaded)

        if dest_format != "obj" and dest_format != "stl":
            logger.warning(f"Mesh Converter only support 'obj' and 'stl' format. Your format is {dest_format}, so we will use 'stl' format instead.")
            dest_format = "stl"

        # 如果是输出格式为 obj ：分离处理
        if dest_format == "obj":
            meshes: List[trimesh.Geometry] = scene.dump(concatenate=False)
            for idx, mesh in enumerate(meshes):
                dest_path = dest_dir / f"{dest_name}_{idx}.obj"
                mesh.export(dest_path, file_type="obj", mtl_name = f"{dest_name}_{idx}.mtl")
                output_paths.append(dest_path)
                logger.debug(f"Converted '{source_path.name}' to {dest_name}_{idx}.obj")
        else:
            # mesh: trimesh.Geometry = scene.dump(concatenate=True)
            mesh = scene.to_geometry()
            dest_path = dest_dir / f"{dest_name}.stl"
            mesh.export(dest_path, file_type="stl")
            output_paths.append(dest_path)
            logger.debug(f"Converted '{source_path.name}' to {dest_name}.stl")

        return output_paths

    @staticmethod
    def _compute_mesh_hash(mesh_path: Union[str, Path]) -> str:
        """
        计算模型文件的哈希值

        :param mesh_path: 模型文件路径
        :return: 模型文件的哈希值前8位
        """
        mesh_path = Path(mesh_path)
        with open(mesh_path, 'rb') as f:
            file_content = f.read()
            md5_hash = hashlib.md5(file_content)
            return md5_hash.hexdigest()[:8]

    # ------------------------------------------------------------------
    # Texture handling (URDF <material><texture .../>)
    # ------------------------------------------------------------------

    def _resolve_texture_path(self, path: str) -> Optional[str]:
        """
        解析 URDF texture filename（package:// / file:// / 相对 / 绝对）为绝对路径。

        相对路径优先相对 URDF 文件所在目录解析，其次相对当前工作目录。
        """
        if not path:
            return None
        p = path.strip()
        if p.startswith("package://") or p.startswith("file://"):
            return MeshConverter._resolve_mesh_path(p, self.package_map)
        p_path = Path(p)
        if p_path.is_absolute():
            return str(p_path) if p_path.is_file() else None
        # 相对 URDF 文件目录
        urdf_dir = self.urdf_model.urdf_file_path.parent if self.urdf_model.urdf_file_path is not None else None
        if urdf_dir is not None:
            cand = (urdf_dir / p_path).resolve()
            if cand.is_file():
                return str(cand)
        cand_cwd = p_path.resolve()
        if cand_cwd.is_file():
            return str(cand_cwd)
        return None

    def _land_texture_file(self, texture_path: str) -> Optional[str]:
        """
        将 URDF 材质中的贴图落地到 meshes_dir（复制/软链或转 PNG），
        返回落地后的**绝对路径**并写入 registry；失败返回 None。

        不修改源文件；若已在 meshes_dir 内则直接返回。
        """
        src = self._resolve_texture_path(texture_path)
        if src is None or not Path(src).is_file():
            logger.warning(f"Texture file not found: '{texture_path}'")
            return None
        src_path = Path(src)
        try:
            if src_path.parent.resolve() == self.meshes_dir.resolve():
                return str(src_path)
        except OSError:
            pass

        landed = mesh_utils.ensure_texture_ready(
            src_path,
            self.meshes_dir,
            is_symlink_copy=self.is_symlink_copy,
            register=self.registry,
        )
        if landed is None:
            return None
        return str(landed.resolve())

    def process_urdf_textures(self) -> int:
        """
        遍历根级 materials 与所有 link 的 visual/collision material，
        将 <material><texture filename=.../> 指定的贴图落地到 meshes_dir，
        并把 material.texture_file 更新为落地后的绝对路径。

        :return: 处理成功的贴图数量
        """
        add_num = 0
        all_materials: List[UrdfParser.UrdfMaterial] = []
        all_materials.extend(self.urdf_model.materials)
        for link in self.urdf_model.links:
            for vs_or_cs in (link.visuals or []) + (link.collisions or []):
                if vs_or_cs.material is not None:
                    all_materials.append(vs_or_cs.material)

        seen: set = set()
        for material in all_materials:
            if not material.texture_file:
                continue
            key = (id(material), material.texture_file)
            if key in seen:
                continue
            seen.add(key)
            landed = self._land_texture_file(material.texture_file)
            if landed is None:
                material.texture_file = None  # 无法落地则忽略，避免坏引用
                continue
            material.texture_file = landed
            add_num += 1
            logger.debug(f"Processed URDF texture: {material.m_name} -> {landed}")
        return add_num

    # ------------------------------------------------------------------
    # ASCII STL -> binary STL
    # ------------------------------------------------------------------

    def _fix_ascii_stl(self, mesh_path: Path) -> Optional[Path]:
        """
        若 mesh_path 是 ASCII STL，则转换为二进制 STL 并落地到 meshes_dir，
        返回转换后的绝对路径（登记到 registry）；否则返回 None。
        """
        if mesh_path.suffix.lower() != ".stl":
            return None
        if not mesh_utils.is_ascii_stl(mesh_path):
            return None
        logger.warning(f"[stl-fix] ASCII STL detected, converting to binary STL: {mesh_path}")
        # 二进制产物直接写入 meshes_dir（避免在源目录产生中间文件）
        self.meshes_dir.mkdir(parents=True, exist_ok=True)
        dest_binary = self.meshes_dir / f"{mesh_path.stem}_binary.stl"
        binary = mesh_utils.convert_ascii_stl_to_binary(mesh_path, dest_path=dest_binary)
        if binary is None:
            return None
        self.registry.mark_converted(binary, origin=mesh_path)
        logger.info(f"[stl-fix] binary STL landed in meshes dir: {binary}")
        return Path(binary).resolve()

    # ------------------------------------------------------------------
    # Main entry
    # ------------------------------------------------------------------

    def convert_meshes(self) -> UrdfParser:
        """
        遍历所有 link 的 visual/collision mesh，执行：
        - 路径解析（package:// 等）
        - ASCII STL -> 二进制 STL 修复
        - 非 stl/obj 格式转换（visual -> obj, collision -> stl）
        - 可选复制/软链源 mesh 到 meshes_dir
        并就地更新 model 中 geometry.filename。
        """

        def handle_vc(v_or_c) -> None:
            """处理单个 visual/collision 的 mesh geometry。"""
            geom = v_or_c.geometry
            if geom.g_type != "mesh":
                return
            raw_filename = geom.filename
            if not raw_filename:
                return

            source = MeshConverter._resolve_mesh_path(raw_filename, self.package_map)
            if source is None or not Path(source).is_file():
                logger.warning(f"Mesh file '{raw_filename}' does not exist. Ignoring...")
                return
            source_path = Path(source)
            mesh_hash = MeshConverter._compute_mesh_hash(source_path)

            # 1) ASCII STL -> 二进制 STL 修复（visual/collision 统一，MuJoCo 需要二进制）
            fixed = self._fix_ascii_stl(source_path)
            if fixed is not None:
                geom.filename = str(fixed)
                return

            # 2) 非 stl/obj 格式 -> 转换（visual -> obj, collision -> stl）
            if source_path.suffix.lower() not in (".stl", ".obj"):
                dest_mesh_name = source_path.stem + "_" + mesh_hash
                dest_format = "obj" if isinstance(v_or_c, UrdfParser.UrdfVisual) else "stl"
                converted_list = MeshConverter._convert_mesh(
                    source_path, self.meshes_dir, dest_mesh_name, dest_format
                )
                # 登记所有转换产物
                for c_path in converted_list:
                    self.registry.mark_converted(c_path, origin=source_path)
                # 将原 visual 替换为转换产物 visual/collision（保持 origin/material）
                self._replace_vc_with_converted(v_or_c, converted_list)
                return

            # 3) stl/obj：ASCII 已修；其余按 copy 策略落地或使用原路径
            if self.is_copy_meshes:
                if source_path.suffix.lower() == ".obj":
                    # OBJ：连带复制其 MTL 与贴图 sidecars
                    landed = mesh_utils.ensure_file_available(
                        source_path,
                        self.meshes_dir,
                        is_symlink_copy=self.is_symlink_copy,
                        register=self.registry,
                        kind="symlink" if self.is_symlink_copy else "copy",
                        force_land=True,
                    )
                    if not self.is_symlink_copy:
                        self._copy_obj_with_sidecars(source_path, Path(landed))
                    geom.filename = str(Path(landed).resolve())
                else:
                    landed = mesh_utils.ensure_file_available(
                        source_path,
                        self.meshes_dir,
                        is_symlink_copy=self.is_symlink_copy,
                        register=self.registry,
                        kind="symlink" if self.is_symlink_copy else "copy",
                        force_land=True,
                    )
                    geom.filename = str(Path(landed).resolve())
            else:
                # 不复制：保持绝对路径引用（mjcf_generator 会做相对化）
                geom.filename = str(source_path.resolve())

        for link in self.urdf_model.links:
            if link.visuals:
                for v in list(link.visuals):
                    handle_vc(v)
            if link.collisions:
                for c in list(link.collisions):
                    handle_vc(c)

        # 落地 OBJ 时若有 MTL 伴生（转换/复制），保证材质文件也进 meshes_dir
        self._land_obj_sidecars()
        return self.urdf_model

    # ------------------------------------------------------------------
    # OBJ sidecars（MTL + 贴图）落地
    # ------------------------------------------------------------------

    def _land_obj_sidecars(self) -> None:
        """
        遍历 model 中所有 mesh geometry：
        - 若为 OBJ，读取其 mtllib 引用；
        - 若该 MTL 缺失（OBJ 已被复制到 meshes_dir 而 MTL 未跟随），
          尝试从原始 OBJ（meshes_dir 之外的源）补齐 —— 源路径已不可知，
          因此在**复制 OBJ 时**（见 _copy_obj_with_sidecars）即连带复制。
        - 本方法为兜底：对已存在 MTL 的 OBJ 校验贴图完整性。
        """
        for link in self.urdf_model.links:
            for vs_or_cs in (link.visuals or []) + (link.collisions or []):
                geom = vs_or_cs.geometry
                if geom.g_type != "mesh" or not geom.filename:
                    continue
                obj_path = Path(geom.filename)
                if obj_path.suffix.lower() != ".obj" or not obj_path.is_file():
                    continue
                mtl_name = mesh_utils.parse_mtl_name_from_obj(obj_path)
                if not mtl_name:
                    continue
                mtl_src = obj_path.parent / mtl_name
                if not mtl_src.is_file():
                    logger.warning(
                        f"[obj-sidecar] MTL missing next to OBJ: {obj_path} "
                        f"(referenced '{mtl_name}')"
                    )
                    continue
                # 将 MTL 引用的贴图落地到 meshes_dir（与 OBJ 同目录）
                _, _, map_kd = mesh_utils.get_mtl_info(obj_path)
                for tex in map_kd.values():
                    if tex is not None and tex.is_file():
                        mesh_utils.ensure_file_available(
                            tex,
                            self.meshes_dir,
                            is_symlink_copy=self.is_symlink_copy,
                            register=self.registry,
                            kind="texture",
                            force_land=True,
                        )

    def _copy_obj_with_sidecars(self, source_obj: Path, dest_obj: Path) -> None:
        """
        复制一个 OBJ 到目标路径，并连带把其 mtllib 引用的 MTL 及 MTL 的
        map_* 贴图复制到 dest_obj 所在目录（保持 basename），保证 OBJ 材质完整。

        - MTL 落为 dest_dir / basename(mtl)
        - 若源 OBJ 内 mtllib 带路径，重写目标 OBJ 的 mtllib 为 basename
        - 所有复制产物登记到 registry
        """
        mtl_name = mesh_utils.parse_mtl_name_from_obj(source_obj)
        if not mtl_name:
            return
        mtl_src = source_obj.parent / mtl_name
        if not mtl_src.is_file():
            logger.warning(f"MTL file not found for OBJ sidecar: {mtl_src}")
            return
        dest_dir = dest_obj.parent
        dest_dir.mkdir(parents=True, exist_ok=True)

        # 复制 MTL -> dest_dir / basename(mtl)
        mtl_dest = dest_dir / Path(mtl_name).name
        if not os.path.lexists(mtl_dest):
            if self.is_symlink_copy:
                os.symlink(os.path.abspath(mtl_src), mtl_dest)
                self.registry.mark_symlink(mtl_dest, origin=mtl_src)
            else:
                shutil.copy2(mtl_src, mtl_dest)
                self.registry.mark_copy(mtl_dest, origin=mtl_src)
            logger.debug(f"Copied MTL sidecar: {mtl_src} -> {mtl_dest}")

        # 解析 MTL 内 map_* 贴图并复制
        try:
            mtl_text = mtl_src.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            mtl_text = ""
        tex_pattern = re.compile(
            r"(?im)^\s*map_\w+\s+(?:-[A-Za-z0-9_]+\s+(?:\S+\s+)*?)?(\S+\.\w+)\s*$"
        )
        for m in tex_pattern.finditer(mtl_text or ""):
            tex_rel = m.group(1).strip()
            if not tex_rel:
                continue
            tex_src = (mtl_src.parent / tex_rel).resolve()
            if not tex_src.is_file():
                continue
            tex_dest = dest_dir / tex_src.name
            if not os.path.lexists(tex_dest):
                if self.is_symlink_copy:
                    os.symlink(os.path.abspath(tex_src), tex_dest)
                    self.registry.mark_symlink(tex_dest, origin=tex_src)
                else:
                    shutil.copy2(tex_src, tex_dest)
                    self.registry.mark_copy(tex_dest, origin=tex_src)
                logger.debug(f"Copied texture sidecar: {tex_src} -> {tex_dest}")

        # 重写目标 OBJ 的 mtllib 为 basename（若原引用带路径）
        if Path(mtl_name).name != mtl_name:
            try:
                obj_text = dest_obj.read_text(encoding="utf-8", errors="ignore")
            except OSError:
                return
            new_obj = re.sub(
                r"(?im)^mtllib\s+.+$",
                f"mtllib {Path(mtl_name).name}",
                obj_text,
                count=1,
            )
            try:
                dest_obj.write_text(new_obj, encoding="utf-8")
            except OSError:
                pass

    def _replace_vc_with_converted(
        self, v_or_c, converted_list: List[Path]
    ) -> None:
        """
        将 v_or_c（所在 list 中）替换为 converted_list 对应的多个 visual/collision。
        转换后每个新 visual 均尝试从伴生 MTL 提取材质（rgba/map_Kd）。
        """
        # 找到 v_or_c 所在的 list
        owner_list: Optional[list] = None
        for link in self.urdf_model.links:
            if link.visuals and v_or_c in link.visuals:
                owner_list = link.visuals
                break
            if link.collisions and v_or_c in link.collisions:
                owner_list = link.collisions
                break
        if owner_list is None:
            return
        idx = owner_list.index(v_or_c)
        is_visual = isinstance(v_or_c, UrdfParser.UrdfVisual)
        src_origin = v_or_c.origin
        src_material = v_or_c.material
        src_geom = v_or_c.geometry

        if is_visual:
            new_items: List[UrdfParser.UrdfVisual] = []
            for added_mesh in converted_list:
                added_path = Path(added_mesh)
                added_geometry = UrdfParser.UrdfGeometry(
                    g_type="mesh",
                    filename=str(added_path),
                    scale=src_geom.scale,
                )
                material = src_material
                if material is None:
                    material = self._extract_mtl_material(added_path)
                new_items.append(
                    UrdfParser.UrdfVisual(src_origin, added_geometry, material)
                )
        else:
            new_items = []  # type: ignore[assignment]
            for added_mesh in converted_list:
                added_path = Path(added_mesh)
                added_geometry = UrdfParser.UrdfGeometry(
                    g_type="mesh",
                    filename=str(added_path),
                    scale=src_geom.scale,
                )
                new_items.append(
                    UrdfParser.UrdfCollision(src_origin, added_geometry, src_material)
                )
        owner_list[idx : idx + 1] = new_items

    def _extract_mtl_material(self, obj_path: Path) -> Optional[UrdfParser.UrdfMaterial]:
        """
        从 OBJ 伴生 MTL 提取材质（rgba + 贴图路径）。

        注意：trimesh 从多材质 DAE 拆出的多个子 OBJ，其 MTL 中的 newmtl 名
        可能都叫 "material_0"，颜色却不同。为避免跨 OBJ 撞名导致 MJCF 中
        材质错误复用，这里把材质名改为 "{obj_stem}_{mtl_name}" 全局唯一。
        """
        mtl_file, rgba_map, map_kd_map = mesh_utils.get_mtl_info(obj_path)
        if not rgba_map:
            return None
        first_name = next(iter(rgba_map))
        unique_name = f"{obj_path.stem}_{first_name}"
        tex = map_kd_map.get(first_name)
        return UrdfParser.UrdfMaterial(
            m_name=unique_name,
            color_rgba=rgba_map[first_name],
            texture_file=str(tex) if tex is not None and tex.is_file() else None,
        )

    def process(self) -> UrdfParser:
        """
        完整执行 mesh_converter 阶段：
        1. convert_meshes()  - 网格路径解析 / ASCII STL 修复 / 格式转换 / 复制落地
        2. process_urdf_textures() - URDF <material><texture> 贴图落地
        """
        self.convert_meshes()
        self.process_urdf_textures()
        return self.urdf_model


def mesh_converter(
    urdf_model: UrdfParser,
    meshes_dir: Union[str, Path],
    json_path: Optional[Union[str, Path]] = None,
    is_copy_meshes: bool = True,
    is_symlink_copy: bool = False,
    registry: Optional[ResourceRegistry] = None,
) -> UrdfParser:
    """
    EN: Process all meshes/textures in the URDF model (model-level, pre-MJCF).
    CN: 处理 URDF model 中的所有 mesh/texture（model 级、写 MJCF 前）。

    :param urdf_model: EN parsed URDF model CN 已解析的 URDF model
    :param meshes_dir: EN target directory for converted/copied meshes
                       CN mesh 落地目录
    :param json_path: EN optional JSON config (mesh_converter.package_map)
                      CN JSON 配置（package_map）
    :param is_copy_meshes: EN copy source meshes into meshes_dir
                           CN 是否将源 mesh 复制进 meshes_dir
    :param is_symlink_copy: EN symlink instead of copy CN 使用软链而非复制
    :param registry: EN ResourceRegistry (shared across pipeline)
                     CN 资源登记表（跨阶段共享）
    :return: EN processed model CN 处理后的 model
    """
    converter = MeshConverter(
        urdf_model=urdf_model,
        meshes_dir=meshes_dir,
        json_path=json_path,
        is_copy_meshes=is_copy_meshes,
        is_symlink_copy=is_symlink_copy,
        registry=registry,
    )
    converter.process()
    return urdf_model


if __name__ == "__main__":
    import logging

    logging.basicConfig(level=logging.INFO)
    urdf_path = "./examples/urdf/UR5e.urdf"
    json_path = "./examples/config/config.json"
    meshes_dir = "./examples/meshes_0/"

    urdf_model = UrdfParser.parse_urdf(urdf_path)
    mesh_converter(
        urdf_model=urdf_model,
        meshes_dir=meshes_dir,
        json_path=json_path,
        is_copy_meshes=True,
        is_symlink_copy=False
    )
