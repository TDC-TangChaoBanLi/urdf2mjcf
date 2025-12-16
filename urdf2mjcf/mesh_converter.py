#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mesh conversion and processing module.

EN:
- Resolve URDF mesh paths (package://, file://, relative/absolute)
- Convert visual meshes to OBJ (try to preserve materials)
- Convert collision meshes to STL
- Deduplicate meshes by content hash (optional hardlink)
- Optional copy/symlink of non-converted meshes
- Use per-mesh subdirectory for converted OBJ to avoid MTL conflicts

CN:
- 解析 URDF mesh 路径（package://、file://、相对/绝对）
- visual 网格转换为 OBJ（尽可能保留材质）
- collision 网格转换为 STL
- 基于内容 hash 去重（可选硬链接）
- 可选复制/软链接未转换网格
- 转换后的 OBJ 使用独立子目录，避免 material.mtl 冲突
"""

from __future__ import annotations

import glob
import hashlib
import logging
import os
import pathlib
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import trimesh

# IMPORTANT:
# EN: Do NOT call logging.basicConfig() here. CLI configures logging globally.
# CN: 不要在此模块内配置 logging.basicConfig()，由 CLI 统一配置日志风格。
logger = logging.getLogger("urdf2mjcf.mesh_converter")

# =============================================================================
# Optional ROS package resolver
# 可选 ROS 包路径解析器
# =============================================================================
try:
    import rospkg  # type: ignore

    _ROSPKG = rospkg.RosPack()
except ImportError:
    _ROSPKG = None


@dataclass
class MeshInfo:
    """
    EN: Mesh record extracted from URDF.
    CN: 从 URDF 中提取的 mesh 记录信息。
    """

    element: ET.Element
    abs_path: str
    scheme: str  # "package" or "file"
    pkg: Optional[str]
    rel_path: str
    mesh_type: str  # "visual" or "collision"


# =============================================================================
# Utility functions
# 工具函数
# =============================================================================
def _compute_file_hash(filepath: str, algorithm: str = "md5") -> str:
    """
    EN: Compute file hash (default md5).
    CN: 计算文件 hash（默认 md5）。
    """
    h = hashlib.new(algorithm)
    with open(filepath, "rb") as f:
        while True:
            chunk = f.read(8192)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def _generate_unique_mesh_name(abs_path: str, target_ext: str) -> str:
    """
    EN: Generate a unique filename to avoid conflicts across different paths.
    CN: 生成唯一文件名，避免不同路径下同名文件冲突。
    """
    path_hash = hashlib.md5(abs_path.encode("utf-8")).hexdigest()[:8]
    original_name = pathlib.Path(abs_path).stem
    if len(original_name) > 20:
        original_name = original_name[:20]
    return f"{original_name}_{path_hash}.{target_ext}"


# =============================================================================
# ROS package path resolution
# ROS 包路径解析
# =============================================================================
def _find_ros_package_path(pkg_name: str) -> Optional[str]:
    """
    EN: Find the filesystem path of a ROS package.
    CN: 查找 ROS 包在文件系统中的路径。

    Search order / 搜索顺序:
    1) rospkg (if installed)
    2) CMAKE_PREFIX_PATH
    3) /opt/ros/*/share/<pkg_name>
    """
    # 1) rospkg
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

    return None


# =============================================================================
# Path parsing and construction
# 路径解析与构造
# =============================================================================
def _resolve_mesh_path(urdf_path: str, filename: str) -> Tuple[str, str, Optional[str], str]:
    """
    EN: Resolve a URDF mesh filename to an absolute path.
    CN: 将 URDF 中 mesh 的 filename 解析为绝对路径。

    Returns / 返回:
        (abs_path, scheme, pkg, rel_path)
        - scheme: "package" or "file"
        - pkg: package name for package://, otherwise None
        - rel_path: internal rel path in package://, otherwise original path
    """
    filename = filename.strip()
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))

    # package://<pkg>/<rel>
    if filename.startswith("package://"):
        scheme = "package"
        s = filename[len("package://") :]
        if "/" not in s:
            raise ValueError(f"Invalid package URI (missing '/'): {filename}")
        pkg, rel = s.split("/", 1)

        pkg_path = _find_ros_package_path(pkg)
        if pkg_path is None:
            cmake_paths = os.environ.get("CMAKE_PREFIX_PATH", "")
            ros_paths = os.environ.get("ROS_PACKAGE_PATH", "")
            msg = f"ROS package not found: '{pkg}'. "
            if cmake_paths:
                msg += f"CMAKE_PREFIX_PATH='{cmake_paths}'. "
            if ros_paths:
                msg += f"ROS_PACKAGE_PATH='{ros_paths}'. "
            msg += "Please ensure the package is installed and the environment is sourced."
            raise FileNotFoundError(msg)

        abs_path = os.path.join(pkg_path, rel)
        if not os.path.exists(abs_path):
            # Provide directory listing to help debugging
            dir_path = os.path.dirname(abs_path)
            extra = ""
            if os.path.exists(dir_path):
                try:
                    files = glob.glob(os.path.join(dir_path, "*"))
                    extra = f" Directory listing: {files}"
                except Exception:
                    extra = ""
            else:
                extra = f" Directory does not exist: {dir_path}"
            raise FileNotFoundError(
                f"Mesh file not found: {abs_path} (pkg='{pkg}', pkg_path='{pkg_path}', rel='{rel}').{extra}"
            )

        # Validate symlink target if needed
        if os.path.islink(abs_path):
            target_path = os.readlink(abs_path)
            if not os.path.isabs(target_path):
                target_path = os.path.join(os.path.dirname(abs_path), target_path)
            if not os.path.exists(target_path):
                raise FileNotFoundError(f"Mesh symlink target not found: {abs_path} -> {target_path}")
        elif not os.path.isfile(abs_path):
            raise FileNotFoundError(f"Mesh path is not a regular file: {abs_path}")

        return abs_path, scheme, pkg, rel

    # file:// or plain path
    scheme = "file"
    pkg = None
    rel = filename
    if filename.startswith("file://"):
        rel = filename[len("file://") :]

    abs_path = rel if os.path.isabs(rel) else os.path.join(urdf_dir, rel)

    if not os.path.exists(abs_path):
        raise FileNotFoundError(f"Mesh file not found: {abs_path}")

    if os.path.islink(abs_path):
        target_path = os.readlink(abs_path)
        if not os.path.isabs(target_path):
            target_path = os.path.join(os.path.dirname(abs_path), target_path)
        if not os.path.exists(target_path):
            raise FileNotFoundError(f"Mesh symlink target not found: {abs_path} -> {target_path}")
    elif not os.path.isfile(abs_path):
        raise FileNotFoundError(f"Mesh path is not a regular file: {abs_path}")

    return abs_path, scheme, pkg, rel


def _build_standard_mesh_path(output_urdf_path: str, mesh_abs_path: str) -> str:
    """
    EN: Build a standardized mesh path string for writing back into URDF.
    CN: 构造写回 URDF 的标准路径字符串。

    Rule / 规则:
    - If mesh is under /opt/ros/, keep absolute path (stable for installed packages)
      若 mesh 位于 /opt/ros/ 下，使用绝对路径（对安装包更稳定）
    - Otherwise, use a path relative to output URDF directory
      否则使用相对输出 URDF 的相对路径
    """
    output_urdf_dir = os.path.dirname(os.path.abspath(output_urdf_path))

    if mesh_abs_path.startswith("/opt/ros/"):
        return mesh_abs_path.replace(os.sep, "/")

    try:
        rel_to_urdf = os.path.relpath(mesh_abs_path, output_urdf_dir)
        return rel_to_urdf.replace(os.sep, "/")
    except ValueError:
        # Different drive on Windows or other edge cases
        # Windows 不同盘符等场景回退为绝对路径
        return mesh_abs_path.replace(os.sep, "/")


# =============================================================================
# Mesh conversion helpers
# Mesh 转换辅助函数
# =============================================================================
def _export_with_scene_preserving_materials(source_path: str, dest_obj_path: str) -> None:
    """
    EN: Export as OBJ using Scene to better preserve multi-material meshes (e.g., DAE).
    CN: 使用 Scene 导出 OBJ，以更好保留多材质（如 DAE）。
    """
    loaded = trimesh.load(source_path, force="scene", process=False)
    scene = loaded if isinstance(loaded, trimesh.Scene) else trimesh.Scene(loaded)
    scene.export(dest_obj_path)


def _fix_mtl_name(dest_obj_path: str) -> None:
    """
    EN:
    - Rename 'material.mtl' -> '<stem>.mtl' in the OBJ folder
    - Patch 'mtllib material.mtl' line inside the OBJ file

    CN:
    - 将 OBJ 同目录下的 'material.mtl' 改名为 '<stem>.mtl'
    - 同时修补 OBJ 文件中的 'mtllib material.mtl'
    """
    obj_path = pathlib.Path(dest_obj_path)
    obj_dir = obj_path.parent
    stem = obj_path.stem

    old_mtl = obj_dir / "material.mtl"
    if not old_mtl.exists():
        return

    new_mtl = obj_dir / f"{stem}.mtl"
    if not new_mtl.exists():
        old_mtl.rename(new_mtl)

    try:
        txt = obj_path.read_text(encoding="utf-8")
        txt = txt.replace("mtllib material.mtl", f"mtllib {stem}.mtl")
        obj_path.write_text(txt, encoding="utf-8")
    except Exception as e:
        logger.warning("Failed to patch OBJ mtllib line: %s (%s)", dest_obj_path, e)


def _convert_mesh(
    source_path: str,
    target_ext: str,
    mesh_dir: str,
    dedup_hashes: Optional[Dict[str, str]] = None,
) -> Tuple[str, Optional[str]]:
    """
    EN:
    Convert source mesh to target_ext ('obj' or 'stl') using trimesh.

    Key behaviors:
    - Each converted mesh uses its own directory: <mesh_dir>/<unique_stem>/
      to avoid material.mtl / texture name conflicts.
    - Dedup happens BEFORE conversion by hashing the source file. If identical content
      has been converted before, return (dest_path, canonical_path) and let the caller
      create a hardlink (or copy fallback).

    CN:
    使用 trimesh 将 source_path 转换为 target_ext（'obj' 或 'stl'）。
    关键点：
    - 每个转换 mesh 使用独立子目录，避免 material.mtl/纹理冲突
    - 转换前先 hash 去重：如果内容相同，直接返回 canonical_path 让上层创建硬链接/复制

    Returns / 返回:
        (dest_mesh_path, canonical_path)
        - canonical_path is not None when dedup hit (caller should hardlink/copy)
    """
    source_path = os.path.abspath(source_path)
    if not os.path.isfile(source_path):
        raise FileNotFoundError(f"Source mesh file not found: {source_path}")

    target_ext = target_ext.lower().lstrip(".")
    unique_stem = pathlib.Path(_generate_unique_mesh_name(source_path, target_ext)).stem
    base_mesh_dir = os.path.abspath(mesh_dir)
    dest_dir = os.path.join(base_mesh_dir, unique_stem)
    os.makedirs(dest_dir, exist_ok=True)
    dest_mesh_path = os.path.join(dest_dir, f"{unique_stem}.{target_ext}")

    # Deduplicate BEFORE conversion
    # 转换前去重
    file_hash = _compute_file_hash(source_path)
    if dedup_hashes is not None and file_hash in dedup_hashes:
        canonical = dedup_hashes[file_hash]
        logger.info("[dedup] source='%s' -> will link to canonical='%s'", source_path, canonical)
        return dest_mesh_path, canonical

    logger.info("[convert] %s -> %s", source_path, dest_mesh_path)

    src_ext = os.path.splitext(source_path)[1].lower()

    # Special handling for DAE/ZAE -> OBJ to preserve materials
    # 针对 DAE/ZAE -> OBJ：尽可能保留材质
    if src_ext in [".dae", ".zae"] and target_ext == "obj":
        _export_with_scene_preserving_materials(source_path, dest_mesh_path)
        _fix_mtl_name(dest_mesh_path)
    else:
        loaded = trimesh.load(source_path, force=None, process=False)

        if isinstance(loaded, trimesh.Scene) and target_ext == "obj":
            loaded.export(dest_mesh_path)
            _fix_mtl_name(dest_mesh_path)
        else:
            if isinstance(loaded, trimesh.Scene):
                # EN: STL doesn't support materials, merge geometry.
                # CN: STL 不支持材质，需要合并几何体。
                mesh = trimesh.util.concatenate(tuple(g for g in loaded.geometry.values()))
            else:
                mesh = loaded
            mesh.export(dest_mesh_path)

    # Record canonical path for future dedup
    # 记录 canonical，用于后续去重
    if dedup_hashes is not None:
        dedup_hashes[file_hash] = dest_mesh_path

    return dest_mesh_path, None


# =============================================================================
# Main processor
# 主处理器
# =============================================================================
class UrdfMeshProcessor:
    """
    EN: URDF mesh processor.
    CN: URDF mesh 处理器。

    External usage / 对外用法:
        - processor = UrdfMeshProcessor(...); processor.process(...)
        - mesh_converter(...) (recommended / 推荐)
    """

    def __init__(
        self,
        input_urdf: str,
        mesh_dir: str,
        copy_meshes: bool = False,
        symlink_copy: bool = False,
        use_hardlinks: bool = True,
    ) -> None:
        """
        EN: Initialize processor.
        CN: 初始化处理器。

        Args:
            input_urdf: input URDF path / 输入 URDF 路径
            mesh_dir: output mesh directory / mesh 输出目录
            copy_meshes: copy non-converted meshes / 是否复制未转换 mesh
            symlink_copy: symlink instead of copy / 是否用软链接替代复制
            use_hardlinks: dedup by hardlink / 是否用硬链接做去重
        """
        self.input_urdf = os.path.abspath(input_urdf)
        self.mesh_dir = os.path.abspath(mesh_dir) if mesh_dir else None
        self.copy_meshes = bool(copy_meshes)
        self.symlink_copy = bool(symlink_copy)
        self.use_hardlinks = bool(use_hardlinks)

        if self.mesh_dir:
            os.makedirs(self.mesh_dir, exist_ok=True)

        # Internal states / 内部状态
        self.tree: Optional[ET.ElementTree] = None
        self.root: Optional[ET.Element] = None

        self.filename_cache: Dict[Tuple[str, str], str] = {}
        self.hardlinks_to_create: Dict[str, str] = {}  # link_path -> canonical_path
        self.meshes_to_copy_or_link: Dict[str, str] = {}  # src_path -> dest_path
        self.mesh_infos: List[MeshInfo] = []

        # EN: track converted/copied canonical paths by hash
        # CN: 记录 hash->canonical_path 用于去重
        self.dedup_hashes: Dict[str, str] = {}

    # -------------------------
    # URDF load / parse
    # URDF 加载 / 解析
    # -------------------------
    def _load_urdf(self) -> None:
        """EN: Load URDF. CN: 加载 URDF。"""
        logger.info("Loading URDF: %s", self.input_urdf)
        self.tree = ET.parse(self.input_urdf)
        self.root = self.tree.getroot()

        # EN: Normalize mesh paths immediately after load.
        # CN: 加载后立刻标准化 mesh 路径。
        self._normalize_all_mesh_paths()

    def _normalize_all_mesh_paths(self) -> None:
        """
        EN: Normalize mesh paths to a stable format for downstream processing.
        CN: 将 mesh 路径标准化为更稳定的格式，便于后续处理。
        """
        if self.root is None:
            raise RuntimeError("URDF root is not loaded.")

        logger.info("Normalizing mesh paths in URDF")
        output_urdf_path = self._get_output_urdf_path()

        for tag_name in ["visual", "collision"]:
            for elem in self.root.iter(tag_name):
                geom = elem.find("geometry")
                if geom is None:
                    continue
                mesh_elem = geom.find("mesh")
                if mesh_elem is None:
                    continue

                filename = mesh_elem.get("filename")
                if not filename:
                    continue

                try:
                    abs_path, _, _, _ = _resolve_mesh_path(self.input_urdf, filename)
                    standard_path = _build_standard_mesh_path(output_urdf_path, abs_path)
                    if filename != standard_path:
                        mesh_elem.set("filename", standard_path)
                        logger.debug("Normalized mesh path: '%s' -> '%s'", filename, standard_path)
                except Exception as e:
                    logger.warning("Failed to normalize mesh path '%s': %s", filename, e)

    def _find_meshes(self) -> None:
        """EN: Find visual/collision meshes in URDF. CN: 查找 URDF 中的 visual/collision mesh。"""
        if self.root is None:
            raise RuntimeError("URDF root is not loaded.")

        logger.info("Scanning URDF for mesh elements")

        self.mesh_infos.clear()

        for tag_name, mesh_type in [("visual", "visual"), ("collision", "collision")]:
            for elem in self.root.iter(tag_name):
                geom = elem.find("geometry")
                if geom is None:
                    continue
                mesh_elem = geom.find("mesh")
                if mesh_elem is None:
                    continue

                filename = mesh_elem.get("filename")
                if not filename:
                    continue

                try:
                    abs_path, scheme, pkg, rel = _resolve_mesh_path(self.input_urdf, filename)
                    self.mesh_infos.append(
                        MeshInfo(
                            element=mesh_elem,
                            abs_path=abs_path,
                            scheme=scheme,
                            pkg=pkg,
                            rel_path=rel,
                            mesh_type=mesh_type,
                        )
                    )
                    logger.debug("Found %s mesh: '%s' -> '%s'", mesh_type, filename, abs_path)
                except Exception as e:
                    logger.warning("Failed to resolve mesh path '%s': %s", filename, e)

    # -------------------------
    # Decision helpers
    # 判断与处理逻辑
    # -------------------------
    def _needs_conversion(self, mesh_info: MeshInfo) -> Tuple[bool, str]:
        """
        EN: Decide whether the mesh needs conversion and determine target extension.
        CN: 判断 mesh 是否需要转换并给出目标扩展名。
        """
        ext = os.path.splitext(mesh_info.abs_path)[1].lower()

        if mesh_info.mesh_type == "visual":
            # EN: visual mesh -> OBJ unless already OBJ/STL
            # CN: visual 统一转 OBJ（已是 OBJ/STL 则不转）
            if ext in [".stl", ".obj"]:
                return False, ext[1:]
            return True, "obj"

        # collision
        # EN: collision mesh -> STL unless already STL
        # CN: collision 统一转 STL（已是 STL 则不转）
        if ext == ".stl":
            return False, "stl"
        return True, "stl"

    def _process_meshes(self) -> None:
        """
        EN: Convert/copy/link meshes and update URDF mesh filenames.
        CN: 转换/复制/链接 mesh，并更新 URDF 中 mesh 的 filename。
        """
        if self.root is None:
            raise RuntimeError("URDF root is not loaded.")

        logger.info("Processing meshes (convert/copy/link)")

        for mesh_info in self.mesh_infos:
            needs_conversion, target_ext = self._needs_conversion(mesh_info)
            if needs_conversion:
                self._process_convertible_mesh(mesh_info, target_ext)
            else:
                self._process_existing_mesh(mesh_info, target_ext)

    def _process_convertible_mesh(self, mesh_info: MeshInfo, target_ext: str) -> None:
        """
        EN: Handle a mesh which needs conversion.
        CN: 处理需要转换的 mesh。
        """
        if not self.mesh_dir:
            raise RuntimeError("mesh_dir is required for conversion but is None.")

        original_filename = mesh_info.element.get("filename") or ""
        cache_key = (original_filename, target_ext)

        if cache_key in self.filename_cache:
            mesh_info.element.set("filename", self.filename_cache[cache_key])
            return

        try:
            dest_mesh_path, canonical_path = _convert_mesh(
                source_path=mesh_info.abs_path,
                target_ext=target_ext,
                mesh_dir=self.mesh_dir,
                dedup_hashes=self.dedup_hashes,
            )

            new_filename = _build_standard_mesh_path(self._get_output_urdf_path(), dest_mesh_path)
            self.filename_cache[cache_key] = new_filename
            mesh_info.element.set("filename", new_filename)

            # EN: dedup hit -> schedule hardlink
            # CN: 若命中去重 -> 记录硬链接关系
            if canonical_path is not None and self.use_hardlinks:
                self.hardlinks_to_create[dest_mesh_path] = canonical_path

        except Exception as e:
            logger.error("Failed to convert mesh '%s': %s", original_filename, e, exc_info=True)

    def _process_existing_mesh(self, mesh_info: MeshInfo, target_ext: str) -> None:
        """
        EN: Handle a mesh that does not need conversion.
        CN: 处理无需转换的 mesh（可选复制/软链接/硬链接去重）。
        """
        original_filename = mesh_info.element.get("filename") or ""
        cache_key = (original_filename, target_ext)

        if cache_key in self.filename_cache:
            mesh_info.element.set("filename", self.filename_cache[cache_key])
            return

        # If user requests copying (or symlink), place it into mesh_dir
        # 如果用户要求复制/软链接，则把文件放入 mesh_dir
        if self.copy_meshes and self.mesh_dir:
            unique_name = _generate_unique_mesh_name(mesh_info.abs_path, target_ext)
            dest_path = os.path.join(self.mesh_dir, unique_name)

            file_hash = _compute_file_hash(mesh_info.abs_path)
            if file_hash in self.dedup_hashes:
                canonical = self.dedup_hashes[file_hash]
                if self.use_hardlinks:
                    self.hardlinks_to_create[dest_path] = canonical
                    logger.info("[dedup] source='%s' -> will link to canonical='%s'", mesh_info.abs_path, canonical)
                else:
                    self.meshes_to_copy_or_link[mesh_info.abs_path] = dest_path
            else:
                self.dedup_hashes[file_hash] = dest_path
                self.meshes_to_copy_or_link[mesh_info.abs_path] = dest_path

            new_filename = _build_standard_mesh_path(self._get_output_urdf_path(), dest_path)
            mesh_info.element.set("filename", new_filename)

        # EN: otherwise, path was already normalized in _load_urdf()
        # CN: 否则在 _load_urdf() 阶段已经标准化过路径，无需再改动

        self.filename_cache[cache_key] = mesh_info.element.get("filename") or original_filename

    # -------------------------
    # File ops (link/copy)
    # 文件操作（硬链接/软链接/复制）
    # -------------------------
    def _create_links_and_copies(self) -> None:
        """
        EN: Create hardlinks first; fallback to copy if hardlink fails; then handle copy/symlink tasks.
        CN: 先创建硬链接（失败则复制兜底），再处理复制/软链接任务。
        """
        # Hardlinks
        for link_path, canonical_path in self.hardlinks_to_create.items():
            try:
                os.makedirs(os.path.dirname(link_path), exist_ok=True)
                if os.path.exists(link_path) or os.path.islink(link_path):
                    os.remove(link_path)
                os.link(canonical_path, link_path)
                logger.info("Created hardlink: %s -> %s", link_path, canonical_path)
            except Exception as e:
                logger.warning(
                    "Hardlink failed, fallback to copy: %s -> %s (%s)",
                    link_path,
                    canonical_path,
                    e,
                )
                shutil.copy2(canonical_path, link_path)

        # Copy or symlink for non-converted meshes
        if self.copy_meshes and self.mesh_dir:
            for src_path, dest_path in self.meshes_to_copy_or_link.items():
                try:
                    os.makedirs(os.path.dirname(dest_path), exist_ok=True)

                    if self.symlink_copy:
                        if os.path.exists(dest_path) or os.path.islink(dest_path):
                            os.remove(dest_path)
                        os.symlink(src_path, dest_path)
                        logger.info("Created symlink: %s -> %s", dest_path, src_path)
                    else:
                        shutil.copy2(src_path, dest_path)
                        logger.info("Copied file: %s -> %s", src_path, dest_path)
                except Exception as e:
                    logger.warning("Failed to copy/symlink: %s -> %s (%s)", src_path, dest_path, e)

    # -------------------------
    # Save URDF
    # 保存 URDF
    # -------------------------
    def _get_output_urdf_path(self) -> str:
        """EN: Default output URDF path. CN: 默认输出 URDF 路径。"""
        in_path = pathlib.Path(self.input_urdf)
        return str(in_path.with_name(in_path.stem + "_fixed.urdf"))

    def _save_urdf(self, output_urdf: Optional[str] = None) -> str:
        """
        EN: Save updated URDF to disk.
        CN: 保存修改后的 URDF 到磁盘。
        """
        if self.tree is None or self.root is None:
            raise RuntimeError("URDF is not loaded. Cannot save.")

        if output_urdf is None:
            output_urdf = self._get_output_urdf_path()

        os.makedirs(os.path.dirname(output_urdf), exist_ok=True)

        # EN: Mark generated file with a comment.
        # CN: 在文件头部添加“自动生成”注释。
        comment = ET.Comment(
            "This file was automatically generated by urdf2mjcf. Do not edit manually.\n"
        )
        # Insert only once if user calls multiple times
        # 避免重复插入
        if len(self.root) == 0 or not isinstance(self.root[0].tag, str) or self.root[0].tag != comment.tag:
            self.root.insert(0, comment)

        self.tree.write(output_urdf, encoding="utf-8", xml_declaration=True)
        logger.info("Saved URDF: %s", output_urdf)
        return output_urdf


# =============================================================================
# Public API
# 对外接口
# =============================================================================
def mesh_converter(
    input_urdf: str,
    mesh_dir: str,
    copy_meshes: bool = False,
    symlink_copy: bool = False,
    use_hardlinks: bool = True,
    output_urdf: Optional[str] = None,
) -> str:
    """
    EN: Public API for processing URDF meshes.
    CN: 处理 URDF mesh 的对外接口。

    Returns:
        output URDF path / 输出 URDF 路径
    """
    logger.info("URDF mesh processing started")

    processor = UrdfMeshProcessor(
        input_urdf=input_urdf,
        mesh_dir=mesh_dir,
        copy_meshes=copy_meshes,
        symlink_copy=symlink_copy,
        use_hardlinks=use_hardlinks,
    )

    processor._load_urdf()
    processor._find_meshes()
    processor._process_meshes()
    processor._create_links_and_copies()
    out_path = processor._save_urdf(output_urdf)

    logger.info("URDF mesh processing finished")
    return out_path
