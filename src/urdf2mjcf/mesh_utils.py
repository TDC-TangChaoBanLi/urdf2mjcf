#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Shared mesh / texture utilities.

EN:
- ASCII STL detection and conversion to binary STL (MuJoCo only supports binary STL)
- Landing derived files (copy/symlink) into a target directory with hash de-duplication
- Texture -> PNG conversion (MuJoCo texture support)

CN:
- ASCII STL 检测与转二进制 STL（MuJoCo 仅支持二进制 STL）
- 派生文件落地（复制/软链 + hash 去重）到目标目录
- 贴图转 PNG（MuJoCo 贴图支持）
"""

from __future__ import annotations

import hashlib
import logging
import os
import re
import shutil
import struct
from pathlib import Path
from typing import Dict, Optional, Tuple, Union

logger = logging.getLogger("urdf2mjcf.mesh_utils")

# EN: MuJoCo recognized mesh suffixes (mesh_converter only rewrites these when fixing ASCII STL).
# CN: MuJoCo 可识别的 mesh 后缀。
_STL_SUFFIXES = (".stl",)
_MUJOCO_TEXTURE_SUFFIXES = (
    ".png", ".jpg", ".jpeg", ".bmp", ".tga", ".ktx2",
)
# EN: suffixes that trimesh can load for texture extraction (MTL/map_Kd) etc.
# CN: trimesh 可加载的常见 3D 格式。
_MESH_SUFFIXES = (".stl", ".obj", ".dae", ".gltf", ".glb", ".ply", ".3ds", ".off")


def is_ascii_stl(path: Union[str, Path]) -> bool:
    """
    EN: Detect whether a file is an ASCII STL.
    CN: 判断文件是否为 ASCII STL。

    Rule: read the 84-byte header; if it starts with b"solid" but fails the
    binary STL triangle-count check (nfaces*50 + 84 == file size), treat as ASCII.
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
            return False  # 头恰以 solid 开头的二进制（极罕见）
        return True
    except (OSError, struct.error):
        return False


def convert_ascii_stl_to_binary(
    stl_path: Union[str, Path],
    dest_path: Optional[Union[str, Path]] = None,
) -> Optional[Path]:
    """
    EN: If `stl_path` is an ASCII STL, convert it to binary STL (trimesh) and
        return the output path (defaults to `{stem}_binary.stl` next to the source).
        Returns None when not an ASCII STL or on failure.
    CN: 若 `stl_path` 为 ASCII STL，则用 trimesh 转为二进制 STL 并返回输出路径
        （默认在源旁生成 `{stem}_binary.stl`）；否则或失败时返回 None。
    """
    stl_path = Path(stl_path)
    if not stl_path.is_file():
        return None
    if stl_path.suffix.lower() not in _STL_SUFFIXES:
        return None
    if not is_ascii_stl(stl_path):
        return None

    dest = (
        Path(dest_path)
        if dest_path is not None
        else stl_path.with_name(f"{stl_path.stem}_binary.stl")
    )
    try:
        import trimesh

        loaded = trimesh.load(stl_path, force="mesh", process=False)
        if loaded is None:
            raise ValueError("trimesh returned None")
        loaded.export(dest, file_type="stl")
    except Exception as e:
        logger.error(f"[stl-fix] Failed to convert ASCII STL '{stl_path}': {e}")
        return None
    logger.info(f"[stl-fix] Converted ASCII STL: {stl_path} -> {dest}")
    return dest


def ensure_file_available(
    src_path: Union[str, Path],
    dest_dir: Union[str, Path],
    is_symlink_copy: bool = False,
    force_land: bool = True,
    register=None,
    kind: str = "copy",
) -> Path:
    """
    EN: Make `src_path` available under `dest_dir` by copying (or symlinking),
        returning the path that exists under dest_dir.
    CN: 将 `src_path` 复制（或软链）到 `dest_dir` 下，返回落地后的目标路径。

    :param src_path: EN source file CN 源文件
    :param dest_dir: EN destination directory CN 目标目录
    :param is_symlink_copy: EN use symlink instead of copy CN 是否使用软链接
    :param force_land: EN always land a copy under dest_dir even if src already
        lives in dest_dir? CN: 即使源已在目标目录内也强制落地副本？
    :param register: EN optional ResourceRegistry to record the created file CN 资源登记表
    :param kind: EN registry kind tag CN 登记类型标签
    :return: EN the path usable under dest_dir CN 目标目录下可用的路径

    - Name conflict: when dest_dir already contains a file with same name and
      different content, a hash-suffixed name is used to avoid overwriting.
    - Never deletes or modifies the source file.
    """
    src_path = Path(src_path)
    dest_dir = Path(dest_dir)
    if not src_path.is_file():
        return src_path

    dest_dir.mkdir(parents=True, exist_ok=True)

    # If the source already lives in dest_dir, no need to copy unless asked.
    try:
        if not force_land and src_path.parent.resolve() == dest_dir.resolve():
            return src_path
    except OSError:
        pass

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
            digest = hashlib.md5(src_path.read_bytes()).hexdigest()[:8]
        except OSError:
            digest = "x"
        dest = dest_dir / f"{src_path.stem}_{digest}{src_path.suffix}"

    if is_symlink_copy:
        if not os.path.lexists(dest):
            os.symlink(os.path.abspath(src_path), dest)
            logger.debug(f"[file] symlinked: {dest} <- {os.path.abspath(src_path)}")
    else:
        shutil.copy2(src_path, dest)
        logger.debug(f"[file] copied: {src_path} -> {dest}")

    if register is not None:
        register.mark(dest, kind=kind, origin=src_path)
    return dest


def convert_texture_to_png(
    src_path: Union[str, Path], dest_path: Union[str, Path]
) -> Optional[Path]:
    """
    EN: Convert an image to PNG via Pillow; plain-copy when already PNG.
    CN: 用 Pillow 将图片转换为 PNG 并写入 dest_path；已是 PNG 则直接复制。
    """
    src_path = Path(src_path)
    dest_path = Path(dest_path)
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
        logger.warning(f"[texture] Failed to convert texture '{src_path}' to PNG: {e}")
        return None


def is_mujoco_texture_file(path: Union[str, Path]) -> bool:
    """
    EN: Whether the file suffix is directly supported by MuJoCo texture.
    CN: 是否 MuJoCo 直接支持的贴图后缀。
    """
    return Path(path).suffix.lower() in _MUJOCO_TEXTURE_SUFFIXES


def ensure_texture_ready(
    src_path: Union[str, Path],
    dest_dir: Union[str, Path],
    is_symlink_copy: bool = False,
    register=None,
) -> Optional[Path]:
    """
    EN: Ensure a texture file is available as a MuJoCo-loadable image under
        `dest_dir` (converting to PNG when the format is not supported), and
        return the landed path (or None on failure).
    CN: 确保贴图以 MuJoCo 可加载的图片形式存在于 `dest_dir`（不支持格式转 PNG），
        返回落地后的路径（失败返回 None）。
    """
    src_path = Path(src_path)
    dest_dir = Path(dest_dir)
    if not src_path.is_file():
        logger.warning(f"[texture] Texture file not found: {src_path}")
        return None

    dest_dir.mkdir(parents=True, exist_ok=True)

    if is_mujoco_texture_file(src_path):
        return ensure_file_available(
            src_path, dest_dir, is_symlink_copy=is_symlink_copy,
            register=register, kind="texture",
        )

    # 不支持的格式 -> 转 PNG（在源同目录生成中间 png，再落地）
    png_tmp = src_path.with_suffix(".png")
    converted = convert_texture_to_png(src_path, png_tmp)
    if converted is None:
        return None
    return ensure_file_available(
        converted, dest_dir, is_symlink_copy=is_symlink_copy,
        register=register, kind="png",
    )


# ======================================================================
# MTL 解析（OBJ 伴随材质文件）
# ======================================================================

def parse_mtl_name_from_obj(obj_file: Union[str, Path]) -> Optional[str]:
    """EN: Find the `mtllib` file name referenced by an OBJ. CN: 读取 OBJ 引用的 mtllib 文件名。"""
    pattern = re.compile(r"^mtllib\s+(.+?\.mtl)(?:\s*#.*)?\s*$", re.IGNORECASE)
    try:
        with Path(obj_file).open("r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                m = pattern.match(line.strip())
                if m:
                    return m.group(1)
    except OSError:
        pass
    return None


def parse_mtl_rgba_map(mtl_file: Union[str, Path]) -> Dict[str, str]:
    """
    EN: Parse an MTL file into {material_name: "r g b a"} (Kd/d/tr only, ignores map_*).
    CN: 解析 MTL 文件为 {材质名: "r g b a"}（只解析 Kd/d/tr，忽略 map_*）。
    """
    result: Dict[str, str] = {}
    mtl_file = Path(mtl_file)
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


def parse_mtl_map_kd(mtl_file: Union[str, Path]) -> Dict[str, Optional[Path]]:
    """
    EN: Parse MTL `map_Kd` statements -> {material_name: texture absolute path | None}.
    CN: 解析 MTL map_Kd 语句 -> {材质名: 贴图绝对路径 | None}。
    """
    result: Dict[str, Optional[Path]] = {}
    mtl_file = Path(mtl_file)
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


def get_mtl_info(obj_file: Union[str, Path]) -> Tuple[Path, Dict[str, str], Dict[str, Optional[Path]]]:
    """
    EN: Resolve the MTL referenced by an OBJ and return its rgba / map_Kd maps.
        On missing MTL returns the sibling MTL path with empty maps.
    CN: 解析 OBJ 引用到的 MTL，返回 (mtl_path, {材质: rgba}, {材质: 贴图路径})。
    """
    obj_file = Path(obj_file)
    mtl_name = parse_mtl_name_from_obj(obj_file)
    if not mtl_name:
        return obj_file.with_suffix(".mtl"), {}, {}
    mtl_file = (obj_file.parent / mtl_name).resolve()
    if not mtl_file.is_file():
        return mtl_file, {}, {}
    return mtl_file, parse_mtl_rgba_map(mtl_file), parse_mtl_map_kd(mtl_file)
