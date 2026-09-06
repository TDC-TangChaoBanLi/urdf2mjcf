#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ResourceRegistry - 记录本流程创建的资源文件，供写 MJCF 后统一清理。

EN:
- The conversion pipeline (mesh_converter / mesh_decomposer / mesh_coacd) copies,
  symlinks, converts or decomposes mesh/texture files. Every file created by the
  pipeline is registered here together with its kind and origin.
- After the MJCF is written, `finalize(used_paths)` removes registered files that
  are NOT referenced by the final MJCF. Only files created by the pipeline
  (i.e. registered) are ever deleted - source files in the ROS package or any
  other external file are never touched.

CN:
- 转换管线（mesh_converter / mesh_decomposer / mesh_coacd）会复制、软链接、转换
  或分解 mesh/texture 文件。管线创建的每个文件都会在此登记（含类型与来源）。
- MJCF 写出后 `finalize(used_paths)` 会删除**未被最终 MJCF 引用**的已登记文件。
  只有管线自己登记过的文件才会被删除——ROS 包内的源文件或其他外部文件绝不会被触碰。
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Union

logger = logging.getLogger("urdf2mjcf.resource_registry")


@dataclass
class CreatedFile:
    """A file created by the pipeline."""

    # EN: How the file was created. CN: 文件创建方式。
    kind: str  # "copy" | "symlink" | "converted" | "split" | "coacd" | "png" | "texture"
    # EN: Original source file this file was derived from (may be missing). CN: 源文件。
    origin: Optional[Path] = None
    # EN: Extra info (e.g. owning module). CN: 额外信息（如所属模块）。
    info: str = ""


class ResourceRegistry:
    """
    EN: Registry of pipeline-created resource files.
    CN: 管线创建的资源文件登记表。
    """

    def __init__(self) -> None:
        self._files: Dict[Path, CreatedFile] = {}

    # ------------------------------------------------------------------
    # Registration
    # ------------------------------------------------------------------

    def mark(
        self,
        path: Union[Path, str],
        kind: str,
        origin: Optional[Union[Path, str]] = None,
        info: str = "",
    ) -> Path:
        """
        EN: Register a file created by the pipeline. CN: 登记一个由管线创建的文件。
        """
        p = Path(path)
        origin_path = Path(origin) if origin is not None else None
        if p not in self._files:
            self._files[p] = CreatedFile(kind=kind, origin=origin_path, info=info)
            logger.debug(f"[registry] marked {kind}: {p} (origin: {origin_path})")
        return p

    def mark_copy(
        self, path: Union[Path, str], origin: Optional[Union[Path, str]] = None
    ) -> Path:
        return self.mark(path, kind="copy", origin=origin, info="copy")

    def mark_symlink(
        self, path: Union[Path, str], origin: Optional[Union[Path, str]] = None
    ) -> Path:
        return self.mark(path, kind="symlink", origin=origin, info="symlink")

    def mark_converted(
        self, path: Union[Path, str], origin: Optional[Union[Path, str]] = None
    ) -> Path:
        return self.mark(path, kind="converted", origin=origin, info="converted")

    def mark_split(
        self, path: Union[Path, str], origin: Optional[Union[Path, str]] = None
    ) -> Path:
        return self.mark(path, kind="split", origin=origin, info="split")

    def mark_coacd(
        self, path: Union[Path, str], origin: Optional[Union[Path, str]] = None
    ) -> Path:
        return self.mark(path, kind="coacd", origin=origin, info="coacd")

    # ------------------------------------------------------------------
    # Query
    # ------------------------------------------------------------------

    def is_created(self, path: Union[Path, str]) -> bool:
        """
        EN: Whether the given file was created (and registered) by the pipeline.
        CN: 指定文件是否由管线创建（已登记）。
        """
        return Path(path) in self._files

    def info(self, path: Union[Path, str]) -> Optional[CreatedFile]:
        return self._files.get(Path(path))

    @property
    def files(self) -> Dict[Path, CreatedFile]:
        return dict(self._files)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def finalize(self, used_paths) -> int:
        """
        EN: Remove all registered files that are NOT in `used_paths`.
        CN: 删除所有不在 `used_paths` 中的已登记文件。

        :param used_paths: EN Iterable of files referenced by the final MJCF
                           (absolute paths preferred).
                           CN: 最终 MJCF 引用到的文件集合（建议绝对路径）。
        :return: EN number of removed files. CN: 删除的文件数量。
        """
        used = {Path(p).resolve() for p in used_paths}
        removed = 0
        for path in sorted(self._files):
            if path.resolve() in used:
                continue
            if self._remove_one(path):
                removed += 1
        self._files.clear()
        logger.info(f"[registry] finalized: removed {removed} unused intermediate files.")
        return removed

    def _remove_one(self, path: Path) -> bool:
        """EN: Remove a single registered file. CN: 删除单个已登记文件。"""
        if not os.path.lexists(path):
            return False
        try:
            if os.path.islink(path):
                os.unlink(path)
                logger.debug(f"[registry] removed symlink: {path}")
            elif path.is_file():
                path.unlink()
                logger.debug(f"[registry] removed file: {path}")
            else:
                return False
            return True
        except OSError as e:
            logger.warning(f"[registry] failed to remove {path}: {e}")
            return False


# EN: Convenience alias for type hints. CN: 便捷类型别名。
CreatedPath = Path
