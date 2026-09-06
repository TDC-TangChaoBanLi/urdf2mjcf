#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convex decomposition (CoACD) module - model level / pre-MJCF.

EN:
- CoacdConfig: CoACD parameters loaded from JSON.
- decompose_mesh_file(...): run CoACD on a single mesh file, output convex STL parts.
- decompose_model_meshes(...): decompose meshes referenced by the URDF model
  (visual or collision) and replace the original geometry in-place with the
  convex parts; register created files in the ResourceRegistry.

CN:
- CoacdConfig: 从 JSON 加载的 CoACD 参数。
- decompose_mesh_file(...): 对单个 mesh 文件执行 CoACD 分解，输出凸包 STL。
- decompose_model_meshes(...): 对 URDF model 引用的 mesh（visual/collision）执行
  凸包分解，用凸包部件就地替换原 geometry，并把创建的文件登记到 ResourceRegistry。
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Union

logger = logging.getLogger("urdf2mjcf.mesh_coacd")


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


def decompose_mesh_file(
    mesh_file: Union[str, Path],
    dest_dir: Optional[Union[str, Path]] = None,
    config: Optional[CoacdConfig] = None,
    registry=None,
) -> List[Path]:
    """
    对单个 mesh 文件执行 CoACD 凸包分解，返回生成的 convex STL 文件列表。

    :param mesh_file: EN source mesh CN 源 mesh 文件
    :param dest_dir: EN output directory (default: mesh_file.parent) CN 输出目录
    :param config: EN CoACD config CN CoACD 配置
    :param registry: EN ResourceRegistry (optional) CN 资源登记表（可选）
    :return: EN list of generated convex files CN 生成的凸包文件列表

    命名规则："{原文件名不带后缀}_convex_[num].stl"
    """
    import trimesh
    try:
        import coacd
    except ImportError:
        logger.error(
            "Convex decomposition requires coacd, please install: pip install coacd"
        )
        return []

    cfg = config or CoacdConfig()
    mesh_file = Path(mesh_file)
    if not mesh_file.exists():
        logger.warning(f"[convex] Mesh file not found: {mesh_file}")
        return []

    logger.info(f"[convex] Starting convex decomposition: {mesh_file}")
    try:
        mesh = trimesh.load(mesh_file, force="mesh", process=False)
        if mesh is None:
            raise ValueError("trimesh returned None")
    except Exception as e:
        logger.warning(f"[convex] Failed to load mesh: {mesh_file}: {e}")
        return []

    m = coacd.Mesh(mesh.vertices, mesh.faces)
    try:
        coacd.set_log_level(cfg.log_level)
    except Exception:
        pass  # 老版本 coacd 可能没有 set_log_level

    try:
        parts = coacd.run_coacd(
            mesh=m,
            threshold=cfg.threshold,
            max_convex_hull=cfg.max_convex_hull,
            preprocess_mode=cfg.preprocess_mode,
            preprocess_resolution=cfg.preprocess_resolution,
            resolution=cfg.resolution,
            mcts_nodes=cfg.mcts_nodes,
            mcts_iterations=cfg.mcts_iterations,
            mcts_max_depth=cfg.mcts_max_depth,
            pca=cfg.pca,
            merge=cfg.merge,
            decimate=cfg.decimate,
            max_ch_vertex=cfg.max_ch_vertex,
            extrude=cfg.extrude,
            extrude_margin=cfg.extrude_margin,
            apx_mode=cfg.apx_mode,
            seed=cfg.seed,
        )
    except Exception as e:
        logger.error(f"[convex] CoACD run failed: {e}")
        return []

    out_dir = Path(dest_dir) if dest_dir is not None else mesh_file.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    out_paths: List[Path] = []
    for idx, (vs, fs) in enumerate(parts):
        part_mesh = trimesh.Trimesh(vs, fs)
        out_path = out_dir / f"{mesh_file.stem}_convex_{idx}.stl"
        part_mesh.export(out_path.as_posix())
        out_paths.append(out_path)
        if registry is not None:
            registry.mark_coacd(out_path, origin=mesh_file)
        logger.info(f"[convex]     generated convex file: {out_path}")

    return out_paths


def decompose_model_meshes(
    urdf_model,
    target: str,
    dest_dir: Union[str, Path],
    registry=None,
    config: Optional[CoacdConfig] = None,
) -> int:
    """
    对 URDF model 中 target（"visual"/"collision"）类的 mesh 执行凸包分解，
    用 convex 部件就地替换 model 中的 geometry，并更新引用。

    - 每个被引用的 mesh 只分解一次（按解析后的绝对路径去重）
    - 若源 mesh 不在 dest_dir 内，先在 dest_dir 落地可写副本再分解
      （避免向 ROS package 等源目录写入）
    - 返回成功替换的 mesh 引用组数
    """
    from . import mesh_utils
    from .urdf_parser import UrdfParser

    if target not in ("visual", "collision"):
        logger.warning(f"[convex] Invalid target: {target}")
        return 0

    dest_dir = Path(dest_dir)
    dest_dir.mkdir(parents=True, exist_ok=True)
    cfg = config or CoacdConfig()

    # 源 mesh（绝对路径）-> convex 文件列表（缓存，避免重复分解同一 mesh）
    cache: dict = {}
    success = 0

    def get_convex_files(mesh_path: Path) -> List[Path]:
        key = str(mesh_path.resolve())
        if key in cache:
            return cache[key]

        # 若源不在 dest_dir 中，先落地一份可写副本（避免写源目录）
        work_file = mesh_path
        try:
            same_dir = mesh_path.parent.resolve() == dest_dir.resolve()
        except OSError:
            same_dir = False
        if not same_dir:
            work_file = mesh_utils.ensure_file_available(
                mesh_path,
                dest_dir,
                is_symlink_copy=False,
                register=registry,
                kind="copy",
                force_land=True,
            )

        convex_files = decompose_mesh_file(
            work_file, dest_dir=dest_dir, config=cfg, registry=registry
        )
        cache[key] = convex_files
        return convex_files

    def process_vc(v_or_c) -> None:
        nonlocal success
        geom = v_or_c.geometry
        if geom.g_type != "mesh":
            return
        raw = geom.filename
        if not raw:
            return

        mesh_path = Path(raw)
        if not mesh_path.is_absolute():
            cand = mesh_path.resolve()
            if cand.is_file():
                mesh_path = cand
            else:
                logger.warning(f"[convex] Mesh file not found: {raw}")
                return
        if not mesh_path.is_file():
            logger.warning(f"[convex] Mesh file not found: {mesh_path}")
            return

        convex_files = get_convex_files(mesh_path)
        if not convex_files:
            return

        # 找到 v_or_c 所在列表并替换
        owner_list = None
        for link in urdf_model.links:
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
        origin_attr = v_or_c.origin
        material_attr = v_or_c.material
        src_scale = v_or_c.geometry.scale

        new_items = []
        for part_path in convex_files:
            new_geom = UrdfParser.UrdfGeometry(
                g_type="mesh", filename=str(Path(part_path).resolve()), scale=src_scale
            )
            if is_visual:
                new_items.append(UrdfParser.UrdfVisual(origin_attr, new_geom, material_attr))
            else:
                new_items.append(UrdfParser.UrdfCollision(origin_attr, new_geom, material_attr))
        owner_list[idx : idx + 1] = new_items
        success += 1

    for link in urdf_model.links:
        if target == "visual" and link.visuals:
            for v in list(link.visuals):
                process_vc(v)
        elif target == "collision" and link.collisions:
            for c in list(link.collisions):
                process_vc(c)

    logger.info(f"[convex] Decomposed {success} mesh reference(s) into convex parts.")
    return success


def mesh_coacd(
    urdf_model,
    meshes_dir: Union[str, Path],
    decompose_target: Optional[str] = None,
    config_path: Optional[Union[str, Path]] = None,
    registry=None,
) -> int:
    """
    EN: Public entry - convex-decompose meshes referenced by the model.
    CN: 公开入口——对 model 引用的 mesh 执行凸包分解。

    :return: EN number of mesh references decomposed CN 成功分解的 mesh 引用组数
    """
    if decompose_target not in ("visual", "collision"):
        return 0
    cfg = CoacdConfig.load_from_json(config_path)
    return decompose_model_meshes(
        urdf_model=urdf_model,
        target=decompose_target,
        dest_dir=meshes_dir,
        registry=registry,
        config=cfg,
    )
