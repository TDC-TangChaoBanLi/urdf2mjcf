#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
urdf2mjcf - Unified URDF to MJCF conversion tool

This tool converts URDF models to MuJoCo-compatible MJCF format and handles related mesh files.

Workflow:
1) Convert meshes referenced in the URDF (e.g., DAE -> OBJ) and store them in -m/--meshes-dir
2) Convert the URDF to MJCF and write to -o/--output
3) Optional: post-process meshes (e.g., material split / convex decomposition) via -d/--decompose

Examples:
    urdf2mjcf robot.urdf -o robot.xml -m ./meshes
    urdf2mjcf robot.urdf -o robot.xml -m ./meshes -c
    urdf2mjcf robot.urdf -o robot.xml -m ./meshes -c -s -d visual
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Optional

from .mesh_converter import mesh_converter
from .mjcf_generator import mjcf_generator
from .mesh_decomposer import mesh_decomposer

# Logger for this module
# 本模块日志记录器
logger = logging.getLogger("urdf2mjcf.cli")


# ----------------------------
# Logging helpers
# 日志配置辅助函数
# ----------------------------
def configure_logging(level: str = "INFO", log_file: Optional[str] = None) -> None:
    """
    Configure global logging style for the whole package.

    配置整个包的全局日志风格（由 CLI 统一设置，子模块仅 getLogger 不自行 basicConfig）。
    """
    numeric_level = getattr(logging, level.upper(), None)
    if not isinstance(numeric_level, int):
        # Fallback to INFO if user passes an invalid level
        # 若用户输入非法等级，则回退到 INFO
        numeric_level = logging.INFO

    # Unified log format (stable, grep-friendly)
    # 统一日志格式（稳定、便于检索）
    fmt = "%(asctime)s | %(name)s | %(levelname)s | %(message)s"
    datefmt = "%Y-%m-%d %H:%M:%S"

    handlers: list[logging.Handler] = []
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(logging.Formatter(fmt=fmt, datefmt=datefmt))
    handlers.append(stream_handler)

    if log_file:
        file_handler = logging.FileHandler(log_file, encoding="utf-8")
        file_handler.setFormatter(logging.Formatter(fmt=fmt, datefmt=datefmt))
        handlers.append(file_handler)

    # NOTE: force=True ensures reconfiguration even if other libs touched logging.
    # 注意：force=True 可确保即便其他库提前配置过 logging，这里也能强制统一。
    logging.basicConfig(level=numeric_level, handlers=handlers, force=True)


def log_step_header(step_idx: int, total_steps: int, title: str) -> None:
    """
    Print a unified step header.

    打印统一的步骤分隔标题。
    """
    bar = "=" * 72
    logger.info(bar)
    logger.info("[STEP %d/%d] %s", step_idx, total_steps, title)
    logger.info(bar)


# ----------------------------
# CLI entry
# 命令行入口
# ----------------------------
def build_parser() -> argparse.ArgumentParser:
    """
    Build CLI argument parser.

    构建命令行参数解析器。
    """
    parser = argparse.ArgumentParser(
        description="urdf2mjcf - URDF to MJCF conversion tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument("input", help="Input URDF file path")

    parser.add_argument(
        "-o", "--output",
        required=True,
        help="Output MJCF file path",
    )

    parser.add_argument(
        "-m", "--meshes-dir",
        required=True,
        help="Directory to store converted mesh files",
    )

    parser.add_argument(
        "-c", "--copy-meshes",
        action="store_true",
        help="Copy mesh files to meshes directory",
    )

    parser.add_argument(
        "-s", "--symlink-copy",
        action="store_true",
        help="Create symlinks instead of copying (requires -c/--copy-meshes)",
    )

    parser.add_argument(
        "-d", "--decompose",
        choices=["visual", "collision"],
        help="Perform convex decomposition on meshes from 'visual' or 'collision', and use results as collision meshes",
    )

    parser.add_argument(
        "-j", "--json-config",
        help="JSON configuration file path",
    )

    # Logging options
    # 日志选项
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="Logging level: DEBUG, INFO, WARNING, ERROR (default: INFO)",
    )
    parser.add_argument(
        "--log-file",
        default=None,
        help="Optional log file path (write logs to file in addition to stdout)",
    )

    return parser


def validate_inputs(input_path: Path) -> None:
    """
    Validate input URDF path.

    校验输入 URDF 路径。
    """
    if not input_path.exists():
        raise FileNotFoundError(f"Input file does not exist: {input_path}")

    if (not input_path.is_file()) or (input_path.suffix.lower() != ".urdf"):
        raise ValueError(f"Input file must be a .urdf file: {input_path}")


def ensure_dirs(meshes_dir: Path, output_path: Path) -> None:
    """
    Ensure output directories exist.

    确保输出目录存在。
    """
    meshes_dir.mkdir(parents=True, exist_ok=True)
    output_path.parent.mkdir(parents=True, exist_ok=True)


def main() -> None:
    """
    Main function - parse command line arguments and execute the conversion pipeline.

    主函数：解析命令行参数并执行完整转换流程。
    """
    parser = build_parser()
    args = parser.parse_args()

    # Configure logging first (before doing any real work)
    # 先配置日志（在做任何实际工作之前）
    configure_logging(level=args.log_level, log_file=args.log_file)

    input_path = Path(args.input)
    meshes_dir = Path(args.meshes_dir)
    output_path = Path(args.output)

    # Input validation
    # 输入校验
    try:
        validate_inputs(input_path)
    except Exception as e:
        logger.error("Input validation failed: %s", e, exc_info=True)
        sys.exit(1)

    # Sanity checks for copy/symlink options
    # 对 copy/symlink 选项做一致性检查
    if args.symlink_copy and (not args.copy_meshes):
        logger.warning("Option --symlink-copy requires --copy-meshes. Falling back to '--copy-meshes' behavior OFF.")
        # Keep behavior explicit: disable symlink if copy is not enabled
        # 行为显式：若未开启 copy，则禁用 symlink
        args.symlink_copy = False

    # Ensure dirs exist
    # 确保目录存在
    try:
        ensure_dirs(meshes_dir, output_path)
    except Exception as e:
        logger.error("Failed to create output directories: %s", e, exc_info=True)
        sys.exit(1)

    total_steps = 3

    # Step 1: convert meshes referenced in the URDF
    log_step_header(1, total_steps, "Convert meshes referenced in URDF")
    try:
        fixed_urdf_path = mesh_converter(
            input_urdf=str(input_path),
            mesh_dir=str(meshes_dir),
            copy_meshes=bool(args.copy_meshes),
            symlink_copy=bool(args.symlink_copy),
        )
        logger.info("Mesh conversion finished successfully.")
        logger.debug("Fixed URDF path: %s", fixed_urdf_path)
    except Exception as e:
        logger.error("Mesh conversion failed: %s", e, exc_info=True)
        sys.exit(1)

    # Step 2: convert URDF to MJCF
    log_step_header(2, total_steps, "Generate MJCF from URDF")
    try:
        mjcf_generator(
            urdf_path=fixed_urdf_path,
            mjcf_path=str(output_path),
            json_config_path=args.json_config,
        )
        logger.info("MJCF generation finished successfully.")
    except Exception as e:
        logger.error("MJCF generation failed: %s", e, exc_info=True)
        sys.exit(1)

    # Step 3: post-process meshes (optional)
    log_step_header(3, total_steps, "Post-process meshes (material split / convex decomposition)")
    try:
        mesh_decomposer(
            xml_path=output_path,
            decompose_target=args.decompose if args.decompose else None,
            config_path=args.json_config,
        )
        logger.info("Mesh post-processing finished successfully.")
    except Exception as e:
        # Keep pipeline tolerant: post-process failures are non-fatal by default
        # 保持流程容错：后处理失败默认不致命
        logger.warning("Mesh post-processing failed (non-fatal): %s", e, exc_info=True)

    # Summary
    # 汇总信息
    bar = "=" * 72
    logger.info(bar)
    logger.info("Pipeline completed.")
    logger.info("Input URDF : %s", input_path)
    logger.info("Output MJCF: %s", output_path)
    logger.info("Meshes dir : %s", meshes_dir)
    if args.json_config:
        logger.info("JSON config: %s", args.json_config)
    logger.info(bar)


if __name__ == "__main__":
    main()
