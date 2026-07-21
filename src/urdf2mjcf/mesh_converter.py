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
from typing import Dict, List, Optional, Tuple, Union

import trimesh



from .urdf_parser import UrdfParser
import json
from pathlib import Path

# IMPORTANT:
# EN: Do NOT call logging.basicConfig() here. CLI configures logging globally.
# CN: 不要在此模块内配置 logging.basicConfig()，由 CLI 统一配置日志风格。
logger = logging.getLogger("urdf2mjcf.mesh_converter")

def _str2vec(x: str) -> tuple:
    """
    EN: Normalize string to numeric triple.
    CN: 将字符串规范化为数值元组格式。
    """
    if x is None:
        return (0.0, 0.0, 0.0)
    vals = x.split()
    return tuple(float(v) for v in vals)


def _vec2str(vec: Optional[list|tuple]) -> Optional[str]:
    """
    EN: Convert numeric list or tuple to space-separated string.
    CN: 将数值列表或元组转换为空格分隔的字符串。
    
    :param vec: A list or tuple containing numeric values
    :return: Space-separated string representation
    """
    if vec is None:
        return None
    return " ".join(str(float(v)) for v in vec)



class MeshConverter:
    def __init__(self, 
                output_path: Union[str, Path],
                urdf_model: UrdfParser, 
                json_path: Optional[Union[str, Path]], 
                meshes_dir: Optional[Union[str, Path]],
                is_copy_meshes: bool = True,
                is_symlink_copy: bool = False
        ):

        self.output_path: Path = Path(output_path)
        self.urdf_model: UrdfParser = urdf_model
        self.json_cfg: dict = {}
        self.package_map: Dict[str,str] = {}
        self.is_copy_meshes: bool = is_copy_meshes
        self.meshes_dir: Path = Path(meshes_dir) if meshes_dir is not None else (self.output_path.parent / "meshes")
        if not self.meshes_dir.exists():
            os.mkdir(self.meshes_dir)
        self.is_symlink_copy: bool = is_symlink_copy

        # load json config
        if json_path is None:
            logger.warning("No json config file provided.")
            self.json_cfg = {}
            return
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




    @staticmethod
    def _find_package_path(pkg_name: str, pkg_map: Dict[str,str]) -> str:
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


    @staticmethod
    def _parse_mtl_to_rgba(mtl_path: Union[Path, str]) -> Optional[Tuple[float, float, float, float]]:
        """
        将 mtl 文件信息解析为 rgba 信息


        """
        mtl_path = Path(mtl_path)
        # rgba = (1,1,1,1)
        if not mtl_path.is_file():
            return None
        current_name: Optional[str] = None
        kd = None
        alpha = 1.0

        with mtl_path.open("r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith("#"):
                    continue
                lower = s.lower()

                if lower.startswith("newmtl"):
                    if current_name and kd:
                        r, g, b = kd
                        mtl_path[current_name] = f"{r} {g} {b} {alpha}"
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
            rgba = tuple((r,g,b,alpha))
            return rgba
        else:
            return None

    def convert_meshes(self) -> UrdfParser:

        def convert_visual_collision(vs_or_cs: Union[List[UrdfParser.UrdfVisual], List[UrdfParser.UrdfCollision]], exclude_meshes: List[Path]) -> Optional[str]:
            """
            转换一组 visual 或 collision

            :param v_or_c: visual 或 collision
            :return: 转换后的模型文件路径
            """
            for v_or_c in vs_or_cs:
                if v_or_c.geometry.g_type != 'mesh':
                    continue
                if Path(v_or_c.geometry.filename) in exclude_meshes:
                    continue

                source_mesh_path = MeshConverter._resolve_mesh_path(v_or_c.geometry.filename, self.package_map) # resolve path to absolute path
                if (source_mesh_path is None) or (not Path(source_mesh_path).is_file()):
                    logger.warning(f"Mesh file '{source_mesh_path}' in link '{link.l_name}' does not exist. Ingnoring...")
                    continue
                mesh_hash = MeshConverter._compute_mesh_hash(source_mesh_path) # compute hash

                # 如果源文件不是 stl 或者 obj ，则直接转换为 obj 到指定 mesh 目录下：
                if not (Path(source_mesh_path).suffix in ['.stl', '.obj', '.STL', '.OBJ']):
                    dest_mesh_name = Path(source_mesh_path).stem + "_" + mesh_hash # dest_mesh_name: SrcMeshName_[hash8].obj
                    dest_mesh_format = "obj" if isinstance(v_or_c, UrdfParser.UrdfVisual) else "stl" # 目标格式： visual: obj | collision: stl
                    added_meshes_once = MeshConverter._convert_mesh(source_mesh_path, self.meshes_dir, dest_mesh_name, dest_mesh_format) # 转换 mesh 格式
                    added_materials_once: List[Path] = []

                    exclude_meshes.extend(added_meshes_once)
                    is_visual = True if isinstance(v_or_c, UrdfParser.UrdfVisual) else False
                    src_origin = v_or_c.origin # 原本的 origin
                    src_material = v_or_c.material # 原本的 material
                    vs_or_cs.remove(v_or_c) # 移除原本的 visual / collision
                    # 添加所有 visual / collision
                    for added_mesh in added_meshes_once:
                        # 创建 geometry
                        added_geometry = UrdfParser.UrdfGeometry(g_type="mesh", filename=str(added_mesh))
                        if is_visual:# 添加 visaul
                            added_material = src_material
                            if src_material is None: # 如果原本没有 material 属性， 添加来自 mtl 文件的 rgba
                                added_mtl = str(added_mesh.parent / added_mesh.stem) + ".mtl"
                                rgba = MeshConverter._parse_mtl_to_rgba(added_mtl)
                                if rgba is not None:
                                    added_material = UrdfParser.UrdfMaterial(m_name=None, color_rgba=_vec2str(rgba))
                            vs_or_cs.append(UrdfParser.UrdfVisual(src_origin, added_geometry, added_material))
                        else: # 添加 collision
                            vs_or_cs.append(UrdfParser.UrdfCollision(src_origin, added_geometry, src_material))
                    continue

                # 如果需要复制原 mesh 文件：
                elif self.is_copy_meshes:
                    dest_mesh_name = Path(source_mesh_path).stem + "_" + mesh_hash + Path(source_mesh_path).suffix
                    dest_mesh_path = self.meshes_dir.joinpath(dest_mesh_name)
                    if self.is_symlink_copy:
                        if not os.path.exists(dest_mesh_path):
                            os.symlink(source_mesh_path, dest_mesh_path) # 创建软链接
                            logger.debug(f"Created mesh symlink: {dest_mesh_path} -> {source_mesh_path}")
                    else:
                        if not os.path.exists(dest_mesh_path):
                            shutil.copy2(source_mesh_path, dest_mesh_path) # 复制文件
                            logger.debug(f"Copied mesh file: {source_mesh_path} -> {dest_mesh_path}")
                    
                    v_or_c.geometry.filename = str(dest_mesh_path)
                    continue

                # 如果不是复制，那么直接使用源 mesh 的路径：
                else: 
                    if Path(source_mesh_path).is_relative_to(Path.cwd()) and Path(source_mesh_path).is_absolute():
                        dest_mesh_path = Path(source_mesh_path).relative_to(Path.cwd())
                    else:
                        dest_mesh_path = Path(source_mesh_path)
                    
                    v_or_c.geometry.filename = str(dest_mesh_path)
                    continue

        # Iterate through links
        added_meshes = []
        for link in self.urdf_model.links:
            
            # Iterate through all the visuals
            if link.visuals is not None:
                convert_visual_collision(link.visuals, added_meshes)
            
            # Iterate through all the collisions
            if link.collisions is not None:
                convert_visual_collision(link.collisions, added_meshes)
        
        return self.urdf_model


    def convert_urdf_element(self) -> UrdfParser:

        def add_new_visuals_or_collisions(link_elem: ET.Element[str], vs_or_cs: Union[List[UrdfParser.UrdfVisual], List[UrdfParser.UrdfCollision]]) -> None:
            """
            根据 'List[UrdfParser.UrdfVisual]' 或 'List[UrdfParser.UrdfCollision]' 为 urdf 文件中的 'link' 创建新的 'visual' 或 'collision' 元素

            :param link_elem: link 元素
            :param vs_or_cs: UrdfParser.UrdfVisual 或 UrdfParser.UrdfCollision 列表
            :return: None
            """
            for model_v_or_c in vs_or_cs:

                # 添加 visual / collision
                tag = "visual" if isinstance(model_v_or_c, UrdfParser.UrdfVisual) else "collision"
                new_vc_elem = ET.SubElement(link_elem, tag)
                
                # 添加 origin
                ET.SubElement(new_vc_elem, "origin", attrib={"xyz": model_v_or_c.origin.xyz, "rpy": model_v_or_c.origin.rpy})

                # 添加 material
                if model_v_or_c.material is not None:
                    new_vc_material_elem_attrib = {}
                    if model_v_or_c.material.m_name is not None:
                        new_vc_material_elem_attrib["name"] = model_v_or_c.material.m_name
                    new_vc_material_elem = ET.SubElement(new_vc_elem, "material", attrib=new_vc_material_elem_attrib)
                    if model_v_or_c.material.color_rgba is not None:
                        ET.SubElement(new_vc_material_elem, "color", attrib={"rgba": model_v_or_c.material.color_rgba})

                # 添加 geometry：
                new_geom_elem = ET.SubElement(new_vc_elem, "geometry")
                if model_v_or_c.geometry.g_type == "mesh":  # 如果是 mesh
                    urdf_dir = Path(self.output_path).parent # urdf 文件所在目录
                    mesh_path = Path(model_v_or_c.geometry.filename) # mesh 文件路径
                    if mesh_path.resolve().is_relative_to(Path.cwd().resolve()): # 如果 mesh_path 在当前目录内 使用相对路径
                        model_v_or_c.geometry.filename = os.path.relpath(str(mesh_path),str(urdf_dir)) 
                    elif mesh_path.resolve().is_relative_to(urdf_dir.parent.resolve()): # 如果 mesh_path 在 urdf_dir 的父目录内 使用相对路径
                        model_v_or_c.geometry.filename = os.path.relpath(str(mesh_path),str(urdf_dir)) 
                    else:
                        model_v_or_c.geometry.filename = str(mesh_path.resolve()) # 使用绝对路径
                    ET.SubElement(new_geom_elem, "mesh", attrib={"filename": model_v_or_c.geometry.filename})
                elif model_v_or_c.geometry.g_type == "box": # 如果是 box
                    ET.SubElement(new_geom_elem, "box", attrib={"size": model_v_or_c.geometry.size})
                elif model_v_or_c.geometry.g_type == "cylinder": # 如果是 cylinder
                    ET.SubElement(new_geom_elem, "cylinder", attrib={"radius": model_v_or_c.geometry.radius, "length": model_v_or_c.geometry.length})
                elif model_v_or_c.geometry.g_type == "sphere": # 如果是 sphere
                    ET.SubElement(new_geom_elem, "sphere", attrib={"radius": model_v_or_c.geometry.radius})


        for link_elem in self.urdf_model.robot_elem.iter("link"):
            link_name = link_elem.attrib["name"]
            # 删除所有 visual 和 collision 子元素
            for visual_elem in link_elem.findall("visual"):
                link_elem.remove(visual_elem)
            for collision_elem in link_elem.findall("collision"):
                link_elem.remove(collision_elem)

            # 获取当前 link 的 model 信息: UrdfParser.UrdfLink
            model_link = list(filter(lambda link: link.l_name == link_name, self.urdf_model.links))
            if len(model_link) > 0:
                model_link = model_link[0]
            else:
                continue
            
            # 添加所有 visual 
            if len(model_link.visuals) > 0:
                add_new_visuals_or_collisions(link_elem, model_link.visuals)

            # 添加所有 collision
            if len(model_link.collisions) > 0:
                add_new_visuals_or_collisions(link_elem, model_link.collisions)

        return self.urdf_model

def mesh_converter(urdf_model: UrdfParser, output_path: Path, meshes_dir: Path, is_copy_meshes: bool = True, is_symlink_copy: bool = False):
    mesh_converter = MeshConverter(
        output_path=output_path,
        urdf_model=urdf_model, 
        json_path=None, 
        meshes_dir=meshes_dir, 
        is_copy_meshes = is_copy_meshes, 
        is_symlink_copy= is_symlink_copy
    )
    mesh_converter.convert_meshes()
    mesh_converter.convert_urdf_element()
    mesh_converter.urdf_model.export_urdf(output_path)

if __name__ == "__main__":
    urdf_path = "./examples/urdf/UR5e.urdf"
    output_path = "./examples/urdf/UR5e_fixed_0.urdf"
    json_path = "./examples/config/config.json"
    meshes_dir = "./examples/meshes_0/"

    urdf_model = UrdfParser.parse_urdf(urdf_path)
    mesh_converter(
        urdf_model=urdf_model,
        output_path=output_path,
        meshes_dir=meshes_dir,
        json_path=json_path,
        is_copy_meshes=True,
        is_symlink_copy=False
    )
