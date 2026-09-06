
# =============================================================================
# Parse URDF
# URDF 解析
# =============================================================================
from __future__ import annotations

import xml.etree.ElementTree as ET
from xml.dom import minidom
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Union
from pathlib import Path
import logging



logger = logging.getLogger("urdf2mjcf.urdf_parser")




def _safe_float(x: Optional[str], default: float = 0.0) -> float:
    """
    EN: Convert str to float safely.
    CN: 安全地把字符串转换为 float。
    """
    if x is None:
        return default
    try:
        return float(x)
    except Exception:
        return default


@dataclass
class UrdfParser:
    """
    EN: Parse URDF XML into UrdfModel.
    CN: 解析 URDF XML 生成 UrdfModel。
    """
    @dataclass
    class UrdfOrigin:
        xyz: str = "0 0 0"
        rpy: str = "0 0 0"
    
    @dataclass
    class UrdfGeometry:
        g_type: str
        filename: str = ""
        scale: str = ""
        size: str = ""
        radius: str = ""
        length: str = ""
    
    @dataclass
    class UrdfMaterial:
        m_name: Optional[str] = None
        color_rgba: Optional[str] = None
        # EN: filename of <material><texture .../> (raw URDF value, may be package:// etc.)
        # CN: <material><texture .../> 的 filename（URDF 原文，可能为 package:// 等）
        texture_file: Optional[str] = None
    
    @dataclass
    class UrdfVisual:
        origin: UrdfParser.UrdfOrigin
        geometry: UrdfParser.UrdfGeometry
        material: Optional[UrdfParser.UrdfMaterial] = None
    
    @dataclass
    class UrdfCollision:
        origin: UrdfParser.UrdfOrigin
        geometry: UrdfParser.UrdfGeometry
        material: Optional[UrdfParser.UrdfMaterial] = None
    
    @dataclass
    class UrdfInertia:
        ixx: float = 0.0
        iyy: float = 0.0
        izz: float = 0.0
        ixy: float = 0.0
        ixz: float = 0.0
        iyz: float = 0.0
    
    @dataclass
    class UrdfInertial:
        origin: UrdfParser.UrdfOrigin# xyz, rpy
        inertia: UrdfParser.UrdfInertia
        mass: float = 0.0

    @dataclass
    class UrdfLink: # link
        l_name: str
        inertial: UrdfParser.UrdfInertial
        visuals: Optional[List[UrdfParser.UrdfVisual]] = None
        collisions: Optional[List[UrdfParser.UrdfCollision]] = None

    @dataclass
    class UrdfJointLimit:
        effort: float
        lower: float
        upper: float
        velocity: float

    @dataclass
    class UrdfJointMimic:
        joint: str
        multiplier: float
        offset: float

    @dataclass
    class UrdfJoint: # joint
        j_name: str
        j_type: str
        parent: str
        child: str
        axis: str 
        origin: UrdfParser.UrdfOrigin
        limit: Optional[UrdfParser.UrdfJointLimit]
        mimic: Optional[UrdfParser.UrdfJointMimic]
    


    @dataclass
    class UrdfMjJoint:
        name: str = field(default_factory=str)
        command_interfaces: List[str] = field(default_factory=list)
        state_interfaces: List[str] = field(default_factory=list)
        param: Dict[str, str] = field(default_factory=dict) # state_interfaces params ("initial_value") 

    @dataclass
    class UrdfMjSensor:
        name: str = field(default_factory=str)
        mujoco_type: str = field(default_factory=str) # from param "mujoco_type"  ["fts" "imu" ("camera", "lidar")]
        param: Dict[str, str] = field(default_factory=dict)   # "mujoco_sensor_name" fot FTS and IMU
                                # "force_mjcf_suffix", "torque_mjcf_suffix" for FTS
                                # "orientation_mjcf_suffix", "angular_velocity_mjcf_suffix", "linear_acceleration_mjcf_suffix" for IMU
                                # "frame_name" for Camera and Lidar
                                # "angle_increment", "min_angle", "max_angle", "range_min", "range_max" for Lidar
    robot_elem: ET.Element[str] = None
    robot_name: str = ""
    root_link: str = ""
    parent_map: Dict[str, List[str]] = field(default_factory=dict) # {"p_1": ["c_1", "c_2"], "p_2": ["c_3"], ...}
    materials: List[UrdfParser.UrdfMaterial] = field(default_factory=list) 
    links: List[UrdfParser.UrdfLink] = field(default_factory=list)
    joints: List[UrdfParser.UrdfJoint] = field(default_factory=list)
    
    mj_joints: List[UrdfParser.UrdfMjJoint] = field(default_factory=list)
    mj_sensors: List[UrdfParser.UrdfMjSensor] = field(default_factory=list)
    urdf_file_path: Path = None

    @classmethod
    def parse_urdf(cls, urdf_path: Union[str, Path]) -> UrdfParser:
        """ 加载 URDF 文件并解析 """
        p = Path(urdf_path)
        if not p.exists():
            raise FileNotFoundError(f"URDF file not found: {p}")
        urdf_tree = ET.parse(p)
        robot_elem = urdf_tree.getroot()
        if robot_elem is None or robot_elem.tag != "robot":
            raise ValueError("Invalid URDF: root element is not <robot>")
        
        # 创建一个新的实例并设置其属性
        instance = cls()
        instance.robot_elem = robot_elem
        instance.robot_name = robot_elem.attrib.get("name", "default_robot")
        instance.materials = instance._parse_materials(robot_elem)
        instance.joints, instance.parent_map = instance._parse_joints(robot_elem)
        instance.links = instance._parse_links(robot_elem)
        instance.root_link = instance._get_root_link(instance.parent_map)
        instance.urdf_file_path = p
        instance.mj_joints, instance.mj_sensors = instance._parse_mujoco_ros_control(urdf_tree)

        logger.info(
            "Loaded URDF: root_link: %s, links: %d, joints: %d, material: %d, mj_joints: %d, mj_sensors: %d",
            instance.root_link, 
            len(instance.links), 
            len(instance.joints), 
            len(instance.materials), 
            len(instance.mj_joints), 
            len(instance.mj_sensors)
        )

        return instance
    
    def export_urdf(self, output_path: Union[str, Path]):
        """
        将 self.robot_elem 的内容导出为 URDF 文件
        
        :param output_path: 输出文件路径
        """
        # 注意：tostring 的结果是 bytes，需要解码为字符串
        # robot_xml_str = ET.tostring(self.robot_elem, encoding='unicode')
        rough = ET.tostring(self.robot_elem, encoding="utf-8")
        reparsed = minidom.parseString(rough)
        pretty = reparsed.toprettyxml(indent="  ")
        pretty = re.sub(r"\n\s*\n", "\n", pretty)
        Path(output_path).write_text(pretty, encoding="utf-8")
        

    @staticmethod
    def _get_root_link(parent_map: Dict[str, List[str]]) -> str:
        """获取根链接名称"""
        all_links = list(set(parent_map.keys()) | set().union(*parent_map.values()))
        all_children = set().union(*parent_map.values())
        for link in all_links:
            if link not in all_children:
                return link

    @staticmethod
    def _parse_material_elem(material_elem: ET.Element[str]) -> 'UrdfParser.UrdfMaterial':
        """
        解析一个 <material> 元素（含 <color> 与 <texture> 子元素）

        URDF 允许：
          <material name="...">
            <color rgba="..."/>
            <texture filename="package://.../gold.png"/>
          </material>
        """
        m_name = material_elem.attrib.get("name", None)
        rgba: Optional[str] = None
        texture_file: Optional[str] = None
        color_elem = material_elem.find("color")
        if color_elem is not None:
            rgba = color_elem.attrib.get("rgba", None)
        texture_elem = material_elem.find("texture")
        if texture_elem is not None:
            texture_file = texture_elem.attrib.get("filename", None)
        return UrdfParser.UrdfMaterial(m_name=m_name, color_rgba=rgba, texture_file=texture_file)

    @staticmethod
    def _parse_materials(robot_elem: ET.Element[str]) -> List[UrdfParser.UrdfMaterial]:
        """解析 robot 根级 material 标签，返回 Material 对象"""
        materials: List[UrdfParser.UrdfMaterial] = []
        # root materials
        for material_elem in robot_elem.findall("material"):
            materials.append(UrdfParser._parse_material_elem(material_elem))
        return materials

    @staticmethod
    def _get_origin(parent_elem: ET.Element[str]) -> UrdfParser.UrdfOrigin:
        """解析 element 下的 origin 标签，返回 UrdfOrigin """
        origin_elem = parent_elem.find("origin")
        if origin_elem is not None:
            xyz = origin_elem.attrib.get("xyz", "0 0 0")
            rpy = origin_elem.attrib.get("rpy", "0 0 0")
        else:
            xyz = "0 0 0"
            rpy = "0 0 0"
        return UrdfParser.UrdfOrigin(xyz=xyz, rpy=rpy)

    @staticmethod
    def _get_geometrys(geom_elem: ET.Element[str]) -> Optional[UrdfParser.UrdfGeometry]:
        """解析 geometry 标签，返回 UrdfGeometry """
        g_type = ""
        filename = ""
        scale = ""
        size = ""
        radius = ""
        length = ""
        mesh_elem = geom_elem.find("mesh")
        box_elem = geom_elem.find("box")
        cylinder_elem = geom_elem.find("cylinder")
        sphere_elem = geom_elem.find("sphere")
        if mesh_elem is not None:
            g_type = "mesh"
            filename = mesh_elem.attrib.get("filename", "")
            scale = mesh_elem.attrib.get("scale", "")
            geometry = UrdfParser.UrdfGeometry(g_type=g_type, filename=filename, scale=scale)
            return geometry
        if box_elem is not None:
            g_type = "box"
            size = box_elem.attrib.get("size", "")
            geometry = UrdfParser.UrdfGeometry(g_type=g_type, size=size)
            return geometry
        if cylinder_elem is not None:
            g_type = "cylinder"
            radius = cylinder_elem.attrib.get("radius", "")
            length = cylinder_elem.attrib.get("length", "")
            geometry = UrdfParser.UrdfGeometry(g_type=g_type, radius=radius, length=length)
            return geometry
        if sphere_elem is not None:
            g_type = "sphere"
            radius = sphere_elem.attrib.get("radius", "")
            geometry = UrdfParser.UrdfGeometry(g_type=g_type, radius=radius)
            return geometry
        else:
            return None

    @staticmethod
    def _parse_links(robot_elem: ET.Element[str]) -> List[UrdfParser.UrdfLink]:
        """解析所有 link 标签到 UrdfParser.links 列表"""
        link: UrdfParser.UrdfLink = None
        links: List[UrdfParser.UrdfLink] = []

        for link_elem in robot_elem.findall("link"): # iterate all link elements
            link_name = link_elem.attrib.get("name")
            if not link_name:
                continue
            link__l_name: str = link_name

            # ==== 解析 visual 信息 ====
            link__visuals: Optional[List[UrdfParser.UrdfVisual]] = None
            visual_elems = link_elem.findall("visual") # get all visual elements
            if visual_elems is not None:
                link__visuals: List[UrdfParser.UrdfVisual] = []
                for visual_elem in visual_elems: # may be more than one visual element
                    # 解析 origin
                    visual_origin = UrdfParser._get_origin(visual_elem)
                    # 解析 geometry
                    geometry_elem = visual_elem.find("geometry") # get all geometry elements
                    geometry = UrdfParser._get_geometrys(geometry_elem) # add geometrys
                    # 解析 material
                    material_elem = visual_elem.find("material") # visual element has only one material element
                    if material_elem is not None:
                        link__visual_material = UrdfParser._parse_material_elem(material_elem)
                    else: 
                        link__visual_material = None
                    # 写入 visual 信息
                    link__visuals.append(UrdfParser.UrdfVisual(origin=visual_origin, geometry=geometry, material=link__visual_material))

            # ==== 解析 collision 信息 ====
            link__collisions: Optional[UrdfParser.UrdfCollision] = None
            collision_elems = link_elem.findall("collision") # get all collision elements
            if collision_elems is not None:
                link__collisions: List[UrdfParser.UrdfCollision] = []
                for collision_elem in collision_elems: # may be more than one collision
                    # 解析 origin
                    collision_origin = UrdfParser._get_origin(collision_elem)
                    # 解析 geometry
                    geometry_elem = collision_elem.find("geometry") # get all geometry elements
                    geometry = UrdfParser._get_geometrys(geometry_elem) # extend geometrys list
                    # 解析 material
                    material_elem = collision_elem.find("material") # collision element has only one material element
                    if material_elem is not None:
                        link__visual_material = UrdfParser._parse_material_elem(material_elem)
                    else: 
                        link__visual_material = None
                    # 写入 collision 信息
                    link__collisions.append(UrdfParser.UrdfCollision(origin=collision_origin, geometry=geometry, material=link__visual_material))

            # ==== 解析 inertial 信息 ====
            link__inertial: Optional[UrdfParser.UrdfInertial] = None
            inertial_elem = link_elem.find("inertial")
            if inertial_elem is not None:
                # 解析 mass
                mass_elem = inertial_elem.find("mass")
                mass = _safe_float(mass_elem.attrib.get("value") if mass_elem is not None else None, 0.0)
                # 解析 origin
                inertial_origin = UrdfParser._get_origin(inertial_elem)
                # 解析 UrdfInertia
                inertia_elem = inertial_elem.find("inertia")
                if inertia_elem is not None:
                    ixx = _safe_float(inertia_elem.attrib.get("ixx"), 0.0)
                    iyy = _safe_float(inertia_elem.attrib.get("iyy"), 0.0)
                    izz = _safe_float(inertia_elem.attrib.get("izz"), 0.0)
                    ixy = _safe_float(inertia_elem.attrib.get("ixy"), 0.0)
                    ixz = _safe_float(inertia_elem.attrib.get("ixz"), 0.0)
                    iyz = _safe_float(inertia_elem.attrib.get("iyz"), 0.0)
                    inertia = UrdfParser.UrdfInertia(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz)
                else:
                    inertia = UrdfParser.UrdfInertia(ixx=0.0, iyy=0.0, izz=0.0, ixy=0.0, ixz=0.0, iyz=0.0)
                # 写入 inertial 信息
                link__inertial = UrdfParser.UrdfInertial(mass=mass, origin=inertial_origin, inertia=inertia)
            # === 添加 link 到列表 ===
            link = UrdfParser.UrdfLink(
                l_name=link__l_name, 
                visuals=link__visuals, 
                collisions=link__collisions, 
                inertial=link__inertial
                )
            links.append(link)
        return links

    @staticmethod
    def _parse_joints(robot_elem: ET.Element[str]) -> Tuple[List['UrdfParser.UrdfJoint'], Dict[str, List[str]]]:
        """解析所有 joint 标签到 UrdfParser.joints 列表"""
        joint: UrdfParser.UrdfJoint = None
        joints: List[UrdfParser.UrdfJoint] = []
        parent_map: Dict[str, List[str]] = {}

        for joint_elem in robot_elem.findall("joint"):
            j_name = joint_elem.attrib.get("name", "")
            j_type = joint_elem.attrib.get("type", "fixed")
            parent_elem = joint_elem.find("parent")
            parent_name = parent_elem.attrib.get("link", "") if parent_elem is not None else ""
            child_elem = joint_elem.find("child")
            child_name = child_elem.attrib.get("link", "") if child_elem is not None else ""
            joint__j_name = j_name
            joint__j_type = j_type
            joint__parent = parent_name
            joint__child = child_name

            # ==== 解析 origin ====
            origin = UrdfParser._get_origin(joint_elem)
            joint__origin = origin

            # ==== 解析 axis ====
            axis_elem = joint_elem.find("axis")
            axis = axis_elem.attrib.get("xyz", "0 0 1") if axis_elem is not None else "0 0 1"
            joint__axis = axis

            # ==== 解析 limit ====
            joint__limit: Optional[UrdfParser.UrdfJointLimit] = None
            limit_elem = joint_elem.find("limit")
            if limit_elem is not None:
                effort = _safe_float(limit_elem.attrib.get("effort"), 0.0)
                lower = _safe_float(limit_elem.attrib.get("lower"), 0.0)
                upper = _safe_float(limit_elem.attrib.get("upper"), 0.0)
                velocity = _safe_float(limit_elem.attrib.get("velocity"), 0.0)
                limit = UrdfParser.UrdfJointLimit(effort=effort, lower=lower, upper=upper, velocity=velocity)
            else:
                limit = None
            joint__limit = limit

            # ==== 解析 mimic ====
            joint__mimic: Optional[UrdfParser.UrdfJointMimic] = None
            mimic_elem = joint_elem.find("mimic")
            if mimic_elem is not None:
                mimic_joint = mimic_elem.attrib.get("joint", "")
                mimic_multiplier = _safe_float(mimic_elem.attrib.get("multiplier"), 1.0)
                mimic_offset = _safe_float(mimic_elem.attrib.get("offset"), 0.0)
                mimic = UrdfParser.UrdfJointMimic(joint=mimic_joint, multiplier=mimic_multiplier, offset=mimic_offset)
            else:
                mimic = None
            joint__mimic = mimic
            # === 添加 joint 到列表 ===
            joint = UrdfParser.UrdfJoint(
                j_name=joint__j_name, 
                j_type=joint__j_type, 
                parent=joint__parent, 
                child=joint__child, 
                origin=joint__origin, 
                axis=joint__axis, 
                limit=joint__limit, 
                mimic=joint__mimic
                )
            joints.append(joint)
            parent_map.setdefault(parent_name, []).append(child_name)
        return joints, parent_map
                

    @staticmethod
    def _parse_mujoco_ros_control(robot_elem: ET.Element[str]) -> Tuple[List['UrdfParser.UrdfMjJoint'], List['UrdfParser.UrdfMjSensor']]:
        if robot_elem.find("ros2_control") is None:
            return [], []
        mj_joints: List[UrdfParser.UrdfMjJoint] = []
        mj_sensors: List[UrdfParser.UrdfMjSensor] = []
        mj_joint: UrdfParser.UrdfMjJoint = None
        mj_sensor: UrdfParser.UrdfMjSensor = None
        for ros2_control_elem in robot_elem.findall("ros2_control"): # iterate all ros2_control elements
            plugin_elem = ros2_control_elem.find("./hardware/plugin")
            if plugin_elem is None or plugin_elem.text is None: # no plugin element or no plugin text -> skip
                continue
            plugin_text = plugin_elem.text
            if plugin_text.find("MujocoSystemInterface") < 0: # not a MujocoSystemInterface plugin -> skip
                continue
            # === joint ===
            for joint_elem in ros2_control_elem.findall("joint"): # iterate all joint elements
                joint_name = joint_elem.attrib.get("name", None)
                if joint_name is None:
                    logger.warning("MujocoSystemInterface joint element without name, skipped this joint")
                    continue
                mj_joint = UrdfParser.UrdfMjJoint(name=joint_name)
                for command_interface in joint_elem.findall("command_interface"): # add all command interfaces
                    command_interface_name = command_interface.attrib.get("name", None)
                    if command_interface_name is None:
                        logger.warning("MujocoSystemInterface command_interface element without name, skipped this interface")
                        continue
                    mj_joint.command_interfaces.append(command_interface_name) # add command interface
                for state_interface in joint_elem.findall("state_interface"): # add all state interfaces
                    state_interface_name = state_interface.attrib.get("name", None)
                    if state_interface_name is None:
                        logger.warning("MujocoSystemInterface state_interface element without name, skipped this interface")
                        continue
                    mj_joint.state_interfaces.append(state_interface_name) # add state interface
                    for state_interface_param in state_interface.findall("param"):
                        param_name = state_interface_param.attrib.get("name", None)
                        if param_name is None:
                            logger.warning("MujocoSystemInterface state_interface_param element without name, skipped this param")
                            continue
                        mj_joint.param[param_name] = state_interface_param.text # add param ("initial_value")
                mj_joints.append(mj_joint) # add joint to list
            # === sensor ===
            for sensor_elem in ros2_control_elem.findall("sensor"): # add all sensors
                mj_sensor = UrdfParser.UrdfMjSensor()
                sensor_name = sensor_elem.attrib.get("name", None) 
                if sensor_name is None:
                    logger.warning("MujocoSystemInterface sensor element without name, skipped this sensor")
                    continue
                mj_sensor.name = sensor_name # add sensor name
                for param_elem in sensor_elem.findall("param"):
                    param_name = param_elem.attrib.get("name", None)
                    if param_name is None:
                        logger.warning("MujocoSystemInterface sensor_interface element without name, skipped this sensor")
                        continue
                    if param_name == "mujoco_type":
                        mj_sensor.mujoco_type = param_elem.text # add mujoco_type for fts and imu
                        continue
                    mj_sensor.param[param_name] = param_elem.text # add param
                if "frame_name" in mj_sensor.param: # add mujoico_type for camera and lidar
                    camera_param_names = ["info_topic", "image_topic", "depth_topic"]
                    lidar_param_names = ["angle_increment", "num_rangefinders", "range_min", "range_max", "laserscan_topic"]
                    if any(param_name in mj_sensor.param for param_name in camera_param_names):
                        mj_sensor.mujoco_type = "camera"
                    elif any(param_name in mj_sensor.param for param_name in lidar_param_names):
                        mj_sensor.mujoco_type = "lidar"

                mj_sensors.append(mj_sensor) # add sensor to list

        return mj_joints, mj_sensors