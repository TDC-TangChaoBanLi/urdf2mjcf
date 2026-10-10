# urdf2mjcf

本项目提供一个用于将 URDF 文件及其 mesh 资源转换为 MuJoCo (MJCF) 可用格式的工具包。

主要功能：

- 标准化并转换 mesh（例如 DAE → OBJ，其他格式 → STL）
- ASCII STL → 二进制 STL 修复（MuJoCo 仅支持二进制 STL）
- 可选地将 mesh 复制或创建链接到指定目录
- 按材质分解多材质 OBJ 并提取颜色/贴图（MTL map_Kd）
- 提取 URDF `<material><texture>` 贴图并落地到 meshes 目录
- 在安装 CoACD 时可选择进行凸包分解
- 由 JSON 配置驱动生成 MJCF：执行器、传感器、site、相机、接触、约束
- 可选重力补偿（`gravcomp` / `actuatorgravcomp`）
- 通过 `joint_force` 配置关节级力/矩限制（`actuatorfrcrange`、`armature`、`damping` 等）
- 为未声明质惯量的 link 显式写入零质量 `<inertial>`

提供命令行工具 `urdf2mjcf` 。

- [README.md](README.md) - 英文文档

## 安装

本项目使用 [uv](https://docs.astral.sh/uv/) 进行依赖管理。

```bash
# 使用 uv 安装（自动创建虚拟环境并安装全部依赖）
uv sync

# 安装含凸包分解支持
uv sync --extra coacd
```

## 命令行使用

转换 URDF 和 mesh：

```bash
uv run urdf2mjcf robot.urdf -o robot.xml -m ./meshes
```

查看全部选项：

```bash
urdf2mjcf --help
```

主要选项：

- `input` (位置参数): URDF 文件路径
- `-o, --output`: MJCF 输出文件路径（必需）
- `-m, --meshes-dir`: Mesh 输出目录（必需）
- `-c, --copy-meshes`: 复制 mesh 到指定目录
- `-s, --symlink-copy`: 使用符号链接而非复制
- `-d, --decompose`: 凸包分解目标 (visual 或 collision)
- `-j, --json-config`: JSON 配置文件路径

JSON 配置文件示例:
```json
{
    "mjcf_generator": {
        "config": {
            "add_default_actuator":     false,

            "add_ros2_mujoco_actuator": false,
            "add_ros2_mujoco_sensor":   false,
            
            "add_json_option":      false,
            "add_json_actuator":    true,
            "add_json_site":        true,
            "add_json_sensor":      true,
            "add_json_camera":      true,

            "add_default_contact": true,
            "add_json_contact": true
        },
        "compiler": { "angle": "radian", "balanceinertia": true },
        "option": {"gravity": "0 0 0" },
        "worldbody": { 
            "add_default_floor": true, 
            "add_default_skybox": true, 
            "add_default_light": true, 
            "add_default_freejoint": false,

            "add_json_freejoint": true,

            "freejoint_body":[
                "cube_A_link", 
                "cube_B_link"
            ]
        },
        "actuator": {
            "motor":[
                {"group": 0, "ctrlrange":"-150 150", "name": "ur_shoulder_pan_joint_ACTUATOR_MOTOR",              "joint": "ur_shoulder_pan_joint"},
                {"group": 0, "ctrlrange":"-150 150", "name": "ur_shoulder_lift_joint_ACTUATOR_MOTOR",             "joint": "ur_shoulder_lift_joint"},
                {"group": 0, "ctrlrange":"-150 150", "name": "ur_elbow_joint_ACTUATOR_MOTOR",                     "joint": "ur_elbow_joint"},
                {"group": 0, "ctrlrange":"-28  28" , "name": "ur_wrist_1_joint_ACTUATOR_MOTOR",                   "joint": "ur_wrist_1_joint"},
                {"group": 0, "ctrlrange":"-28  28" , "name": "ur_wrist_2_joint_ACTUATOR_MOTOR",                   "joint": "ur_wrist_2_joint"},
                {"group": 0, "ctrlrange":"-28  28" , "name": "ur_wrist_3_joint_ACTUATOR_MOTOR",                   "joint": "ur_wrist_3_joint"},
                {"group": 0, "ctrlrange":"-10  10" , "name": "robotiq_85_left_knuckle_joint_ACTUATOR_MOTOR",      "joint": "robotiq_85_left_knuckle_joint"}
            ],
            "position":[
                {"group": 1, "kp":200, "dampratio":1.2, "ctrlrange":"-3.1416 3.1416", "name": "ur_shoulder_pan_joint_ACTUATOR_POSITION",           "joint": "ur_shoulder_pan_joint"},
                {"group": 1, "kp":200, "dampratio":1.2, "ctrlrange":"-3.1416 3.1416", "name": "ur_shoulder_lift_joint_ACTUATOR_POSITION",          "joint": "ur_shoulder_lift_joint"},
                {"group": 1, "kp":100, "dampratio":1.2, "ctrlrange":"-3.1416 3.1416", "name": "ur_elbow_joint_ACTUATOR_POSITION",                  "joint": "ur_elbow_joint"},
                {"group": 1, "kp":50,  "dampratio":1.2, "ctrlrange":"-3.1416 3.1416", "name": "ur_wrist_1_joint_ACTUATOR_POSITION",                "joint": "ur_wrist_1_joint"},
                {"group": 1, "kp":50,  "dampratio":1.2, "ctrlrange":"-3.1416 3.1416", "name": "ur_wrist_2_joint_ACTUATOR_POSITION",                "joint": "ur_wrist_2_joint"},
                {"group": 1, "kp":50,  "dampratio":1.2, "ctrlrange":"-3.1416 3.1416", "name": "ur_wrist_3_joint_ACTUATOR_POSITION",                "joint": "ur_wrist_3_joint"},
                {"group": 1, "kp":10,  "dampratio":1.2, "ctrlrange":"0.0 0.7929",     "name": "robotiq_85_left_knuckle_joint_ACTUATOR_POSITION",   "joint": "robotiq_85_left_knuckle_joint"}
            ]
        },
        "site":{
            "ur_ft_frame" :[
                {"name": "ur_ft_frame_SITE", "pos":"0 0 0", "quat": "1 0 0 0" }
            ]
        },
        "sensor": {
            "force":[
                { "name": "ur_ft_frame_SENSOR_FORCE", "site": "ur_ft_frame_SITE"}
            ],
            "torque": [
                { "name": "ur_ft_frame_SENSOR_TORQUE", "site": "ur_ft_frame_SITE"}
            ]
        },
        "camera": {
            "realsense_link" :[
                {"name": "realsense_link_CAMERA", "mode": "fixed", "pos": "0 0 0", "euler": "0 -1.5708 -1.5708", "fovy": "75", "resolution": "640 480" }
            ]
        },
        "gravcomp":{
            "add_gravcomp": false,          // 设为 true 则为全部 body 开启重力补偿；也可用列表指定 body: ["body_1", "body_2"]
            "add_actuatorgravcomp": false   // 设为 true 则为全部 joint 开启控制力重力补偿；也可用列表指定 joint: ["joint_1", "joint_2"]
        },
        "joint_force":{
            // 关节级力/矩限制，写到 <joint> 上。
            // true      -> 用 URDF <limit effort="..."> 自动生成 actuatorfrcrange
            // "lo hi"   -> {"actuatorfrcrange": "lo hi"} 的简写
            // {attr: v} -> 任意 <joint> 属性，例如 armature / damping
            "elbow_joint": true,
            "wrist_joint": "-28 28",
            "shoulder_joint": {"actuatorfrcrange": "-150 150", "armature": "0.1"}
        },
        "contact":{
            "exclude": [
                {"body1": "robotiq_85_left_finger_tip_link", "body2": "robotiq_85_left_inner_knuckle_link"},
                {"body1": "robotiq_85_right_finger_tip_link", "body2": "robotiq_85_right_inner_knuckle_link"}
            ]
        }
    }
}
```

## 项目结构

```
src/urdf2mjcf/
├── __init__.py              # 包入口
├── cli.py                   # 命令行界面（流程编排）
├── urdf_parser.py           # URDF 解析（link/joint/mesh/material/texture）
├── mesh_converter.py        # mesh 复制/软链/格式转换/ASCII-STL 修复/URDF 贴图落地
├── mesh_decomposer.py       # 多材质 OBJ 分解（model 级）
├── mesh_coacd.py            # 凸包分解（CoACD，可选）
├── mesh_utils.py            # 公共 mesh/texture 工具
├── resource_registry.py     # 资源登记与写 MJCF 后统一清理
├── mjcf_generator.py        # URDF 到 MJCF 转换
└── py.typed                 # 类型提示标记

pyproject.toml               # 项目元数据和依赖
uv.lock                      # 锁定文件（uv 自动生成）
README.md                    # 英文文档
README_zh.md                 # 本文件
LICENSE                      # MIT 许可证
```

## 主要特性

### Mesh 转换

- DAE/其他格式 → OBJ（保留材质）
- DAE/其他格式 → STL（用于碰撞网格）
- ASCII STL → 二进制 STL（MuJoCo 兼容）
- 贴图：URDF `<material><texture>` 或 mesh 自带（OBJ/MTL map_Kd 等）
- 转换/拆分/凸包的中间文件由 ResourceRegistry 登记，写 MJCF 后自动清理
- 自动 MTL 文件重命名和修复
- 每个 mesh 独立子目录（避免 MTL 冲突）
- 基于内容的 mesh 去重和硬链接支持
- ROS package:// 路径解析
- 绝对/相对路径智能转换
- 符号链接支持

### MJCF 生成

- 由 JSON 配置中的 `mjcf_generator` 段驱动
- 执行器（`motor` / `position`）、传感器、site、相机、灯光与接触
- 支持 `ros2_control` 的 `<hardware><MimicJoint>`，且 mimic 关节顺序正确
- 关节 `scale` 正确写入 MJCF geom
- 由 `equality` 配置段生成约束（equality）
- 编译器设置（`angle`、`balanceinertia` 等）与世界 `<option>`（如重力）
- 可选的默认地面 / 天空盒 / 光源
- 自由关节：支持把嵌套 body 从 URDF 树中提升为 worldbody 下的自由漂浮 body，
  并保持其原有世界位姿

#### 质惯量处理

- 声明了 `<inertial>` 的 link：转换其惯量并对角化后写入
- **未**声明质惯量的 link：显式写入零质量 `<inertial>`。不能简单地不写该元素——
  只要 body 自带 geom，MuJoCo 就会用几何体反推质量（visual mesh 走 shell 惯量，
  即「三角面总面积 × 默认密度」），导致整机质量被静默放大
- 带可动关节却没有 `<inertial>` 的 link：仅给出告警，不擅自置零——置零会让 MuJoCo
  编译失败（运动 body 的质量必须大于 `mjMINVAL`），数据不完整时应由用户修正 URDF

#### 关节级力/矩限制（`joint_force`）

`actuatorfrcrange` 限制的是「作用在该关节自由度上的全部 actuator 力」（**包含**重力
补偿），而 actuator 自身的 `forcerange` 只限制伺服输出项。URDF `<limit effort>` 的语义
是关节总力矩上限，要完整复现它必须使用关节级的 `actuatorfrcrange`，这正是
`joint_force` 配置段的用途：

- `true` —— 用 URDF `<limit effort="...">` 自动生成 `actuatorfrcrange`；若 URDF
  未声明 effort，则不生成限制并输出告警
- `"lo hi"` —— `{"actuatorfrcrange": "lo hi"}` 的简写
- `{attr: value}` —— 任意 `<joint>` 属性，例如 `armature` 或 `damping`

### 依赖项

**必需：**
- trimesh >= 3.12.0
- numpy >= 1.21.0
- pycollada >= 0.9.3
- Pillow >= 9.0.0

**可选：**
- coacd >= 1.0.0（凸包分解）
- rospkg（ROS 包路径解析）

## 许可证

MIT

