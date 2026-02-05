# urdf2mjcf

urdf2mjcf is a small utility to help convert URDF files and their mesh
assets into formats usable by MuJoCo (MJCF). It provides a CLI entry point
and utilities for:

- Normalizing and converting mesh files (DAE → OBJ, others → STL)
- Optionally copying or linking mesh assets into a dedicated directory
- Splitting multi-material OBJ files and extracting simple material colors
- Optionally running convex decomposition via CoACD (when available)

This package exposes a `urdf2mjcf` console script .

- [README_zh.md](README_zh.md) - Chinese documentation

## Installation

Install the package locally:

```bash
pip install .
```

Install with optional convex-decomposition support:

```bash
pip install .[coacd]
```

## Basic CLI Usage

Convert a URDF and its meshes:

```bash
urdf2mjcf robot.urdf -o robot.xml -m ./meshes
```

For full options:

```bash
urdf2mjcf --help
```

Options:

- `input` (positional): Input URDF file path
- `-o, --output`: Output MJCF file path (required)
- `-m, --meshes-dir`: Mesh output directory (required)
- `-c, --copy-meshes`: Copy mesh files to the meshes directory
- `-s, --symlink-copy`: Use symbolic links instead of copying
- `-d, --decompose {visual,collision}`: Perform convex decomposition
- `-j, --json-config`: JSON config file path

JSON config file example:
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

            "add_json_texture":   true,

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
        "texture": {
            "cube_A_link": [
                {"name": "cube_A_link_TEXTURE", "file": "../assets/tag36h11-100.png", "pos": "0.0 0.0 0.025", "euler": "0.0 0.0 0.0", "size": "0.05 0.05"}
            ],
            "cube_B_link": [
                {"name": "cube_B_link_TEXTURE", "file": "../assets/tag36h11-101.png", "pos": "0.0 0.0 0.025", "euler": "0.0 0.0 0.0", "size": "0.05 0.05"}
            ]
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

## Project Structure

```
urdf2mjcf/
├── __init__.py              # Main package entry
├── cli.py                   # Command-line interface
├── mesh_converter.py        # Core mesh conversion logic
├── mjcf_generator.py        # URDF to MJCF conversion
├── mesh_decomposer.py       # OBJ post-processing and decomposition
└── py.typed                 # Type hints marker

setup.py                      # Package configuration
pyproject.toml               # Project metadata
README.md                    # This file
README_zh.md                 # Chinese documentation
LICENSE                      # MIT License
```

## Features

### Mesh Conversion

- DAE/other formats → OBJ (preserving materials where possible)
- DAE/other formats → STL (for collision meshes)
- Automatic MTL file renaming and fixing
- Separate subdirectory per mesh (avoiding MTL conflicts)
- File content hashing for deduplication
- Hardlink support for identical meshes
- ROS `package://` path resolution
- Intelligent absolute/relative path conversion
- Support for symbolic links

### URDF Parsing & Path Handling

- Full URDF mesh path resolution
- Support for `package://`, `file://`, relative, and absolute paths
- ROS package lookup via rospkg or environment variables
- Automatic search in `/opt/ros` directories

### Optional Features

- OBJ splitting by material
- Material color extraction from MTL files
- Convex decomposition via CoACD (requires `pip install coacd`)

## Dependencies

**Required:**
- trimesh >= 3.12.0

**Optional:**
- coacd >= 1.0.0 (for convex decomposition)
- rospkg (for ROS package path resolution)


## License

MIT

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

URDF2MJCF Contributors

