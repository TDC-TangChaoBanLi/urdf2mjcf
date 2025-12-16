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
    "mjcf": {
        "config": {
            "add_default_actuator":     false,

            "add_ros2_mujoco_actuator": false,
            "add_ros2_mujoco_sensor":   false,
            
            "add_json_actuator":    true,
            "add_json_sensor":      true,
            "add_json_camera":      true
        },
        "compiler": { "angle": "radian", "balanceinertia": true },
        "option": { "timestep": 0.002, "gravity": "0 0 0" },
        "worldbody": { "add_floor": true, "add_skybox": true, "add_light": true, "add_freejoint": false },
        "actuator": {
            "motor":[
                {"group": 0, "ctrlrange":"-3.1416 3.1416", "name": "shoulder_pan_joint_ACTUATOR_MOTOR",              "joint": "shoulder_pan_joint"},
                {"group": 0, "ctrlrange":"-3.1416 3.1416", "name": "shoulder_lift_joint_ACTUATOR_MOTOR",             "joint": "shoulder_lift_joint"},
                {"group": 0, "ctrlrange":"-3.1416 3.1416", "name": "elbow_joint_ACTUATOR_MOTOR",                     "joint": "elbow_joint"},
                {"group": 0, "ctrlrange":"-3.1416 3.1416", "name": "wrist_1_joint_ACTUATOR_MOTOR",                   "joint": "wrist_1_joint"},
                {"group": 0, "ctrlrange":"-3.1416 3.1416", "name": "wrist_2_joint_ACTUATOR_MOTOR",                   "joint": "wrist_2_joint"},
                {"group": 0, "ctrlrange":"-3.1416 3.1416", "name": "wrist_3_joint_ACTUATOR_MOTOR",                   "joint": "wrist_3_joint"},
            ],
            "position":[
                {"group": 1, "kp":1 , "dampratio":1, "ctrlrange":"-3.1416 3.1416", "name": "robotiq_85_left_knuckle_joint_ACTUATOR_POSITION",   "joint": "robotiq_85_left_knuckle_joint"},
            ]
        },
        "sensor": {
            "force":[
                { "name": "ft_frame_SENSOR_FORCE", "body": "ft_frame"}
            ],
            "torque": [
                { "name": "ft_frame_SENSOR_TORQUE", "body": "ft_frame"}
            ]
        },
        "camera": {
            "realsense_link" :[
                {"name": "realsense_link_CAMERA", "mode": "fixed", "pos": "0 0 0", "euler": "0 -1.5708 -1.5708", "fovy": 75, "resolution": "640 480" }
            ]
        }
    },
    "coacd":{
        "threshold": 0.03,
        "max_convex_hull": 20,
        "preprocess_mode": "auto",
        "preprocess_resolution": 60,
        "resolution": 3000,
        "mcts_nodes": 20,
        "mcts_iterations": 200,
        "mcts_max_depth": 3,
        "pca": true,
        "merge": true,
        "decimate": false,
        "max_ch_vertex": 256,
        "extrude": false,
        "extrude_margin": 0.01,
        "apx_mode": "ch",
        "seed": 0,
        "log_level": "error"
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

