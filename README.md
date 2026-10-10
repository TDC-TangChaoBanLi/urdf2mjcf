# urdf2mjcf

urdf2mjcf is a small utility to help convert URDF files and their mesh
assets into formats usable by MuJoCo (MJCF). It provides a CLI entry point
and utilities for:

- Normalizing and converting mesh files (DAE → OBJ, others → STL)
- Fixing ASCII STL → binary STL (MuJoCo only supports binary STL)
- Optionally copying or linking mesh assets into a dedicated directory
- Splitting multi-material OBJ files and extracting colors / textures (MTL map_Kd)
- Extracting URDF `<material><texture>` textures into the meshes directory
- Optionally running convex decomposition via CoACD (when available)
- Generating a JSON-driven MJCF: actuators, sensors, sites, cameras, contacts, equality constraints
- Optional gravity compensation (`gravcomp` / `actuatorgravcomp`)
- Per-joint force / torque limits (`actuatorfrcrange`, `armature`, `damping`, ...) via `joint_force`
- Explicit zero-mass `<inertial>` for links that declare no inertia

This package exposes a `urdf2mjcf` console script .

- [README_zh.md](README_zh.md) - Chinese documentation

## Installation

This project uses [uv](https://docs.astral.sh/uv/) for dependency management.

```bash
# Install with uv (creates virtual environment and installs all dependencies)
uv sync

# Install with optional convex-decomposition support
uv sync --extra coacd
```

## Basic CLI Usage

Convert a URDF and its meshes:

```bash
uv run urdf2mjcf robot.urdf -o robot.xml -m ./meshes
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
            "add_gravcomp": false,          // true = enable for all bodies; or a list of body names: ["body_1", "body_2"]
            "add_actuatorgravcomp": false   // true = enable for all joints; or a list of joint names: ["joint_1", "joint_2"]
        },
        "joint_force":{
            // Per-joint force/torque limits written onto <joint>.
            // true      -> derive actuatorfrcrange from the URDF <limit effort="...">
            // "lo hi"   -> shorthand for {"actuatorfrcrange": "lo hi"}
            // {attr: v} -> any <joint> attribute, e.g. armature / damping
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

## Project Structure

```
src/urdf2mjcf/
├── __init__.py              # Main package entry
├── cli.py                   # Command-line interface (pipeline orchestration)
├── urdf_parser.py           # URDF parsing (link/joint/mesh/material/texture)
├── mesh_converter.py        # Mesh copy/symlink/format conversion / ASCII-STL fix / URDF texture landing
├── mesh_decomposer.py       # Multi-material OBJ decomposition (model level)
├── mesh_coacd.py            # Convex decomposition via CoACD (optional)
├── mesh_utils.py            # Shared mesh/texture utilities
├── resource_registry.py     # Resource registry & post-MJCF cleanup
├── mjcf_generator.py        # URDF to MJCF conversion
└── py.typed                 # Type hints marker

pyproject.toml               # Project metadata and dependencies
uv.lock                      # Lock file (generated by uv)
README.md                    # This file
README_zh.md                 # Chinese documentation
LICENSE                      # MIT License
```

## Features

### Mesh Conversion

- DAE/other formats → OBJ (preserving materials where possible)
- DAE/other formats → STL (for collision meshes)
- ASCII STL → binary STL (MuJoCo compatible)
- Textures: from URDF `<material><texture>` or from the mesh itself (OBJ/MTL map_Kd etc.)
- Intermediate files from conversion/splitting/convex decomposition are tracked by a
  ResourceRegistry and cleaned up automatically after the MJCF is written
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

### MJCF Generation

- Driven by the `mjcf_generator` section of the JSON config
- Actuators (`motor` / `position`), sensors, sites, cameras, lights and contacts
- `ros2_control` `<hardware><MimicJoint>` support, with correct mimic ordering
- Joint `scale` values propagated onto MJCF geoms
- Equality constraints from the `equality` config section
- Compiler settings (`angle`, `balanceinertia`, ...), world `<option>` (e.g. gravity)
- Optional default floor / skybox / light
- Free joints, including promoting a nested body out of the URDF tree to a
  worldbody free-floating body while preserving its world pose

#### Inertia handling

- Links that declare `<inertial>` are converted with their inertia diagonalized
- Links that declare **no** inertia get an explicit zero-mass `<inertial>`.
  Omitting the element is not equivalent: MuJoCo would infer mass from the body
  geometry (visual meshes use shell inertia = surface area × default density),
  which silently inflates the model's total mass
- A link with a movable joint but no `<inertial>` is only warned about — zeroing
  it would make MuJoCo compilation fail (a moving body must exceed `mjMINVAL`),
  so the incomplete data is left for the user to fix in the URDF

#### Joint force / torque limits (`joint_force`)

`actuatorfrcrange` caps the *total* actuator force applied to a joint's DOFs,
gravity compensation included; an actuator's own `forcerange` only caps the
servo output. Since URDF `<limit effort>` means total joint torque, reproducing
it faithfully requires a joint-level `actuatorfrcrange`. This is what the
`joint_force` section is for:

- `true` — derive `actuatorfrcrange` from the URDF `<limit effort="...">`;
  if the URDF declares no effort, no limit is emitted and a warning is logged
- `"lo hi"` — shorthand for `{"actuatorfrcrange": "lo hi"}`
- `{attr: value}` — any `<joint>` attribute, e.g. `armature` or `damping`

### Optional Features

- OBJ splitting by material
- Material color extraction from MTL files
- Convex decomposition via CoACD (requires `uv sync --extra coacd`)

## Dependencies

**Required:**
- trimesh >= 3.12.0
- numpy >= 1.21.0
- pycollada >= 0.9.3
- Pillow >= 9.0.0

**Optional:**
- coacd >= 1.0.0 (for convex decomposition)
- rospkg (for ROS package path resolution)


## License

MIT

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

URDF2MJCF Contributors

