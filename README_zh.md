# urdf2mjcf

本项目提供一个用于将 URDF 文件及其 mesh 资源转换为 MuJoCo (MJCF) 可用格式的工具包。

主要功能：

- 标准化并转换 mesh（例如 DAE → OBJ，其他格式 → STL）
- 可选地将 mesh 复制或创建链接到指定目录
- 按材质分解多材质 OBJ 并提取简单材质颜色
- 在安装 CoACD 时可选择进行凸包分解

提供命令行工具 `urdf2mjcf` 。

- [README.md](README.md) - 英文文档

## 安装

基础安装：

```bash
pip install .
```

安装含凸包分解支持：

```bash
pip install .[coacd]
```

## 命令行使用

转换 URDF 和 mesh：

```bash
urdf2mjcf robot.urdf -o robot.xml -m ./meshes
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


## 项目结构

```
urdf2mjcf/
├── __init__.py              # 包入口
├── cli.py                   # 命令行界面
├── mesh_converter.py        # 核心 mesh 转换逻辑
├── mjcf_generator.py        # URDF 到 MJCF 转换
├── mesh_decomposer.py       # OBJ 后处理和分解
└── py.typed                 # 类型提示标记
```

## 主要特性

### Mesh 转换

- DAE/其他格式 → OBJ（保留材质）
- DAE/其他格式 → STL（用于碰撞网格）
- 自动 MTL 文件重命名和修复
- 每个 mesh 独立子目录（避免 MTL 冲突）
- 基于内容的 mesh 去重和硬链接支持
- ROS package:// 路径解析
- 绝对/相对路径智能转换
- 符号链接支持

### 依赖项

**必需：**
- trimesh >= 3.12.0
- numpy >= 1.21.0

**可选：**
- coacd >= 1.0.0（凸包分解）
- rospkg（ROS 包路径解析）

## 许可证

MIT

