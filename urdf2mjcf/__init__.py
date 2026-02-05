"""urdf2mjcf - URDF to MJCF conversion package

Provides a simple command-line entry point and core utilities for
converting URDF files and processing mesh assets.
"""

from .cli import main
from .urdf_parser import UrdfParser
from .mesh_converter import MeshConverter,  mesh_converter
from .mjcf_generator import MjcfBuilder, mjcf_generator


__all__ = [
    "main",
    "UrdfParser",
    "MeshConverter",
    "mesh_converter",
    "MjcfBuilder",
    "mjcf_generator"
]
__version__ = "0.2.0"
__author__ = "TangChaoBanLi-TDC"
