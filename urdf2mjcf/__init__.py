"""urdf2mjcf - URDF to MJCF conversion package

Provides a simple command-line entry point and core utilities for
converting URDF files and processing mesh assets.
"""

from .cli import main
from .mesh_converter import UrdfMeshProcessor

__all__ = [
    "main",
    "UrdfMeshProcessor",
]
__version__ = "0.1.0"
__author__ = "URDF2MJCF Contributors"
