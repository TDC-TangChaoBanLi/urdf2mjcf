"""Setup configuration for urdf2mjcf package."""

from setuptools import setup, find_packages

setup(
    name="urdf2mjcf",
    version="0.1.0",
    description="URDF to MJCF conversion tool with mesh processing and optional decomposition",
    author="URDF2MJCF Contributors",
    url="https://github.com/your-org/urdf2mjcf",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "trimesh>=3.12.0",
        "numpy>=1.21.0",
    ],
    extras_require={
        "coacd": ["coacd>=1.0.0"],
        "dev": [
            "pytest>=7.0",
            "pytest-cov>=4.0",
            "black>=22.0",
            "isort>=5.0",
            "mypy>=0.990",
            "ruff>=0.0.250",
        ],
    },
    entry_points={
        "console_scripts": [
            "urdf2mjcf=urdf2mjcf.cli:main",
        ],
    },
    package_data={
        "urdf2mjcf": ["py.typed"],
    },
    long_description="Small utility for converting URDF to MJCF with mesh processing",
    long_description_content_type="text/plain",
)
