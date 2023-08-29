import os

import pkg_resources
from setuptools import find_packages, setup

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name="pcdarray",
    version="1.0.0",
    description="Converts PointCloud2 message to Numpy Array",
    package_dir={"": "pointcloud_array"},
    packages=find_packages(where="pointcloud_array"),
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/reachpranjal/pcdarray_pkg",
    author="reachpranjal",
    author_email="reachpranjal19@gmail.com",
    license="MIT",
    classifiers=[
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3.10",
        "Operating System :: OS Independent",
        "Development Status :: 4 - Beta",
    ],
    install_requires=[
        str(r)
        for r in pkg_resources.parse_requirements(
            open(os.path.join(os.path.dirname(__file__), "requirements.txt"))
        )
    ],
    extras_require={
        "dev": ["twine>=4.0.2"],
    },
    python_requires=">=3.10",
)