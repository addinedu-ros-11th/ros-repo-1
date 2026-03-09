from setuptools import setup
import os
from glob import glob

package_name = "pinky_comms"

setup(
    name="pinky-comms",
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Pinky Pro AI Server communication package (UDP).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bridge_node = pinky_comms.bridge_node:main",
        ],
    },
)
