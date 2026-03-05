from setuptools import setup
import os
from glob import glob

package_name = "pinky_control"

setup(
    name="pinky-control",
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Pinky Pro control package including executor and safety supervisor.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "executor_node = pinky_control.executor_node:main",
            "safety_node = pinky_control.safety_node:main",
            "camera_node = pinky_control.camera_node:main",
            "initial_pose_setter = pinky_control.initial_pose_setter:main",
        ],
    },
)
