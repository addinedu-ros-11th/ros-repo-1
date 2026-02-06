from setuptools import setup

package_name = "office_robot_executor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/executor.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Task executor node for office robot (mock-ready).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "office_robot_executor_node = office_robot_executor.node:main",
        ],
    },
)
