from setuptools import setup

package_name = "office_robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bringup.launch.py"]),
        ("share/" + package_name + "/config", ["config/executor.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Bringup launch and params for office robot.",
    license="Apache-2.0",
)
