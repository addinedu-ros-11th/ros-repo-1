from setuptools import setup, find_packages

setup(
    name="pinkylib",
    version="1.4",
    packages=find_packages(),
    install_requires=[
        "rpi-lgpio",
        "smbus2",
        "opencv-python",
        "dynamixel_sdk",
    ],
)
