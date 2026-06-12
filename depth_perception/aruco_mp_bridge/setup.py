from glob import glob
import os

from setuptools import find_packages, setup


package_name = "aruco_mp_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ktj",
    maintainer_email="kweontj0701@naver.com",
    description="Bridge EEF ArUco marker pose into existing mp_control topics.",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "aruco_to_mp_control_bridge = aruco_mp_bridge.aruco_to_mp_control_bridge:main",
        ],
    },
)
