from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_eef_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ktj',
    maintainer_email='kweontj0701@naver.com',
    description='EEF camera ArUco marker pose tracker for manipulator grasping.',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "aruco_eef_tracker_node = aruco_eef_tracker.aruco_eef_tracker_node:main",
        ],
    },
)
