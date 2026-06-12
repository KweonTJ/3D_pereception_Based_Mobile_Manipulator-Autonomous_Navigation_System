from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'turtlebot3_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KweonTJ',
    maintainer_email='kweontj0701@naver.com',
    description='Integrated TurtleBot3 leader rover and ArUco pick-place control.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_mux_node = turtlebot3_control.cmd_vel_mux_node:main',
            'leader_pick_coordinator_node = turtlebot3_control.leader_pick_coordinator_node:main',
        ],
    },
)
