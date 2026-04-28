#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: ChanHyeong Lee

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether execute rviz2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_manipulation_gazebo'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'start_rviz': start_rviz,
                'prefix': '""',
                'use_sim': 'true',
                'world': PathJoinSubstitution([
                    FindPackageShare('turtlebot3_manipulation_gazebo'),
                    'worlds',
                    'turtlebot3_home_service_challenge.world'
                ]),
                'x_pose': '0.0',
                'y_pose': '0.0',
                'z_pose': '0.0',
                'roll': '0.0',
                'pitch': '0.0',
                'yaw': '0.0',
            }.items(),
        ),
    ])
