#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix of the joint and link names'),

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation.'),

        DeclareLaunchArgument(
            'start_depth_camera',
            default_value='true',
            description='Whether bridge simulated depth camera topics'),

        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Ignition Gazebo 6 world file or resource name'),

        DeclareLaunchArgument(
            'x_pose',
            default_value='-2.00',
            description='position of turtlebot3'),

        DeclareLaunchArgument(
            'y_pose',
            default_value='-0.50',
            description='position of turtlebot3'),

        DeclareLaunchArgument(
            'z_pose',
            default_value='0.01',
            description='position of turtlebot3'),

        DeclareLaunchArgument(
            'roll',
            default_value='0.00',
            description='orientation roll of turtlebot3'),

        DeclareLaunchArgument(
            'pitch',
            default_value='0.00',
            description='orientation pitch of turtlebot3'),

        DeclareLaunchArgument(
            'yaw',
            default_value='0.00',
            description='orientation yaw of turtlebot3'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_manipulation_gazebo'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'start_rviz': 'true',
                'start_depth_camera': LaunchConfiguration('start_depth_camera'),
                'prefix': LaunchConfiguration('prefix'),
                'use_sim': LaunchConfiguration('use_sim'),
                'world': LaunchConfiguration('world'),
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'z_pose': LaunchConfiguration('z_pose'),
                'roll': LaunchConfiguration('roll'),
                'pitch': LaunchConfiguration('pitch'),
                'yaw': LaunchConfiguration('yaw'),
            }.items(),
        ),
    ])
