# Copyright 2026 ktj
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_camera = LaunchConfiguration("start_camera")
    start_eef_camera_driver = LaunchConfiguration("start_eef_camera_driver")
    start_lidar = LaunchConfiguration("start_lidar")
    start_rviz = LaunchConfiguration("start_rviz")
    move_to_stay_pose = LaunchConfiguration("move_to_stay_pose")
    start_state_relays = LaunchConfiguration("start_state_relays")
    start_leader_task_manager = LaunchConfiguration("start_leader_task_manager")
    start_leader_beacon = LaunchConfiguration("start_leader_beacon")
    start_domain_bridge = LaunchConfiguration("start_domain_bridge")
    start_monitor_uploader = LaunchConfiguration("start_monitor_uploader")
    monitor_server = LaunchConfiguration("monitor_server")
    monitor_token = LaunchConfiguration("monitor_token")
    monitor_video_enabled = LaunchConfiguration("monitor_video_enabled")
    monitor_status_period = LaunchConfiguration("monitor_status_period")
    monitor_video_period = LaunchConfiguration("monitor_video_period")
    monitor_jpeg_quality = LaunchConfiguration("monitor_jpeg_quality")
    monitor_image_width = LaunchConfiguration("monitor_image_width")
    monitor_image_height = LaunchConfiguration("monitor_image_height")
    monitor_http_timeout = LaunchConfiguration("monitor_http_timeout")

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_bringup"),
                "launch",
                "hardware.launch.py",
            ])
        ]),
        launch_arguments={
            "start_rviz": start_rviz,
            "start_camera": start_camera,
            "start_eef_camera_driver": start_eef_camera_driver,
            "start_lidar": start_lidar,
            "move_to_stay_pose": move_to_stay_pose,
            "start_state_relays": start_state_relays,
        }.items(),
    )

    leader_task_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("leader_task_manager"),
                "launch",
                "leader_task_manager.launch.py",
            ])
        ]),
        condition=IfCondition(start_leader_task_manager),
    )

    leader_beacon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("leader_platooning_beacon"),
                "launch",
                "leader_platooning_beacon.launch.py",
            ])
        ]),
        condition=IfCondition(start_leader_beacon),
    )

    domain_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("platooning_bridge_config"),
                "launch",
                "bridge.launch.py",
            ])
        ]),
        condition=IfCondition(start_domain_bridge),
    )

    monitor_uploader_node = Node(
        package="leader_platooning_beacon",
        executable="robot_status_uploader.py",
        name="leader_status_uploader",
        output="screen",
        arguments=[
            "--robot", "leader",
            "--server", monitor_server,
            "--token", monitor_token,
            "--status-period", monitor_status_period,
            "--video-period", monitor_video_period,
            "--jpeg-quality", monitor_jpeg_quality,
            "--image-width", monitor_image_width,
            "--image-height", monitor_image_height,
            "--http-timeout", monitor_http_timeout,
            "--video-enabled", monitor_video_enabled,
        ],
        condition=IfCondition(start_monitor_uploader),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_camera",
            default_value="false",
            description="Whether to start the front Astra camera for teleop monitoring.",
        ),
        DeclareLaunchArgument(
            "start_eef_camera_driver",
            default_value="false",
            description="Whether to start the end-effector USB camera for teleop monitoring.",
        ),
        DeclareLaunchArgument(
            "start_lidar",
            default_value="false",
            description="Whether to start a lidar driver.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Whether to start RViz.",
        ),
        DeclareLaunchArgument(
            "move_to_stay_pose",
            default_value="true",
            description="Move manipulator to stay pose after hardware starts.",
        ),
        DeclareLaunchArgument(
            "start_state_relays",
            default_value="true",
            description="Publish /battery_state and /sensor_state from /dynamic_joint_states.",
        ),
        DeclareLaunchArgument(
            "start_leader_task_manager",
            default_value="true",
            description="Publish /leader/task_state, cargo_state, follower_enable, and platoon_mode.",
        ),
        DeclareLaunchArgument(
            "start_leader_beacon",
            default_value="true",
            description="Relay /odom and /cmd_vel to /leader/odom and /leader/cmd_vel.",
        ),
        DeclareLaunchArgument(
            "start_domain_bridge",
            default_value="true",
            description="Bridge /leader/* topics from leader domain to follower domain.",
        ),
        DeclareLaunchArgument(
            "start_monitor_uploader",
            default_value="true",
            description="Start upload-only monitor status uploader.",
        ),
        DeclareLaunchArgument(
            "monitor_server",
            default_value=EnvironmentVariable(
                "MONITOR_SERVER_URL",
                default_value="http://192.168.0.13:8000",
            ),
        ),
        DeclareLaunchArgument(
            "monitor_token",
            default_value=EnvironmentVariable("MONITOR_TOKEN", default_value=""),
        ),
        DeclareLaunchArgument("monitor_video_enabled", default_value="false"),
        DeclareLaunchArgument("monitor_status_period", default_value="0.2"),
        DeclareLaunchArgument("monitor_video_period", default_value="0.5"),
        DeclareLaunchArgument("monitor_jpeg_quality", default_value="50"),
        DeclareLaunchArgument("monitor_image_width", default_value="424"),
        DeclareLaunchArgument("monitor_image_height", default_value="318"),
        DeclareLaunchArgument("monitor_http_timeout", default_value="1.0"),
        hardware_launch,
        leader_task_manager_launch,
        leader_beacon_launch,
        domain_bridge_launch,
        monitor_uploader_node,
    ])
