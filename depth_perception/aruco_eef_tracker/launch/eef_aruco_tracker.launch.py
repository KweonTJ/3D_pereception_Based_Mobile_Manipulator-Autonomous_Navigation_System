from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("aruco_eef_tracker"),
                "config",
                "eef_aruco_tracker.yaml",
            ]),
        ),
        Node(
            package="aruco_eef_tracker",
            executable="aruco_eef_tracker_node",
            name="aruco_eef_tracker_node",
            output="screen",
            parameters=[config_file],
        ),
    ])