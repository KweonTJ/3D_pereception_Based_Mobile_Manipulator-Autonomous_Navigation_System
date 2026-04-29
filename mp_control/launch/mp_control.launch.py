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
                FindPackageShare("mp_control"),
                "config",
                "mp_control_params.yaml",
            ]),
            description="YAML parameter file for camera-guided manipulator grasp control.",
        ),
        Node(
            package="mp_control",
            executable="mp_control_node",
            name="mp_control_node",
            output="screen",
            parameters=[config_file],
        ),
    ])
