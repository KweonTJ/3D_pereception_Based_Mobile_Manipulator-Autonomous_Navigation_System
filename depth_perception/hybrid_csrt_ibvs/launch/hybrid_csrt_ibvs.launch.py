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
                FindPackageShare("hybrid_csrt_ibvs"),
                "config",
                "turtlebot3_waffle_pi_orbbec.yaml",
            ]),
            description="YAML parameter file for the CSRT + IBVS hybrid node.",
        ),
        Node(
            package="hybrid_csrt_ibvs",
            executable="csrt_ibvs_node",
            name="csrt_ibvs_node",
            output="screen",
            parameters=[config_file],
        ),
    ])
