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
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_gazebo"),
                "worlds",
                "grasp_far_test.world",
            ]),
            description="Gazebo world containing the far red pick object.",
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value=["-r --headless-rendering ", LaunchConfiguration("world")],
            description="Arguments passed to Gazebo Sim.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz to show robot, object marker, and place marker.",
        ),
        DeclareLaunchArgument(
            "demo_start_delay",
            default_value="10.0",
            description="Seconds to wait before publishing bbox and executing the demo trajectory.",
        ),
        DeclareLaunchArgument(
            "return_to_stow",
            default_value="false",
            description="Return the arm to the stow pose after the demo.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("turtlebot3_manipulation_gazebo"),
                    "launch",
                    "sim_pick_place_demo.launch.py",
                ])
            ]),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                "gz_args": LaunchConfiguration("gz_args"),
                "start_rviz": LaunchConfiguration("start_rviz"),
                "demo_start_delay": LaunchConfiguration("demo_start_delay"),
                "return_to_stow": LaunchConfiguration("return_to_stow"),
            }.items(),
        ),
    ])
