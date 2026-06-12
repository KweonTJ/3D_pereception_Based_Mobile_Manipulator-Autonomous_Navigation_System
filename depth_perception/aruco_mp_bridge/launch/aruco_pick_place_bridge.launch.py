from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mp_control_config_file = LaunchConfiguration("mp_control_config_file")
    aruco_config_file = LaunchConfiguration("aruco_config_file")
    bridge_config_file = LaunchConfiguration("bridge_config_file")
    force_object_x_m = LaunchConfiguration("force_object_x_m")
    control_start_delay = LaunchConfiguration("control_start_delay")
    bridge_start_delay = LaunchConfiguration("bridge_start_delay")

    aruco_pick_place = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("mp_control"),
                "launch",
                "aruco_pick_place.launch.py",
            ])
        ]),
        launch_arguments={
            "start_camera": "false",
            "start_aruco_tracker": "true",
            "start_mp_control": "true",
            "mp_control_config_file": mp_control_config_file,
            "aruco_config_file": aruco_config_file,
            "control_start_delay": control_start_delay,
        }.items(),
    )

    aruco_bridge = Node(
        package="aruco_mp_bridge",
        executable="aruco_to_mp_control_bridge",
        name="aruco_to_mp_control_bridge",
        output="screen",
        parameters=[
            bridge_config_file,
            {
                "force_object_x_m": force_object_x_m,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mp_control_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mp_control"),
                "config",
                "mp_control_aruco_params.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "aruco_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("aruco_eef_tracker"),
                "config",
                "eef_aruco_tracker.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "bridge_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("aruco_mp_bridge"),
                "config",
                "aruco_to_mp_control_bridge.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "force_object_x_m",
            default_value="0.22",
        ),
        DeclareLaunchArgument(
            "control_start_delay",
            default_value="8.0",
        ),
        DeclareLaunchArgument(
            "bridge_start_delay",
            default_value="10.0",
            description="Delay bridge startup until mp_control has created its subscriptions.",
        ),
        aruco_pick_place,
        TimerAction(
            period=bridge_start_delay,
            actions=[aruco_bridge],
        ),
    ])
