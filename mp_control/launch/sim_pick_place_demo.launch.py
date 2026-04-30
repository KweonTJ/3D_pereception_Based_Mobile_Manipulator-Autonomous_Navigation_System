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
    start_rviz = LaunchConfiguration("start_rviz")
    gz_args = LaunchConfiguration("gz_args")
    world = LaunchConfiguration("world")
    demo_start_delay = LaunchConfiguration("demo_start_delay")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("mp_control"),
                "launch",
                "sim_hybrid_grasp.launch.py",
            ])
        ]),
        launch_arguments={
            "start_rviz": start_rviz,
            "world": world,
            "gz_args": gz_args,
            "start_tracker": "true",
            "start_servo": "false",
            "start_mp_control": "false",
            "control_start_delay": "1.0",
        }.items(),
    )

    demo_node = Node(
        package="mp_control",
        executable="sim_pick_place_demo.py",
        name="sim_pick_place_demo",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"start_delay_s": demo_start_delay},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_gazebo"),
                "worlds",
                "grasp_test.world",
            ]),
            description="Gazebo world containing the red pick object.",
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value=["-r --headless-rendering ", world],
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
        sim_launch,
        TimerAction(period=2.0, actions=[demo_node]),
    ])
