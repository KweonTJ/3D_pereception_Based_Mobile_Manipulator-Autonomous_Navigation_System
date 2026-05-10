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
                "grasp_10m_room.world",
            ]),
            description="Gazebo 10 m x 10 m room world containing the far red pick object.",
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
            default_value="true",
            description="Return the arm to the stow pose after placing the object.",
        ),
        DeclareLaunchArgument(
            "base_approach_distance",
            default_value="0.28",
            description="Meters to drive before grasping. Matches the hybrid depth simulation desired depth.",
        ),
        DeclareLaunchArgument(
            "base_approach_speed",
            default_value="0.12",
            description="Base linear speed in m/s during the approach stage.",
        ),
        DeclareLaunchArgument(
            "base_transport_distance",
            default_value="1.00",
            description="Meters to drive after grasping and before placing.",
        ),
        DeclareLaunchArgument(
            "base_transport_speed",
            default_value="0.12",
            description="Base linear speed in m/s during the transport stage.",
        ),
        DeclareLaunchArgument(
            "base_turn_angle",
            default_value="1.5708",
            description="Radians to rotate after picking before transport. Positive turns left; 0 keeps the old straight path.",
        ),
        DeclareLaunchArgument(
            "base_turn_speed",
            default_value="0.45",
            description="Base angular speed in rad/s during the post-pick turn.",
        ),
        DeclareLaunchArgument(
            "post_place_move_distance",
            default_value="1.00",
            description="Meters to drive forward after placing, reversing, and turning.",
        ),
        DeclareLaunchArgument(
            "post_place_move_speed",
            default_value="0.12",
            description="Base forward speed in m/s after placing, reversing, and turning.",
        ),
        DeclareLaunchArgument(
            "post_place_reverse_distance",
            default_value="0.35",
            description="Meters to back up after placing cargo on the follower.",
        ),
        DeclareLaunchArgument(
            "post_place_reverse_speed",
            default_value="0.10",
            description="Base reverse speed in m/s after placing cargo on the follower.",
        ),
        DeclareLaunchArgument(
            "post_place_turn_angle",
            default_value="1.5708",
            description="Radians to rotate after the post-place reverse.",
        ),
        DeclareLaunchArgument(
            "post_place_turn_speed",
            default_value="0.45",
            description="Base angular speed in rad/s during the post-place turn.",
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
                "base_approach_distance": LaunchConfiguration("base_approach_distance"),
                "base_approach_speed": LaunchConfiguration("base_approach_speed"),
                "base_transport_distance": LaunchConfiguration("base_transport_distance"),
                "base_transport_speed": LaunchConfiguration("base_transport_speed"),
                "base_turn_angle": LaunchConfiguration("base_turn_angle"),
                "base_turn_speed": LaunchConfiguration("base_turn_speed"),
                "post_place_move_distance": LaunchConfiguration("post_place_move_distance"),
                "post_place_move_speed": LaunchConfiguration("post_place_move_speed"),
                "post_place_reverse_distance": LaunchConfiguration("post_place_reverse_distance"),
                "post_place_reverse_speed": LaunchConfiguration("post_place_reverse_speed"),
                "post_place_turn_angle": LaunchConfiguration("post_place_turn_angle"),
                "post_place_turn_speed": LaunchConfiguration("post_place_turn_speed"),
            }.items(),
        ),
    ])
