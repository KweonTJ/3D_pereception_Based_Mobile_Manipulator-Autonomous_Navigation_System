from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rviz = LaunchConfiguration("start_rviz")
    gz_args = LaunchConfiguration("gz_args")
    world = LaunchConfiguration("world")
    demo_start_delay = LaunchConfiguration("demo_start_delay")
    return_to_stow = LaunchConfiguration("return_to_stow")
    base_approach_distance = LaunchConfiguration("base_approach_distance")
    base_approach_speed = LaunchConfiguration("base_approach_speed")
    base_transport_distance = LaunchConfiguration("base_transport_distance")
    base_transport_speed = LaunchConfiguration("base_transport_speed")
    base_turn_angle = LaunchConfiguration("base_turn_angle")
    base_turn_speed = LaunchConfiguration("base_turn_speed")
    show_follower = LaunchConfiguration("show_follower")
    follower_x = LaunchConfiguration("follower_x")
    follower_y = LaunchConfiguration("follower_y")
    follower_yaw = LaunchConfiguration("follower_yaw")

    follower_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_gazebo"),
                "urdf",
                "turtlebot3_platooning_follower.urdf.xacro",
            ]),
        ]),
        value_type=str,
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_gazebo"),
                "launch",
                "sim_hybrid_grasp.launch.py",
            ])
        ]),
        launch_arguments={
            "start_rviz": start_rviz,
            "start_gazebo": "true",
            "world": world,
            "gz_args": gz_args,
            "start_tracker": "true",
            "start_eef_tracker": "true",
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
            {"return_to_stow": return_to_stow},
            {"base_approach_distance_m": base_approach_distance},
            {"base_approach_speed_mps": base_approach_speed},
            {"base_transport_distance_m": base_transport_distance},
            {"base_transport_speed_mps": base_transport_speed},
            {"base_turn_angle_rad": base_turn_angle},
            {"base_turn_speed_radps": base_turn_speed},
            {"cmd_vel_topic": "/diff_drive_controller/cmd_vel_unstamped"},
            {"cmd_vel_wait_timeout_s": 90.0},
            {"trajectory_wait_timeout_s": 90.0},
            {"gripper_wait_timeout_s": 90.0},
            {"require_cmd_vel_subscriber": True},
            {"require_trajectory_subscriber": True},
            {"require_gripper_action_server": True},
            {"publish_demo_base_tf": False},
            {"publish_demo_joint_states": False},
            {"cargo_id_prefix": "SIM-PKG"},
        ],
    )

    follower_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="follower_robot_state_publisher",
        output="screen",
        condition=IfCondition(show_follower),
        parameters=[
            {"robot_description": follower_description},
            {"use_sim_time": True},
        ],
        remappings=[
            ("robot_description", "/follower/robot_description"),
            ("joint_states", "/follower/joint_states"),
        ],
    )

    follower_joint_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="follower_joint_state_publisher",
        output="screen",
        condition=IfCondition(show_follower),
        parameters=[
            {"robot_description": follower_description},
            {"use_sim_time": True},
        ],
        remappings=[
            ("robot_description", "/follower/robot_description"),
            ("joint_states", "/follower/joint_states"),
        ],
    )

    follower_anchor_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="follower_anchor_tf",
        output="screen",
        condition=IfCondition(show_follower),
        arguments=[
            "--x", follower_x,
            "--y", follower_y,
            "--z", "0.0",
            "--yaw", follower_yaw,
            "--frame-id", "base_footprint",
            "--child-frame-id", "follower_base_footprint",
        ],
    )

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
        DeclareLaunchArgument(
            "return_to_stow",
            default_value="true",
            description="Return the arm to the stow pose after placing the object.",
        ),
        DeclareLaunchArgument(
            "base_approach_distance",
            default_value="0.80",
            description="Meters to drive before grasping. Object pose is generated from the post-drive grasp pose.",
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
            "show_follower",
            default_value="true",
            description="Show a follower TurtleBot3 model in RViz for platooning visualization.",
        ),
        DeclareLaunchArgument(
            "follower_x",
            default_value="-1.00",
            description="Follower visualization x offset from the leader base_footprint frame.",
        ),
        DeclareLaunchArgument(
            "follower_y",
            default_value="0.00",
            description="Follower visualization y offset from the leader base_footprint frame.",
        ),
        DeclareLaunchArgument(
            "follower_yaw",
            default_value="0.00",
            description="Follower visualization yaw offset from the leader base_footprint frame.",
        ),
        follower_state_publisher,
        follower_joint_publisher,
        follower_anchor_tf,
        sim_launch,
        TimerAction(period=2.0, actions=[demo_node]),
    ])
