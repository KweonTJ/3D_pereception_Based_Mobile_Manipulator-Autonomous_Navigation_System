from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hybrid_config_file = LaunchConfiguration("hybrid_config_file")
    mp_control_config_file = LaunchConfiguration("mp_control_config_file")
    use_sim = LaunchConfiguration("use_sim")
    start_tracker = LaunchConfiguration("start_tracker")
    start_servo = LaunchConfiguration("start_servo")
    start_mp_control = LaunchConfiguration("start_mp_control")

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_moveit_config"),
                "launch",
                "servo.launch.py",
            ])
        ]),
        launch_arguments={"use_sim": use_sim}.items(),
    )

    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("hybrid_csrt_ibvs"),
                "launch",
                "hybrid_csrt_ibvs.launch.py",
            ])
        ]),
        launch_arguments={"config_file": hybrid_config_file}.items(),
        condition=IfCondition(start_tracker),
    )

    mp_control_node = Node(
        package="mp_control",
        executable="mp_control_node",
        name="mp_control_node",
        output="screen",
        parameters=[mp_control_config_file],
        condition=IfCondition(start_mp_control),
    )

    start_servo_call = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    FindExecutable(name="ros2"),
                    "service",
                    "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
                condition=IfCondition(start_servo),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "hybrid_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("hybrid_csrt_ibvs"),
                "config",
                "turtlebot3_waffle_pi_orbbec.yaml",
            ]),
            description="YAML parameter file for hybrid_csrt_ibvs.",
        ),
        DeclareLaunchArgument(
            "mp_control_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mp_control"),
                "config",
                "mp_control_params.yaml",
            ]),
            description="YAML parameter file for mp_control.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Pass through to MoveIt Servo.",
        ),
        DeclareLaunchArgument(
            "start_tracker",
            default_value="true",
            description="Launch hybrid_csrt_ibvs tracker.",
        ),
        DeclareLaunchArgument(
            "start_servo",
            default_value="true",
            description="Call /servo_node/start_servo after Servo starts.",
        ),
        DeclareLaunchArgument(
            "start_mp_control",
            default_value="true",
            description="Launch mp_control node.",
        ),
        servo_launch,
        tracker_launch,
        mp_control_node,
        start_servo_call,
    ])
