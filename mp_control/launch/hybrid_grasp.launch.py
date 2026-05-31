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
    eef_hybrid_config_file = LaunchConfiguration("eef_hybrid_config_file")
    mp_control_config_file = LaunchConfiguration("mp_control_config_file")
    use_sim = LaunchConfiguration("use_sim")
    start_tracker = LaunchConfiguration("start_tracker")
    start_eef_tracker = LaunchConfiguration("start_eef_tracker")
    start_servo = LaunchConfiguration("start_servo")
    servo_command_out_topic = LaunchConfiguration("servo_command_out_topic")
    start_joint_trajectory_transformer = LaunchConfiguration("start_joint_trajectory_transformer")
    joint_trajectory_raw_topic = LaunchConfiguration("joint_trajectory_raw_topic")
    joint_trajectory_output_topic = LaunchConfiguration("joint_trajectory_output_topic")
    start_mp_control = LaunchConfiguration("start_mp_control")

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_moveit_config"),
                "launch",
                "servo.launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim": use_sim,
            "command_out_topic": servo_command_out_topic,
        }.items(),
    )

    joint_trajectory_transformer_node = Node(
        package="mp_control",
        executable="joint_trajectory_transformer.py",
        name="joint_trajectory_transformer",
        output="screen",
        parameters=[{
            "input_topic": joint_trajectory_raw_topic,
            "output_topic": joint_trajectory_output_topic,
            "joint_state_topic": "/joint_states",
            "reverse_joint_names": ["joint3"],
            "preserve_roll_sum": False,
            "roll_parent_joint": "joint2",
            "roll_reversed_joint": "joint3",
            "roll_compensation_joint": "joint4",
        }],
        condition=IfCondition(start_joint_trajectory_transformer),
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

    eef_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("hybrid_csrt_ibvs"),
                "launch",
                "hybrid_csrt_ibvs.launch.py",
            ])
        ]),
        launch_arguments={
            "node_name": "eef_csrt_ibvs_node",
            "config_file": eef_hybrid_config_file,
        }.items(),
        condition=IfCondition(start_eef_tracker),
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
            "eef_hybrid_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("hybrid_csrt_ibvs"),
                "config",
                "eef_usb_camera.yaml",
            ]),
            description="YAML parameter file for the end-effector camera tracker.",
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
            "start_eef_tracker",
            default_value="false",
            description="Launch a second tracker for the end-effector USB camera.",
        ),
        DeclareLaunchArgument(
            "start_servo",
            default_value="true",
            description="Call /servo_node/start_servo after Servo starts.",
        ),
        DeclareLaunchArgument(
            "servo_command_out_topic",
            default_value="/arm_controller/joint_trajectory",
            description="JointTrajectory topic published by MoveIt Servo.",
        ),
        DeclareLaunchArgument(
            "start_joint_trajectory_transformer",
            default_value="false",
            description="Mirror selected joint deltas before the real arm controller.",
        ),
        DeclareLaunchArgument(
            "joint_trajectory_raw_topic",
            default_value="/arm_controller/joint_trajectory_raw",
            description="Raw JointTrajectory topic consumed by the transformer.",
        ),
        DeclareLaunchArgument(
            "joint_trajectory_output_topic",
            default_value="/arm_controller/joint_trajectory",
            description="Controller JointTrajectory topic published by the transformer.",
        ),
        DeclareLaunchArgument(
            "start_mp_control",
            default_value="true",
            description="Launch mp_control node.",
        ),
        joint_trajectory_transformer_node,
        servo_launch,
        tracker_launch,
        eef_tracker_launch,
        mp_control_node,
        start_servo_call,
    ])
