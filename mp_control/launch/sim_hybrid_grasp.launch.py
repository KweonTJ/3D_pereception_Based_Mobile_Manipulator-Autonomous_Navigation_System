from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")
    gz_args = LaunchConfiguration("gz_args")
    start_rviz = LaunchConfiguration("start_rviz")
    start_depth_camera = LaunchConfiguration("start_depth_camera")
    hybrid_config_file = LaunchConfiguration("hybrid_config_file")
    mp_control_config_file = LaunchConfiguration("mp_control_config_file")
    start_tracker = LaunchConfiguration("start_tracker")
    start_eef_tracker = LaunchConfiguration("start_eef_tracker")
    start_servo = LaunchConfiguration("start_servo")
    start_mp_control = LaunchConfiguration("start_mp_control")
    control_start_delay = LaunchConfiguration("control_start_delay")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_gazebo"),
                "launch",
                "gazebo.launch.py",
            ])
        ]),
        launch_arguments={
            "start_rviz": start_rviz,
            "start_depth_camera": start_depth_camera,
            "use_sim": "true",
            "world": world,
            "gz_args": gz_args,
            "x_pose": "-2.00",
            "y_pose": "-0.50",
            "z_pose": "0.01",
            "roll": "0.00",
            "pitch": "0.00",
            "yaw": "0.00",
        }.items(),
    )

    grasp_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("mp_control"),
                "launch",
                "hybrid_grasp.launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim": "true",
            "hybrid_config_file": hybrid_config_file,
            "mp_control_config_file": mp_control_config_file,
            "start_tracker": start_tracker,
            "start_eef_tracker": start_eef_tracker,
            "start_servo": start_servo,
            "start_mp_control": start_mp_control,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_gazebo"),
                "worlds",
                "grasp_test.world",
            ]),
            description="Gazebo world containing a simple grasp target.",
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value=["-r --headless-rendering ", world],
            description="Arguments passed to Gazebo Sim. Use '-r -s --headless-rendering <world>' for server-only tests.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz with the Gazebo robot description.",
        ),
        DeclareLaunchArgument(
            "start_depth_camera",
            default_value="true",
            description="Bridge simulated RGB-D camera topics.",
        ),
        DeclareLaunchArgument(
            "hybrid_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("hybrid_csrt_ibvs"),
                "config",
                "turtlebot3_waffle_pi_orbbec_sim.yaml",
            ]),
            description="Simulation parameter file for hybrid_csrt_ibvs.",
        ),
        DeclareLaunchArgument(
            "mp_control_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mp_control"),
                "config",
                "mp_control_sim_params.yaml",
            ]),
            description="Simulation parameter file for mp_control.",
        ),
        DeclareLaunchArgument(
            "start_tracker",
            default_value="true",
            description="Launch the main RGB-D tracker.",
        ),
        DeclareLaunchArgument(
            "start_eef_tracker",
            default_value="false",
            description="Start the end-effector camera tracker. Disabled until a simulated EEF stream is added.",
        ),
        DeclareLaunchArgument(
            "start_servo",
            default_value="true",
            description="Call /servo_node/start_servo after Servo starts.",
        ),
        DeclareLaunchArgument(
            "start_mp_control",
            default_value="true",
            description="Launch mp_control.",
        ),
        DeclareLaunchArgument(
            "control_start_delay",
            default_value="6.0",
            description="Seconds to wait before starting Servo, tracker, and mp_control.",
        ),
        gazebo_launch,
        TimerAction(
            period=control_start_delay,
            actions=[grasp_stack_launch],
        ),
    ])
