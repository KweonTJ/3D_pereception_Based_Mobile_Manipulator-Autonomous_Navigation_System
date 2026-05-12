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
                "grasp_test.world",
            ]),
            description="Gazebo world containing a simple grasp target.",
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value=["-r --headless-rendering ", LaunchConfiguration("world")],
            description="Arguments passed to Gazebo Sim.",
        ),
        DeclareLaunchArgument(
            "start_gazebo",
            default_value="false",
            description="Start Gazebo simulation. If false, launch RViz model visualization only.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz with the Gazebo robot description.",
        ),
        DeclareLaunchArgument(
            "start_depth_camera",
            default_value="true",
            description="Bridge simulated RGB-D and end-effector camera topics.",
        ),
        DeclareLaunchArgument(
            "hybrid_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("hybrid_csrt_ibvs"),
                "config",
                "turtlebot3_waffle_pi_orbbec_sim.yaml",
            ]),
            description="Simulation parameter file for the base RGB-D tracker.",
        ),
        DeclareLaunchArgument(
            "eef_hybrid_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("hybrid_csrt_ibvs"),
                "config",
                "eef_usb_camera_sim.yaml",
            ]),
            description="Simulation parameter file for the end-effector camera tracker.",
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
            description="Launch the base RGB-D tracker.",
        ),
        DeclareLaunchArgument(
            "start_eef_tracker",
            default_value="false",
            description="Launch the legacy end-effector CSRT tracker. Keep false for EEF IBVS feature correction.",
        ),
        DeclareLaunchArgument(
            "start_eef_ibvs_feature",
            default_value="true",
            description="Publish an EEF visual-feature bbox for near-field IBVS correction.",
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
            description="Seconds to wait before starting Servo, trackers, and mp_control.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("turtlebot3_manipulation_gazebo"),
                    "launch",
                    "sim_hybrid_grasp.launch.py",
                ])
            ]),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                "gz_args": LaunchConfiguration("gz_args"),
                "start_gazebo": LaunchConfiguration("start_gazebo"),
                "start_rviz": LaunchConfiguration("start_rviz"),
                "start_depth_camera": LaunchConfiguration("start_depth_camera"),
                "hybrid_config_file": LaunchConfiguration("hybrid_config_file"),
                "eef_hybrid_config_file": LaunchConfiguration("eef_hybrid_config_file"),
                "mp_control_config_file": LaunchConfiguration("mp_control_config_file"),
                "start_tracker": LaunchConfiguration("start_tracker"),
                "start_eef_tracker": LaunchConfiguration("start_eef_tracker"),
                "start_eef_ibvs_feature": LaunchConfiguration("start_eef_ibvs_feature"),
                "start_servo": LaunchConfiguration("start_servo"),
                "start_mp_control": LaunchConfiguration("start_mp_control"),
                "control_start_delay": LaunchConfiguration("control_start_delay"),
            }.items(),
        ),
    ])
