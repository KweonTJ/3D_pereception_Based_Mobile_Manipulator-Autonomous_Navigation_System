from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rviz = LaunchConfiguration("start_rviz")
    start_camera = LaunchConfiguration("start_camera")
    start_lidar = LaunchConfiguration("start_lidar")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    move_to_stay_pose = LaunchConfiguration("move_to_stay_pose")
    use_camera_driver_tf = LaunchConfiguration("use_camera_driver_tf")
    use_eef_usb_camera = LaunchConfiguration("use_eef_usb_camera")
    eef_usb_camera_parent = LaunchConfiguration("eef_usb_camera_parent")
    eef_usb_camera_xyz = LaunchConfiguration("eef_usb_camera_xyz")
    eef_usb_camera_rpy = LaunchConfiguration("eef_usb_camera_rpy")

    hybrid_config_file = LaunchConfiguration("hybrid_config_file")
    eef_hybrid_config_file = LaunchConfiguration("eef_hybrid_config_file")
    mp_control_config_file = LaunchConfiguration("mp_control_config_file")
    start_tracker = LaunchConfiguration("start_tracker")
    start_eef_tracker = LaunchConfiguration("start_eef_tracker")
    start_servo = LaunchConfiguration("start_servo")
    start_mp_control = LaunchConfiguration("start_mp_control")
    control_start_delay = LaunchConfiguration("control_start_delay")

    start_leader_task_manager = LaunchConfiguration("start_leader_task_manager")
    start_leader_beacon = LaunchConfiguration("start_leader_beacon")
    start_domain_bridge = LaunchConfiguration("start_domain_bridge")

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_bringup"),
                "launch",
                "hardware.launch.py",
            ])
        ]),
        launch_arguments={
            "start_rviz": start_rviz,
            "start_camera": start_camera,
            "start_lidar": start_lidar,
            "lidar_port": lidar_port,
            "lidar_frame_id": lidar_frame_id,
            "move_to_stay_pose": move_to_stay_pose,
            "use_camera_driver_tf": use_camera_driver_tf,
            "use_eef_usb_camera": use_eef_usb_camera,
            "eef_usb_camera_parent": eef_usb_camera_parent,
            "eef_usb_camera_xyz": eef_usb_camera_xyz,
            "eef_usb_camera_rpy": eef_usb_camera_rpy,
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
            "use_sim": "false",
            "hybrid_config_file": hybrid_config_file,
            "eef_hybrid_config_file": eef_hybrid_config_file,
            "mp_control_config_file": mp_control_config_file,
            "start_tracker": start_tracker,
            "start_eef_tracker": start_eef_tracker,
            "start_servo": start_servo,
            "start_mp_control": start_mp_control,
        }.items(),
    )

    leader_task_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("leader_task_manager"),
                "launch",
                "leader_task_manager.launch.py",
            ])
        ]),
        condition=IfCondition(start_leader_task_manager),
    )

    leader_beacon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("leader_platooning_beacon"),
                "launch",
                "leader_platooning_beacon.launch.py",
            ])
        ]),
        condition=IfCondition(start_leader_beacon),
    )

    domain_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("platooning_bridge_config"),
                "launch",
                "bridge.launch.py",
            ])
        ]),
        condition=IfCondition(start_domain_bridge),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Start RViz on the robot computer.",
        ),
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Start the Astra Mini camera driver.",
        ),
        DeclareLaunchArgument(
            "start_lidar",
            default_value="false",
            description="Start the TurtleBot3 lidar driver.",
        ),
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyUSB0",
            description="Connected USB port for the lidar.",
        ),
        DeclareLaunchArgument(
            "lidar_frame_id",
            default_value="base_scan",
            description="Frame id used by the lidar driver.",
        ),
        DeclareLaunchArgument(
            "move_to_stay_pose",
            default_value="true",
            description="Move the manipulator to the saved stay pose after startup.",
        ),
        DeclareLaunchArgument(
            "use_camera_driver_tf",
            default_value="true",
            description="Let the camera driver publish camera internal TF frames.",
        ),
        DeclareLaunchArgument(
            "use_eef_usb_camera",
            default_value="true",
            description="Attach the end-effector USB camera frames to robot_description.",
        ),
        DeclareLaunchArgument(
            "eef_usb_camera_parent",
            default_value="dummy_mimic_fix",
            description="Parent link for the end-effector USB camera frame.",
        ),
        DeclareLaunchArgument(
            "eef_usb_camera_xyz",
            default_value="0.02 0.0 0.065",
            description="End-effector USB camera translation relative to parent.",
        ),
        DeclareLaunchArgument(
            "eef_usb_camera_rpy",
            default_value="0.0 0.0 0.0",
            description="End-effector USB camera rotation relative to parent.",
        ),
        DeclareLaunchArgument(
            "hybrid_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("hybrid_csrt_ibvs"),
                "config",
                "turtlebot3_waffle_pi_orbbec.yaml",
            ]),
            description="Real robot parameter file for the base RGB-D tracker.",
        ),
        DeclareLaunchArgument(
            "eef_hybrid_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("hybrid_csrt_ibvs"),
                "config",
                "eef_usb_camera.yaml",
            ]),
            description="Real robot parameter file for the end-effector camera tracker.",
        ),
        DeclareLaunchArgument(
            "mp_control_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mp_control"),
                "config",
                "mp_control_params.yaml",
            ]),
            description="Real robot parameter file for mp_control.",
        ),
        DeclareLaunchArgument(
            "start_tracker",
            default_value="true",
            description="Launch the base RGB-D object tracker.",
        ),
        DeclareLaunchArgument(
            "start_eef_tracker",
            default_value="false",
            description="Launch the optional end-effector camera tracker.",
        ),
        DeclareLaunchArgument(
            "start_servo",
            default_value="true",
            description="Start MoveIt Servo after the hardware stack is ready.",
        ),
        DeclareLaunchArgument(
            "start_mp_control",
            default_value="true",
            description="Launch mp_control.",
        ),
        DeclareLaunchArgument(
            "control_start_delay",
            default_value="8.0",
            description="Seconds to wait before starting Servo, trackers, and mp_control.",
        ),
        DeclareLaunchArgument(
            "start_leader_task_manager",
            default_value="true",
            description="Publish leader task/cargo/platoon state.",
        ),
        DeclareLaunchArgument(
            "start_leader_beacon",
            default_value="true",
            description="Publish leader heartbeat.",
        ),
        DeclareLaunchArgument(
            "start_domain_bridge",
            default_value="false",
            description="Start the optional leader-to-follower domain bridge.",
        ),
        hardware_launch,
        TimerAction(
            period=control_start_delay,
            actions=[
                grasp_stack_launch,
                leader_task_manager_launch,
                leader_beacon_launch,
                domain_bridge_launch,
            ],
        ),
    ])
