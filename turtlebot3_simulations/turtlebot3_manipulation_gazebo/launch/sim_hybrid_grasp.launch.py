from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")
    gz_args = LaunchConfiguration("gz_args")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_rviz = LaunchConfiguration("start_rviz")
    start_depth_camera = LaunchConfiguration("start_depth_camera")
    hybrid_config_file = LaunchConfiguration("hybrid_config_file")
    eef_hybrid_config_file = LaunchConfiguration("eef_hybrid_config_file")
    mp_control_config_file = LaunchConfiguration("mp_control_config_file")
    start_tracker = LaunchConfiguration("start_tracker")
    start_eef_tracker = LaunchConfiguration("start_eef_tracker")
    start_eef_ibvs_feature = LaunchConfiguration("start_eef_ibvs_feature")
    eef_ibvs_feature_image_topic = LaunchConfiguration("eef_ibvs_feature_image_topic")
    eef_ibvs_feature_bbox_topic = LaunchConfiguration("eef_ibvs_feature_bbox_topic")
    eef_ibvs_feature_status_topic = LaunchConfiguration("eef_ibvs_feature_status_topic")
    eef_ibvs_feature_color_mode = LaunchConfiguration("eef_ibvs_feature_color_mode")
    eef_ibvs_feature_min_mask_pixels = LaunchConfiguration("eef_ibvs_feature_min_mask_pixels")
    eef_ibvs_feature_min_bbox_width_px = LaunchConfiguration("eef_ibvs_feature_min_bbox_width_px")
    eef_ibvs_feature_min_bbox_height_px = LaunchConfiguration("eef_ibvs_feature_min_bbox_height_px")
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
        condition=IfCondition(start_gazebo),
    )

    rviz_only_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_description"),
                "launch",
                "model.launch.py",
            ])
        ]),
        launch_arguments={
            "start_rviz": start_rviz,
            "use_gui": "false",
        }.items(),
        condition=UnlessCondition(start_gazebo),
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
            "eef_hybrid_config_file": eef_hybrid_config_file,
            "mp_control_config_file": mp_control_config_file,
            "start_tracker": start_tracker,
            "start_eef_tracker": start_eef_tracker,
            "start_servo": start_servo,
            "start_mp_control": start_mp_control,
        }.items(),
    )

    eef_ibvs_feature_node = Node(
        package="mp_control",
        executable="auto_init_bbox.py",
        name="eef_ibvs_feature",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "image_topic": eef_ibvs_feature_image_topic,
            "bbox_topic": eef_ibvs_feature_bbox_topic,
            "status_topic": eef_ibvs_feature_status_topic,
            "color_mode": eef_ibvs_feature_color_mode,
            "min_mask_pixels": ParameterValue(eef_ibvs_feature_min_mask_pixels, value_type=int),
            "min_bbox_width_px": ParameterValue(eef_ibvs_feature_min_bbox_width_px, value_type=float),
            "min_bbox_height_px": ParameterValue(eef_ibvs_feature_min_bbox_height_px, value_type=float),
            "max_bbox_area_ratio": 0.65,
            "min_bbox_aspect_ratio": 0.15,
            "max_bbox_aspect_ratio": 6.0,
            "continuous_publish": True,
            "continuous_publish_period_s": 0.2,
            "timeout_s": 0.0,
        }],
        condition=IfCondition(start_eef_ibvs_feature),
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
            "eef_ibvs_feature_image_topic",
            default_value="/eef_camera/image_raw",
            description="EEF image topic used for visual-feature bbox detection.",
        ),
        DeclareLaunchArgument(
            "eef_ibvs_feature_bbox_topic",
            default_value="/target/eef_ibvs_bbox",
            description="Continuous EEF visual-feature bbox topic consumed by mp_control.",
        ),
        DeclareLaunchArgument(
            "eef_ibvs_feature_status_topic",
            default_value="/target/eef_ibvs_feature_status",
            description="Status topic for EEF visual-feature bbox detection.",
        ),
        DeclareLaunchArgument(
            "eef_ibvs_feature_color_mode",
            default_value="auto",
            description="Visual feature mode for the EEF RGB camera.",
        ),
        DeclareLaunchArgument(
            "eef_ibvs_feature_min_mask_pixels",
            default_value="80",
            description="Minimum colored pixel count for EEF visual-feature detection.",
        ),
        DeclareLaunchArgument(
            "eef_ibvs_feature_min_bbox_width_px",
            default_value="6.0",
            description="Minimum EEF visual-feature bbox width in pixels.",
        ),
        DeclareLaunchArgument(
            "eef_ibvs_feature_min_bbox_height_px",
            default_value="6.0",
            description="Minimum EEF visual-feature bbox height in pixels.",
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
        rviz_only_launch,
        gazebo_launch,
        TimerAction(
            period=control_start_delay,
            actions=[grasp_stack_launch, eef_ibvs_feature_node],
            condition=IfCondition(start_gazebo),
        ),
    ])
