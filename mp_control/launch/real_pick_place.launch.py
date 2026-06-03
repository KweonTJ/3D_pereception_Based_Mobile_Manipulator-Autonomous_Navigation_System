from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    start_joint_trajectory_transformer = LaunchConfiguration("start_joint_trajectory_transformer")
    joint_trajectory_raw_topic = LaunchConfiguration("joint_trajectory_raw_topic")
    joint_trajectory_output_topic = LaunchConfiguration("joint_trajectory_output_topic")
    start_mp_control = LaunchConfiguration("start_mp_control")
    control_start_delay = LaunchConfiguration("control_start_delay")

    start_leader_task_manager = LaunchConfiguration("start_leader_task_manager")
    start_leader_beacon = LaunchConfiguration("start_leader_beacon")
    start_domain_bridge = LaunchConfiguration("start_domain_bridge")
    start_auto_init_bbox = LaunchConfiguration("start_auto_init_bbox")
    auto_init_bbox_start_delay = LaunchConfiguration("auto_init_bbox_start_delay")
    auto_init_bbox_image_topic = LaunchConfiguration("auto_init_bbox_image_topic")
    auto_init_alternate_image_topics = LaunchConfiguration("auto_init_alternate_image_topics")
    auto_init_bbox_topic = LaunchConfiguration("auto_init_bbox_topic")
    auto_init_tracked_bbox_topic = LaunchConfiguration("auto_init_tracked_bbox_topic")
    auto_init_publish_tracked_bbox = LaunchConfiguration("auto_init_publish_tracked_bbox")
    auto_init_bbox_status_topic = LaunchConfiguration("auto_init_bbox_status_topic")
    auto_init_color_mode = LaunchConfiguration("auto_init_color_mode")
    auto_init_min_mask_pixels = LaunchConfiguration("auto_init_min_mask_pixels")
    auto_init_min_bbox_width_px = LaunchConfiguration("auto_init_min_bbox_width_px")
    auto_init_min_bbox_height_px = LaunchConfiguration("auto_init_min_bbox_height_px")
    auto_init_max_bbox_area_ratio = LaunchConfiguration("auto_init_max_bbox_area_ratio")
    auto_init_min_bbox_aspect_ratio = LaunchConfiguration("auto_init_min_bbox_aspect_ratio")
    auto_init_max_bbox_aspect_ratio = LaunchConfiguration("auto_init_max_bbox_aspect_ratio")
    auto_init_roi_min_x_ratio = LaunchConfiguration("auto_init_roi_min_x_ratio")
    auto_init_roi_max_x_ratio = LaunchConfiguration("auto_init_roi_max_x_ratio")
    auto_init_roi_min_y_ratio = LaunchConfiguration("auto_init_roi_min_y_ratio")
    auto_init_roi_max_y_ratio = LaunchConfiguration("auto_init_roi_max_y_ratio")
    auto_init_timeout_s = LaunchConfiguration("auto_init_timeout_s")
    auto_init_black_max = LaunchConfiguration("auto_init_black_max")
    auto_init_black_min_contrast = LaunchConfiguration("auto_init_black_min_contrast")
    auto_init_red_min = LaunchConfiguration("auto_init_red_min")
    auto_init_red_margin = LaunchConfiguration("auto_init_red_margin")
    auto_init_red_ratio = LaunchConfiguration("auto_init_red_ratio")
    auto_init_depth_min_m = LaunchConfiguration("auto_init_depth_min_m")
    auto_init_depth_max_m = LaunchConfiguration("auto_init_depth_max_m")
    auto_init_depth_near_percentile = LaunchConfiguration("auto_init_depth_near_percentile")
    auto_init_depth_band_m = LaunchConfiguration("auto_init_depth_band_m")
    auto_init_box_depth_band_m = LaunchConfiguration("auto_init_box_depth_band_m")
    auto_init_box_min_fill_ratio = LaunchConfiguration("auto_init_box_min_fill_ratio")
    auto_init_box_max_depth_std_m = LaunchConfiguration("auto_init_box_max_depth_std_m")
    auto_init_box_max_center_distance_ratio = LaunchConfiguration(
        "auto_init_box_max_center_distance_ratio")
    auto_init_box_center_weight = LaunchConfiguration("auto_init_box_center_weight")
    auto_init_box_area_weight = LaunchConfiguration("auto_init_box_area_weight")
    auto_init_box_depth_weight = LaunchConfiguration("auto_init_box_depth_weight")
    auto_init_yolo_model_path = LaunchConfiguration("auto_init_yolo_model_path")
    auto_init_yolo_confidence = LaunchConfiguration("auto_init_yolo_confidence")
    auto_init_yolo_imgsz = LaunchConfiguration("auto_init_yolo_imgsz")
    auto_init_yolo_class_name = LaunchConfiguration("auto_init_yolo_class_name")
    auto_init_yolo_max_detections = LaunchConfiguration("auto_init_yolo_max_detections")
    start_auto_eef_init_bbox = LaunchConfiguration("start_auto_eef_init_bbox")
    auto_eef_init_bbox_start_delay = LaunchConfiguration("auto_eef_init_bbox_start_delay")
    auto_eef_init_bbox_image_topic = LaunchConfiguration("auto_eef_init_bbox_image_topic")
    auto_eef_init_bbox_topic = LaunchConfiguration("auto_eef_init_bbox_topic")
    auto_eef_init_bbox_status_topic = LaunchConfiguration("auto_eef_init_bbox_status_topic")
    auto_eef_init_color_mode = LaunchConfiguration("auto_eef_init_color_mode")
    auto_eef_init_min_mask_pixels = LaunchConfiguration("auto_eef_init_min_mask_pixels")
    auto_eef_init_min_bbox_width_px = LaunchConfiguration("auto_eef_init_min_bbox_width_px")
    auto_eef_init_min_bbox_height_px = LaunchConfiguration("auto_eef_init_min_bbox_height_px")
    auto_eef_init_roi_min_x_ratio = LaunchConfiguration("auto_eef_init_roi_min_x_ratio")
    auto_eef_init_roi_max_x_ratio = LaunchConfiguration("auto_eef_init_roi_max_x_ratio")
    auto_eef_init_roi_min_y_ratio = LaunchConfiguration("auto_eef_init_roi_min_y_ratio")
    auto_eef_init_roi_max_y_ratio = LaunchConfiguration("auto_eef_init_roi_max_y_ratio")
    start_calibrated_camera_info = LaunchConfiguration("start_calibrated_camera_info")
    calibrated_camera_info_json_path = LaunchConfiguration("calibrated_camera_info_json_path")
    calibrated_camera_image_topic = LaunchConfiguration("calibrated_camera_image_topic")
    calibrated_camera_info_topic = LaunchConfiguration("calibrated_camera_info_topic")
    start_eef_camera_driver = LaunchConfiguration("start_eef_camera_driver")
    eef_camera_video_device = LaunchConfiguration("eef_camera_video_device")
    eef_camera_frame_id = LaunchConfiguration("eef_camera_frame_id")
    eef_camera_pixel_format = LaunchConfiguration("eef_camera_pixel_format")
    eef_camera_output_encoding = LaunchConfiguration("eef_camera_output_encoding")
    eef_camera_image_width = LaunchConfiguration("eef_camera_image_width")
    eef_camera_image_height = LaunchConfiguration("eef_camera_image_height")
    eef_camera_name = LaunchConfiguration("eef_camera_name")
    eef_camera_info_url = LaunchConfiguration("eef_camera_info_url")
    start_monitor_uploader = LaunchConfiguration("start_monitor_uploader")
    monitor_server = LaunchConfiguration("monitor_server")
    monitor_token = LaunchConfiguration("monitor_token")
    monitor_video_enabled = LaunchConfiguration("monitor_video_enabled")
    monitor_status_period = LaunchConfiguration("monitor_status_period")
    monitor_video_period = LaunchConfiguration("monitor_video_period")
    monitor_jpeg_quality = LaunchConfiguration("monitor_jpeg_quality")
    monitor_image_width = LaunchConfiguration("monitor_image_width")
    monitor_image_height = LaunchConfiguration("monitor_image_height")
    monitor_http_timeout = LaunchConfiguration("monitor_http_timeout")

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_bringup"),
                "launch",
                "hardware.launch.py",
            ])
        ]),
        launch_arguments={
            "start_rviz": "false",
            "start_camera": start_camera,
            "start_lidar": "false",
            "lidar_port": lidar_port,
            "lidar_frame_id": lidar_frame_id,
            "move_to_stay_pose": move_to_stay_pose,
            "use_camera_driver_tf": use_camera_driver_tf,
            "use_eef_usb_camera": use_eef_usb_camera,
            "eef_usb_camera_parent": eef_usb_camera_parent,
            "eef_usb_camera_xyz": eef_usb_camera_xyz,
            "eef_usb_camera_rpy": eef_usb_camera_rpy,
            "start_eef_camera_driver": start_eef_camera_driver,
            "eef_camera_video_device": eef_camera_video_device,
            "eef_camera_frame_id": eef_camera_frame_id,
            "eef_camera_pixel_format": eef_camera_pixel_format,
            "eef_camera_output_encoding": eef_camera_output_encoding,
            "eef_camera_image_width": eef_camera_image_width,
            "eef_camera_image_height": eef_camera_image_height,
            "eef_camera_name": eef_camera_name,
            "eef_camera_info_url": eef_camera_info_url,
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
            "servo_command_out_topic": joint_trajectory_raw_topic,
            "start_joint_trajectory_transformer": start_joint_trajectory_transformer,
            "joint_trajectory_raw_topic": joint_trajectory_raw_topic,
            "joint_trajectory_output_topic": joint_trajectory_output_topic,
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
    )

    auto_init_bbox_node = Node(
        package="mp_control",
        executable="auto_init_bbox.py",
        name="auto_init_bbox",
        output="screen",
        parameters=[{
            "image_topic": auto_init_bbox_image_topic,
            "alternate_image_topics": auto_init_alternate_image_topics,
            "bbox_topic": auto_init_bbox_topic,
            "tracked_bbox_topic": auto_init_tracked_bbox_topic,
            "publish_tracked_bbox": ParameterValue(auto_init_publish_tracked_bbox, value_type=bool),
            "status_topic": auto_init_bbox_status_topic,
            "color_mode": auto_init_color_mode,
            "min_mask_pixels": ParameterValue(auto_init_min_mask_pixels, value_type=int),
            "min_bbox_width_px": ParameterValue(auto_init_min_bbox_width_px, value_type=float),
            "min_bbox_height_px": ParameterValue(auto_init_min_bbox_height_px, value_type=float),
            "max_bbox_area_ratio": ParameterValue(auto_init_max_bbox_area_ratio, value_type=float),
            "min_bbox_aspect_ratio": ParameterValue(auto_init_min_bbox_aspect_ratio, value_type=float),
            "max_bbox_aspect_ratio": ParameterValue(auto_init_max_bbox_aspect_ratio, value_type=float),
            "roi_min_x_ratio": ParameterValue(auto_init_roi_min_x_ratio, value_type=float),
            "roi_max_x_ratio": ParameterValue(auto_init_roi_max_x_ratio, value_type=float),
            "roi_min_y_ratio": ParameterValue(auto_init_roi_min_y_ratio, value_type=float),
            "roi_max_y_ratio": ParameterValue(auto_init_roi_max_y_ratio, value_type=float),
            "timeout_s": ParameterValue(auto_init_timeout_s, value_type=float),
            "black_max": ParameterValue(auto_init_black_max, value_type=int),
            "black_min_contrast": ParameterValue(auto_init_black_min_contrast, value_type=int),
            "red_min": ParameterValue(auto_init_red_min, value_type=int),
            "red_margin": ParameterValue(auto_init_red_margin, value_type=int),
            "red_ratio": ParameterValue(auto_init_red_ratio, value_type=float),
            "depth_min_m": ParameterValue(auto_init_depth_min_m, value_type=float),
            "depth_max_m": ParameterValue(auto_init_depth_max_m, value_type=float),
            "depth_near_percentile": ParameterValue(auto_init_depth_near_percentile, value_type=float),
            "depth_band_m": ParameterValue(auto_init_depth_band_m, value_type=float),
            "box_depth_band_m": ParameterValue(auto_init_box_depth_band_m, value_type=float),
            "box_min_fill_ratio": ParameterValue(auto_init_box_min_fill_ratio, value_type=float),
            "box_max_depth_std_m": ParameterValue(auto_init_box_max_depth_std_m, value_type=float),
            "box_max_center_distance_ratio": ParameterValue(
                auto_init_box_max_center_distance_ratio, value_type=float),
            "box_center_weight": ParameterValue(auto_init_box_center_weight, value_type=float),
            "box_area_weight": ParameterValue(auto_init_box_area_weight, value_type=float),
            "box_depth_weight": ParameterValue(auto_init_box_depth_weight, value_type=float),
            "yolo_model_path": auto_init_yolo_model_path,
            "yolo_confidence": ParameterValue(auto_init_yolo_confidence, value_type=float),
            "yolo_imgsz": ParameterValue(auto_init_yolo_imgsz, value_type=int),
            "yolo_class_name": auto_init_yolo_class_name,
            "yolo_max_detections": ParameterValue(auto_init_yolo_max_detections, value_type=int),
            "yolo_lock_target": False,
            "yolo_min_accept_confidence": 0.15,
            "yolo_locked_min_accept_confidence": 0.35,
            "yolo_max_center_jump_ratio": 0.08,
            "yolo_anchor_max_center_jump_ratio": 0.18,
            "yolo_min_reselect_iou": 0.10,
            "yolo_min_area_ratio_change": 0.50,
            "yolo_max_area_ratio_change": 1.80,
            "continuous_publish": True,
            "continuous_publish_period_s": 0.35,
            "reuse_last_bbox_on_loss": False,
            "lock_first_bbox": False,
        }],
        condition=IfCondition(start_auto_init_bbox),
    )

    auto_eef_init_bbox_node = Node(
        package="mp_control",
        executable="auto_init_bbox.py",
        name="auto_eef_init_bbox",
        output="screen",
        parameters=[{
            "image_topic": auto_eef_init_bbox_image_topic,
            "bbox_topic": auto_eef_init_bbox_topic,
            "status_topic": auto_eef_init_bbox_status_topic,
            "enable_topic": "/target/eef_auto_init_enable",
            "start_enabled": True,
            "color_mode": auto_eef_init_color_mode,
            "min_mask_pixels": ParameterValue(auto_eef_init_min_mask_pixels, value_type=int),
            "min_bbox_width_px": ParameterValue(auto_eef_init_min_bbox_width_px, value_type=float),
            "min_bbox_height_px": ParameterValue(auto_eef_init_min_bbox_height_px, value_type=float),
            "max_bbox_area_ratio": 0.98,
            "min_bbox_aspect_ratio": ParameterValue(auto_init_min_bbox_aspect_ratio, value_type=float),
            "max_bbox_aspect_ratio": ParameterValue(auto_init_max_bbox_aspect_ratio, value_type=float),
            "roi_min_x_ratio": ParameterValue(auto_eef_init_roi_min_x_ratio, value_type=float),
            "roi_max_x_ratio": ParameterValue(auto_eef_init_roi_max_x_ratio, value_type=float),
            "roi_min_y_ratio": ParameterValue(auto_eef_init_roi_min_y_ratio, value_type=float),
            "roi_max_y_ratio": ParameterValue(auto_eef_init_roi_max_y_ratio, value_type=float),
            "timeout_s": ParameterValue(auto_init_timeout_s, value_type=float),
            "black_max": ParameterValue(auto_init_black_max, value_type=int),
            "black_min_contrast": ParameterValue(auto_init_black_min_contrast, value_type=int),
            "red_min": ParameterValue(auto_init_red_min, value_type=int),
            "red_margin": ParameterValue(auto_init_red_margin, value_type=int),
            "red_ratio": ParameterValue(auto_init_red_ratio, value_type=float),
            "depth_min_m": ParameterValue(auto_init_depth_min_m, value_type=float),
            "depth_max_m": ParameterValue(auto_init_depth_max_m, value_type=float),
            "depth_near_percentile": ParameterValue(auto_init_depth_near_percentile, value_type=float),
            "depth_band_m": ParameterValue(auto_init_depth_band_m, value_type=float),
            "box_depth_band_m": ParameterValue(auto_init_box_depth_band_m, value_type=float),
            "box_min_fill_ratio": ParameterValue(auto_init_box_min_fill_ratio, value_type=float),
            "box_max_depth_std_m": ParameterValue(auto_init_box_max_depth_std_m, value_type=float),
            "box_max_center_distance_ratio": ParameterValue(
                auto_init_box_max_center_distance_ratio, value_type=float),
            "box_center_weight": ParameterValue(auto_init_box_center_weight, value_type=float),
            "box_area_weight": ParameterValue(auto_init_box_area_weight, value_type=float),
            "box_depth_weight": ParameterValue(auto_init_box_depth_weight, value_type=float),
            "yolo_model_path": auto_init_yolo_model_path,
            "yolo_confidence": ParameterValue(auto_init_yolo_confidence, value_type=float),
            "yolo_imgsz": ParameterValue(auto_init_yolo_imgsz, value_type=int),
            "yolo_class_name": auto_init_yolo_class_name,
            "yolo_max_detections": ParameterValue(auto_init_yolo_max_detections, value_type=int),
            "yolo_lock_target": False,
            "continuous_publish": True,
            "continuous_publish_period_s": 0.35,
            "reuse_last_bbox_on_loss": False,
            "lock_first_bbox": False,
        }],
        condition=IfCondition(start_auto_eef_init_bbox),
    )

    calibrated_camera_info_node = Node(
        package="astra_mini_calibration",
        executable="camera_info_from_json.py",
        name="astra_mini_camera_info_from_json",
        output="screen",
        parameters=[{
            "json_path": calibrated_camera_info_json_path,
            "camera_info_key": "camera_info",
        }],
        remappings=[
            ("image", calibrated_camera_image_topic),
            ("camera_info", calibrated_camera_info_topic),
        ],
        condition=IfCondition(start_calibrated_camera_info),
    )

    monitor_uploader_node = Node(
        package="leader_platooning_beacon",
        executable="robot_status_uploader.py",
        name="leader_status_uploader",
        output="screen",
        arguments=[
            "--robot", "leader",
            "--server", monitor_server,
            "--token", monitor_token,
            "--status-period", monitor_status_period,
            "--video-period", monitor_video_period,
            "--jpeg-quality", monitor_jpeg_quality,
            "--image-width", monitor_image_width,
            "--image-height", monitor_image_height,
            "--http-timeout", monitor_http_timeout,
            "--video-enabled", monitor_video_enabled,
        ],
        condition=IfCondition(start_monitor_uploader),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "move_to_stay_pose",
            default_value="true",
            description="Move the manipulator to the saved stay pose after startup.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Leader default keeps RViz off on the robot computer.",
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
                "mp_control_real_params.yaml",
            ]),
            description="Real robot parameter file for mp_control.",
        ),
        DeclareLaunchArgument(
            "start_tracker",
            default_value="true",
            description="Launch the front Astra RGB-D/depth tracker used for primary object judgement.",
        ),
        DeclareLaunchArgument(
            "start_eef_tracker",
            default_value="true",
            description="Launch the EEF tracker for near-field refinement only; it must not initialize the primary target.",
        ),
        DeclareLaunchArgument(
            "start_servo",
            default_value="true",
            description="Start MoveIt Servo after the hardware stack is ready.",
        ),
        DeclareLaunchArgument(
            "start_joint_trajectory_transformer",
            default_value="true",
            description="Route MoveIt Servo through a joint3 delta-mirror transformer before the real arm controller.",
        ),
        DeclareLaunchArgument(
            "joint_trajectory_raw_topic",
            default_value="/arm_controller/joint_trajectory_raw",
            description="Raw MoveIt Servo trajectory topic before real joint3 direction conversion.",
        ),
        DeclareLaunchArgument(
            "joint_trajectory_output_topic",
            default_value="/arm_controller/joint_trajectory",
            description="Arm-controller trajectory topic after real joint3 direction conversion.",
        ),
        DeclareLaunchArgument(
            "start_mp_control",
            default_value="true",
            description="Launch mp_control.",
        ),
        DeclareLaunchArgument(
            "control_start_delay",
            default_value="12.0",
            description="Seconds to wait for manipulator alignment before starting Servo, trackers, and mp_control.",
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
            default_value="true",
            description="Leader default starts the leader-to-follower domain bridge.",
        ),
        DeclareLaunchArgument(
            "start_auto_init_bbox",
            default_value="true",
            description="Automatically detect the box target and publish /target/init_bbox.",
        ),
        DeclareLaunchArgument(
            "auto_init_bbox_start_delay",
            default_value="12.0",
            description="Seconds to wait before auto-detecting the initial bbox.",
        ),
        DeclareLaunchArgument(
            "auto_init_bbox_image_topic",
            default_value="/camera/color/image_raw",
            description="Front Astra image topic used for automatic primary box detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_alternate_image_topics",
            default_value="/camera/rgb/image_raw,/camera/image_raw,/camera/color/image",
            description="Comma-separated fallback image topics for automatic primary box detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_bbox_topic",
            default_value="/target/init_bbox",
            description="Initial bbox topic published by the auto detector.",
        ),
        DeclareLaunchArgument(
            "auto_init_tracked_bbox_topic",
            default_value="/target/tracked_bbox",
            description="Tracked bbox topic for optional direct detector publishing.",
        ),
        DeclareLaunchArgument(
            "auto_init_publish_tracked_bbox",
            default_value="false",
            description="Also publish detector bbox to tracked-bbox. Keep false when hybrid tracking is enabled.",
        ),
        DeclareLaunchArgument(
            "auto_init_bbox_status_topic",
            default_value="/target/auto_init_bbox_status",
            description="Status topic for automatic initial bbox detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_color_mode",
            default_value="yolo",
            description="Detection mode for the initial bbox detector.",
        ),
        DeclareLaunchArgument(
            "auto_init_min_mask_pixels",
            default_value="300",
            description="Minimum target-mask pixel count required to initialize tracking.",
        ),
        DeclareLaunchArgument(
            "auto_init_min_bbox_width_px",
            default_value="12.0",
            description="Minimum detected bbox width in pixels.",
        ),
        DeclareLaunchArgument(
            "auto_init_min_bbox_height_px",
            default_value="12.0",
            description="Minimum detected bbox height in pixels.",
        ),
        DeclareLaunchArgument(
            "auto_init_max_bbox_area_ratio",
            default_value="0.65",
            description="Maximum bbox area ratio allowed for automatic YOLO box detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_min_bbox_aspect_ratio",
            default_value="0.35",
            description="Minimum bbox width/height ratio allowed for automatic depth detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_max_bbox_aspect_ratio",
            default_value="3.0",
            description="Maximum bbox width/height ratio allowed for automatic box detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_roi_min_x_ratio",
            default_value="0.0",
            description="Left boundary of the automatic detection ROI as an image-width ratio.",
        ),
        DeclareLaunchArgument(
            "auto_init_roi_max_x_ratio",
            default_value="1.0",
            description="Right boundary of the automatic detection ROI as an image-width ratio.",
        ),
        DeclareLaunchArgument(
            "auto_init_roi_min_y_ratio",
            default_value="0.0",
            description="Top boundary of the automatic detection ROI as an image-height ratio.",
        ),
        DeclareLaunchArgument(
            "auto_init_roi_max_y_ratio",
            default_value="1.00",
            description="Bottom boundary of the automatic detection ROI as an image-height ratio.",
        ),
        DeclareLaunchArgument(
            "auto_init_timeout_s",
            default_value="0.0",
            description="Seconds before auto bbox detection gives up. 0 means keep waiting.",
        ),
        DeclareLaunchArgument(
            "auto_init_black_max",
            default_value="85",
            description="Maximum luma value considered black.",
        ),
        DeclareLaunchArgument(
            "auto_init_black_min_contrast",
            default_value="20",
            description="Minimum luma contrast against the scene median for black target detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_red_min",
            default_value="80",
            description="Minimum red-channel value for red target detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_red_margin",
            default_value="35",
            description="Minimum red-channel margin over other channels.",
        ),
        DeclareLaunchArgument(
            "auto_init_red_ratio",
            default_value="1.25",
            description="Minimum red-channel ratio against green and blue.",
        ),
        DeclareLaunchArgument(
            "auto_init_depth_min_m",
            default_value="0.12",
            description="Minimum valid depth for depth_near automatic bbox detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_depth_max_m",
            default_value="1.2",
            description="Maximum valid depth for depth_near automatic bbox detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_depth_near_percentile",
            default_value="8.0",
            description="Nearest valid depth percentile used by depth_near automatic bbox detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_depth_band_m",
            default_value="0.15",
            description="Depth band above the near percentile used by depth_near detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_box_depth_band_m",
            default_value="0.08",
            description="Depth band above the near percentile used by box detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_box_min_fill_ratio",
            default_value="0.25",
            description="Minimum filled-pixel ratio inside a depth component bbox for box detection.",
        ),
        DeclareLaunchArgument(
            "auto_init_box_max_depth_std_m",
            default_value="0.16",
            description="Maximum depth standard deviation inside a box candidate.",
        ),
        DeclareLaunchArgument(
            "auto_init_box_max_center_distance_ratio",
            default_value="0.24",
            description="Maximum normalized distance from image center for a box candidate.",
        ),
        DeclareLaunchArgument(
            "auto_init_box_center_weight",
            default_value="0.55",
            description="Weight for centered box candidates.",
        ),
        DeclareLaunchArgument(
            "auto_init_box_area_weight",
            default_value="0.30",
            description="Weight for box candidate image area.",
        ),
        DeclareLaunchArgument(
            "auto_init_box_depth_weight",
            default_value="0.15",
            description="Weight for box candidate depth flatness.",
        ),
        DeclareLaunchArgument(
            "auto_init_yolo_model_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("mp_control"),
                "models",
                "cardboard_box_yolov8_best.pt",
            ]),
            description="YOLO model path used when auto_init_color_mode:=yolo.",
        ),
        DeclareLaunchArgument(
            "auto_init_yolo_confidence",
            default_value="0.01",
            description="YOLO inference threshold; accepted boxes are still filtered by the lock/accept confidence gates.",
        ),
        DeclareLaunchArgument(
            "auto_init_yolo_imgsz",
            default_value="1280",
            description="YOLO inference image size.",
        ),
        DeclareLaunchArgument(
            "auto_init_yolo_class_name",
            default_value="box",
            description="YOLO class name to accept.",
        ),
        DeclareLaunchArgument(
            "auto_init_yolo_max_detections",
            default_value="10",
            description="Maximum YOLO detections to inspect per frame.",
        ),
        DeclareLaunchArgument(
            "start_auto_eef_init_bbox",
            default_value="true",
            description="Auto-detect the target from the EEF camera when front depth is unavailable for stereo triangulation.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_bbox_start_delay",
            default_value="12.0",
            description="Seconds to wait before EEF camera auto bbox detection starts.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_bbox_image_topic",
            default_value="/eef_camera/image_raw",
            description="EEF camera image topic used for automatic bbox detection.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_bbox_topic",
            default_value="/target/eef_init_bbox",
            description="Initial bbox topic published for the EEF tracker.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_bbox_status_topic",
            default_value="/target/auto_eef_init_bbox_status",
            description="Status topic for EEF automatic bbox detection.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_color_mode",
            default_value="yolo",
            description="Target appearance to detect from the EEF RGB camera when EEF auto init is enabled.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_min_mask_pixels",
            default_value="250",
            description="Minimum colored pixel count required for EEF bbox initialization.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_min_bbox_width_px",
            default_value="8.0",
            description="Minimum EEF detected bbox width in pixels.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_min_bbox_height_px",
            default_value="8.0",
            description="Minimum EEF detected bbox height in pixels.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_roi_min_x_ratio",
            default_value="0.25",
            description="Left boundary of the EEF automatic detection ROI as an image-width ratio.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_roi_max_x_ratio",
            default_value="0.98",
            description="Right boundary of the EEF automatic detection ROI as an image-width ratio.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_roi_min_y_ratio",
            default_value="0.10",
            description="Top boundary of the EEF automatic detection ROI as an image-height ratio.",
        ),
        DeclareLaunchArgument(
            "auto_eef_init_roi_max_y_ratio",
            default_value="1.00",
            description="Bottom boundary of the EEF automatic detection ROI as an image-height ratio.",
        ),
        DeclareLaunchArgument(
            "start_calibrated_camera_info",
            default_value="true",
            description="Publish calibrated Astra color camera_info from the saved JSON.",
        ),
        DeclareLaunchArgument(
            "calibrated_camera_info_json_path",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"),
                "turtlebot3_ws",
                "src",
                "depth_perception",
                "astra_mini_calibration",
                "config",
                "astra_mini_color.json",
            ]),
            description="Astra color calibration JSON used after the depth near-limit handoff.",
        ),
        DeclareLaunchArgument(
            "calibrated_camera_image_topic",
            default_value="/camera/color/image_raw",
            description="Image topic paired with the calibrated Astra color camera_info.",
        ),
        DeclareLaunchArgument(
            "calibrated_camera_info_topic",
            default_value="/camera/color/camera_info_calibrated",
            description="Calibrated Astra color camera_info topic.",
        ),
        DeclareLaunchArgument(
            "start_eef_camera_driver",
            default_value="true",
            description="Start the v4l2 end-effector USB camera driver.",
        ),
        DeclareLaunchArgument(
            "eef_camera_video_device",
            default_value="/dev/video0",
            description="Linux video device for the end-effector USB camera.",
        ),
        DeclareLaunchArgument(
            "eef_camera_frame_id",
            default_value="eef_usb_camera_optical_frame",
            description="Frame id used by the end-effector camera images.",
        ),
        DeclareLaunchArgument(
            "eef_camera_pixel_format",
            default_value="YUYV",
            description="V4L2 pixel format requested from the end-effector camera.",
        ),
        DeclareLaunchArgument(
            "eef_camera_output_encoding",
            default_value="rgb8",
            description="ROS image encoding published by the end-effector camera.",
        ),
        DeclareLaunchArgument(
            "eef_camera_image_width",
            default_value="320",
            description="EEF USB camera image width.",
        ),
        DeclareLaunchArgument(
            "eef_camera_image_height",
            default_value="240",
            description="EEF USB camera image height.",
        ),
        DeclareLaunchArgument(
            "eef_camera_name",
            default_value="eef_usb_camera",
            description="Camera name used when loading and saving EEF USB camera calibration.",
        ),
        DeclareLaunchArgument(
            "eef_camera_info_url",
            default_value=[
                "file://",
                PathJoinSubstitution([
                    EnvironmentVariable("HOME"),
                    "turtlebot3_ws",
                    "src",
                    "mp_control",
                    "calibration",
                    "eef_camera",
                    "eef_usb_camera.yaml",
                ]),
            ],
            description="Camera calibration URL for the EEF USB camera.",
        ),
        DeclareLaunchArgument(
            "start_monitor_uploader",
            default_value="true",
            description="Start the upload-only Raspberry Pi monitor uploader.",
        ),
        DeclareLaunchArgument(
            "monitor_server",
            default_value=EnvironmentVariable(
                "MONITOR_SERVER_URL",
                default_value="http://192.168.0.13:8000",
            ),
            description="Upload-only monitor server URL, for example http://192.168.0.10:8080.",
        ),
        DeclareLaunchArgument(
            "monitor_token",
            default_value=EnvironmentVariable("MONITOR_TOKEN", default_value=""),
            description="Upload-only monitor token.",
        ),
        DeclareLaunchArgument("monitor_video_enabled", default_value="true"),
        DeclareLaunchArgument("monitor_status_period", default_value="0.2"),
        DeclareLaunchArgument("monitor_video_period", default_value="0.5"),
        DeclareLaunchArgument("monitor_jpeg_quality", default_value="50"),
        DeclareLaunchArgument("monitor_image_width", default_value="424"),
        DeclareLaunchArgument("monitor_image_height", default_value="318"),
        DeclareLaunchArgument("monitor_http_timeout", default_value="3.0"),
        hardware_launch,
        TimerAction(
            period=control_start_delay,
            actions=[
                calibrated_camera_info_node,
                grasp_stack_launch,
                leader_task_manager_launch,
                leader_beacon_launch,
                domain_bridge_launch,
                monitor_uploader_node,
            ],
        ),
        TimerAction(
            period=auto_init_bbox_start_delay,
            actions=[auto_init_bbox_node],
            condition=IfCondition(start_auto_init_bbox),
        ),
        TimerAction(
            period=auto_eef_init_bbox_start_delay,
            actions=[auto_eef_init_bbox_node],
            condition=IfCondition(start_auto_eef_init_bbox),
        ),
    ])
