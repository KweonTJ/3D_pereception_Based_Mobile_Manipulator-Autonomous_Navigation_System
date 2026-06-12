from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    start_camera = LaunchConfiguration("start_camera")
    start_eef_camera_driver = LaunchConfiguration("start_eef_camera_driver")
    start_aruco_tracker = LaunchConfiguration("start_aruco_tracker")
    start_servo = LaunchConfiguration("start_servo")
    start_joint_trajectory_transformer = LaunchConfiguration("start_joint_trajectory_transformer")
    start_mp_control = LaunchConfiguration("start_mp_control")
    control_start_delay = LaunchConfiguration("control_start_delay")

    joint_trajectory_raw_topic = LaunchConfiguration("joint_trajectory_raw_topic")
    joint_trajectory_output_topic = LaunchConfiguration("joint_trajectory_output_topic")

    move_to_stay_pose = LaunchConfiguration("move_to_stay_pose")
    stay_pose_joint_trajectory_topic = LaunchConfiguration("stay_pose_joint_trajectory_topic")

    eef_camera_video_device = LaunchConfiguration("eef_camera_video_device")
    eef_camera_frame_id = LaunchConfiguration("eef_camera_frame_id")
    eef_camera_pixel_format = LaunchConfiguration("eef_camera_pixel_format")
    eef_camera_output_encoding = LaunchConfiguration("eef_camera_output_encoding")
    eef_camera_image_width = LaunchConfiguration("eef_camera_image_width")
    eef_camera_image_height = LaunchConfiguration("eef_camera_image_height")
    eef_camera_name = LaunchConfiguration("eef_camera_name")
    eef_camera_info_url = LaunchConfiguration("eef_camera_info_url")

    aruco_config_file = LaunchConfiguration("aruco_config_file")
    mp_control_config_file = LaunchConfiguration("mp_control_config_file")

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

            # 기존 stay pose 유지
            "move_to_stay_pose": move_to_stay_pose,
            "stay_pose_joint_trajectory_topic": stay_pose_joint_trajectory_topic,

            # EEF USB camera 유지
            "use_eef_usb_camera": "true",
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
            "max_reversed_joint_delta_rad": 0.17,
            "reversed_joint_min_position_rad": -2.70,
            "reversed_joint_max_position_rad": 1.38,
        }],
        condition=IfCondition(start_joint_trajectory_transformer),
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_moveit_config"),
                "launch",
                "servo.launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim": "false",
            "command_out_topic": joint_trajectory_raw_topic,
        }.items(),
        condition=IfCondition(start_servo),
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

    aruco_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("aruco_eef_tracker"),
                "launch",
                "eef_aruco_tracker.launch.py",
            ])
        ]),
        launch_arguments={
            "config_file": aruco_config_file,
        }.items(),
        condition=IfCondition(start_aruco_tracker),
    )

    mp_control_node = Node(
        package="mp_control",
        executable="mp_control_node",
        name="mp_control_node",
        output="screen",
        parameters=[mp_control_config_file],
        condition=IfCondition(start_mp_control),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_camera",
            default_value="false",
            description="Astra depth camera is disabled in ArUco-only manipulator launch.",
        ),
        DeclareLaunchArgument(
            "start_eef_camera_driver",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "start_aruco_tracker",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "start_mp_control",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "start_servo",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "start_joint_trajectory_transformer",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "control_start_delay",
            default_value="8.0",
        ),

        # 기존 stay pose 유지
        DeclareLaunchArgument(
            "move_to_stay_pose",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "stay_pose_joint_trajectory_topic",
            default_value="/arm_controller/joint_trajectory",
        ),

        # 기존 trajectory 경로 유지
        DeclareLaunchArgument(
            "joint_trajectory_raw_topic",
            default_value="/arm_controller/joint_trajectory_raw",
        ),
        DeclareLaunchArgument(
            "joint_trajectory_output_topic",
            default_value="/arm_controller/joint_trajectory",
        ),

        DeclareLaunchArgument(
            "aruco_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("aruco_eef_tracker"),
                "config",
                "eef_aruco_tracker.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "mp_control_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mp_control"),
                "config",
                "mp_control_real_params.yaml",
            ]),
        ),

        DeclareLaunchArgument(
            "eef_camera_video_device",
            default_value="/dev/video0",
        ),
        DeclareLaunchArgument(
            "eef_camera_frame_id",
            default_value="eef_usb_camera_optical_frame",
        ),
        DeclareLaunchArgument(
            "eef_camera_pixel_format",
            default_value="YUYV",
        ),
        DeclareLaunchArgument(
            "eef_camera_output_encoding",
            default_value="rgb8",
        ),
        DeclareLaunchArgument(
            "eef_camera_image_width",
            default_value="320",
        ),
        DeclareLaunchArgument(
            "eef_camera_image_height",
            default_value="240",
        ),
        DeclareLaunchArgument(
            "eef_camera_name",
            default_value="eef_usb_camera",
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
        ),

        hardware_launch,
        TimerAction(
            period=control_start_delay,
            actions=[
                joint_trajectory_transformer_node,
                servo_launch,
                start_servo_call,
                aruco_tracker_launch,
                mp_control_node,
            ],
        ),
    ])
