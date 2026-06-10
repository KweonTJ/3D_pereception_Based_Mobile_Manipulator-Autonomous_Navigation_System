from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_camera = LaunchConfiguration("start_camera")
    start_eef_camera_driver = LaunchConfiguration("start_eef_camera_driver")
    start_aruco_tracker = LaunchConfiguration("start_aruco_tracker")
    start_servo = LaunchConfiguration("start_servo")
    control_start_delay = LaunchConfiguration("control_start_delay")

    eef_camera_video_device = LaunchConfiguration("eef_camera_video_device")
    eef_camera_frame_id = LaunchConfiguration("eef_camera_frame_id")
    eef_camera_pixel_format = LaunchConfiguration("eef_camera_pixel_format")
    eef_camera_output_encoding = LaunchConfiguration("eef_camera_output_encoding")
    eef_camera_image_width = LaunchConfiguration("eef_camera_image_width")
    eef_camera_image_height = LaunchConfiguration("eef_camera_image_height")
    eef_camera_name = LaunchConfiguration("eef_camera_name")
    eef_camera_info_url = LaunchConfiguration("eef_camera_info_url")

    aruco_config_file = LaunchConfiguration("aruco_config_file")

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
            "move_to_stay_pose": "true",
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
            "command_out_topic": "/arm_controller/joint_trajectory_raw",
        }.items(),
        condition=IfCondition(start_servo),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_camera",
            default_value="false",
            description="Do not start Astra depth camera for ArUco-only manipulator test.",
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
            "start_servo",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "control_start_delay",
            default_value="8.0",
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
                aruco_tracker_launch,
                servo_launch,
            ],
        ),
    ])