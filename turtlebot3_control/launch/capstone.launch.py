from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    frame_id = LaunchConfiguration('frame_id')
    map_path = LaunchConfiguration('map_path')
    command_timeout_sec = LaunchConfiguration('command_timeout_sec')

    start_camera = LaunchConfiguration('start_camera')
    start_eef_camera_driver = LaunchConfiguration('start_eef_camera_driver')
    start_lidar = LaunchConfiguration('start_lidar')
    move_to_stay_pose = LaunchConfiguration('move_to_stay_pose')
    start_state_relays = LaunchConfiguration('start_state_relays')

    eef_camera_video_device = LaunchConfiguration('eef_camera_video_device')
    eef_camera_frame_id = LaunchConfiguration('eef_camera_frame_id')
    eef_camera_pixel_format = LaunchConfiguration('eef_camera_pixel_format')
    eef_camera_output_encoding = LaunchConfiguration('eef_camera_output_encoding')
    eef_camera_image_width = LaunchConfiguration('eef_camera_image_width')
    eef_camera_image_height = LaunchConfiguration('eef_camera_image_height')
    eef_camera_name = LaunchConfiguration('eef_camera_name')
    eef_camera_info_url = LaunchConfiguration('eef_camera_info_url')

    align_start_delay = LaunchConfiguration('align_start_delay')
    heading_aligned_topic = LaunchConfiguration('heading_aligned_topic')
    align_linear_x = LaunchConfiguration('align_linear_x')
    align_stop_duration = LaunchConfiguration('align_stop_duration')
    align_max_drive_sec = LaunchConfiguration('align_max_drive_sec')
    align_publish_rate = LaunchConfiguration('align_publish_rate')

    control_start_delay = LaunchConfiguration('control_start_delay')
    force_object_x_m = LaunchConfiguration('force_object_x_m')
    mp_control_config_file = LaunchConfiguration('mp_control_config_file')
    aruco_config_file = LaunchConfiguration('aruco_config_file')
    bridge_config_file = LaunchConfiguration('bridge_config_file')
    mux_config_file = LaunchConfiguration('mux_config_file')
    coordinator_config_file = LaunchConfiguration('coordinator_config_file')

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_manipulation_bringup'),
                'launch',
                'hardware.launch.py',
            ])
        ]),
        launch_arguments={
            'start_camera': start_camera,
            'start_eef_camera_driver': start_eef_camera_driver,
            'start_lidar': start_lidar,
            'move_to_stay_pose': move_to_stay_pose,
            'start_state_relays': start_state_relays,
            'eef_camera_video_device': eef_camera_video_device,
            'eef_camera_frame_id': eef_camera_frame_id,
            'eef_camera_pixel_format': eef_camera_pixel_format,
            'eef_camera_output_encoding': eef_camera_output_encoding,
            'eef_camera_image_width': eef_camera_image_width,
            'eef_camera_image_height': eef_camera_image_height,
            'eef_camera_name': eef_camera_name,
            'eef_camera_info_url': eef_camera_info_url,
        }.items(),
    )

    imu_forward_align = Node(
        package='leader_line_follower',
        executable='imu_forward_align',
        name='imu_forward_align',
        output='screen',
        parameters=[{
            'robot': robot,
            'cmd_vel_topic': '/cmd_vel',
            'heading_aligned_topic': heading_aligned_topic,
            'linear_x': align_linear_x,
            'stop_duration': align_stop_duration,
            'max_drive_sec': align_max_drive_sec,
            'publish_rate': align_publish_rate,
        }],
    )

    rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('leader_line_follower'),
                'launch',
                'leader_rover.launch.py',
            ])
        ]),
        launch_arguments={
            'robot': robot,
            'frame_id': frame_id,
            'map_path': map_path,
            'command_timeout_sec': command_timeout_sec,
            'publish_description': 'false',
            'cmd_vel_topic': '/leader/cmd_vel',
        }.items(),
    )

    aruco_pick_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_control'),
                'launch',
                'leader_rover_aruco_pick_place.launch.py',
            ])
        ]),
        launch_arguments={
            'start_rover': 'false',
            'start_pick_place': 'true',
            'start_aruco_bridge': 'true',
            'start_mux': 'true',
            'start_coordinator': 'true',
            'start_hardware': 'false',
            'robot': robot,
            'frame_id': frame_id,
            'map_path': map_path,
            'command_timeout_sec': command_timeout_sec,
            'control_start_delay': control_start_delay,
            'force_object_x_m': force_object_x_m,
            'mp_control_config_file': mp_control_config_file,
            'aruco_config_file': aruco_config_file,
            'bridge_config_file': bridge_config_file,
            'mux_config_file': mux_config_file,
            'coordinator_config_file': coordinator_config_file,
            'start_camera': 'false',
            'start_eef_camera_driver': 'false',
            'eef_camera_video_device': eef_camera_video_device,
            'eef_camera_frame_id': eef_camera_frame_id,
            'eef_camera_pixel_format': eef_camera_pixel_format,
            'eef_camera_output_encoding': eef_camera_output_encoding,
            'eef_camera_image_width': eef_camera_image_width,
            'eef_camera_image_height': eef_camera_image_height,
            'eef_camera_name': eef_camera_name,
            'eef_camera_info_url': eef_camera_info_url,
        }.items(),
    )

    start_after_align = RegisterEventHandler(
        OnProcessExit(
            target_action=imu_forward_align,
            on_exit=[
                rover_launch,
                aruco_pick_place_launch,
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='leader'),
        DeclareLaunchArgument('frame_id', default_value='uwb_global'),
        DeclareLaunchArgument(
            'map_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('leader_line_follower'),
                'maps',
                'map.json',
            ]),
        ),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.0'),

        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('start_eef_camera_driver', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='false'),
        DeclareLaunchArgument('move_to_stay_pose', default_value='true'),
        DeclareLaunchArgument('start_state_relays', default_value='true'),

        DeclareLaunchArgument('eef_camera_video_device', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'eef_camera_frame_id',
            default_value='eef_usb_camera_optical_frame',
        ),
        DeclareLaunchArgument('eef_camera_pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('eef_camera_output_encoding', default_value='rgb8'),
        DeclareLaunchArgument('eef_camera_image_width', default_value='320'),
        DeclareLaunchArgument('eef_camera_image_height', default_value='240'),
        DeclareLaunchArgument('eef_camera_name', default_value='eef_usb_camera'),
        DeclareLaunchArgument(
            'eef_camera_info_url',
            default_value=[
                'file://',
                PathJoinSubstitution([
                    EnvironmentVariable('HOME'),
                    'turtlebot3_ws',
                    'src',
                    'mp_control',
                    'calibration',
                    'eef_camera',
                    'eef_usb_camera.yaml',
                ]),
            ],
        ),

        DeclareLaunchArgument('align_start_delay', default_value='8.0'),
        DeclareLaunchArgument('heading_aligned_topic', default_value=''),
        DeclareLaunchArgument('align_linear_x', default_value='2.0'),
        DeclareLaunchArgument('align_stop_duration', default_value='0.5'),
        DeclareLaunchArgument('align_max_drive_sec', default_value='15.0'),
        DeclareLaunchArgument('align_publish_rate', default_value='20.0'),

        DeclareLaunchArgument('control_start_delay', default_value='8.0'),
        DeclareLaunchArgument('force_object_x_m', default_value='0.29'),
        DeclareLaunchArgument(
            'mp_control_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('turtlebot3_control'),
                'config',
                'mp_control_aruco_integrated_params.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'aruco_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('aruco_eef_tracker'),
                'config',
                'eef_aruco_tracker.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'bridge_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('turtlebot3_control'),
                'config',
                'aruco_to_mp_control_bridge_integrated.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'mux_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('turtlebot3_control'),
                'config',
                'cmd_vel_mux.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'coordinator_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('turtlebot3_control'),
                'config',
                'leader_pick_coordinator.yaml',
            ]),
        ),

        hardware_launch,
        TimerAction(
            period=align_start_delay,
            actions=[imu_forward_align],
        ),
        start_after_align,
    ])
