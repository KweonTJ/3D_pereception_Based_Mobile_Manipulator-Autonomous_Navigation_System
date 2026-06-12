from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rover = LaunchConfiguration('start_rover')
    start_pick_place = LaunchConfiguration('start_pick_place')
    start_aruco_bridge = LaunchConfiguration('start_aruco_bridge')
    start_mux = LaunchConfiguration('start_mux')
    start_coordinator = LaunchConfiguration('start_coordinator')

    robot = LaunchConfiguration('robot')
    frame_id = LaunchConfiguration('frame_id')
    map_path = LaunchConfiguration('map_path')
    command_timeout_sec = LaunchConfiguration('command_timeout_sec')

    control_start_delay = LaunchConfiguration('control_start_delay')
    force_object_x_m = LaunchConfiguration('force_object_x_m')

    mp_control_config_file = LaunchConfiguration('mp_control_config_file')
    aruco_config_file = LaunchConfiguration('aruco_config_file')
    bridge_config_file = LaunchConfiguration('bridge_config_file')
    mux_config_file = LaunchConfiguration('mux_config_file')
    coordinator_config_file = LaunchConfiguration('coordinator_config_file')

    start_camera = LaunchConfiguration('start_camera')
    start_eef_camera_driver = LaunchConfiguration('start_eef_camera_driver')
    eef_camera_video_device = LaunchConfiguration('eef_camera_video_device')
    eef_camera_frame_id = LaunchConfiguration('eef_camera_frame_id')
    eef_camera_pixel_format = LaunchConfiguration('eef_camera_pixel_format')
    eef_camera_output_encoding = LaunchConfiguration('eef_camera_output_encoding')
    eef_camera_image_width = LaunchConfiguration('eef_camera_image_width')
    eef_camera_image_height = LaunchConfiguration('eef_camera_image_height')
    eef_camera_name = LaunchConfiguration('eef_camera_name')
    eef_camera_info_url = LaunchConfiguration('eef_camera_info_url')

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
        }.items(),
        condition=IfCondition(start_rover),
    )

    pick_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mp_control'),
                'launch',
                'aruco_pick_place.launch.py',
            ])
        ]),
        launch_arguments={
            'start_camera': start_camera,
            'start_eef_camera_driver': start_eef_camera_driver,
            'start_aruco_tracker': 'true',
            'start_mp_control': 'true',
            'start_servo': 'true',
            'start_joint_trajectory_transformer': 'true',
            'mp_control_config_file': mp_control_config_file,
            'aruco_config_file': aruco_config_file,
            'control_start_delay': control_start_delay,
            'eef_camera_video_device': eef_camera_video_device,
            'eef_camera_frame_id': eef_camera_frame_id,
            'eef_camera_pixel_format': eef_camera_pixel_format,
            'eef_camera_output_encoding': eef_camera_output_encoding,
            'eef_camera_image_width': eef_camera_image_width,
            'eef_camera_image_height': eef_camera_image_height,
            'eef_camera_name': eef_camera_name,
            'eef_camera_info_url': eef_camera_info_url,
        }.items(),
        condition=IfCondition(start_pick_place),
    )

    aruco_bridge = Node(
        package='aruco_mp_bridge',
        executable='aruco_to_mp_control_bridge',
        name='aruco_to_mp_control_bridge',
        output='screen',
        parameters=[
            bridge_config_file,
            {
                'force_object_x_m': force_object_x_m,
                'publish_start_on_visible': False,
                'continuous_start_publish': False,
                'start_publish_count': 0,
            },
        ],
        condition=IfCondition(start_aruco_bridge),
    )

    cmd_vel_mux = Node(
        package='turtlebot3_control',
        executable='cmd_vel_mux_node',
        name='cmd_vel_mux_node',
        output='screen',
        parameters=[mux_config_file],
        condition=IfCondition(start_mux),
    )

    coordinator = Node(
        package='turtlebot3_control',
        executable='leader_pick_coordinator_node',
        name='leader_pick_coordinator_node',
        output='screen',
        parameters=[coordinator_config_file],
        condition=IfCondition(start_coordinator),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_rover', default_value='true'),
        DeclareLaunchArgument('start_pick_place', default_value='true'),
        DeclareLaunchArgument('start_aruco_bridge', default_value='true'),
        DeclareLaunchArgument('start_mux', default_value='true'),
        DeclareLaunchArgument('start_coordinator', default_value='true'),

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

        DeclareLaunchArgument(
            'start_camera',
            default_value='false',
            description='Astra camera is not required for ArUco-only pick-place.',
        ),
        DeclareLaunchArgument('start_eef_camera_driver', default_value='true'),
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

        pick_place_launch,
        rover_launch,
        aruco_bridge,
        cmd_vel_mux,
        coordinator,
    ])
