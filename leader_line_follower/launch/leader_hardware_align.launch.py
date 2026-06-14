from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_camera = LaunchConfiguration('start_camera')
    start_eef_camera_driver = LaunchConfiguration('start_eef_camera_driver')
    start_lidar = LaunchConfiguration('start_lidar')
    move_to_stay_pose = LaunchConfiguration('move_to_stay_pose')
    align_start_delay = LaunchConfiguration('align_start_delay')

    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    heading_aligned_topic = LaunchConfiguration('heading_aligned_topic')
    linear_x = LaunchConfiguration('linear_x')
    stop_duration = LaunchConfiguration('stop_duration')
    max_drive_sec = LaunchConfiguration('max_drive_sec')
    publish_rate = LaunchConfiguration('publish_rate')

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
        }.items(),
    )

    imu_forward_align = Node(
        package='leader_line_follower',
        executable='imu_forward_align',
        name='imu_forward_align',
        output='screen',
        parameters=[{
            'robot': 'leader',
            'cmd_vel_topic': cmd_vel_topic,
            'heading_aligned_topic': heading_aligned_topic,
            'linear_x': linear_x,
            'stop_duration': stop_duration,
            'max_drive_sec': max_drive_sec,
            'publish_rate': publish_rate,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('start_eef_camera_driver', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='false'),
        DeclareLaunchArgument(
            'move_to_stay_pose',
            default_value='true',
            description='Avoid automatic arm stay-pose motion during base heading align.',
        ),
        DeclareLaunchArgument(
            'align_start_delay',
            default_value='8.0',
            description='Delay before imu_forward_align starts after hardware bringup.',
        ),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('heading_aligned_topic', default_value=''),
        DeclareLaunchArgument('linear_x', default_value='2.0'),
        DeclareLaunchArgument('stop_duration', default_value='0.5'),
        DeclareLaunchArgument('max_drive_sec', default_value='15.0'),
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        hardware_launch,
        TimerAction(
            period=align_start_delay,
            actions=[imu_forward_align],
        ),
    ])
