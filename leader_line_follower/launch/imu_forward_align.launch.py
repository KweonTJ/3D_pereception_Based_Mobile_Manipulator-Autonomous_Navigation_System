from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    heading_aligned_topic = LaunchConfiguration('heading_aligned_topic')
    linear_x = LaunchConfiguration('linear_x')
    stop_duration = LaunchConfiguration('stop_duration')
    max_drive_sec = LaunchConfiguration('max_drive_sec')
    publish_rate = LaunchConfiguration('publish_rate')

    imu_forward_align = Node(
        package='leader_line_follower',
        executable='imu_forward_align',
        name='imu_forward_align',
        output='screen',
        parameters=[{
            'robot': robot,
            'cmd_vel_topic': cmd_vel_topic,
            'heading_aligned_topic': heading_aligned_topic,
            'linear_x': linear_x,
            'stop_duration': stop_duration,
            'max_drive_sec': max_drive_sec,
            'publish_rate': publish_rate,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='leader'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('heading_aligned_topic', default_value=''),
        DeclareLaunchArgument('linear_x', default_value='2.0'),
        DeclareLaunchArgument('stop_duration', default_value='0.5'),
        DeclareLaunchArgument('max_drive_sec', default_value='15.0'),
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        imu_forward_align,
    ])
