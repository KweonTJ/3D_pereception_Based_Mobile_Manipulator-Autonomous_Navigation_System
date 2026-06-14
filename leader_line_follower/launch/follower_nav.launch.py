"""Follower navigation: rover_nav -> base."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    map_path = LaunchConfiguration('map_path')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    rover_nav = Node(
        package='leader_line_follower',
        executable='leader_rover_base_nav_node',
        name='follower_rover_nav_node',
        output='screen',
        parameters=[{
            'robot': robot,
            'map_path': map_path,
            'goal_tolerance': 0.10,
            'goal_pass_tolerance': 0.15,
            'goal_slowdown_distance': 0.30,
            'normal_linear_x': 0.20,
            'min_linear_x': 0.03,
            'default_speed_cap': 0.20,
            'max_linear_speed': 0.20,
            'max_angular_speed': 1.2,
            'lookahead_distance': 0.50,
            'near_goal_deadband_speed': 0.02,
            'goal_lookahead_extension': 0.50,
            'align_start_angle_deg': 50.0,
            'align_finish_angle_deg': 5.0,
            'slowdown_at_intermediate_waypoints': True,
        }],
        remappings=[('/follower/cmd_vel', cmd_vel_topic)],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='follower'),
        DeclareLaunchArgument('map_path', default_value=''),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        rover_nav,
    ])
