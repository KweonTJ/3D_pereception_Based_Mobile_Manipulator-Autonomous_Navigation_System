"""Leader navigation: rover_nav -> base.

Runs the leader mission-contract navigation and connects its velocity command
straight to the base by default. The node subscribes /leader/target_cmd and
/leader/global/position, plans over map.json, publishes /leader/nav_feedback and
/leader/route, and sends velocity to cmd_vel_topic.

Standalone default:
    /leader/target_cmd + /leader/global/position
        -> leader_rover_nav_node
        -> /cmd_vel -> turtlebot3 base

The integrated leader_runtime.launch.py overrides cmd_vel_topic back to
/leader/cmd_vel so cmd_vel_mux can arbitrate nav, PnP, and startup alignment.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('leader_line_follower')
    default_map_path = os.path.join(package_share, 'maps', 'map.json')
    default_xacro_path = os.path.join(
        package_share, 'urdf', 'turtlebot3_manipulation.urdf.xacro'
    )

    robot = LaunchConfiguration('robot')
    frame_id = LaunchConfiguration('frame_id')
    map_path = LaunchConfiguration('map_path')
    publish_description = LaunchConfiguration('publish_description')
    xacro_path = LaunchConfiguration('xacro_path')
    use_sim = LaunchConfiguration('use_sim')
    command_timeout_sec = LaunchConfiguration('command_timeout_sec')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    invert_angular_z = LaunchConfiguration('invert_angular_z')

    leader_nav = Node(
        package='leader_line_follower',
        executable='leader_rover_nav_node',
        name='leader_rover_nav_node',
        output='screen',
        parameters=[{
            'robot': robot,
            'map_path': map_path,
            'frame_id': frame_id,
            'command_timeout_sec': command_timeout_sec,
            'cmd_vel_topic': cmd_vel_topic,
            'invert_angular_z': invert_angular_z,
        }],
    )

    robot_state_publisher = Node(
        condition=IfCondition(publish_description),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='leader_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                xacro_path,
                ' prefix:=',
                robot,
                '_ ',
                'use_sim:=',
                use_sim,
            ]),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='leader',
            description='Leader robot namespace prefix used for topics.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='uwb_global',
            description='Global frame used by TargetCommand and NavFeedback.',
        ),
        DeclareLaunchArgument(
            'map_path',
            default_value=default_map_path,
            description='Absolute path to the uwb_global map.json.',
        ),
        DeclareLaunchArgument(
            'command_timeout_sec',
            default_value='0.0',
            description='TargetCommand deadman timeout. 0 disables timeout.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Velocity output topic. Standalone default drives the base directly.',
        ),
        DeclareLaunchArgument(
            'invert_angular_z',
            default_value='true',
            description='Invert angular.z output for the leader base/IMU heading sign convention.',
        ),
        DeclareLaunchArgument(
            'publish_description',
            default_value='false',
            description='Whether to publish robot_description from the leader Xacro.',
        ),
        DeclareLaunchArgument(
            'xacro_path',
            default_value=default_xacro_path,
            description='Leader robot Xacro file.',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Forwarded to the leader Xacro.',
        ),
        leader_nav,
        robot_state_publisher,
    ])
