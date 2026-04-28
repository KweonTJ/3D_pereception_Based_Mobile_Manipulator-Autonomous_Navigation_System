from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('hybrid_single_target_tracker')
    gazebo_share = get_package_share_directory('turtlebot3_manipulation_gazebo')
    default_params = os.path.join(package_share, 'config', 'params.yaml')
    gazebo_launch = os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the hybrid tracker parameter file.',
    )

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Whether to start RViz together with Gazebo.',
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Ignition Gazebo 6 world file or resource name.',
    )

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='-2.00',
        description='Robot x position in Gazebo.',
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='-0.50',
        description='Robot y position in Gazebo.',
    )

    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.01',
        description='Robot z position in Gazebo.',
    )

    roll_arg = DeclareLaunchArgument(
        'roll',
        default_value='0.00',
        description='Robot roll in Gazebo.',
    )

    pitch_arg = DeclareLaunchArgument(
        'pitch',
        default_value='0.00',
        description='Robot pitch in Gazebo.',
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.00',
        description='Robot yaw in Gazebo.',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'start_rviz': LaunchConfiguration('start_rviz'),
            'start_depth_camera': 'true',
            'use_sim': 'true',
            'world': LaunchConfiguration('world'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': LaunchConfiguration('z_pose'),
            'roll': LaunchConfiguration('roll'),
            'pitch': LaunchConfiguration('pitch'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    tracker = Node(
        package='hybrid_single_target_tracker',
        executable='hybrid_single_target_tracker_node',
        name='hybrid_single_target_tracker',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        params_file_arg,
        start_rviz_arg,
        world_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        gazebo,
        tracker,
    ])
