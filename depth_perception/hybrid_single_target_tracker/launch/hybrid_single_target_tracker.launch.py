from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('hybrid_single_target_tracker')
    astra_share = get_package_share_directory('astra_camera')
    default_params = os.path.join(package_share, 'config', 'params.yaml')
    astra_launch = os.path.join(astra_share, 'launch', 'astra_mini.launch.py')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the hybrid tracker parameter file.',
    )

    start_camera_arg = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description='Whether to launch the camera interface stack together.',
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use Gazebo depth camera topics instead of the physical Astra Mini.',
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(astra_launch),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    node = Node(
        package='hybrid_single_target_tracker',
        executable='hybrid_single_target_tracker_node',
        name='hybrid_single_target_tracker',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim')},
        ],
    )

    return LaunchDescription([
        params_file_arg,
        start_camera_arg,
        use_sim_arg,
        camera_launch,
        node,
    ])
