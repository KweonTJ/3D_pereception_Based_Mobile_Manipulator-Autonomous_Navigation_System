import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_share = get_package_share_directory('astra_mini_calibration')
    astra_share = get_package_share_directory('astra_camera')
    astra_launch = os.path.join(astra_share, 'launch', 'astra_mini.launch.py')
    workspace_root = os.path.dirname(
        os.path.dirname(
            os.path.dirname(
                os.path.dirname(calibration_share)
            )
        )
    )
    default_output_dir = os.path.join(
        workspace_root,
        'src',
        'astra_mini_calibration',
        'config',
    )

    start_camera_arg = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description='Whether to launch the Astra Mini camera stack together.',
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Monocular image topic to calibrate.',
    )

    camera_namespace_arg = DeclareLaunchArgument(
        'camera_namespace',
        default_value='/camera',
        description='Camera namespace used for set_camera_info service remapping.',
    )

    board_size_arg = DeclareLaunchArgument(
        'board_size',
        default_value='10x7',
        description='Checkerboard interior corners in NxM format.',
    )

    square_size_arg = DeclareLaunchArgument(
        'square_size',
        default_value='0.024',
        description='Checkerboard square size in meters.',
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='astra_mini_color',
        description='Camera name stored in the calibration output.',
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=default_output_dir,
        description='Directory where the calibration JSON file is saved when SAVE is pressed.',
    )

    output_prefix_arg = DeclareLaunchArgument(
        'output_prefix',
        default_value='astra_mini_color',
        description='Filename prefix for the saved calibration JSON file.',
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(astra_launch),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    calibrator_node = Node(
        package='astra_mini_calibration',
        executable='cameracalibrator_with_save.py',
        name='astra_mini_cameracalibrator',
        output='screen',
        arguments=[
            '--size',
            LaunchConfiguration('board_size'),
            '--square',
            LaunchConfiguration('square_size'),
            '--camera_name',
            LaunchConfiguration('camera_name'),
            '--no-service-check',
        ],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('camera', LaunchConfiguration('camera_namespace')),
        ],
        additional_env={
            'ASTRA_MINI_CALIB_OUTPUT_DIR': LaunchConfiguration('output_dir'),
            'ASTRA_MINI_CALIB_OUTPUT_PREFIX': LaunchConfiguration('output_prefix'),
        },
    )

    return LaunchDescription([
        # ROS Humble camera_calibration/cv_bridge is not compatible with the
        # user-level NumPy 2.x currently installed in this environment.
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        start_camera_arg,
        image_topic_arg,
        camera_namespace_arg,
        board_size_arg,
        square_size_arg,
        camera_name_arg,
        output_dir_arg,
        output_prefix_arg,
        camera_launch,
        calibrator_node,
    ])
