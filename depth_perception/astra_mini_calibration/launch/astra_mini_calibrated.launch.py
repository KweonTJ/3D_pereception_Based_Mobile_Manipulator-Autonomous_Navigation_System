import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    default_json_path = os.path.join(
        workspace_root,
        'src',
        'astra_mini_calibration',
        'config',
        'astra_mini_color.json',
    )

    start_camera_arg = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description='Whether to launch the Astra Mini camera stack together.',
    )

    json_path_arg = DeclareLaunchArgument(
        'json_path',
        default_value=default_json_path,
        description='Calibration JSON file produced by the calibration launcher.',
    )

    camera_info_key_arg = DeclareLaunchArgument(
        'camera_info_key',
        default_value='camera_info',
        description='JSON key to load camera info from.',
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Raw image topic paired with the calibrated camera info.',
    )

    calibrated_camera_info_topic_arg = DeclareLaunchArgument(
        'calibrated_camera_info_topic',
        default_value='/camera/color/camera_info_calibrated',
        description='Output topic for calibrated camera info.',
    )

    rectify_image_arg = DeclareLaunchArgument(
        'rectify_image',
        default_value='true',
        description='Whether to publish a rectified image using image_proc.',
    )

    rectified_image_topic_arg = DeclareLaunchArgument(
        'rectified_image_topic',
        default_value='/camera/color/image_rect_calibrated',
        description='Output topic for the rectified image.',
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(astra_launch),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    calibrated_camera_info_node = Node(
        package='astra_mini_calibration',
        executable='camera_info_from_json.py',
        name='astra_mini_camera_info_from_json',
        output='screen',
        parameters=[{
            'json_path': LaunchConfiguration('json_path'),
            'camera_info_key': LaunchConfiguration('camera_info_key'),
        }],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('calibrated_camera_info_topic')),
        ],
    )

    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='astra_mini_rectify_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rectify_image')),
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('calibrated_camera_info_topic')),
            ('image_rect', LaunchConfiguration('rectified_image_topic')),
        ],
    )

    return LaunchDescription([
        start_camera_arg,
        json_path_arg,
        camera_info_key_arg,
        image_topic_arg,
        calibrated_camera_info_topic_arg,
        rectify_image_arg,
        rectified_image_topic_arg,
        camera_launch,
        calibrated_camera_info_node,
        rectify_node,
    ])
