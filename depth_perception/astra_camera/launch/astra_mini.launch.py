from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory
import yaml


def generate_launch_description():
    params_file = get_package_share_directory(
        "astra_camera") + "/params/astra_mini_params.yaml"
    with open(params_file, 'r') as file:
        config_params = yaml.safe_load(file)

    use_sim = LaunchConfiguration('use_sim')

    hardware_container = ComposableNodeContainer(
        name='astra_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::OBCameraNodeFactory',
                           name='camera',
                           namespace='camera',
                           parameters=[config_params]),
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::PointCloudXyzNode',
                           namespace='camera',
                           name='point_cloud_xyz'),
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::PointCloudXyzrgbNode',
                           namespace='camera',
                           name='point_cloud_xyzrgb')
        ],
        output='screen',
        condition=UnlessCondition(use_sim))

    sim_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            'camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            'camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            'camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            'camera/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen',
        condition=IfCondition(use_sim))

    sim_container = ComposableNodeContainer(
        name='astra_camera_sim_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::PointCloudXyzNode',
                           namespace='camera',
                           name='point_cloud_xyz'),
        ],
        output='screen',
        condition=IfCondition(use_sim))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use Gazebo camera topics instead of the physical Astra Mini device.',
        ),
        hardware_container,
        sim_bridge,
        sim_container,
    ])
