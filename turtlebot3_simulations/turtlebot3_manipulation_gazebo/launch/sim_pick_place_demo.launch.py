from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rviz = LaunchConfiguration("start_rviz")
    gz_args = LaunchConfiguration("gz_args")
    world = LaunchConfiguration("world")
    demo_start_delay = LaunchConfiguration("demo_start_delay")
    return_to_stow = LaunchConfiguration("return_to_stow")
    direct_place_on_follower = LaunchConfiguration("direct_place_on_follower")
    base_approach_distance = LaunchConfiguration("base_approach_distance")
    base_approach_speed = LaunchConfiguration("base_approach_speed")
    base_transport_distance = LaunchConfiguration("base_transport_distance")
    base_transport_speed = LaunchConfiguration("base_transport_speed")
    base_turn_angle = LaunchConfiguration("base_turn_angle")
    base_turn_speed = LaunchConfiguration("base_turn_speed")
    post_place_move_distance = LaunchConfiguration("post_place_move_distance")
    post_place_move_speed = LaunchConfiguration("post_place_move_speed")
    post_place_reverse_distance = LaunchConfiguration("post_place_reverse_distance")
    post_place_reverse_speed = LaunchConfiguration("post_place_reverse_speed")
    post_place_turn_angle = LaunchConfiguration("post_place_turn_angle")
    post_place_turn_speed = LaunchConfiguration("post_place_turn_speed")
    show_follower = LaunchConfiguration("show_follower")
    follower_x = LaunchConfiguration("follower_x")
    follower_y = LaunchConfiguration("follower_y")
    follower_yaw = LaunchConfiguration("follower_yaw")
    follower_distance = LaunchConfiguration("follower_distance")
    follower_handoff_distance = LaunchConfiguration("follower_handoff_distance")
    follower_max_speed = LaunchConfiguration("follower_max_speed")
    follower_max_turn_speed = LaunchConfiguration("follower_max_turn_speed")
    follower_gazebo_entity_name = LaunchConfiguration("follower_gazebo_entity_name")
    follower_gazebo_x = LaunchConfiguration("follower_gazebo_x")
    follower_gazebo_y = LaunchConfiguration("follower_gazebo_y")
    follower_gazebo_z = LaunchConfiguration("follower_gazebo_z")
    follower_leader_odom_topic = LaunchConfiguration("follower_leader_odom_topic")
    follower_leader_reference_frame = LaunchConfiguration("follower_leader_reference_frame")
    follower_reference_frame = LaunchConfiguration("follower_reference_frame")
    place_on_follower = LaunchConfiguration("place_on_follower")
    follower_place_z = LaunchConfiguration("follower_place_z")
    leader_domain_id = LaunchConfiguration("leader_domain_id")
    start_leader_task_manager = LaunchConfiguration("start_leader_task_manager")
    start_leader_beacon = LaunchConfiguration("start_leader_beacon")
    start_domain_bridge = LaunchConfiguration("start_domain_bridge")
    description_share_parent = PathJoinSubstitution([
        FindPackageShare("turtlebot3_manipulation_description"),
        "..",
    ])
    gazebo_share_parent = PathJoinSubstitution([
        FindPackageShare("turtlebot3_manipulation_gazebo"),
        "..",
    ])

    follower_platooning_urdf = (
        "/home/ktj/Desktop/Turtlebot3_Platooning/src/"
        "turtlebot3_manipulation/turtlebot3_manipulation_description/"
        "urdf/turtlebot3_platooning.urdf"
    )
    follower_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="cat")]),
            " ",
            follower_platooning_urdf,
        ]),
        value_type=str,
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_gazebo"),
                "launch",
                "sim_hybrid_grasp.launch.py",
            ])
        ]),
        launch_arguments={
            "start_rviz": start_rviz,
            "start_gazebo": "true",
            "world": world,
            "gz_args": gz_args,
            "start_tracker": "true",
            "start_eef_tracker": "false",
            "start_eef_ibvs_feature": "true",
            "start_servo": "false",
            "start_mp_control": "false",
            "control_start_delay": "1.0",
        }.items(),
    )

    demo_node = Node(
        package="mp_control",
        executable="sim_pick_place_demo.py",
        name="sim_pick_place_demo",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"start_delay_s": demo_start_delay},
            {"return_to_stow": return_to_stow},
            {"direct_place_on_follower": direct_place_on_follower},
            {"base_approach_distance_m": base_approach_distance},
            {"base_approach_speed_mps": base_approach_speed},
            {"base_transport_distance_m": base_transport_distance},
            {"base_transport_speed_mps": base_transport_speed},
            {"base_turn_angle_rad": base_turn_angle},
            {"base_turn_speed_radps": base_turn_speed},
            {"post_place_move_distance_m": post_place_move_distance},
            {"post_place_move_speed_mps": post_place_move_speed},
            {"post_place_reverse_distance_m": post_place_reverse_distance},
            {"post_place_reverse_speed_mps": post_place_reverse_speed},
            {"post_place_turn_angle_rad": post_place_turn_angle},
            {"post_place_turn_speed_radps": post_place_turn_speed},
            {"place_on_follower": place_on_follower},
            {"follower_place_frame": "follower_base_footprint"},
            {"follower_place_xyz": [0.0, 0.0, 0.12]},
            {"follower_place_z_m": follower_place_z},
            {"follower_handoff_wait_s": 2.5},
            {"cmd_vel_topic": "/diff_drive_controller/cmd_vel_unstamped"},
            {"cmd_vel_wait_timeout_s": 90.0},
            {"trajectory_wait_timeout_s": 90.0},
            {"gripper_wait_timeout_s": 90.0},
            {"require_cmd_vel_subscriber": True},
            {"require_trajectory_subscriber": True},
            {"require_gripper_action_server": True},
            {"publish_demo_base_tf": False},
            {"publish_demo_joint_states": False},
            {"gazebo_pose_update_period_s": 0.033},
            {"cargo_id_prefix": "SIM-PKG"},
        ],
    )

    leader_task_manager_node = Node(
        package="leader_task_manager",
        executable="leader_task_manager_node",
        name="leader_task_manager",
        output="screen",
        condition=IfCondition(start_leader_task_manager),
        parameters=[{
            "mp_control_status_topic": "/mp_control/pick_place_status",
            "cargo_events_topic": "/cargo/events",
            "cargo_current_id_topic": "/cargo/current_id",
            "task_state_topic": "/leader/task_state",
            "cargo_state_topic": "/leader/cargo_state",
            "follower_enable_topic": "/leader/follower_enable",
            "platoon_mode_topic": "/leader/platoon_mode",
            "initial_task_state": "IDLE",
            "initial_cargo_state": "EMPTY",
            "initial_platoon_mode": "STOP",
            "initial_follower_enable": False,
        }],
    )

    leader_beacon_node = Node(
        package="leader_platooning_beacon",
        executable="leader_platooning_beacon_node",
        name="leader_platooning_beacon",
        output="screen",
        condition=IfCondition(start_leader_beacon),
        parameters=[{
            "heartbeat_rate_hz": 10.0,
            "publish_odom_backup": True,
            "publish_cmd_vel_backup": True,
            "odom_topic": "/diff_drive_controller/odom",
            "cmd_vel_topic": "/diff_drive_controller/cmd_vel_unstamped",
            "heartbeat_topic": "/leader/heartbeat",
            "leader_odom_topic": "/leader/odom",
            "leader_cmd_vel_topic": "/leader/cmd_vel",
        }],
    )

    domain_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("platooning_bridge_config"),
                "launch",
                "bridge.launch.py",
            ])
        ]),
        condition=IfCondition(start_domain_bridge),
    )

    follower_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="follower_robot_state_publisher",
        output="screen",
        condition=IfCondition(show_follower),
        parameters=[
            {"robot_description": follower_description},
            {"use_sim_time": True},
            {"frame_prefix": "follower_"},
        ],
        remappings=[
            ("robot_description", "/follower/robot_description"),
            ("joint_states", "/follower/joint_states"),
        ],
    )

    follower_gazebo_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_platooning_follower",
        output="screen",
        condition=IfCondition(show_follower),
        arguments=[
            "-name", follower_gazebo_entity_name,
            "-topic", "/follower/robot_description",
            "-x", follower_gazebo_x,
            "-y", follower_gazebo_y,
            "-z", follower_gazebo_z,
            "-R", "0.0",
            "-P", "0.0",
            "-Y", follower_yaw,
        ],
    )

    follower_platooning_node = Node(
        package="turtlebot3_manipulation_gazebo",
        executable="follower_platooning_visualizer.py",
        name="follower_platooning_visualizer",
        output="screen",
        condition=IfCondition(show_follower),
        parameters=[
            {"use_sim_time": True},
            {"leader_odom_topic": follower_leader_odom_topic},
            {"parent_frame": "odom"},
            {"follower_frame": "follower_base_footprint"},
            {"leader_reference_frame": follower_leader_reference_frame},
            {"follower_reference_frame": follower_reference_frame},
            {"follower_odom_topic": "/follower/odom"},
            {"target_distance_m": follower_distance},
            {"handoff_distance_m": follower_handoff_distance},
            {"initial_offset_x_m": follower_x},
            {"initial_offset_y_m": follower_y},
            {"initial_yaw_offset_rad": follower_yaw},
            {"status_topic": "/mp_control/pick_place_status"},
            {"max_linear_speed_mps": follower_max_speed},
            {"max_angular_speed_radps": follower_max_turn_speed},
            {"linear_gain": 0.85},
            {"angular_gain": 2.4},
            {"distance_deadband_m": 0.03},
            {"publish_rate_hz": 30.0},
            {"sync_gazebo_entity": True},
            {"gazebo_set_pose_service": "/world/default/set_pose"},
            {"gazebo_entity_name": follower_gazebo_entity_name},
            {"gazebo_world_origin_xyz": [-2.12, -0.5, 0.0]},
            {"gazebo_pose_z_m": follower_gazebo_z},
            {"gazebo_pose_update_period_s": 0.033},
            {"gazebo_pose_smoothing_alpha": 0.35},
            {"post_place_reverse_distance_m": post_place_reverse_distance},
            {"post_place_reverse_speed_mps": post_place_reverse_speed},
            {"post_place_turn_angle_rad": post_place_turn_angle},
            {"post_place_turn_speed_radps": post_place_turn_speed},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "leader_domain_id",
            default_value="25",
            description="ROS_DOMAIN_ID used by the leader simulation side and host monitor bridge.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_gazebo"),
                "worlds",
                "grasp_10m_room.world",
            ]),
            description="Gazebo 10 m x 10 m room world containing the far red pick object.",
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value=["-r --headless-rendering ", world],
            description="Arguments passed to Gazebo Sim.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz to show robot, object marker, and place marker.",
        ),
        DeclareLaunchArgument(
            "demo_start_delay",
            default_value="10.0",
            description="Seconds to wait before publishing bbox and executing the demo trajectory.",
        ),
        DeclareLaunchArgument(
            "return_to_stow",
            default_value="true",
            description="Return the arm to the stow pose after placing the object.",
        ),
        DeclareLaunchArgument(
            "direct_place_on_follower",
            default_value="true",
            description="After grasping, place directly onto the follower without base transport.",
        ),
        DeclareLaunchArgument(
            "base_approach_distance",
            default_value="0.30",
            description="Meters to drive before grasping. Matches the hybrid depth simulation desired depth; object pose is generated from the post-drive grasp pose.",
        ),
        DeclareLaunchArgument(
            "base_approach_speed",
            default_value="0.12",
            description="Base linear speed in m/s during the approach stage.",
        ),
        DeclareLaunchArgument(
            "base_transport_distance",
            default_value="0.00",
            description="Meters to drive after grasping and before placing when direct placement is disabled.",
        ),
        DeclareLaunchArgument(
            "base_transport_speed",
            default_value="0.12",
            description="Base linear speed in m/s during the transport stage.",
        ),
        DeclareLaunchArgument(
            "base_turn_angle",
            default_value="0.00",
            description="Radians to rotate after picking when direct placement is disabled.",
        ),
        DeclareLaunchArgument(
            "base_turn_speed",
            default_value="0.45",
            description="Base angular speed in rad/s during the post-pick turn.",
        ),
        DeclareLaunchArgument(
            "post_place_move_distance",
            default_value="1.00",
            description="Meters to drive after placing cargo on the follower.",
        ),
        DeclareLaunchArgument(
            "post_place_move_speed",
            default_value="0.12",
            description="Base forward speed in m/s after reversing and turning.",
        ),
        DeclareLaunchArgument(
            "post_place_reverse_distance",
            default_value="0.35",
            description="Meters to back up after placing cargo on the follower.",
        ),
        DeclareLaunchArgument(
            "post_place_reverse_speed",
            default_value="0.10",
            description="Base reverse speed in m/s after placing cargo on the follower.",
        ),
        DeclareLaunchArgument(
            "post_place_turn_angle",
            default_value="1.5708",
            description="Radians to rotate after post-place reverse before driving forward.",
        ),
        DeclareLaunchArgument(
            "post_place_turn_speed",
            default_value="0.45",
            description="Base angular speed in rad/s during the post-place turn.",
        ),
        DeclareLaunchArgument(
            "show_follower",
            default_value="true",
            description="Show and spawn a follower TurtleBot3 model for platooning visualization.",
        ),
        DeclareLaunchArgument(
            "follower_x",
            default_value="-0.45",
            description="Follower visualization initial x offset from the leader reference frame.",
        ),
        DeclareLaunchArgument(
            "follower_y",
            default_value="0.00",
            description="Follower visualization initial y offset from the leader reference frame.",
        ),
        DeclareLaunchArgument(
            "follower_yaw",
            default_value="0.00",
            description="Follower visualization yaw offset from the leader reference frame.",
        ),
        DeclareLaunchArgument(
            "follower_distance",
            default_value="0.45",
            description="Target IMU-to-IMU following distance behind the leader in meters.",
        ),
        DeclareLaunchArgument(
            "follower_handoff_distance",
            default_value="0.45",
            description="IMU-to-IMU follower spacing used while the leader places cargo on the follower deck.",
        ),
        DeclareLaunchArgument(
            "follower_leader_odom_topic",
            default_value="/diff_drive_controller/odom",
            description="Leader odometry topic used by the follower platooning visualizer.",
        ),
        DeclareLaunchArgument(
            "follower_leader_reference_frame",
            default_value="imu_link",
            description="Leader TF frame used as the platooning distance reference.",
        ),
        DeclareLaunchArgument(
            "follower_reference_frame",
            default_value="follower_imu_link",
            description="Follower TF frame used as the platooning distance reference.",
        ),
        DeclareLaunchArgument(
            "place_on_follower",
            default_value="true",
            description="Place the released object marker on the follower cargo deck frame.",
        ),
        DeclareLaunchArgument(
            "follower_place_z",
            default_value="0.12",
            description="Object center height above follower_base_footprint when placed on the follower.",
        ),
        DeclareLaunchArgument(
            "follower_max_speed",
            default_value="0.24",
            description="Follower visualization maximum linear speed in m/s.",
        ),
        DeclareLaunchArgument(
            "follower_max_turn_speed",
            default_value="1.20",
            description="Follower visualization maximum angular speed in rad/s.",
        ),
        DeclareLaunchArgument(
            "follower_gazebo_entity_name",
            default_value="turtlebot3_platooning_follower",
            description="Gazebo entity name used for the spawned platooning follower.",
        ),
        DeclareLaunchArgument(
            "follower_gazebo_x",
            default_value="-2.45",
            description="Initial Gazebo world x position for the platooning follower.",
        ),
        DeclareLaunchArgument(
            "follower_gazebo_y",
            default_value="-0.50",
            description="Initial Gazebo world y position for the platooning follower.",
        ),
        DeclareLaunchArgument(
            "follower_gazebo_z",
            default_value="0.00",
            description="Initial Gazebo world z position for the platooning follower.",
        ),
        DeclareLaunchArgument(
            "start_leader_task_manager",
            default_value="true",
            description="Publish leader task/cargo/platoon state for follower communication.",
        ),
        DeclareLaunchArgument(
            "start_leader_beacon",
            default_value="true",
            description="Publish leader heartbeat, odometry, and cmd_vel backup topics.",
        ),
        DeclareLaunchArgument(
            "start_domain_bridge",
            default_value="true",
            description="Bridge /leader topics from the leader simulation domain to the follower domain.",
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [
                description_share_parent,
                ":",
                gazebo_share_parent,
                ":",
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            [
                description_share_parent,
                ":",
                gazebo_share_parent,
                ":",
                EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", leader_domain_id),
        leader_task_manager_node,
        leader_beacon_node,
        domain_bridge_launch,
        follower_state_publisher,
        TimerAction(period=2.0, actions=[follower_gazebo_spawn]),
        follower_platooning_node,
        sim_launch,
        TimerAction(period=2.0, actions=[demo_node]),
    ])
