#!/usr/bin/env python3
"""leader_runtime.launch.py — 리더 단일 통합 런타임 (option B).

구성:
  real_pick_place(bringup+grasp+task_manager+beacon+bridge, mp_control은 끔)
  + mp_control 재기동(base→/cmd_vel_pnp, param override만)
  + rover_nav 주행(rsp 중복 회피: publish_description:=false)
  + imu_forward_align(startup 정렬, base→/cmd_vel_align, align_delay 후 1회)
  + cmd_vel_mux(state-select 중재 → /cmd_vel)

base 명령 흐름:
  imu_align → /cmd_vel_align ┐
  mp_control → /cmd_vel_pnp   ├ cmd_vel_mux(align > pnp[/pnp/working] > nav) → /cmd_vel → diff_drive
  rover_nav  → /leader/cmd_vel┘

정렬은 이 런치(=같은 bringup/IMU 세션) startup에 1회 → 재기동마다 fresh 정렬.
기존 노드 코드 수정 0 (토픽은 전부 launch param override).
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mp_cfg = LaunchConfiguration('mp_control_config_file')
    start_align = LaunchConfiguration('start_align')

    real_pick_place = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('mp_control'), 'launch', 'real_pick_place.launch.py'])]),
        launch_arguments={
            'start_mp_control': 'false',                 # 아래서 redirect 재기동
            'mp_control_config_file': mp_cfg,
        }.items())

    mp_control = Node(
        package='mp_control', executable='mp_control_node', name='mp_control_node',
        output='screen',
        parameters=[mp_cfg, {'base_cmd_vel_topic': '/cmd_vel_pnp'}])

    rover_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('leader_line_follower'), 'launch', 'leader_rover.launch.py'])]),
        launch_arguments={
            'publish_description': 'false',
            'cmd_vel_topic': '/leader/cmd_vel',
        }.items())   # rsp는 hardware 것 사용, nav 출력은 mux 입력으로 유지

    imu_align = TimerAction(period=6.0, condition=IfCondition(start_align), actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('leader_line_follower'), 'launch',
                'imu_forward_align.launch.py'])]),
            launch_arguments={'cmd_vel_topic': '/cmd_vel_align', 'linear_x': '0.2'}.items())])

    mux = Node(
        package='leader_line_follower', executable='cmd_vel_mux', name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'output_topic': '/cmd_vel',
            'align_topic': '/cmd_vel_align',
            'pnp_topic': '/cmd_vel_pnp',
            'nav_topic': '/leader/cmd_vel',
            'pnp_working_topic': '/pnp/working',
        }])

    return LaunchDescription([
        DeclareLaunchArgument('mp_control_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mp_control'), 'config', 'mp_control_real_params.yaml'])),
        DeclareLaunchArgument('start_align', default_value='true'),
        real_pick_place,
        mp_control,
        rover_nav,
        imu_align,
        mux,
    ])
