# TurtleBot3 with OpenMANIPULATOR
<img src="https://raw.githubusercontent.com/ROBOTIS-GIT/emanual/master/assets/images/platform/turtlebot3/manipulation/tb3_with_opm_logo.png" width="500">

<img src="https://raw.githubusercontent.com/ROBOTIS-GIT/emanual/master/assets/images/platform/turtlebot3/manipulation/hardware_setup.png" width="500">

## 로컬 실제 리더 joint3 명령 경로

실제 리더 로봇은 MoveIt Servo YAML의 `command_out_topic`을 그대로 쓰지 않는다.
`servo.launch.py`는 `command_out_topic` launch argument를 받고,
`mp_control/launch/real_pick_place.launch.py`는 이 값을
`/arm_controller/joint_trajectory_raw`로 설정한 뒤
`mp_control/tools/joint_trajectory_transformer.py`를 시작한다.

이 변환 노드는 현재 joint3 위치를 기준으로 joint3 이동량만 반전한 뒤
`/arm_controller/joint_trajectory`로 다시 발행한다. 그래서 Servo의
cancel/hold 명령은 현재 joint 각도를 유지하면서, 실제 매니퓰레이터에서 위험한
joint3 이동 방향만 반대로 바꾼다.

- Active Branches: noetic, humble, jazzy, main(rolling)
- Legacy Branches: *-devel

## Open Source Projects Related to TurtleBot3 and OpenMANIPULATOR
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [turtlebot3_manipulation](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation)
- [turtlebot3_manipulation_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations)
- [turtlebot3_applications](https://github.com/ROBOTIS-GIT/turtlebot3_applications)
- [turtlebot3_applications_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs)
- [turtlebot3_machine_learning](https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning)
- [turtlebot3_autorace](https://github.com/ROBOTIS-GIT/turtlebot3_autorace)
- [turtlebot3_home_service_challenge](https://github.com/ROBOTIS-GIT/turtlebot3_home_service_challenge)
- [hls_lfcd_lds_driver](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver)
- [ld08_driver](https://github.com/ROBOTIS-GIT/ld08_driver)
- [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [OpenCR-Hardware](https://github.com/ROBOTIS-GIT/OpenCR-Hardware)
- [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR)

## Documentation, Videos, and Community

### Official Documentation
- ⚙️ **[ROBOTIS DYNAMIXEL](https://dynamixel.com/)**
- 📚 **[ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)**
- 📚 **[ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)**
- 📚 **[ROBOTIS e-Manual for OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/)**

### Learning Resources
- 🎥 **[ROBOTIS YouTube Channel](https://www.youtube.com/@ROBOTISCHANNEL)**
- 🎥 **[ROBOTIS Open Source YouTube Channel](https://www.youtube.com/@ROBOTISOpenSourceTeam)**
- 🎥 **[ROBOTIS TurtleBot3 YouTube Playlist](https://www.youtube.com/playlist?list=PLRG6WP3c31_XI3wlvHlx2Mp8BYqgqDURU)**
- 🎥 **[ROBOTIS OpenMANIPULATOR YouTube Playlist](https://www.youtube.com/playlist?list=PLRG6WP3c31_WpEsB6_Rdt3KhiopXQlUkb)**

### Community & Support
- 💬 **[ROBOTIS Community Forum](https://forum.robotis.com/)**
- 💬 **[TurtleBot category from ROS Community](https://discourse.ros.org/c/turtlebot/)**
