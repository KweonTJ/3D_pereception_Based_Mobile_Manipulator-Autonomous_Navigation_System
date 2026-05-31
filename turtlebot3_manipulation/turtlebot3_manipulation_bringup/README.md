# turtlebot3_manipulation_bringup

리더 터틀봇 매니퓰레이터의 실제 하드웨어 브링업 패키지다. `hardware.launch.py`는 ros2_control 기반으로 OpenCR, 바퀴, 매니퓰레이터, 그리퍼, IMU broadcaster, 카메라 드라이버를 실행한다.

## 실행

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

## 배터리와 센서 상태

이 하드웨어 런치는 표준 `turtlebot3_node`를 실행하지 않는다. 따라서 TurtleBot3 기본 노드가 직접 발행하던 `/battery_state`, `/sensor_state`는 자동으로 생기지 않는다.

현재 런치는 `leader_platooning_beacon` 패키지의 relay 노드를 기본으로 함께 실행해서 ros2_control의 `/dynamic_joint_states`를 다음 토픽으로 변환한다.

```text
/dynamic_joint_states[battery] -> /battery_state
/dynamic_joint_states[battery, wheel_left_joint, wheel_right_joint] -> /sensor_state
```

확인:

```bash
ros2 topic echo /dynamic_joint_states --once
ros2 topic echo /battery_state --once
ros2 topic echo /sensor_state --once
```

relay를 끄려면 다음 옵션을 사용한다.

```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py start_state_relays:=false
```
