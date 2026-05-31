# leader_platooning_beacon

리더 터틀봇의 플래투닝 상태를 팔로워와 모니터로 내보내는 패키지다.

## 주요 노드

`leader_platooning_beacon_node`는 리더의 작업 상태, 화물 상태, 추종 활성화 상태, heartbeat, odom, cmd_vel 백업 토픽을 발행한다.

`battery_state_from_dynamic_joint_state_node`는 매니퓰레이터 하드웨어 런치에서 생성되는 `/dynamic_joint_states`의 `battery` 센서 인터페이스를 표준 `/battery_state`로 변환한다.

`sensor_state_from_dynamic_joint_state_node`는 `/dynamic_joint_states`의 배터리와 바퀴 position 값을 이용해서 TurtleBot3 호환 `/sensor_state`를 발행한다. bumper, cliff, sonar, illumination, button 값은 ros2_control 하드웨어 인터페이스에서 제공되지 않으므로 0으로 유지한다.

`include/leader_platooning_beacon/dynamic_joint_state_utils.hpp`는 두 relay 노드가 공통으로 쓰는 `/dynamic_joint_states` 조회 유틸이다. 센서나 조인트 이름으로 `InterfaceValue`를 찾고, 특정 state interface 값을 안전하게 가져오는 역할만 담당한다.

## 하드웨어 런치 연동

리더 매니퓰레이터의 기본 하드웨어 런치인 `turtlebot3_manipulation_bringup/launch/hardware.launch.py`는 기본값으로 두 relay를 함께 실행한다.

```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

확인 토픽:

```bash
ros2 topic echo /battery_state --once
ros2 topic echo /sensor_state --once
ros2 topic echo /dynamic_joint_states --once
```

relay를 끄고 싶으면 다음처럼 실행한다.

```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py start_state_relays:=false
```
