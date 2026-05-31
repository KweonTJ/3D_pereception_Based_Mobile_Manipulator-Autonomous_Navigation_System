# turtlebot3_manipulation_teleop

터틀봇3 매니퓰레이터 실제 로봇을 SSH 터미널에서 키보드로 조작하는 패키지다.

## 실행

리더 로봇 SSH 세션에서 실행한다.

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run turtlebot3_manipulation_teleop turtlebot3_manipulation_teleop
```

## 베이스 텔레옵

키보드의 `i`, `k`, `j`, `l`, `space` 입력은 `/cmd_vel`로 발행된다.

```text
i: 전진 속도 증가
k: 후진 속도 증가
j: 좌회전
l: 우회전
space: 정지
```

이 패키지는 `cmd_vel` publisher를 직접 만들고 10 ms 주기로 현재 명령을 발행한다. `/cmd_vel` 발행이 꺼져 있으면 키 입력 로그는 보여도 실제 베이스는 움직이지 않는다.

실제 리더 하드웨어 기준으로 `j`는 음수 `angular.z`, `l`은 양수 `angular.z`를 발행한다. 전진/후진 방향은 그대로 유지하고 좌/우 회전 부호만 하드웨어 방향에 맞춘다.

## 확인

텔레옵이 동작하지 않을 때는 다른 SSH 터미널에서 다음을 확인한다.

```bash
ros2 topic info -v /cmd_vel
ros2 topic echo /cmd_vel --once
ros2 topic info -v /diff_drive_controller/cmd_vel_unstamped
ros2 control list_controllers
```

`/cmd_vel`에 `servo_keyboard_input` publisher가 보여야 한다. `diff_drive_controller`가 active가 아니면 `/cmd_vel`이 발행되어도 로봇이 움직이지 않는다.
