# mp_control

리더 TurtleBot3 매니퓰레이터의 실제 로봇 파지 흐름을 제어하는 패키지다.

이 패키지는 전면 RGB-D 추적기, EEF RGB 카메라, MoveIt Servo, 그리퍼 액션을
하나의 파지 시퀀스로 연결한다.

## 현재 실제 로봇 파지 흐름

실제 로봇에서 사용하는 주요 설정은 아래 두 파일이다.

```text
mp_control/launch/real_pick_place.launch.py
mp_control/config/mp_control_real_params.yaml
```

실행 흐름:

1. 전면 YOLO가 `/target/init_bbox`를 발행한다.
2. `hybrid_csrt_ibvs`가 전면 카메라 물체를 추적하고 `/target/tracked_bbox`를 발행한다.
3. Astra 전면 depth가 유효한 구간에서는 RGB-D 기반으로 베이스가 물체에 접근한다.
4. EEF YOLO는 런치 시작부터 켜서 depth handoff 전에 bbox가 준비되도록 한다.
5. 전면 depth가 근접 한계에 도달하면 이후 depth 거리값은 새 거리 추정에 쓰지 않는다.
6. 전면 RGB와 EEF RGB bbox로 컬러 삼각 측량을 수행해 물체와 로봇 사이 거리를 좁힌다.
7. 베이스를 정지시키고, 조인트 pregrasp 자세로 팔을 이동한 뒤 EEF RGB 보정 후 그리퍼를 닫는다.

## 주요 실제 로봇 파라미터

현재 depth, 거리, pregrasp 관련 핵심 값은 아래와 같다.

```yaml
min_valid_depth_m: 0.47
eef_refinement_start_depth_m: 0.47
eef_yolo_pre_enable_depth_m: 0.60
min_depth_handoff_bbox_area_ratio: 0.08
min_depth_handoff_bbox_height_ratio: 0.40
color_triangulation_base_stop_object_x_m: 0.30
arm_start_max_object_x_m: 0.30
use_fallback_bbox_for_control: true
start_servo_on_start: false
use_joint_pregrasp: true
joint_trajectory_topic: /arm_controller/joint_trajectory
pregrasp_ready_joint_positions: [0.0, 0.65, -0.85, -1.20]
pregrasp_preserve_gripper_roll: true
pregrasp_reverse_joint3_delta: true
pregrasp_hold_current_duration_s: 0.0
pregrasp_sync_steps: 1
pregrasp_joint_tolerance_rad: 0.04
pregrasp_republish_period_s: 1.0
object_pregrasp_standoff_m: 0.08
use_eef_rpy_refinement: true
eef_hold_current_rpy: true
eef_hold_stay_roll: true
eef_forward_after_align: true
eef_forward_distance_m: 0.05
eef_forward_speed_mps: 0.012
gripper_grasp_clearance_m: 0.004
position_tolerance_m: 0.035
close_after_stable_cycles: 4
```

의미:

- `0.60 m`: 전면 depth가 아직 유효할 때 EEF YOLO를 미리 켠다.
- `0.47 m`: 이 거리부터 전면 depth를 새 물체 거리 추정에 신뢰하지 않는다.
- `0.08 / 0.40`: 전면 bbox가 이미지 면적 8% 이상이거나 높이 40% 이상이면 depth 대기를 끝내고 근접 RGB/EEF handoff 경로로 넘어간다.
- `0.30 m`: 컬러 삼각 측량 기반 베이스 접근 목표 거리다. 이 거리 이후 팔 파지 단계로 넘어간다.
- `use_fallback_bbox_for_control`: CSRT `/target/tracked_bbox`가 멈추면 전면 YOLO `/target/init_bbox`를 fallback으로 사용한다.
- `start_servo_on_start`: `mp_control` 내부 Servo 자동 시작은 꺼져 있다. 실제 런치에서는 Servo 출력이 먼저 `/arm_controller/joint_trajectory_raw`로 나가고, 조인트 trajectory 변환 노드가 joint3 이동량만 반전해 `/arm_controller/joint_trajectory`로 다시 발행한다.
- `use_joint_pregrasp`: 실제 로봇 pregrasp는 Cartesian Servo가 아니라 `/arm_controller/joint_trajectory`로 직접 보낸다.
- `pregrasp_preserve_gripper_roll`: pregrasp 목표를 만들 때 현재 stay 자세의 `joint2 + joint3 + joint4` 합을 유지하도록 `joint4`를 계산한다. 이 값은 joint3 실제 모터 방향 보정 전에 계산하므로, joint2/3/4가 같은 trajectory point에서 동시에 움직이면서 그리퍼 roll이 무너지지 않게 한다.
- `pregrasp_hold_current_duration_s`: 기본값은 `0.0`이다. pregrasp 시작 전에 현재 자세 hold point를 추가하지 않아서 시작 지연을 만들지 않는다.
- `pregrasp_sync_steps`: 기본값은 `1`이다. pregrasp trajectory에는 joint1~4가 모두 들어간 단일 목표 point만 들어가며, joint2/3/4가 같은 `time_from_start`로 동시에 목표에 도달하도록 한다. 이 값을 2 이상으로 올리면 중간 waypoint가 생겨 실제 로봇에서 끊긴 동작처럼 보일 수 있다.
- `pregrasp_joint_tolerance_rad`: `/joint_states`가 pregrasp 목표에 이 오차 안으로 들어와야 EEF 보정과 그리퍼 닫기를 허용한다.
- `pregrasp_republish_period_s`: arm controller가 1회 trajectory를 놓치면 같은 pregrasp trajectory를 주기적으로 재발행한다.
- `pregrasp_reverse_joint3_delta`: 실제 로봇에서는 켜져 있다. 소프트웨어 grasp 표의 ready 목표는 `joint3=-0.85 rad`로 유지하지만, 실제 명령은 현재 joint3 기준 이동량만 반대로 보낸다.
- `0.08 m`: 삼각 측량된 물체 위치에서 EEF pregrasp standoff로 남기는 거리다.

## 실제 로봇 joint3 trajectory 변환

리더 실제 로봇의 joint3 모터 회전 방향은 소프트웨어 trajectory 방향과 반대다. 이 문제는 하드웨어 드라이버에서 수정하지 않고 trajectory 변환 계층으로 처리한다.

실제 런치의 경로:

```text
MoveIt Servo
  -> /arm_controller/joint_trajectory_raw
  -> joint_trajectory_transformer.py
  -> /arm_controller/joint_trajectory
  -> arm_controller
```

변환 노드는 `/joint_states`를 구독하고 joint3 이동량만 현재 joint3 위치 기준으로 반전한다. trajectory 첫 번째 point를 기준점으로 쓰지 않는다. 첫 point를 기준으로 잡으면 pregrasp/Servo 목표점이 그대로 기준점이 되어 joint3 반전이 사라지기 때문이다.

```text
joint3_out = current_joint3 - (joint3_in - current_joint3)
```

Servo가 런치 종료나 cancel 과정에서 현재 위치 hold 명령을 보내면, 입력 목표와 `/joint_states` 기준점이 같아서 출력도 현재 위치 그대로 유지된다. 단순히 절대각을 `-joint3`으로 바꾸지 않기 때문에 취소 시 반대 절대각으로 튀지 않는다.

확인 명령:

```bash
ros2 topic info -v /arm_controller/joint_trajectory_raw
ros2 topic info -v /arm_controller/joint_trajectory
ros2 topic echo /arm_controller/joint_trajectory_raw --once
ros2 topic echo /arm_controller/joint_trajectory --once
```

실제 런치에서는 `/servo_node`가 raw 토픽을 발행하고, `joint_trajectory_transformer`가 실제 arm controller 토픽을 발행해야 한다. `mp_control`의 1회 pregrasp 명령은 기존처럼 controller 토픽으로 직접 나가지만, 내부의 `pregrasp_reverse_joint3_delta` 로직이 같은 현재 위치 기준 delta 반전을 수행한다.

## EEF 카메라 경로

EEF 카메라는 depth가 없는 RGB 전용 근접 보정 카메라다. 실제 런치는 EEF YOLO를 즉시 시작하고 `/target/eef_init_bbox`를 계속 발행한다. `mp_control`은 근접 파지 단계에서만 이 bbox를 사용한다.

현재 제어 입력 토픽:

```yaml
eef_bbox_topic: /target/eef_init_bbox
```

즉 실제 파지 제어는 EEF YOLO bbox를 직접 사용한다. EEF용 `hybrid_csrt_ibvs`는 디버그용으로 띄울 수 있지만 `/target/eef_tracked_bbox`는 현재 실제 파지 제어 입력이 아니다.

EEF 보정은 픽셀 정렬과 stay-roll/current-pitch-yaw 유지를 같이 쓴다.

```yaml
eef_close_tolerance_px: 70.0
```

이 tolerance는 설정된 EEF 카메라 해상도를 기준으로 스케일된다. 따라서 320x240 EEF 카메라와 640x480 전면 카메라의 픽셀 오차를 같은 기준으로 보지 않는다. 현재 실제 EEF 카메라에서는 bbox가 물체에 딱 맞으면 물체 중심이 optical center보다 약 60 px 아래로 보일 수 있으므로, `70 px`를 close-ready 기준으로 사용한다.

EEF bbox 정렬이 완료되면 `mp_control`은 초기 stay 자세에서 캡처한 EE roll을 유지하고, refinement 진입 시점의 pitch/yaw를 유지한다. 그 상태에서 `base_link` 기준으로 짧게 전진한 뒤 그리퍼를 닫는다.

```yaml
use_eef_rpy_refinement: true
eef_hold_current_rpy: true
eef_hold_stay_roll: true
eef_rpy_tolerance_rad: 0.12
eef_rpy_gain: 0.8
eef_refine_max_angular_speed: 0.25
eef_forward_after_align: true
eef_forward_distance_m: 0.05
eef_forward_speed_mps: 0.012
```

이 단계에서 `/mp_control/status`에는 `rpy_err=(roll, pitch, yaw)`, `roll_ref=stay_roll`, `rpy_ready`, forward advance 진행 상태가 표시된다.

EEF 카메라는 그리퍼 폭을 보정하지 않는다. 그리퍼 폭은 전면 depth에서 얻은 물체 폭만 사용한다. 유효한 전면 depth 폭이 없으면 fallback 폭을 쓴다. EEF RGB는 EE가 약간 틀어진 경우 최종 위치 보정에만 사용한다.

최종 그리퍼 명령은 `object_width + gripper_grasp_clearance_m`를 사용한다. 그래서 측정된 물체 폭보다 약간 크게 벌린 상태로 닫기 명령을 보내며, `gripper_grasp_clearance_m: 0.0`이면 기존 `gripper_grasp_compression_m` 방식으로 돌아간다.

## 전면-EEF RGB 삼각 측량

0.47 m depth handoff 이후 `mp_control`은 아래 정보를 사용해 물체를 삼각 측량한다.

```text
/target/tracked_bbox      전면 RGB bbox
/target/eef_init_bbox     EEF RGB YOLO bbox
/camera/color/camera_info_calibrated
/eef_camera/camera_info
/tf
```

EEF 카메라와 전면 카메라의 측정 위치 차이는 삼각 측량에만 적용한다.

```yaml
use_eef_front_camera_extrinsic_override: true
eef_front_camera_offset_x_m: -0.05
eef_front_camera_offset_y_m: 0.0
eef_front_camera_offset_z_m: 0.15
```

이는 `base_link` 축 기준으로 EEF 카메라가 전면 카메라보다 x축 -5 cm, z축 +15 cm 위치에 있다는 뜻이다. 이 값은 MoveIt/arm TF 자체를 바꾸지 않는다.

## 캘리브레이션 파일

전면 Astra color 캘리브레이션:

```text
../depth_perception/astra_mini_calibration/config/astra_mini_color.json
```

EEF USB 카메라 캘리브레이션:

```text
calibration/eef_camera/eef_usb_camera.yaml
```

현재 EEF 캘리브레이션은 320x240 EEF 카메라를 약 50 cm 거리에서 캘리브레이션한 값이다. 실제 런타임 토픽은 아래와 같다.

```text
/eef_camera/image_raw
/eef_camera/camera_info
/eef_camera/set_camera_info
```

카메라 캘리브레이션 명령과 service remapping은 아래 문서를 사용한다.

```text
calibration/eef_camera/README.md
```

전면-EEF stereo 캘리브레이션 캡처와 검증 스크립트는 아래 문서에 정리되어 있다.

```text
scripts/STEREO_CALIBRATION_USAGE.md
```

실행 기준 디렉토리:

```bash
cd ~/turtlebot3_ws/src/mp_control
python3 scripts/...
```

스크립트는 캡처 시 `/camera/color/image_raw`, `/eef_camera/image_raw`를 사용한다. Intrinsics는 `../depth_perception/.../astra_mini_color.json`과 `calibration/eef_camera/eef_usb_camera.yaml`에서 읽는다.

각 스크립트는 GUI 경로가 있다. 캡처는 `--manual` 또는 `--display`, calibration/validation/YAML summary는 `--display` 옵션을 사용한다.

Stereo 스크립트 출력은 진단용이다. 실제 파지 런타임은 기존 front JSON과 EEF YAML을 `/camera/color/camera_info_calibrated`, `/eef_camera/camera_info`를 통해 계속 사용한다.

## 실행 중 확인 명령

```bash
ros2 topic echo /mp_control/status --once
ros2 topic echo /target/tracked_bbox --once
ros2 topic echo /target/eef_init_bbox --once
ros2 topic echo /eef_camera/camera_info --once
ros2 topic echo /servo_node/status --once
```

RGB 삼각 측량이 활성화되면 `/mp_control/status`에 아래 값이 포함되어야 한다.

```text
offset_override=true
eef_front_offset=(-0.05, 0, 0.15)
```

joint3 trajectory 변환 경로 확인:

```bash
ros2 topic info -v /arm_controller/joint_trajectory_raw
ros2 topic info -v /arm_controller/joint_trajectory
```

## 빌드

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mp_control --symlink-install
```
