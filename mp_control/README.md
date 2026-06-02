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
color_triangulation_base_stop_object_x_m: 0.20
arm_start_max_object_x_m: 0.20
use_fallback_bbox_for_control: true
start_servo_on_start: false
require_visual_grasp_confirmation: true
use_joint_pregrasp: true
joint_trajectory_topic: /arm_controller/joint_trajectory_raw
pregrasp_ready_joint_positions: [0.0, 0.50, -0.85, -1.05]
pregrasp_preserve_gripper_roll: true
pregrasp_roll_joint2_weight: 1.0
pregrasp_roll_joint3_weight: 1.0
pregrasp_roll_joint4_weight: 1.0
pregrasp_reverse_joint3_delta: true
pregrasp_hold_current_duration_s: 0.0
pregrasp_sync_steps: 1
pregrasp_joint_tolerance_rad: 0.04
pregrasp_republish_period_s: 1.0
object_pregrasp_standoff_m: 0.06
use_eef_rpy_refinement: true
eef_hold_current_rpy: true
eef_hold_stay_roll: true
eef_forward_after_align: true
eef_forward_distance_m: 0.08
eef_forward_speed_mps: 0.018
eef_forward_start_tolerance_px: 90.0
eef_forward_use_joint_nudge: true
eef_forward_joint2_delta_rad: 0.0
eef_forward_joint3_delta_rad: -0.050
eef_forward_joint_nudge_duration_s: 0.45
eef_forward_joint_nudge_period_s: 0.35
eef_forward_joint3_first_duration_ratio: 0.0
eef_forward_roll_joint2_weight: 0.0
eef_forward_roll_joint3_weight: 1.5
eef_forward_roll_joint4_weight: 1.0
eef_forward_joint4_rpy_roll_gain: 0.6
eef_forward_joint4_rpy_roll_max_delta_rad: 0.04
gripper_down_joint4_offset_rad: 0.0
close_on_front_bbox_shrink: true
front_bbox_close_area_ratio: 0.60
close_on_eef_bbox_shrink: true
eef_bbox_close_area_ratio: 0.60
handoff_after_grasp: true
handoff_lift_joint2_delta_rad: 0.25
handoff_place_joint2_delta_rad: -0.20
handoff_rotate_angle_rad: 3.14159265
handoff_rotate_angular_speed_rad_s: 0.45
gripper_grasp_clearance_m: 0.004
gripper_grasp_width_scale: 1.50
position_tolerance_m: 0.035
close_after_stable_cycles: 4
grasp_completion_front_max_age_s: 2.0
grasp_completion_eef_lost_timeout_s: 0.8
```

의미:

- `0.60 m`: 전면 depth가 아직 유효할 때 EEF YOLO를 미리 켠다.
- `0.47 m`: 이 거리부터 전면 depth를 새 물체 거리 추정에 신뢰하지 않는다.
- `0.08 / 0.40`: 전면 bbox가 이미지 면적 8% 이상이거나 높이 40% 이상이면 depth 대기를 끝내고 근접 RGB/EEF handoff 경로로 넘어간다.
- `0.20 m`: 컬러 삼각 측량 기반 베이스 접근 목표 거리다. 이 거리 이후 팔 파지 단계로 넘어간다. 실제 전면 카메라는 `base_link`보다 앞에 있으므로, `base_link` 변환 x가 20cm를 넘더라도 전면 bbox 크기 기반 camera-range가 20cm 이하이면 close-range로 보고 팔 pregrasp를 시작한다.
- `require_visual_grasp_confirmation`: 그리퍼 close 명령 직후 바로 파지 완료로 보지 않는다. 전면 bbox는 계속 보이고, EEF bbox는 사라져야 `/cargo/events`에 `picked`를 발행한다.
- `grasp_completion_front_max_age_s`: 파지 완료 판정에 사용할 전면 bbox freshness 한계다.
- `grasp_completion_eef_lost_timeout_s`: 이 시간 동안 EEF bbox가 새로 들어오지 않으면 EEF에서 물체가 사라진 것으로 본다.
- `handoff_after_grasp`: 파지 시각 확인 후 바로 종료하지 않고 팔로워 적재 동작으로 이어간다. 현재 구현은 물체를 든 뒤 리더 베이스를 180도 회전하고, 뒤쪽 팔로워 방향으로 내려놓은 뒤 그리퍼를 연다.
- `handoff_lift_joint2_delta_rad / handoff_place_joint2_delta_rad`: 파지 자세 기준 상대 joint2 동작량이다. joint3/joint4를 절대 자세로 다시 풀지 않아서 기존 joint3/4 토크 충돌을 피한다.
- `handoff_rotate_angle_rad / handoff_rotate_angular_speed_rad_s`: `/cmd_vel`로 리더 베이스를 제자리 회전시키는 각도와 각속도다. 기본값은 180도 회전이다.
- `use_fallback_bbox_for_control`: CSRT `/target/tracked_bbox`가 멈추면 전면 YOLO `/target/init_bbox`를 fallback으로 사용한다.
- `start_servo_on_start`: `mp_control` 내부 Servo 자동 시작은 꺼져 있다. 실제 런치에서는 Servo 출력이 먼저 `/arm_controller/joint_trajectory_raw`로 나가고, 조인트 trajectory 변환 노드가 joint3 이동량만 반전해 `/arm_controller/joint_trajectory`로 다시 발행한다.
- `use_joint_pregrasp`: 실제 로봇 pregrasp는 Cartesian Servo가 아니라 joint trajectory로 보낸다. 이 trajectory도 `/arm_controller/joint_trajectory_raw`로 나가며, 최종 arm controller에는 transformer를 거친 `/arm_controller/joint_trajectory`만 들어간다.
- `pregrasp_preserve_gripper_roll`: pregrasp 목표를 만들 때 현재 stay 자세의 `joint2 + joint3 + joint4` orientation proxy를 유지하도록 `joint4`를 계산한다. 실제 로봇에서 joint3을 크게 펴면 그리퍼 방향도 같이 바뀌므로, joint3을 제외하면 그리퍼가 로봇 베이스와 수평을 유지하지 못하고 위를 향한다. 실제 로봇에서는 joint3 이동량 반전만 raw trajectory transformer에서 처리한다. joint4 roll 보정은 `mp_control` 한 곳에서만 계산해서 중복 보정으로 joint4가 크게 꺾이는 상황을 막는다.
- `pregrasp_roll_joint*_weight`: pregrasp에서 그리퍼가 베이스/몸체와 수평에 가깝게 유지되도록 쓰는 조인트 orientation proxy 가중치다. 현재 실제 리더는 `joint2=1.0`, `joint3=1.0`, `joint4=1.0`을 사용한다.
- `pregrasp_ready_joint_positions`: 실제 리더에서는 전면 Astra bbox가 파지 완료 판정에도 필요하므로, pregrasp 자세가 팔을 너무 아래로 떨어뜨려 Astra 시야를 가리지 않게 둔다. 현재 값은 이전 `joint2=0.65`, `joint4=-1.20`보다 높은 `joint2=0.50`, `joint4=-1.05`를 사용한다.
- `pregrasp_hold_current_duration_s`: 기본값은 `0.0`이다. pregrasp 시작 전에 현재 자세 hold point를 추가하지 않아서 시작 지연을 만들지 않는다.
- `pregrasp_sync_steps`: 기본값은 `1`이다. pregrasp trajectory에는 joint1~4가 모두 들어간 단일 목표 point만 들어가며, joint2/3/4가 같은 `time_from_start`로 동시에 목표에 도달하도록 한다. 이 값을 2 이상으로 올리면 중간 waypoint가 생겨 실제 로봇에서 끊긴 동작처럼 보일 수 있으므로 실제 로봇에서는 1을 유지한다.
- `pregrasp_joint_tolerance_rad`: `/joint_states`가 pregrasp 목표에 이 오차 안으로 들어와야 EEF 보정과 그리퍼 닫기를 허용한다.
- `pregrasp_republish_period_s`: arm controller가 1회 trajectory를 놓치면 같은 pregrasp trajectory를 주기적으로 재발행한다.
- `pregrasp_reverse_joint3_delta`: 실제 로봇 설정에서는 켜져 있다. `pregrasp_ready_joint_positions`는 최종 controller 기준 목표이고, `mp_control`은 raw 토픽으로 보내기 전에 joint3만 미리 반전한다. 이후 `joint_trajectory_transformer.py`가 다시 반전해서 arm controller에는 의도한 최종 joint3 방향으로 들어간다. 이때 도달 판정은 raw 목표가 아니라 controller 목표와 `/joint_states`를 비교한다.
- `0.06 m`: 삼각 측량된 물체 위치에서 EEF pregrasp standoff로 남기는 거리다. 실제 파지 직전에는 EEF 고정자세 전진을 별도로 수행하므로, 이 값은 물체 앞에서 너무 일찍 멈추지 않게 작게 둔다.
- `eef_forward_use_joint_nudge`: 실제 로봇에서는 최종 전진을 MoveIt Servo twist에만 맡기지 않는다. Servo가 singularity/collision scaling으로 멈추는 경우가 있어서, joint trajectory nudge로 EEF를 추가 전진시킨다. 실제 리더에서는 joint3과 joint4가 시간차를 두고 움직이면 팔이 최대로 펴지는 순간 베이스/그리퍼 방향이 튀므로, joint3과 joint4를 같은 trajectory point에서 동시에 움직인다. 이때 joint4 자체를 고정하는 것이 아니라, 그리퍼의 roll proxy와 TF roll 오차를 기준으로 joint4를 움직인다.
- `eef_forward_start_tolerance_px`: final forward 시작 기준이다. `eef_close_tolerance_px`보다 넓게 잡아, Servo가 마지막 픽셀 오차를 줄이다가 멈추는 경우에도 조인트 nudge 전진 단계로 넘어가게 한다.
- `eef_forward_joint2_delta_rad / eef_forward_joint3_delta_rad`: EEF bbox가 아직 보이는 동안 반복 적용하는 controller 기준 조인트 전진량이다. 전면 Astra가 아직 파지 완료 판정에 필요하므로 추가 전진 단계에서는 joint2를 더 내리지 않는다. joint3 추가 전개가 실제 로봇에서 보이도록 실제 설정은 joint3 delta를 `-0.050 rad`로 둔다. raw 토픽으로 나가기 전 joint3 delta는 기존 transformer 경로와 맞게 pre-invert된다. 이 EEF forward nudge에서는 실제 joint3 현재값이 `pregrasp_joint_min_positions` 밖에 있을 수 있으므로, joint3을 pregrasp clamp로 다시 `-0.94` 근처에 끌어올리지 않는다.
- `eef_forward_joint3_first_duration_ratio`: 전체 nudge 시간 중 첫 joint3-only point에 배정하는 비율이다. 실제 리더 설정은 `0.0`이라 joint3-only 중간 point를 만들지 않고, joint3과 joint4 보정 목표를 같은 final point에 넣어 동시에 움직인다.
- `eef_forward_roll_joint*_weight`: 추가 전진에서 그리퍼 roll proxy를 계산하는 조인트 가중치다. 최신 실제 리더 확인에서 joint3이 음수 방향으로 펴질 때 joint4도 음수 방향으로 움직이면 그리퍼와 EEF 카메라가 계속 공중을 보는 문제가 확인되었다. 따라서 현재 실제 설정은 `eef_forward_roll_joint3_weight: 1.5`, `eef_forward_roll_joint4_weight: 1.0`으로 두어 joint3이 음수 방향으로 펴질 때 joint4는 더 큰 양수 방향으로 보상되게 한다. 로그 기준 `controller_joint3_delta=-0.05`이면 joint4 roll proxy 보정은 약 `+0.075 rad`가 된다.
- `eef_forward_joint4_rpy_roll_gain / eef_forward_joint4_rpy_roll_max_delta_rad`: `/tf`에서 계산한 EE roll 오차를 joint4 목표에 추가로 반영하는 값이다. joint trajectory nudge가 Servo twist를 우회하더라도 gripper roll feedback을 잃지 않게 한다.
- `gripper_down_joint4_offset_rad`: pregrasp와 EEF forward joint nudge의 roll 보정 이후 joint4에 더하는 추가 오프셋이다. 현재 실제 리더는 `0.0 rad`로 둔다. 로그에서 `-0.10 rad` 오프셋이 joint4 하한 근처에서 clamp를 만들고 `eef_usb_camera_link`와 `link3` 충돌을 유발했기 때문에, 방향 보정은 roll proxy 부호로 처리하고 별도 하향 오프셋은 끈다.
- `close_on_front_bbox_shrink / front_bbox_close_area_ratio`: EEF fixed-pose forward 시작 시점의 전면 bbox 면적을 기준으로 저장하고, 이후 전면 bbox 면적이 그 기준의 `0.60` 이하로 줄면 그리퍼 close를 시작한다.
- `close_on_eef_bbox_shrink / eef_bbox_close_area_ratio`: 전면 bbox가 계속 크게 유지되는 상황을 보완한다. 실제 로그에서 전면 bbox ratio는 약 `1.0`으로 유지되었지만 EEF bbox는 약 `9000 px`대에서 `1700 px`대로 줄었으므로, EEF bbox 면적도 시작 면적의 `0.60` 이하가 되면 물체가 그리퍼 안쪽으로 들어온 것으로 보고 close를 시작한다.

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

joint4 보정은 변환 노드에서 하지 않는다. `mp_control` 내부 pregrasp 목표 생성 시 최종 controller 기준으로 한 번만 계산한다. 그 다음 raw 토픽 전송용으로 joint3만 pre-invert한다. 변환 노드와 `mp_control`이 동시에 joint4를 보정하거나, raw joint3 기준으로 joint4를 계산하면 joint4 목표가 과도하게 튀어 그리퍼가 베이스/지지대와 충돌할 수 있다.

pregrasp trajectory는 joint1~4가 모두 들어간 단일 point로 발행된다. 따라서 joint2, joint3, joint4는 순차 명령이 아니라 하나의 동시 trajectory 명령으로 arm controller에 들어간다. 런타임에서 순차 동작처럼 보이면 `/arm_controller/joint_trajectory`에 transformer 외 publisher가 붙었거나, 설치본 config가 최신이 아닌지 확인해야 한다.

Servo가 런치 종료나 cancel 과정에서 현재 위치 hold 명령을 보내면, 입력 목표와 `/joint_states` 기준점이 같아서 출력도 현재 위치 그대로 유지된다. 단순히 절대각을 `-joint3`으로 바꾸지 않기 때문에 취소 시 반대 절대각으로 튀지 않는다.

확인 명령:

```bash
ros2 topic info -v /arm_controller/joint_trajectory_raw
ros2 topic info -v /arm_controller/joint_trajectory
ros2 topic echo /arm_controller/joint_trajectory_raw --once
ros2 topic echo /arm_controller/joint_trajectory --once
```

실제 런치에서는 `/servo_node`와 `mp_control_node`가 모두 raw 토픽을 발행하고, `joint_trajectory_transformer`만 실제 arm controller 토픽을 발행해야 한다. 따라서 `/arm_controller/joint_trajectory`의 publisher는 transformer 하나만 보이는 것이 정상이다. `mp_control` status의 `raw_target`은 transformer 입력이고, `controller_target`은 실제 `/joint_states`가 도달해야 하는 목표다.

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

이 tolerance는 설정된 EEF 카메라 해상도를 기준으로 스케일된다. 따라서 320x240 EEF 카메라와 640x480 전면 카메라의 픽셀 오차를 같은 기준으로 보지 않는다. 현재 실제 EEF 카메라에서는 bbox가 물체에 딱 맞으면 물체 중심이 optical center보다 약 60 px 아래로 보일 수 있으므로, `70 px`를 close-ready 기준으로 사용한다. 다만 실제 로봇에서 Servo가 마지막 lateral 오차 보정 중 singularity/collision scaling으로 멈출 수 있어, final forward 시작은 `eef_forward_start_tolerance_px: 90.0`으로 더 넓게 허용한다.

EEF bbox 정렬이 완료되면 `mp_control`은 초기 stay 자세에서 캡처한 EE roll을 유지하고, refinement 진입 시점의 pitch/yaw를 유지한다. 그 상태에서 `base_link` 기준으로 짧게 전진한 뒤 그리퍼를 닫는다.

```yaml
use_eef_rpy_refinement: true
eef_hold_current_rpy: true
eef_hold_stay_roll: true
eef_rpy_tolerance_rad: 0.12
eef_rpy_gain: 0.8
eef_refine_max_angular_speed: 0.25
eef_forward_after_align: true
eef_forward_distance_m: 0.08
eef_forward_speed_mps: 0.018
```

이 단계에서 `/mp_control/status`에는 `rpy_err=(roll, pitch, yaw)`, `roll_ref=stay_roll`, `rpy_ready`, forward advance 진행 상태가 표시된다. 실제 로봇은 EEF bbox와 stay roll이 맞은 뒤 8 cm를 0.018 m/s로 더 전진한 다음 그리퍼를 닫는다.

EEF 카메라는 그리퍼 폭을 보정하지 않는다. 그리퍼 폭은 전면 depth에서 얻은 물체 폭만 사용한다. 유효한 전면 depth 폭이 없으면 fallback 폭을 쓴다. EEF RGB는 EE가 약간 틀어진 경우 최종 위치 보정에만 사용한다.

최종 그리퍼 명령은 `object_width * gripper_grasp_width_scale + gripper_grasp_clearance_m`를 사용한다. 실제 설정은 `1.50`이라서 전면 depth로 측정된 물체 폭보다 약 50% 크게 잡고, 여기에 작은 여유폭을 더한다. `gripper_grasp_clearance_m: 0.0`이면 같은 50% scale을 적용한 뒤 기존 `gripper_grasp_compression_m` 방식으로 돌아간다.

## 파지 완료 판정

파지 완료 이벤트는 그리퍼 close 명령 시점이 아니라 시각 확인 이후에만 발행한다.

완료 조건:

```text
전면 카메라 /target/tracked_bbox 가 fresh 상태
그리퍼 close 이후 EEF 카메라 /target/eef_init_bbox 를 한 번 이상 관측
이후 EEF 카메라 /target/eef_init_bbox 가 lost/stale 상태
```

즉 전면 카메라에서는 물체 bbox가 계속 보이고, EEF 카메라에서는 close 이후 물체 bbox가 보이다가 더 이상 생성되지 않을 때 물체가 그리퍼 안으로 들어왔다고 판단한다. 이 조건을 만족해야 `/cargo/events`에 `picked`가 발행되고 `/leader/cargo_state`가 `GRASPED`로 넘어간다. close 직후 EEF bbox를 한 번도 보지 못한 상태에서는 EEF bbox가 없더라도 파지 완료로 처리하지 않는다.

그리퍼 close 이후에도 EEF bbox가 계속 보이면 `mp_control`은 완료로 빠지지 않고 stay roll/current pitch/yaw를 유지한 채 EEF fixed-pose 전진 명령을 계속 보낸다. EEF bbox가 점점 작아지거나 그리퍼/물체 접촉으로 더 이상 검출되지 않고, 전면 bbox가 유지되는 순간 파지 완료로 확정한다.

그리퍼 close 자체는 EEF forward 거리만으로 기다리지 않는다. EEF forward 시작 시점의 전면 bbox와 EEF bbox 면적을 각각 저장하고, 현재 전면 bbox 면적이 `front_bbox_close_area_ratio` 이하이거나 현재 EEF bbox 면적이 `eef_bbox_close_area_ratio` 이하가 되면 물체가 그리퍼 안쪽으로 충분히 들어왔다고 보고 close 명령을 보낸다. 현재 실제 설정은 둘 다 `0.60`이다. 실제 로그에서는 전면 bbox ratio가 약 `1.0`으로 유지되어 전면 조건만으로는 close되지 않았고, EEF bbox가 약 `9000 px`대에서 `1700 px`대로 줄어드는 현상이 확인되어 EEF bbox shrink 조건을 같이 사용한다.

실제 로봇에서 이 fixed-pose 전진은 `eef_forward_use_joint_nudge: true`일 때 joint trajectory로 보낸다. 로그에 `Very close to a singularity` 또는 `Close to a collision`이 반복되면 Servo twist가 막힌 것이므로, `/mp_control/status`의 `eef forward joint nudge`와 `/arm_controller/joint_trajectory_raw`, `/arm_controller/joint_trajectory`를 같이 확인한다. status에는 `controller_target`, `controller_joint3_delta`, `raw_joint3_delta`, `joint3_first_time`, `roll_proxy_weights`, `joint4_roll_feedback`, `rpy_roll_err`가 같이 나온다. 정상이라면 `controller_joint3_delta`가 0이 아니고, transformer 입력인 `raw_joint3_delta`는 부호가 반대로 보인다. 실제 리더 설정에서는 `joint3_first_time=0`으로 나와야 하며, 이 경우 joint3-only 중간 point 없이 final target 하나에서 joint3과 joint4가 동시에 움직인다. 실제 리더에서 joint4가 음수 방향으로 보정될 때 그리퍼와 EEF 카메라가 공중을 보는 문제가 확인되었으므로, 추가 전진 roll proxy는 joint3 음수 전개에 대해 joint4가 양수 방향으로 보상되도록 둔다.

파지가 시각적으로 확인되면 `handoff_after_grasp: true` 설정에 따라 후속 적재 동작으로 넘어간다. 순서는 `picked` 이벤트 발행, joint2 상대 상승, `/cmd_vel` 180도 회전, joint2 상대 하강, 그리퍼 open, `placed`/`loaded` 이벤트 발행이다. 회전 중에는 `/target/base_hold`를 켜서 전면 tracker가 `/cmd_vel`을 덮어쓰지 않게 한다.

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

## 런치 종료 확인

실제 리더 런치 종료 시 `joint_trajectory_transformer.py`는 `Ctrl-C`로 들어오는
`KeyboardInterrupt`를 정상 종료로 처리한다. 종료 로그에서 transformer가
`exit code -2`로 ERROR 표시되면 설치본이 최신이 아닌 상태일 수 있으므로
`mp_control`을 다시 빌드한다.

전면 Astra camera container는 드라이버 종료가 오래 걸릴 수 있어서
`astra_mini.launch.py`의 container shutdown timeout을 짧게 둔다. 따라서
`Ctrl-C` 이후에도 container가 즉시 내려가지 않으면 launch가 짧게 기다린 뒤
강제 종료하며, 다음 실행 전에 남은 프로세스가 있는지만 확인하면 된다.

## 빌드

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mp_control --symlink-install
```
