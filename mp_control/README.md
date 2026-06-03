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
color_triangulation_base_stop_object_x_m: 0.195
arm_start_max_object_x_m: 0.195
front_yolo_min_bbox_width_px: 6.0
front_yolo_min_bbox_height_px: 6.0
front_yolo_min_accept_confidence: 0.05
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
pregrasp_move_duration_s: 1.6
pregrasp_joint3_lead_enabled: true
pregrasp_joint3_lead_duration_ratio: 0.45
pregrasp_joint3_lead_fraction: 0.5
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
eef_forward_fixed_duration_s: 1.0
eef_forward_start_tolerance_px: 90.0
eef_forward_use_joint_nudge: true
eef_forward_joint2_delta_rad: 0.025
eef_forward_joint3_delta_rad: -0.050
eef_forward_joint_nudge_duration_s: 0.80
eef_forward_joint_nudge_period_s: 0.35
eef_forward_joint3_first_duration_ratio: 0.65
eef_forward_roll_joint2_weight: 0.0
eef_forward_roll_joint3_weight: 0.6
eef_forward_roll_joint4_weight: 1.0
eef_forward_joint4_rpy_roll_gain: 0.3
eef_forward_joint4_rpy_roll_max_delta_rad: 0.015
eef_forward_joint4_ground_parallel_limit_rad: -1.05
eef_forward_joint4_ground_limit_tolerance_rad: 0.01
eef_forward_joint4_down_positive: true
gripper_down_joint4_offset_rad: 0.0
close_on_front_bbox_shrink: true
front_bbox_close_area_ratio: 0.55
close_on_eef_bbox_shrink: true
eef_bbox_close_area_ratio: 0.55
eef_forward_min_advance_before_close_m: 0.04
handoff_after_grasp: true
handoff_lift_joint2_delta_rad: 0.0
handoff_place_joint2_delta_rad: 0.0
handoff_rotate_angle_rad: 3.14159265
handoff_rotate_angular_speed_rad_s: 0.45
handoff_stay_joint_positions: [0.104311, 0.027612, -0.001534, -1.638291]
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
- `0.195 m`: 컬러 삼각 측량 기반 베이스 접근 목표 거리다. 이 거리 이후 팔 파지 단계로 넘어간다. `arm_start_max_object_x_m`도 같은 `0.195 m`로 맞춰 베이스가 충분히 가까워지기 전에는 팔 단계로 넘어가지 않게 한다. 단, 전면 bbox가 close-range handoff 크기를 넘고 EEF bbox가 fresh하면 실제 시각적으로 가까운 상태로 보고, 거리 추정값이 `0.195 m`보다 약간 크게 나와도 joint pregrasp를 시작한다. `0.19 m` 실험에서는 로봇과 물체가 너무 가까워져 팔이 뻗기 전에 EEF 시야와 전개 공간이 부족했기 때문에, 현재 기준은 기존 `0.20 m`에서 조금만 줄인 `0.195 m`다. 전면 bbox 크기 기반 3D point 변환이 TF 문제로 실패해도, 기존 depth reference가 남아 있으면 bbox 크기 close 판정만으로도 pregrasp를 허용한다.
- 전면 YOLO는 런치 시작 직후 원거리 박스가 작게 잡히는 구간을 놓치지 않도록 최소 bbox 크기를 `6 px`, accept confidence를 `0.05`로 둔다. 로그에서 전면 후보가 `small/conf`로 버려지는 경우를 줄이기 위한 값이다. EEF YOLO는 반사/옆면 오검출을 막기 위해 strict ROI와 same-object lock을 사용하고, 짧은 검출 실패에서는 마지막 bbox를 재사용한다.
- `EEF_REFINE`와 handoff 작업 단계에서는 `/target/base_hold=true`와 zero `/cmd_vel`을 반복 발행한다. 그리퍼가 전면 카메라를 가려 전면 bbox/depth가 왜곡되어도 베이스 접근 루프로 되돌아가 직진하지 않게 한다.
- `require_visual_grasp_confirmation`: 그리퍼 close 명령 직후 바로 파지 완료로 보지 않는다. 전면 bbox는 계속 보이고, EEF bbox는 사라져야 `/cargo/events`에 `picked`를 발행한다.
- `grasp_completion_front_max_age_s`: 파지 완료 판정에 사용할 전면 bbox freshness 한계다.
- `grasp_completion_eef_lost_timeout_s`: 이 시간 동안 EEF bbox가 새로 들어오지 않으면 EEF에서 물체가 사라진 것으로 본다.
- `handoff_after_grasp`: 파지 시각 확인 후 바로 종료하지 않고 팔로워 적재 동작으로 이어간다. 현재 구현은 물체를 들어 올리지 않고, 현재 파지 높이를 유지한 채 매니퓰레이터 `joint1`만 180도 회전해 팔로워 쪽으로 보낸 뒤 그리퍼를 연다.
- `handoff_lift_joint2_delta_rad / handoff_place_joint2_delta_rad`: 실제 리더에서는 둘 다 `0.0`이다. 파지 후 joint2를 들어 올리거나 다시 내리지 않는다.
- `handoff_rotate_angle_rad`: 베이스 회전이 아니라 매니퓰레이터 `joint1` 상대 회전각이다. 기본값은 180도 회전이다.
- `handoff_stay_joint_positions`: 팔로워 쪽에 내려놓고 그리퍼를 연 뒤 복귀할 saved stay pose다. bringup의 stay trajectory와 같은 `[0.104311, 0.027612, -0.001534, -1.638291]`를 사용한다.
- `use_fallback_bbox_for_control`: CSRT `/target/tracked_bbox`가 멈추면 전면 YOLO `/target/init_bbox`를 fallback으로 사용한다.
- `start_servo_on_start`: `mp_control` 내부 Servo 자동 시작은 꺼져 있다. 실제 런치에서는 Servo 출력이 먼저 `/arm_controller/joint_trajectory_raw`로 나가고, 조인트 trajectory 변환 노드가 joint3 이동량만 반전해 `/arm_controller/joint_trajectory`로 다시 발행한다.
- `use_joint_pregrasp`: 실제 로봇 pregrasp는 Cartesian Servo가 아니라 joint trajectory로 보낸다. 이 trajectory도 `/arm_controller/joint_trajectory_raw`로 나가며, 최종 arm controller에는 transformer를 거친 `/arm_controller/joint_trajectory`만 들어간다.
- `pregrasp_preserve_gripper_roll`: pregrasp 목표를 만들 때 현재 stay 자세의 `joint2 + joint3 + joint4` orientation proxy를 유지하도록 `joint4`를 계산한다. 실제 로봇에서 joint3을 크게 펴면 그리퍼 방향도 같이 바뀌므로, joint3을 제외하면 그리퍼가 로봇 베이스와 수평을 유지하지 못하고 위를 향한다. 실제 로봇에서는 joint3 이동량 반전만 raw trajectory transformer에서 처리한다. joint4 roll 보정은 `mp_control` 한 곳에서만 계산해서 중복 보정으로 joint4가 크게 꺾이는 상황을 막는다.
- `pregrasp_roll_joint*_weight`: pregrasp에서 그리퍼가 베이스/몸체와 수평에 가깝게 유지되도록 쓰는 조인트 orientation proxy 가중치다. 현재 실제 리더는 `joint2=1.0`, `joint3=1.0`, `joint4=1.0`을 사용한다.
- `pregrasp_ready_joint_positions`: 실제 리더에서는 전면 Astra bbox가 파지 완료 판정에도 필요하므로, pregrasp 자세가 팔을 너무 아래로 떨어뜨려 Astra 시야를 가리지 않게 둔다. 현재 값은 이전 `joint2=0.65`, `joint4=-1.20`보다 높은 `joint2=0.50`, `joint4=-1.05`를 사용한다.
- `pregrasp_hold_current_duration_s`: 기본값은 `0.0`이다. pregrasp 시작 전에 현재 자세 hold point를 추가하지 않아서 시작 지연을 만들지 않는다.
- `pregrasp_joint3_lead_enabled`: 실제 리더에서는 `true`다. pregrasp 시작 시 첫 trajectory point는 joint3만 목표 방향으로 먼저 펴고, joint2와 joint4는 현재 위치를 유지한다. joint2가 먼저 숙여지면 joint3에 하중이 걸려 토크 루프가 깨질 수 있으므로, joint3 선행 point를 둔다.
- `pregrasp_joint3_lead_duration_ratio`: joint3 선행 point가 전체 pregrasp 시간 중 어느 시점에 도달할지 정한다. 현재 값은 `0.45`라서 전체 `1.6 s` 중 약 `0.72 s` 동안 joint3을 먼저 이동시킨다.
- `pregrasp_joint3_lead_fraction`: joint3 선행 point에서 최종 joint3 이동량의 몇 %를 먼저 보낼지 정한다. 현재 값은 `0.5`이므로 joint3은 첫 point에서 최종 pregrasp joint3 이동량의 절반만 먼저 간다. `1.0`은 joint3이 한 번에 너무 크게 펴져 실제 모터 토크가 깨질 수 있어 사용하지 않는다.
- `pregrasp_sync_steps`: 기본값은 `1`이다. joint3 선행 point 이후에는 joint1~4가 모두 들어간 최종 목표 point 하나만 추가한다. 이 값을 2 이상으로 올리면 중간 waypoint가 생겨 실제 로봇에서 끊긴 동작처럼 보일 수 있으므로 실제 로봇에서는 1을 유지한다.
- `pregrasp_joint_tolerance_rad`: `/joint_states`가 pregrasp 목표에 이 오차 안으로 들어와야 EEF 보정과 그리퍼 닫기를 허용한다.
- `pregrasp_republish_period_s`: arm controller가 1회 trajectory를 놓치면 같은 pregrasp trajectory를 주기적으로 재발행한다.
- `pregrasp_reverse_joint3_delta`: 실제 리더 런치에서는 `true`로 둔다. `mp_control`이 `/arm_controller/joint_trajectory_raw`에 보낼 때 joint3 delta를 현재값 기준으로 미리 반전하고, `joint_trajectory_transformer.py`가 이를 다시 controller 목표로 mirror해서 `/arm_controller/joint_trajectory`에 발행한다. 이렇게 해야 최종 controller target은 `pregrasp_ready_joint_positions`를 유지하면서도, 실제 하드웨어에 필요한 joint3 명령 방향을 raw 경로에서 보존할 수 있다. 다음 실행 로그에는 조인트별 `joint_err`, `current`, `controller_target`, `raw_target`, `joint3_lead_*` 값이 함께 출력된다.
- `0.06 m`: 삼각 측량된 물체 위치에서 EEF pregrasp standoff로 남기는 거리다. 실제 파지 직전에는 EEF 고정자세 전진을 별도로 수행하므로, 이 값은 물체 앞에서 너무 일찍 멈추지 않게 작게 둔다.
- `eef_forward_use_joint_nudge`: 실제 로봇에서는 최종 전진을 MoveIt Servo twist에만 맡기지 않는다. Servo가 singularity/collision scaling으로 멈추는 경우가 있어서, joint2/joint3/joint4를 하나의 trajectory point에 함께 넣어 같은 `time_from_start`로 동시에 움직인다. 이때 joint4 자체를 고정하는 것이 아니라, 그리퍼의 roll proxy와 TF roll 오차를 기준으로 joint4를 움직인다.
- `eef_forward_start_tolerance_px`: final forward 시작 기준이다. `eef_close_tolerance_px`보다 넓게 잡아, Servo가 마지막 픽셀 오차를 줄이다가 멈추는 경우에도 조인트 nudge 전진 단계로 넘어가게 한다.
- `eef_forward_fixed_duration_s`: final forward가 시작된 뒤 그리퍼 close를 허용하기 전 최소 직진 시간이다. 현재 실제 리더는 `1.0 s`로 둔다. 따라서 bbox 면적이 먼저 줄어도 1초 전에는 close하지 않고, 1초가 지나면 distance 기준을 기다리지 않고 close 단계로 넘어간다.
- `eef_forward_joint2_delta_rad / eef_forward_joint3_delta_rad`: EEF bbox가 아직 보이는 동안 반복 적용하는 controller 기준 조인트 전진량이다. 실제 설정은 `joint2=0.025 rad`, `joint3=-0.050 rad`이며, joint4는 같은 목표 point에서 그리퍼 roll 유지값으로 계산된다. `joint3=-0.075 rad`는 실제 로봇에서 joint3이 너무 크게 펴지고 토크가 깨질 수 있어 기존 회전 크기인 `-0.050 rad`로 되돌렸다. raw 토픽으로 나간 뒤 joint3 delta 반전은 `joint_trajectory_transformer.py`가 단독으로 처리한다. 이 EEF forward nudge에서는 실제 joint3 현재값이 `pregrasp_joint_min_positions` 밖에 있을 수 있으므로, joint3을 pregrasp clamp로 다시 `-0.94` 근처에 끌어올리지 않는다.
- `eef_forward_joint3_first_duration_ratio`: 기존 순차 전개 호환용 파라미터다. 현재 실제 리더의 final nudge는 단일 목표 point를 사용하므로 joint3-only 선행 point를 만들지 않는다.
- `eef_forward_joint_nudge_duration_s`: 추가 전진 trajectory 시간이다. joint4 방향 반전 후 토크 충격을 줄이기 위해 실제 설정은 `0.80 s`로 늦춘다.
- `eef_forward_roll_joint*_weight`: 추가 전진에서 그리퍼 roll proxy를 계산하는 조인트 가중치다. 현재 실제 설정은 `eef_forward_roll_joint3_weight: 0.6`, `eef_forward_roll_joint4_weight: 1.0`이다. 실제 리더에서 음수 weight는 그리퍼와 EEF 카메라가 하늘을 보는 방향으로 joint4를 돌렸기 때문에 양수 weight를 사용하되, joint3 `-0.050 rad` 전개 기준 joint4가 joint2/3보다 과하게 먼저 꺾이지 않도록 roll feedback과 ground-parallel limit으로 제한한다.
- `eef_forward_joint4_rpy_roll_gain / eef_forward_joint4_rpy_roll_max_delta_rad`: `/tf`에서 계산한 EE roll 오차를 joint4 목표에 추가로 반영하는 값이다. joint trajectory nudge가 Servo twist를 우회하더라도 gripper roll feedback을 잃지 않게 한다. 현재 실제 설정은 gain `0.3`, max `0.015 rad`로 제한해서 joint4가 joint2/joint3보다 크게 먼저 꺾이지 않게 한다.
- `eef_forward_joint4_ground_parallel_limit_rad`: 추가 전진 중 joint4가 이 값에 도달하면 그리퍼/EEF 카메라가 땅과 수평에 가까운 한계 자세로 본다. 실제 리더는 `-1.05 rad`를 기준으로 두고, 이 이후에는 joint4를 더 아래로 숙이는 방향으로 회전시키지 않는다.
- `eef_forward_joint4_ground_limit_tolerance_rad`: joint4가 수평 한계 근처에 이미 도달했을 때 제한값으로 다시 끌어당기지 않고 현재 각도를 유지하는 허용 오차다. 로그에서 joint4가 `-1.053 rad` 근처인데 매 nudge마다 `-1.05 rad`로 미세하게 당겨지며 그리퍼 자세가 흔들렸기 때문에, 실제 설정은 `0.01 rad`로 둔다.
- `eef_forward_joint4_down_positive`: 실제 리더에서는 joint4 값이 증가하는 방향이 그리퍼와 EEF 카메라를 바닥으로 숙이는 방향이다. 따라서 `true`로 두고, `eef_forward_joint4_ground_parallel_limit_rad`보다 큰 joint4 목표를 막는다.
- `gripper_down_joint4_offset_rad`: pregrasp와 EEF forward joint nudge의 roll 보정 이후 joint4에 더하는 추가 오프셋이다. 현재 실제 리더는 `0.0 rad`로 둔다. 로그에서 `-0.10 rad` 오프셋이 joint4 하한 근처에서 clamp를 만들고 `eef_usb_camera_link`와 `link3` 충돌을 유발했기 때문에, 방향 보정은 roll proxy 부호로 처리하고 별도 하향 오프셋은 끈다.
- `close_on_front_bbox_shrink / front_bbox_close_area_ratio`: EEF fixed-pose forward 시작 시점의 전면 bbox 면적을 기준으로 저장하고, 이후 전면 bbox 면적이 그 기준의 `0.55` 이하로 줄면 그리퍼 close를 시작한다. 베이스 정지 거리를 `0.195 m`로 맞춰 forward 시작 bbox가 `0.20 m` 설정보다 약간 크지만 `0.19 m` 설정보다 덜 극단적이므로, close 기준은 유지한다.
- `close_on_eef_bbox_shrink / eef_bbox_close_area_ratio`: 전면 bbox가 계속 크게 유지되는 상황을 보완한다. 실제 로그에서 전면 bbox ratio는 약 `1.0`으로 유지되었지만 EEF bbox는 약 `9000 px`대에서 `1700 px`대로 줄었으므로, EEF bbox 면적도 시작 면적의 `0.55` 이하가 되면 물체가 그리퍼 안쪽으로 들어온 것으로 보고 close를 시작한다.
- `eef_forward_min_advance_before_close_m`: fixed-duration 모드를 끈 경우의 보조 안전거리다. 현재 실제 리더는 `eef_forward_fixed_duration_s: 1.0`이 primary gate이므로, bbox 축소 close도 1초 전에는 동작하지 않는다.
- EEF bbox가 ROI/aspect 필터 때문에 중간에 사라지거나 stale 상태가 되면, 기존에는 `prepareEefRefinement()`에서 멈춰 final forward가 시작되지 않았다. 현재는 전면 bbox가 close-size 조건을 만족하면 EEF bbox가 불안정해도 전면 bbox fallback으로 EEF fixed-pose forward를 시작한다. 한 번 forward가 시작된 뒤에는 EEF bbox freshness를 다시 요구하지 않고, saved stay roll/current pitch-yaw를 유지한 채 joint2/joint3/joint4 nudge를 계속 보낸다.

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

EEF YOLO는 전면 카메라와 별도 ROI를 사용한다. 마지막 실험에서 EEF 화면 왼쪽 반사/배경을 물체로 잡아 `/mp_control/status`가 `after depth limit; waiting for close-range color triangulation`에 머물렀기 때문에, EEF 자동 bbox는 기본적으로 화면 왼쪽 25%와 최상단 10%를 제외한다.

```text
auto_eef_init_roi_min_x_ratio:=0.25
auto_eef_init_roi_max_x_ratio:=0.98
auto_eef_init_roi_min_y_ratio:=0.10
auto_eef_init_roi_max_y_ratio:=1.00
```

EEF YOLO는 런치 직후부터 켜지고, `mp_control` 파지 시퀀스가 시작되어도 꺼지지 않는다. EEF bbox가 중간에 켜지면 첫 lock이 반사면이나 배경에 걸릴 수 있어서, 현재 기본값은 `auto_eef_init_bbox_start_delay:=0.0`이다.

또한 EEF YOLO는 같은 물체를 계속 따라가도록 target lock을 사용한다. 새 후보 bbox가 이전 bbox/anchor bbox에서 너무 멀리 튀면 `/target/eef_init_bbox`로 발행하지 않고, `/target/auto_eef_init_bbox_status`에 `YOLO target locked; no same-object box near last bbox`가 찍힌다. 짧은 검출 실패는 `reuse_last_bbox_on_loss: true`로 마지막 bbox를 재발행해서 EEF refinement가 바로 끊기지 않게 한다. 단, `/target/eef_auto_init_enable`이 false에서 true로 다시 바뀌면 이전 lock과 anchor는 초기화된다.

실행 중에는 `/target/auto_eef_init_bbox_status`에서 `roi=x[0.25,0.98] y[0.10,1.00]` 거부 로그와 target lock 로그를 같이 확인한다.

EEF YOLO는 bbox 중심점뿐 아니라 bbox 전체가 EEF ROI 안에 들어와야 `/target/eef_init_bbox`로 발행한다. 로그에서 `[1,0,159,162]`처럼 왼쪽/상단 반사 영역에 걸친 큰 bbox가 중심점만으로 ROI를 통과해 물체가 아닌 곳을 보던 문제가 있었기 때문이다. 전면 YOLO는 기존처럼 중심점 ROI 기준을 유지하고, 이 strict ROI 필터는 EEF 자동 bbox에만 적용한다.

`mp_control_node`는 EEF refinement 요청 플래그가 켜진 뒤에만 EEF bbox를 저장하지 않는다. EEF YOLO는 depth handoff 전에 미리 켜지므로, `/target/eef_init_bbox`가 들어오는 즉시 최신 EEF bbox로 저장한다. 그렇지 않으면 로그에는 `auto_eef_init_bbox`가 정상 검출을 찍어도 제어 노드 내부의 `eef_bbox_ready`가 false로 남아 `after depth limit` 단계에서 팔 pregrasp로 넘어가지 못한다.

전면 bbox 크기와 EEF bbox가 모두 close-range 조건을 만족하면, RGB 삼각 측량이나 전면 bbox TF 변환이 일시적으로 실패해도 joint pregrasp로 넘어간다. 이때는 `visual_bbox_fallback=true` 상태로 표시되고, EEF refinement는 임시 3D 투영점이 아니라 실제 EEF bbox 중심을 기준으로 보정한다. 마지막 실험처럼 물체가 이미 EEF 화면 중앙에 들어왔는데 `/mp_control/status`가 `after depth limit; waiting for close-range color triangulation`에 머무는 상황을 막기 위한 fallback이다.

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
eef_forward_fixed_duration_s: 1.0
```

이 단계에서 `/mp_control/status`에는 `rpy_err=(roll, pitch, yaw)`, `roll_ref=stay_roll`, `rpy_ready`, forward advance 진행 상태가 표시된다. 실제 파지 순서는 `베이스 정지 -> joint pregrasp 자세 생성 -> EEF fixed-pose 1초 직진 -> 그리퍼 close`다. `eef_forward_fixed_duration_s: 1.0`이 켜져 있으면 bbox 면적 축소 조건이 먼저 들어와도 1초 전에는 그리퍼를 닫지 않는다.

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

그리퍼 close는 EEF fixed-pose 전진 1초가 지난 뒤에만 허용한다. EEF forward 시작 시점의 전면 bbox와 EEF bbox 면적도 각각 저장하고, 현재 전면 bbox 면적이 `front_bbox_close_area_ratio` 이하이거나 현재 EEF bbox 면적이 `eef_bbox_close_area_ratio` 이하가 되면 보조 close 조건으로 쓴다. 다만 bbox 축소 조건도 `eef_forward_fixed_duration_s: 1.0` 전에는 무시된다. 현재 close ratio는 둘 다 `0.55`이다.

실제 로봇에서 이 fixed-pose 전진은 `eef_forward_use_joint_nudge: true`일 때 joint trajectory로 보낸다. 로그에 `Very close to a singularity` 또는 `Close to a collision`이 반복되면 Servo twist가 막힌 것이므로, `/mp_control/status`의 `eef forward joint nudge`와 `/arm_controller/joint_trajectory_raw`, `/arm_controller/joint_trajectory`를 같이 확인한다. status에는 `controller_target`, `simultaneous_joints=joint2,joint3,joint4`, `controller_joint2_delta`, `controller_joint3_delta`, `controller_joint4_delta`, `raw_joint3_delta`, `roll_proxy_weights`, `joint4_roll_feedback`, `rpy_roll_err`가 같이 나온다. 정상이라면 trajectory point가 하나이고, joint2/joint3/joint4가 같은 `time_from_start`로 동시에 움직인다. transformer 입력인 `raw_joint3_delta`는 controller 기준 joint3 delta와 부호가 반대로 보인다. joint4는 gripper roll feedback까지 반영한다. 실제 리더에서 `eef_forward_roll_joint3_weight: -1.5`일 때 그리퍼와 EEF 카메라가 하늘을 보는 문제가 확인되었으므로, 현재는 joint4 보정 방향을 반대로 만들기 위해 양수 weight를 사용한다. 다만 `1.5`는 팔이 완전히 펴지기 전에 그리퍼가 아래를 보게 만들어, 현재 실제 설정은 `0.6`으로 줄였다.

파지가 시각적으로 확인되면 `handoff_after_grasp: true` 설정에 따라 후속 적재 동작으로 넘어간다. 순서는 `picked` 이벤트 발행, 현재 파지 자세에서 `joint1` 상대 180도 회전, 그리퍼 open, saved stay pose 복귀, `placed`/`loaded` 이벤트 발행이다. handoff 중에는 `/target/base_hold`를 켜서 전면 tracker가 `/cmd_vel`을 덮어쓰지 않게 한다.

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
