# hybrid_single_target_tracker

기존 Python 노드(`depth_object_detector.py`)를 기반으로, 단일 타깃 추적과 하이브리드 깊이 추정을 C++로 다시 구성한 ROS 2 패키지입니다.

## 왜 새로 만드는가

기존 Python 코드의 구조적 문제는 다음과 같습니다.

1. `next_id`가 계속 증가하지만 실제 퍼블리셔는 `fruit_1 ~ fruit_4`만 존재해서, 일정 시간이 지나면 추적은 되어도 토픽 발행이 끊길 수 있습니다.
2. `latest_color`를 depth 콜백에서 가져오는 방식이라 색상/깊이 프레임 타임스탬프가 어긋날 수 있습니다.
3. 매 프레임 큰 이미지를 복사하고 여러 객체를 리스트로 관리해 CPU/메모리 비용이 누적됩니다.
4. `header.frame_id`를 `object_x`로 바꾸지만 실제 좌표는 카메라 좌표계 기준이라 의미가 섞여 있습니다.

이 패키지는 위 문제를 해결하기 위해 **단일 타깃**만 유지하고, **고정 ID**, **동기화된 컬러/깊이 입력**, **정리된 상태 구조체**, **선택적 디버그 출력**으로 다시 설계했습니다.

## 핵심 동작

- 컬러/깊이 이미지를 `message_filters`의 ApproximateTime 동기화로 함께 받음
- 컬러 프레임에서 CSRT tracker로 단일 타깃 추적
- 깊이 ROI 중앙부 median으로 sensor depth 계산
- 센서 깊이가 신뢰 구간보다 크면 sensor depth 사용
- 근거리이거나 센서 실패 시, 학습된 실제 폭 기반 visual depth 사용
- 둘 다 실패하면 마지막 유효 깊이 유지

## 기존 Python 대비 변경점

### 1) ID 정책 단순화
- 임의 증가 ID 제거
- 타깃 ID는 고정 (`fixed_target_id`, 기본값 1)
- 단일 타깃이므로 `/object/fruit_1/*` 토픽만 유지

### 2) 메모리/CPU 절감
- 객체 리스트 제거, `TargetState` 하나만 유지
- 디버그 영상은 구독자가 있을 때만 `clone()` 수행
- raw view는 동기화된 color 이미지를 그대로 재발행
- depth median 계산용 버퍼 재사용
- depth meter 변환 버퍼 재사용

### 3) 좌표계 정리
- `PointStamped.header.frame_id`는 카메라 프레임 유지
- 물체 identity는 토픽 이름으로 구분

### 4) 더 안전한 입출력
- `/object/fruit_1/valid` 토픽 추가
- 타깃 유효 여부를 로봇이 직접 확인 가능

## 토픽

### Subscribe
- `color_topic` (기본 `/camera/color/image_raw`)
- `depth_topic` (기본 `/camera/aligned_depth_to_color/image_raw`)
- `camera_info_topic` (기본 `/camera/color/camera_info`)

### Publish
- `/object/fruit_1/position` (`geometry_msgs/msg/PointStamped`)
- `/object/fruit_1/size` (`geometry_msgs/msg/PointStamped`)
- `/object/fruit_1/depth_comp` (`geometry_msgs/msg/PointStamped`)
- `/object/fruit_1/valid` (`std_msgs/msg/Bool`)
- `/object/picking_view` (`sensor_msgs/msg/Image`, optional)
- `/object/close_range_view` (`sensor_msgs/msg/Image`, optional)

## 빌드

```bash
cd ~/ros2_ws/src
cp -r hybrid_single_target_tracker ./
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select hybrid_single_target_tracker
source install/setup.bash
```

## 실행

### 직접 실행
```bash
ros2 run hybrid_single_target_tracker hybrid_single_target_tracker_node \
  --ros-args --params-file $(ros2 pkg prefix hybrid_single_target_tracker)/share/hybrid_single_target_tracker/config/params.yaml
```

### launch 실행
```bash
ros2 launch hybrid_single_target_tracker hybrid_single_target_tracker.launch.py
```

위 launch는 기본적으로 `astra_camera`의 `astra_mini.launch.py`도 함께 실행합니다. 실기에서는 Orbbec Astra Mini 드라이버를 올리고, 시뮬레이션에서는 같은 토픽 이름으로 Gazebo 카메라를 브리지할 수 있습니다.

실기 카메라 없이 트래커만 올리려면 아래처럼 실행합니다.

```bash
ros2 launch hybrid_single_target_tracker hybrid_single_target_tracker.launch.py start_camera:=false
```

Gazebo 카메라를 사용할 때는 Gazebo가 먼저 떠 있어야 하며, 아래처럼 `use_sim:=true`를 주면 동일한 `/camera/*` 인터페이스를 사용합니다.

```bash
ros2 launch hybrid_single_target_tracker hybrid_single_target_tracker.launch.py use_sim:=true
```

이미 `turtlebot3_manipulation_gazebo`의 Gazebo 런치가 카메라 브리지를 올리고 있다면, 중복 실행을 피하기 위해 아래처럼 트래커만 올리면 됩니다.

```bash
ros2 launch hybrid_single_target_tracker hybrid_single_target_tracker.launch.py start_camera:=false use_sim:=true
```

Gazebo와 트래커를 한 번에 올리려면 아래 런치를 사용하면 됩니다.

```bash
ros2 launch hybrid_single_target_tracker hybrid_single_target_tracker_sim.launch.py
```

## 파라미터 조정 포인트

- `min_depth_trust_m`: 이 값보다 가까우면 visual depth 쪽으로 전환
- `detection_interval`: 재탐지 주기
- `tracker_skip`: tracker update 간격
- `max_missing_frames`: 타깃 제거 기준
- `min_contour_area_px`: depth contour 최소 면적
- `publish_debug_image`: 디버그 영상 on/off

## 주의

1. 컬러 bbox를 깊이 ROI에 그대로 쓰므로, **depth는 color 기준으로 정렬된(aligned) 토픽**을 쓰는 것이 가장 안전합니다.
2. 카메라 정보는 **컬러 프레임 기준 intrinsics**를 쓰는 것이 맞습니다.
3. OpenCV tracking 모듈(CSRT)이 포함된 환경이어야 합니다.

## 다음 단계 추천

1. 로봇 제어부가 `/object/fruit_1/valid`를 보고 grasp를 허용하도록 연결
2. 필요하면 contour 기반 초기 탐지를 segmentation 또는 detector로 교체
3. 최종적으로는 position/size/depth를 custom msg 하나로 합쳐 전송
