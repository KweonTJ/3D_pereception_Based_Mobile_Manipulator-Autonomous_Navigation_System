# 3D_pereception_Based_Mobile_Manipulator-Autonomous_Navigation_System

# 개요

본 프로젝트는 2026년 UROP 연구 프로젝트로, **3차원 인식 기반 모바일 매니퓰레이터 및 자율 이동 시스템** 구축을 목표로 한다.

산업 및 물류 환경에서는 단순 이동 로봇이나 고정형 매니퓰레이터만으로는 작업 범위와 활용성이 제한된다.  본 프로젝트는 모바일 로봇, 로봇 매니퓰레이터, 3D 비전 센서, 자율주행, 플래투닝 기술을 통합하여 객체 인식, 3차원 위치 추정, Pick & Place, 자율 이동, 협업 운반이 가능한 소형 통합 로봇 시스템을 개발한다.

최종적으로 ROS 2 Humble 기반 C++ 패키지를 구현하고, 실제 하드웨어 실험 결과를 바탕으로 A1 포스터 논문 형태의 연구 결과물을 작성한다.

---
# 연구 목표

- 3D Depth Camera 기반 객체 인식 및 거리 추정
- RGB-D 데이터 기반 물체의 3차원 좌표 계산
- 근거리 Depth 인식 불가 문제를 보완하는 Hybrid Depth Perception 구조 개발
- 모바일 매니퓰레이터 기반 Pick & Place 작업 수행
- 3D 좌표 기반 매니퓰레이터 정밀 제어
- TurtleBot3 기반 자율 이동 및 물체 운반
- Leader-Follower 구조의 플래투닝 기반 협업 이동 시스템 구현
- ROS 2 Humble 기반 C++ 패키지 구조 설계 및 실시간 처리 최적화
---
# 전체 시스템 구조

```
[3D Camera / RGB Camera]
        ↓
[Object Detection & Depth Estimation]
        ↓
[3D Position Estimation]
        ↓
[TF Coordinate Transform]
        ↓
[Manipulator Control]
        ↓
[Pick & Place]
        ↓
[Autonomous Mobility / Platooning]
```
---
# 세부 프로젝트 구성
- 본 프로젝트는 다음 3개의 소형 프로젝트로 나누어 진행한다.
## 1. 3차원 비전 기반 객체 인지 시스템
[[1. 3D Vision Based Object Perception System]]
- Depth Camera와 RGB Camera를 활용하여 물체를 인식하고, 카메라 좌표계 기준 3차원 위치 추정
- **주요 기능**
	- RGB-D 기능 수신
	- 객체 검출 및 Bounding Box 추출
	- Depth ROI 기반 거리 계산
	- 근거리 Depth 손실 문제 대응
	- C++ 기반 실시간 처리 구조 구현
- **관련 패키지**
	- depth_perception
		- astra_camera
		- astra_camera_msgs
		- astra_mini_calibration
		- hybrid_single_target_tracker
## 2. 3D 공간 좌표 기반 매니퓰레이터 정밀 제어
[[2. 3D Spatial Coordinate-Based Manipulator Precision Control]]
- 인식된 객체의 3D 좌표를 로봇 기준 좌표계로 변환한 뒤, Open Manipulator-X를 이용하여 Pick & Place 작업 수행
- **주요 기능**
	- Camera TF - Robot TF 좌표 변환
	- TF2 기반 좌표계 관리
	- Manipulator Joint 제어
	- Gripper 제어
	- Pick & Place 동작 수행
- **관련 패키지**
	- 
## 3. 플래투닝 및 협업 기반 객체 전달 시스템
[[3. Autonomous Platooning and Cooperative Object Transfer System]]
- 작업 로봇이 물체를 인식하고 조작한 뒤, 별도의 이동 로봇이 협업하여 물체를 운반하는 구조 구현
- **주요 기능**
	- Leader-Follower 기반 플래투닝
	- 로봇 간 거리 유지 제어
	- 속도 명령 발행
	- 협업 운반 시나리오 구성
	- TurtleBot3 다중 로봇 운용
- 관련 패키지
	- 
---
# 개발 환경
## 하드웨어 구성
### Mobile Manipulator

| 구분                  | 사용 하드웨어                   |
| ------------------- | ------------------------- |
| Moblie Base         | Turtlebot3 Waffle Pi      |
| Manipulator         | Open Manipulator - X      |
| 3D Camera           | Orbbec Astra 3D Camera    |
| End-effector Camear | USB Camera                |
| Gripper             | OpenManipulator-X Gripper |
| Control System      | ROS2 Control System       |
### Orbbec Astra 3D Camera
- TurtleBot 전면에 장착된 3D Deptha Camera
- 역할
	- RGB Image 획득
	- Depth Image 획득
	- 객체까지의 거리 추정
	- 3차원 좌표 계선
- 한계점
	- 물체와 카메라 사이 거리가 50cm이하로 가까워질 경우, Depth Data 불안정
	- 근거리에서 Depth Map이 뭉개진거나 유혀 Depth 값이 사라지는 문제 발생
- 본 프로젝트는 근거리 인식 문제를 보완하기 위해 Hybrid Depth Perception 구조를 사용
## 소프트웨어 구성

| 항목         | 사양                 | 항목                   | 환경                    |
| ---------- | ------------------ | -------------------- | --------------------- |
| Laptop     | Lenovo Yoga Pro 7i | OS                   | Ubuntu 22.04          |
| OS         | Ubuntu 22.04 LTS   | ROS                  | ROS2 Humble           |
| Kerner     | Linux 6.8.0        | Build System         | colcon                |
| CPU        | Intel i7 13th      | Main Language        | C++                   |
| GPU        | RTX 4050 Latop     | Visualization        | RViz2, rqt_image_view |
| GPU Driver | 580.65.06          | Camera Driver        | astra_camera          |
| CUDA       | 13.0               | Message System       | ROS2 custom msg       |
| VRAM       | 6GB                | Coordinate Transform | tf2                   |
| RAM        | 16GB               | Image Processing     | OpenCV                |
| Python     | 3.10               | Middleware           | DDS                   |
| PyTorch    | 2.9.1 + cu130      |                      |                       |
| OpenCV     | 4.12.0             |                      |                       |
## 개발 방향
- 본 프로젝트는 Python 기반 프로토타입보다 C++ 기반 구현을 우선시 한다.
	- RGB-D 데이터 실시간 처리
	- 실시간 객체 추적
	- PointCloud2, Image, CameraInfo 등 대용량 ROS  메시지
	- 장시간 Python 구현시 메모리 병목
	- ROS2 노드 최적화, 메모리 관리, 실시간 처리 측면에서 C++ 유리
---
# Repository Structure
```
turtlebot3_ws/src
├── depth_perception
│   ├── astra_camera
│   ├── astra_camera_msgs
│   ├── astra_mini_calibration
│   └── hybrid_single_target_tracker
├── mp_control
│   ├── config
│   ├── include
│   ├── launch
│   └── src
├── po_delivery
│   ├── config
│   ├── include
│   ├── launch
│   └── src
├── turtlebot3
├── turtlebot3_msgs
├── turtlebot3_manipulation
├── turtlebot3_simulations
└── DynamixelSDK
```
## Run




## Point ROS2 Topic
### Perception

| Topic                     | Type                          | Description                 |
| ------------------------- | ----------------------------- | --------------------------- |
| /camera/color/image_raw   | sensor_msgs/msg/Image         | RGB Image                   |
| /camera/depth/image_raw   | sensor_msgs/msg/Image         | Depth image                 |
| /camera/color/camera_info | sensor_msgs/msg/CameraInfo    | Camera intrinsic parameters |
| /target_object_pose       | geometry_msgs/msg/PoseStamped | Estimated object pose       |
### Manipulator Control

| Topic               | Type                          | Description                        |
| ------------------- | ----------------------------- | ---------------------------------- |
| /target_object_pose | geometry_msgs/msg/PoseStamped | Object pose from perception system |
| /mp_control/status  | std_msgs/msg/String           | Manipulator control status         |
| /joint_states       | sensor_msgs/msg/JointState    | Manipulator joint states           |
### Platooning Delivery

| Topic               | Type                    | Description                     |
| ------------------- | ----------------------- | ------------------------------- |
| /leader/odom        | nav_msgs/msg/Odometry   | Leader robot odometry           |
| /follower/odom      | nav_msgs/msg/Odometry   | Follower robot odometry         |
| /follower/cmd_vel   | geometry_msgs/msg/Twist | Follower robot velocity command |
| /po_delivery/status | std_msgs/msg/String     | Platooning delivery status      |

---
# Expected Result
- 본 프로젝트를 통해 RGB-D 기반 객체 인식, 3차원 좌표 추정, 모바일 매니퓰레이터 제어, Pick & Place, 플래투닝 기반 협업 운반 기능을 하나의 ROS 2 시스템으로 통합하는 것을 목표로 한다.
- 최종적으로 Leader Robot은 객체를 인식하고 조작하며, Follower Robot은 Leader Robot을 추종하여 협업 운반을 수행하는 구조를 구현한다.
# License
- This project is licensed under the Apache License 2.0.
