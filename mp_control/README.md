# mp_control

Real-robot grasp orchestration for the leader TurtleBot3 manipulation stack.

This package connects the front RGB-D tracker, the end-effector RGB detector,
MoveIt Servo, and the gripper action into one pick sequence.

## Current Real Grasp Flow

The active real configuration is:

```text
mp_control/launch/real_pick_place.launch.py
mp_control/config/mp_control_real_params.yaml
```

Runtime flow:

1. Front YOLO publishes `/target/init_bbox`.
2. `hybrid_csrt_ibvs` tracks the front-camera object and publishes `/target/tracked_bbox`.
3. The base approaches using front RGB-D until the Astra depth near limit.
4. EEF YOLO is pre-enabled before the depth handoff so its bbox is ready.
5. After the depth near limit, front depth is ignored.
6. Front RGB + EEF RGB triangulation narrows the robot-object distance.
7. The base stops, the arm moves to pregrasp, EEF RGB refinement runs, then the gripper closes.

## Important Real Parameters

Current depth and distance settings:

```yaml
min_valid_depth_m: 0.47
eef_refinement_start_depth_m: 0.47
eef_yolo_pre_enable_depth_m: 0.60
color_triangulation_base_stop_object_x_m: 0.30
arm_start_max_object_x_m: 0.30
object_pregrasp_standoff_m: 0.08
position_tolerance_m: 0.035
close_after_stable_cycles: 4
```

Meaning:

- `0.60 m`: EEF YOLO is enabled early, while front depth is still valid.
- `0.47 m`: front depth is no longer trusted for new object range estimates.
- `0.30 m`: RGB triangulation target distance before the arm enters the grasp phase.
- `0.08 m`: EEF pregrasp standoff from the triangulated object point.

## EEF Camera Path

The EEF camera has no depth stream. It is used as an RGB-only near-field camera.

Current control topic:

```yaml
eef_bbox_topic: /target/eef_init_bbox
```

That means real grasp control consumes the EEF YOLO bbox directly. The EEF
`hybrid_csrt_ibvs` node may still be launched for debugging, but `/target/eef_tracked_bbox`
is not the active grasp-control input.

EEF refinement uses pixel alignment only:

```yaml
eef_close_tolerance_px: 55.0
```

The tolerance is scaled from the configured EEF camera resolution, so the 320x240
EEF camera and 640x480 front camera are not treated as equivalent pixel grids.

## Front-EEF RGB Triangulation

After the 0.47 m depth handoff, `mp_control` triangulates the object using:

```text
/target/tracked_bbox      front RGB bbox
/target/eef_init_bbox     EEF RGB YOLO bbox
/camera/color/camera_info_calibrated
/eef_camera/camera_info
/tf
```

The measured EEF camera position relative to the front camera is applied only in
the triangulation step:

```yaml
use_eef_front_camera_extrinsic_override: true
eef_front_camera_offset_x_m: -0.05
eef_front_camera_offset_y_m: 0.0
eef_front_camera_offset_z_m: 0.15
```

This means the EEF camera is 5 cm behind and 15 cm above the front camera in
`base_link` axes. This does not change the MoveIt/arm TF used for Servo.

## Calibration Files

Front Astra color calibration:

```text
depth_perception/astra_mini_calibration/config/astra_mini_color.json
```

EEF USB camera calibration:

```text
mp_control/calibration/eef_camera/eef_usb_camera.yaml
```

The current EEF calibration is for the 320x240 EEF camera at about 50 cm.

## Useful Runtime Checks

```bash
ros2 topic echo /mp_control/status --once
ros2 topic echo /target/tracked_bbox --once
ros2 topic echo /target/eef_init_bbox --once
ros2 topic echo /eef_camera/camera_info --once
ros2 topic echo /servo_node/status --once
```

When RGB triangulation is active, `/mp_control/status` should include:

```text
offset_override=true
eef_front_offset=(-0.05, 0, 0.15)
```

## Build

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mp_control --symlink-install
```
