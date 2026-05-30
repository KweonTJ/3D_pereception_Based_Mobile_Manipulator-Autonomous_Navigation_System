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
4. EEF YOLO runs from launch start so its bbox is already ready before the depth handoff.
5. After the depth near limit, front depth is ignored.
6. Front RGB + EEF RGB triangulation narrows the robot-object distance.
7. The base stops, the arm moves to joint pregrasp, EEF RGB refinement runs, then the gripper closes.

## Important Real Parameters

Current depth and distance settings:

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
pregrasp_ready_joint_positions: [0.0, 0.65, -0.85, -1.20]
pregrasp_reverse_joint3_delta: true
pregrasp_joint_tolerance_rad: 0.04
pregrasp_republish_period_s: 1.0
object_pregrasp_standoff_m: 0.08
use_eef_rpy_refinement: true
eef_hold_current_rpy: true
eef_forward_after_align: true
eef_forward_distance_m: 0.05
eef_forward_speed_mps: 0.012
position_tolerance_m: 0.035
close_after_stable_cycles: 4
```

Meaning:

- `0.60 m`: EEF YOLO is enabled early, while front depth is still valid.
- `0.47 m`: front depth is no longer trusted for new object range estimates.
- `0.08 / 0.40`: if the front bbox already fills at least 8% of the image or
  40% of image height, the system switches out of depth wait and starts the
  close-range RGB/EEF handoff path.
- `0.30 m`: RGB triangulation target distance before the arm enters the grasp phase.
- `use_fallback_bbox_for_control`: `mp_control` subscribes to the front YOLO
  `/target/init_bbox` as a fallback when `/target/tracked_bbox` becomes stale,
  so the close-range pregrasp handoff does not wait forever on CSRT output.
- `start_servo_on_start`: disabled for real joint pregrasp. `servo_node` also
  publishes `/arm_controller/joint_trajectory`, so starting it before pregrasp
  can overwrite the one-shot joint trajectory with a hold/current command.
  `mp_control` starts MoveIt Servo after joint pregrasp completes, just before
  EEF visual refinement.
- `use_joint_pregrasp`: real hardware sends `/arm_controller/joint_trajectory`
  before EEF refinement, avoiding MoveIt Servo collision scaling during arm
  extension.
- `pregrasp_joint_tolerance_rad`: EEF refinement and gripper close are blocked
  until `/joint_states` is actually within this tolerance of the pregrasp
  target. If the arm controller misses the one-shot command, `mp_control`
  republishes the same trajectory every `pregrasp_republish_period_s`.
- `pregrasp_reverse_joint3_delta`: keeps the configured ready target as the
  reference but moves joint3 in the opposite direction from the current real
  hardware pose.
- `0.08 m`: EEF pregrasp standoff from the triangulated object point.

## EEF Camera Path

The EEF camera has no depth stream. It is used as an RGB-only near-field camera.
The real launch starts EEF YOLO immediately and publishes `/target/eef_init_bbox`
continuously; `mp_control` only consumes it during the near-field grasp phase.

Current control topic:

```yaml
eef_bbox_topic: /target/eef_init_bbox
```

That means real grasp control consumes the EEF YOLO bbox directly. The EEF
`hybrid_csrt_ibvs` node may still be launched for debugging, but `/target/eef_tracked_bbox`
is not the active grasp-control input.

EEF refinement uses pixel alignment plus current-RPY hold:

```yaml
eef_close_tolerance_px: 70.0
```

The tolerance is scaled from the configured EEF camera resolution, so the 320x240
EEF camera and 640x480 front camera are not treated as equivalent pixel grids.
The real EEF camera can show a tightly fitted box with the object center around
60 px below the optical center; `70 px` treats that as close-ready. Once it is
ready, `mp_control` keeps the captured EE roll/pitch/yaw, drives the EE forward
for `eef_forward_distance_m`, then closes the gripper.

The EEF camera must not update gripper width. Width-aware gripper commands use
only the latest object width measured from the front depth image; if no valid
front-depth width exists, the configured fallback width is used. EEF RGB is only
for final position correction when the end effector is slightly misaligned.

After EEF bbox alignment, the real grasp path captures the current EE
roll/pitch/yaw as the hold reference, commands a short forward motion in
`base_link`, then closes the gripper. This keeps the hand from reorienting while
it advances into the object:

```yaml
use_eef_rpy_refinement: true
eef_hold_current_rpy: true
eef_rpy_tolerance_rad: 0.12
eef_rpy_gain: 0.8
eef_refine_max_angular_speed: 0.25
eef_forward_after_align: true
eef_forward_distance_m: 0.05
eef_forward_speed_mps: 0.012
```

During this phase `/mp_control/status` reports `rpy_err=(roll, pitch, yaw)`,
`rpy_ready`, and fixed-pose forward advance progress.

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
../depth_perception/astra_mini_calibration/config/astra_mini_color.json
```

EEF USB camera calibration:

```text
calibration/eef_camera/eef_usb_camera.yaml
```

The current EEF calibration is for the 320x240 EEF camera at about 50 cm. The
active runtime topics are:

```text
/eef_camera/image_raw
/eef_camera/camera_info
/eef_camera/set_camera_info
```

For the calibration command and service remapping, use:

```text
calibration/eef_camera/README.md
```

Front-EEF stereo calibration capture and validation scripts are documented in:

```text
scripts/STEREO_CALIBRATION_USAGE.md
```

Run them from `~/turtlebot3_ws/src/mp_control` with `python3 scripts/...`.
They use `/camera/color/image_raw` and `/eef_camera/image_raw` for capture, then
load intrinsics from `../depth_perception/.../astra_mini_color.json` and
`calibration/eef_camera/eef_usb_camera.yaml`.
Each script has a GUI path: use `--manual` or `--display` for capture, and
`--display` for calibration, validation, and YAML summary.

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
