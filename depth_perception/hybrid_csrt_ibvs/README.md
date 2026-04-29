# hybrid_csrt_ibvs

ROS 2 Humble C++ package for a **CSRT + IBVS hybrid tracker/controller**.

This package is designed for the UROP I mobile-manipulator setup:

- TurtleBot3 Waffle Pi base
- OpenManipulator-X, optionally via MoveIt Servo-style `TwistStamped`
- Orbbec Astra / RGB-D camera, with USB-camera fallback
- object tracking after one bounding-box initialization
- image-based visual servoing directly from pixel error and depth/area error

## Why this package

The previous object-tracking pipeline can be simplified into one loop:

```text
RGB image + optional depth
        │
        ▼
initial bbox ──► CSRT tracker ──► target center / bbox area / depth
                                   │
                                   ▼
                              IBVS controller
                                   │
                   ┌───────────────┴────────────────┐
                   ▼                                ▼
              /cmd_vel                         optional arm twist
        TurtleBot3 base                /servo_node/delta_twist_cmds
```

CSRT tracks the target box frame-by-frame after initialization. IBVS then minimizes the image error:

- horizontal pixel error → base yaw command
- depth error → base forward/backward command
- if depth is unavailable, bounding-box area error → forward command fallback
- optional vertical/lateral error → arm Cartesian twist command

## Package contents

```text
hybrid_csrt_ibvs/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── turtlebot3_waffle_pi_orbbec.yaml
│   └── usb_cam_area_fallback.yaml
├── include/hybrid_csrt_ibvs/csrt_ibvs_node.hpp
├── launch/hybrid_csrt_ibvs.launch.py
├── src/
│   ├── csrt_ibvs_main.cpp
│   └── csrt_ibvs_node.cpp
└── tools/publish_bbox.py
```

## Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  libopencv-dev \
  libopencv-contrib-dev
```

`libopencv-contrib-dev` is important because OpenCV CSRT tracking is usually provided by the tracking/contrib module.

## Build

```bash
cd ~/ros2_ws/src
# copy or unzip hybrid_csrt_ibvs here
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select hybrid_csrt_ibvs
source install/setup.bash
```

## Run with Orbbec / RGB-D camera

```bash
ros2 launch hybrid_csrt_ibvs hybrid_csrt_ibvs.launch.py
```

If your camera topics differ, edit:

```bash
config/turtlebot3_waffle_pi_orbbec.yaml
```

Typical topic parameters:

```yaml
image_topic: /camera/color/image_raw
depth_topic: /camera/depth/image_raw
camera_info_topic: /camera/color/camera_info
cmd_vel_topic: /cmd_vel
```

## Run with USB camera only

```bash
ros2 launch hybrid_csrt_ibvs hybrid_csrt_ibvs.launch.py \
  config_file:=$(ros2 pkg prefix hybrid_csrt_ibvs)/share/hybrid_csrt_ibvs/config/usb_cam_area_fallback.yaml
```

This disables depth and uses bounding-box area as the approach signal.

## Initialize the CSRT target

The node waits for one bounding box on `/target/init_bbox`.

Message format:

```text
std_msgs/Float32MultiArray.data = [x, y, width, height]
```

Example:

```bash
ros2 run hybrid_csrt_ibvs publish_bbox.py 280 170 120 100
```

Equivalent raw ROS command:

```bash
ros2 topic pub --once /target/init_bbox std_msgs/msg/Float32MultiArray \
  "{data: [280.0, 170.0, 120.0, 100.0]}"
```

After initialization, CSRT owns the frame-to-frame target location. If tracking fails for `loss_frame_limit` frames, the node stops the robot and waits for a fresh bbox.

## Watch debug output

```bash
ros2 topic echo /hybrid_csrt_ibvs/status
rqt_image_view /hybrid_csrt_ibvs/debug_image
```

## Controller behavior

Default base command law:

```text
x_error = (target_u - desired_u) / focal_length_or_image_width
angular.z = -yaw_gain * x_error

if depth is available:
    z_error = depth_m - desired_depth_m
    linear.x = linear_gain * z_error
else:
    area_error = desired_area_ratio - bbox_area_ratio
    linear.x = area_gain * area_error
```

Forward motion is gated when the target is far from the image center. This prevents the robot from driving forward aggressively while still misaligned.

## Recommended tuning order

1. Keep `max_linear_x <= 0.10` and `max_angular_z <= 0.45` for first real-robot tests.
2. Tune `desired_depth_m` first. For grasping/pickup experiments, start around `0.40–0.50 m`.
3. Tune `yaw_gain` until the target recenters smoothly without oscillation.
4. Tune `linear_gain` only after yaw behavior is stable.
5. If depth is noisy, increase `depth_roi_radius_px` from `6` to `8` or `10`.
6. If the robot loses the target too easily, increase `loss_frame_limit`; if it reacts too slowly to failure, decrease it.

## Optional OpenManipulator-X output

The package can also publish `geometry_msgs/TwistStamped` to:

```text
/servo_node/delta_twist_cmds
```

Enable it only after MoveIt Servo or your own arm velocity bridge is ready:

```yaml
enable_arm_twist: true
arm_twist_topic: /servo_node/delta_twist_cmds
arm_command_frame_id: camera_color_optical_frame
```

For early experiments, leave `enable_arm_twist: false` and control only the TurtleBot3 base.

## YOLO / detector bridge

This package does not require YOLO every frame. The clean hybrid strategy is:

1. YOLO, HSV, or manual click produces the first bbox.
2. CSRT tracks that bbox at camera rate.
3. If CSRT fails, YOLO or manual selection republishes a bbox.
4. IBVS continuously drives the robot while CSRT is healthy.

Any detector can connect by publishing `[x, y, w, h]` to `/target/init_bbox`.

## Safety defaults

- publishes zero velocity when tracking is lost
- has a watchdog timeout for stale image/tracking updates
- clamps base velocity and angular velocity
- stops forward/depth motion below `emergency_stop_depth_m`
- uses small conservative gains for TurtleBot3 first tests

