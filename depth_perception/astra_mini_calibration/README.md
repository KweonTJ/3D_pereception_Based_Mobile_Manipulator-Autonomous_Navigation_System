# Astra Mini And EEF Camera Calibration Scripts

This package provides run-based calibration helpers in `scripts/`. Use them when
you want to calibrate from the current ROS topics without starting an extra
launch file.

## Front Astra Color Camera

The front RGB-D camera uses `/camera/color/image_raw` after the depth reading is
latched near the minimum reliable range. Recalibrate the color camera around
0.47 m from the checkerboard, because this is the handoff distance where grasp
control stops trusting new depth samples.

Current checkerboard:

```text
interior corners: 9x11
square size: 0.020 m
target distance: about 0.47 m
```

Run calibration on the leader while the front camera is already publishing:

```bash
PYTHONNOUSERSITE=1 ros2 run astra_mini_calibration cameracalibrator_with_save.py \
  --size 9x11 \
  --square 0.020 \
  --camera_name astra_mini_color \
  --no-service-check \
  --ros-args \
  -r image:=/camera/color/image_raw \
  -r camera:=/camera
```

When SAVE is pressed, the calibration JSON is written to:

```text
~/turtlebot3_ws/src/depth_perception/astra_mini_calibration/config/astra_mini_color.json
```

To override the save target without using a launch file:

```bash
ASTRA_MINI_CALIB_OUTPUT_DIR=/absolute/output/dir \
ASTRA_MINI_CALIB_OUTPUT_PREFIX=astra_mini_color \
PYTHONNOUSERSITE=1 ros2 run astra_mini_calibration cameracalibrator_with_save.py \
  --size 9x11 \
  --square 0.020 \
  --camera_name astra_mini_color \
  --no-service-check \
  --ros-args \
  -r image:=/camera/color/image_raw \
  -r camera:=/camera
```

After recalibration, publish the calibrated camera info with `ros2 run`:

```bash
ros2 run astra_mini_calibration camera_info_from_json.py \
  --ros-args \
  -p json_path:=$HOME/turtlebot3_ws/src/depth_perception/astra_mini_calibration/config/astra_mini_color.json \
  -p camera_info_key:=camera_info \
  -r image:=/camera/color/image_raw \
  -r camera_info:=/camera/color/camera_info_calibrated
```

This path only republishes `CameraInfo`; it does not start the camera and does
not require `image_proc`.

The same publisher can also be launched through:

```bash
ros2 launch astra_mini_calibration astra_mini_calibrated.launch.py \
  start_camera:=false \
  image_topic:=/camera/color/image_raw \
  calibrated_camera_info_topic:=/camera/color/camera_info_calibrated \
  rectify_image:=false
```

## EEF USB Camera

The same `cameracalibrator_with_save.py` script can be used for the EEF USB
camera. Current EEF interfaces are:

```text
image topic:       /eef_camera/image_raw
camera info topic: /eef_camera/camera_info
set-info service:  /eef_camera/set_camera_info
camera name:       eef_usb_camera
resolution:        320x240
```

Run this while `/eef_camera/image_raw` is publishing:

```bash
ASTRA_MINI_CALIB_OUTPUT_DIR=$HOME/turtlebot3_ws/src/mp_control/calibration/eef_camera \
ASTRA_MINI_CALIB_OUTPUT_PREFIX=eef_usb_camera \
PYTHONNOUSERSITE=1 ros2 run astra_mini_calibration cameracalibrator_with_save.py \
  --size 9x11 \
  --square 0.020 \
  --camera_name eef_usb_camera \
  --no-service-check \
  --ros-args \
  -r image:=/eef_camera/image_raw \
  -r camera:=/eef_camera
```

Use the UI buttons this way:

```text
COMMIT  writes through /eef_camera/set_camera_info to the active v4l2 camera_info_url
SAVE    writes JSON to mp_control/calibration/eef_camera/eef_usb_camera.json
```

The runtime EEF camera calibration file used by the robot is still:

```text
mp_control/calibration/eef_camera/eef_usb_camera.yaml
```

After committing, verify the active EEF calibration:

```bash
ros2 topic echo /eef_camera/camera_info --once
ros2 topic hz /eef_camera/image_raw
```
