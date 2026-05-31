# Astra Mini Color Calibration

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

## 종료 처리

`camera_info_from_json.py`는 launch 종료나 `Ctrl-C` 때 `KeyboardInterrupt`를
정상 종료로 처리한다. ROS 컨텍스트가 이미 내려간 상태에서 다시
`rclpy.shutdown()`을 호출하지 않으므로, 종료 로그에
`rcl_shutdown already called` 예외가 떠서 전체 launch 종료를 지연시키는 상황을
피한다.

The same publisher can also be launched through:

```bash
ros2 launch astra_mini_calibration astra_mini_calibrated.launch.py \
  start_camera:=false \
  image_topic:=/camera/color/image_raw \
  calibrated_camera_info_topic:=/camera/color/camera_info_calibrated \
  rectify_image:=false
```
