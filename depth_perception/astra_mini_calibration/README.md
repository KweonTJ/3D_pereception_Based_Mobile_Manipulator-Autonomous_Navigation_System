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

Run calibration on the leader while the front camera is publishing:

```bash
ros2 launch astra_mini_calibration astra_mini_calibration.launch.py \
  start_camera:=false \
  image_topic:=/camera/color/image_raw \
  camera_namespace:=/camera \
  board_size:=9x11 \
  square_size:=0.020 \
  camera_name:=astra_mini_color \
  output_prefix:=astra_mini_color
```

When SAVE is pressed, the calibration JSON is written to:

```text
~/turtlebot3_ws/src/depth_perception/astra_mini_calibration/config/astra_mini_color.json
```

After recalibration, launch the calibrated camera-info publisher with:

```bash
ros2 launch astra_mini_calibration astra_mini_calibrated.launch.py \
  start_camera:=false \
  image_topic:=/camera/color/image_raw \
  calibrated_camera_info_topic:=/camera/color/camera_info_calibrated
```
