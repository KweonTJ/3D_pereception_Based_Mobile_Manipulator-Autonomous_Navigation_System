# EEF Camera Calibration

Default calibration file:

```text
~/turtlebot3_ws/src/mp_control/calibration/eef_camera/eef_usb_camera.yaml
```

The real hardware launch files point `eef_camera_info_url` to this path by
default so the `camera_calibration` COMMIT button saves the EEF camera YAML into
the repository workspace.

Run calibration on the leader while `/eef_camera/image_raw` is publishing:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 10x8 \
  --square 0.026 \
  --camera_name eef_usb_camera \
  --no-service-check \
  --ros-args \
  -r image:=/eef_camera/image_raw \
  -r camera/set_camera_info:=/eef_camera/set_camera_info
```

If the workspace is not `~/turtlebot3_ws`, override the launch argument:

```bash
eef_camera_info_url:=file:///absolute/path/to/eef_usb_camera.yaml
```
