# EEF Camera Calibration

Default calibration file:

```text
~/turtlebot3_ws/src/mp_control/calibration/eef_camera/eef_usb_camera.yaml
```

The real hardware launch files point `eef_camera_info_url` to this path by
default so the `camera_calibration` COMMIT button saves the EEF camera YAML into
the repository workspace.

Current saved calibration:

```text
camera: eef_usb_camera
resolution: 320x240
target distance: about 0.50 m
checkerboard: 9x11 inner corners, 0.020 m square
```

## Current ROS Interfaces

The EEF USB camera is launched in the `eef_camera` namespace by
`turtlebot3_manipulation_bringup/launch/hardware.launch.py`.

Use these names for calibration and runtime checks:

```text
image topic:       /eef_camera/image_raw
camera info topic: /eef_camera/camera_info
set-info service:  /eef_camera/set_camera_info
frame id:          eef_usb_camera_optical_frame
camera name:       eef_usb_camera
```

`camera_calibration` creates its service client as `camera/set_camera_info`;
the command below remaps that internal service to the active EEF camera service.
Do not use `/left` or `/right` for this mono EEF camera.

Run calibration on the leader while `/eef_camera/image_raw` is publishing:

```bash
PYTHONNOUSERSITE=1 \
ros2 run camera_calibration cameracalibrator \
  --size 9x11 \
  --square 0.020 \
  --camera_name eef_usb_camera \
  --no-service-check \
  --ros-args \
  -r image:=/eef_camera/image_raw \
  -r camera/set_camera_info:=/eef_camera/set_camera_info
```

After pressing `COMMIT`, the `v4l2_camera` node writes the calibration to the
URL configured by `eef_camera_info_url`. In the default launch this is:

```text
file://$HOME/turtlebot3_ws/src/mp_control/calibration/eef_camera/eef_usb_camera.yaml
```

Check that the new calibration is being used:

```bash
ros2 topic echo /eef_camera/camera_info --once
ros2 topic hz /eef_camera/image_raw
```

If the workspace is not `~/turtlebot3_ws`, override the launch argument:

```bash
eef_camera_info_url:=file:///absolute/path/to/eef_usb_camera.yaml
```

## Extrinsic Note

The measured EEF camera position relative to the front camera is not stored in
this intrinsic calibration YAML. Real RGB triangulation applies that value in
`mp_control/config/mp_control_real_params.yaml`:

```yaml
use_eef_front_camera_extrinsic_override: true
eef_front_camera_offset_x_m: -0.05
eef_front_camera_offset_y_m: 0.0
eef_front_camera_offset_z_m: 0.15
```

The EEF camera is 5 cm behind and 15 cm above the front camera in `base_link`
axes. Keep this separate from `camera_matrix`, `distortion_coefficients`, and
`projection_matrix`.
