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

Run calibration on the leader while `/eef_camera/image_raw` is publishing:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 9x11 \
  --square 0.020 \
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
