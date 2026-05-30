# Front/EEF Stereo Calibration Usage

This guide explains the generated calibration files and the exact commands for
using them.

## Files And Roles

### `capture_front_eef_stereo_pairs.py`

Purpose:

```text
Capture synchronized image pairs from:
- Front Astra RGB camera
- EEF USB camera
```

This is a ROS 2 Python node. It does not run the grasp controller, tracker, YOLO,
navigation, or pick/place sequence. It only needs the two camera image topics.

Output:

```text
mp_control/calibration/stereo/images/front/front_000.png
mp_control/calibration/stereo/images/eef/eef_000.png
mp_control/calibration/stereo/images/pairs.csv
```

### `stereo_calibrate_front_eef.py`

Purpose:

```text
Read saved front/eef image pairs
detect the calibration board
calibrate the two cameras as a stereo pair
save the canonical calibration result as .npz
```

The `.npz` is the source of truth.

Output:

```text
mp_control/calibration/stereo/front_eef_stereo_calib.npz
```

### `validate_stereo_calib_offline.py`

Purpose:

```text
Validate the .npz without running the robot control stack
```

It re-detects checkerboard corners from saved image pairs, triangulates the
board corners, computes board distance, reprojection error, epipolar error,
undistort shift, square-size reconstruction error, and plane fitting error.

It can also run against videos or directly connected OpenCV camera indices.

Output:

```text
mp_control/calibration/stereo/validation/report.txt
mp_control/calibration/stereo/validation/report.csv
mp_control/calibration/stereo/validation/per_pair_errors.csv
mp_control/calibration/stereo/validation/overlays/
```

### `stereo_npz_to_yaml.py`

Purpose:

```text
Convert the canonical .npz into an OpenCV YAML file for C++ runtime loading
```

Output:

```text
mp_control/calibration/stereo/front_eef_stereo_calib.yaml
```

The YAML is derived from the `.npz`. Do not edit the YAML by hand.

## Board Settings

Use the current project board settings:

```text
checkerboard inner corners: 9 x 11
square size: 0.020 m
```

`--board-cols` and `--board-rows` mean inner corner count. They do not mean the
number of black/white squares.

## Step 1: Capture Image Pairs

Start only the camera drivers needed to publish:

```text
/camera/color/image_raw
/eef_camera/image_raw
```

Manual capture:

```bash
ros2 run mp_control capture_front_eef_stereo_pairs.py \
  --front-topic /camera/color/image_raw \
  --eef-topic /eef_camera/image_raw \
  --output-dir mp_control/calibration/stereo/images \
  --manual
```

Manual controls:

```text
s       save current synchronized pair
q/Esc   quit
```

Automatic capture:

```bash
ros2 run mp_control capture_front_eef_stereo_pairs.py \
  --front-topic /camera/color/image_raw \
  --eef-topic /eef_camera/image_raw \
  --output-dir mp_control/calibration/stereo/images \
  --save-every-s 0.7 \
  --max-pairs 40
```

Recommended image count:

```text
absolute minimum for solving: 3 valid pairs
minimum for validation PASS: 10 valid pairs
recommended real capture: 20 to 40 valid pairs
```

Move the board around the shared field of view. Include center, left, right,
top, bottom, near, far, and tilted views. Both cameras must see the same board
pose.

## Step 2: Generate The `.npz`

Full stereo calibration:

```bash
python3 mp_control/scripts/stereo_calibrate_front_eef.py \
  --front-dir mp_control/calibration/stereo/images/front \
  --eef-dir mp_control/calibration/stereo/images/eef \
  --board-cols 9 \
  --board-rows 11 \
  --square-size 0.020 \
  --output mp_control/calibration/stereo/front_eef_stereo_calib.npz \
  --overlay-dir mp_control/calibration/stereo/calibration_overlays \
  --run-validation \
  --validation-output-dir mp_control/calibration/stereo/validation
```

Use existing front/eef intrinsics as fixed values and solve only stereo
extrinsics:

```bash
python3 mp_control/scripts/stereo_calibrate_front_eef.py \
  --front-dir mp_control/calibration/stereo/images/front \
  --eef-dir mp_control/calibration/stereo/images/eef \
  --board-cols 9 \
  --board-rows 11 \
  --square-size 0.020 \
  --front-intrinsics-json depth_perception/astra_mini_calibration/config/astra_mini_color.json \
  --eef-intrinsics-yaml mp_control/calibration/eef_camera/eef_usb_camera.yaml \
  --fix-intrinsics \
  --output mp_control/calibration/stereo/front_eef_stereo_calib.npz
```

Important output values printed by this step:

```text
valid pair count
front/eef image size
RMS error
baseline norm
det(R)
failed pair list
```

## Step 3: Validate The `.npz`

Run validation against the saved image pairs:

```bash
python3 mp_control/scripts/validate_stereo_calib_offline.py \
  mp_control/calibration/stereo/front_eef_stereo_calib.npz \
  --front-dir mp_control/calibration/stereo/images/front \
  --eef-dir mp_control/calibration/stereo/images/eef \
  --board-cols 9 \
  --board-rows 11 \
  --square-size 0.020 \
  --output-dir mp_control/calibration/stereo/validation
```

Check:

```text
final verdict
valid pair count
total reprojection error
epipolar error
positive depth ratio
mean_square_size_m
front_board_distance_mean_m
eef_board_distance_mean_m
plane fitting error
undistort shift
```

A useful quick expectation:

```text
mean_square_size_m should be close to 0.020 m
positive depth ratio should be close to 100%
reprojection and epipolar errors should be small
```

The overlay images are important. If a pair has bad corner detection, remove
that pair and regenerate the calibration.

## Step 4: Optional Synthetic Check

This checks math consistency of `K`, `D`, `R`, `T`. It does not prove real-world
calibration quality.

```bash
python3 mp_control/scripts/validate_stereo_calib_offline.py \
  mp_control/calibration/stereo/front_eef_stereo_calib.npz \
  --synthetic-test-only
```

If synthetic test fails but image-pair validation passes, trust the image-pair
validation more. Synthetic points can be outside a real camera overlap for
unusual camera geometry.

## Step 5: Manual Bbox Center Test

Use this to see what a specific front/eef bbox center pair triangulates to.

```bash
python3 mp_control/scripts/validate_stereo_calib_offline.py \
  mp_control/calibration/stereo/front_eef_stereo_calib.npz \
  --manual-points \
  --front-u 320 --front-v 240 \
  --eef-u 160 --eef-v 120
```

Output includes:

```text
front raw point
eef raw point
front/eef undistorted normalized points
triangulated point in front camera frame
triangulated point in eef camera frame
depth validity
```

## Step 6: Real-Time Video Validation

Saved videos:

```bash
python3 mp_control/scripts/validate_stereo_calib_offline.py \
  mp_control/calibration/stereo/front_eef_stereo_calib.npz \
  --stream-validation \
  --front-source mp_control/calibration/stereo/videos/front.mp4 \
  --eef-source mp_control/calibration/stereo/videos/eef.mp4 \
  --board-cols 9 \
  --board-rows 11 \
  --square-size 0.020 \
  --output-dir mp_control/calibration/stereo/validation_stream
```

Camera indices:

```bash
python3 mp_control/scripts/validate_stereo_calib_offline.py \
  mp_control/calibration/stereo/front_eef_stereo_calib.npz \
  --stream-validation \
  --front-source 0 \
  --eef-source 2 \
  --board-cols 9 \
  --board-rows 11 \
  --square-size 0.020 \
  --output-dir mp_control/calibration/stereo/validation_stream \
  --display
```

Useful options:

```bash
--max-frames 300
--frame-stride 5
--stream-log-every 10
--stream-overlay-every 30
--display
```

## Step 7: Generate Runtime YAML

Only do this after validation looks acceptable.

```bash
python3 mp_control/scripts/stereo_npz_to_yaml.py \
  mp_control/calibration/stereo/front_eef_stereo_calib.npz \
  --output mp_control/calibration/stereo/front_eef_stereo_calib.yaml
```

The C++ node reads this YAML. The `.npz` remains the canonical calibration file.

## Step 8: Enable Runtime Stereo Triangulation

In `mp_control/config/mp_control_real_params.yaml`:

```yaml
use_stereo_npz_calibration: true
stereo_calibration_file: "/absolute/path/to/mp_control/calibration/stereo/front_eef_stereo_calib.npz"
stereo_calibration_yaml_file: "/absolute/path/to/mp_control/calibration/stereo/front_eef_stereo_calib.yaml"
stereo_camera1_frame: "camera_color_optical_frame"
stereo_camera2_frame: "eef_usb_camera_optical_frame"
use_distortion_undistort_points: true
input_images_are_rectified: false
stereo_calibration_valid_only_when_eef_static: true
reject_stereo_tf_delta: false
debug_stereo_triangulation: true
max_stereo_reprojection_error_px: 8.0
min_triangulated_depth_m: 0.06
max_triangulated_depth_m: 1.0
max_stereo_tf_translation_delta_m: 0.03
max_stereo_tf_rotation_delta_deg: 5.0
```

`input_images_are_rectified: false` means runtime uses distortion-aware
`undistortPoints()` before triangulation. Set it to `true` only if both image
topics are already rectified.

## Rollback

To return to the previous TF-based ray closest-point triangulation:

```yaml
use_stereo_npz_calibration: false
```

No other runtime files need to change.

## Troubleshooting

High reprojection error:

```text
bad corner detection
wrong image pairs
bad intrinsics
distortion mismatch
```

High epipolar error:

```text
wrong R/T
unsynchronized or mismatched image pairs
corner order mismatch
```

Bad reconstructed square size:

```text
wrong square-size argument
bad T scale
poor board coverage
```

Runtime object position is wrong but validation passes:

```text
front bbox center and eef bbox center may not refer to the same physical point
EEF arm pose may differ from calibration pose
runtime image may be rectified while parameter says raw, or vice versa
```