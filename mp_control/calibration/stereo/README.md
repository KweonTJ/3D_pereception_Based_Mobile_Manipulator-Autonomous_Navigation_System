# Front/EEF Stereo Calibration

This calibration treats the Front Astra RGB camera and the EEF USB camera as one
two-view camera pair during the perception/triangulation phase.

The goal is to stop relying only on separate intrinsics plus hand-written
URDF/TF extrinsics. The canonical calibration artifact is:

```bash
mp_control/calibration/stereo/front_eef_stereo_calib.npz
```

Runtime C++ reads a derived OpenCV YAML generated from that `.npz`:

```bash
mp_control/calibration/stereo/front_eef_stereo_calib.yaml
```

Do not edit the YAML by hand. Regenerate it from the `.npz`.

Recommended layout:

```bash
mp_control/calibration/stereo/
  front_eef_stereo_calib.npz
  front_eef_stereo_calib.yaml
  images/
    front/
    eef/
  validation/
```

For detailed command-by-command usage, see `USAGE.md` in this directory.

## Coordinate Convention

OpenCV `stereoCalibrate()` is used with:

```text
camera1 = Front Astra RGB camera
camera2 = EEF USB camera
```

The saved transform follows:

```text
X_eef = R_front_to_eef * X_front + T_front_to_eef
```

Runtime triangulation uses normalized points with:

```text
P_front = [I | 0]
P_eef   = [R_front_to_eef | T_front_to_eef]
```

The triangulated 3D point is in the front camera optical frame, then transformed
to `base_link` with TF.

## Important Pose Requirement

The EEF camera must be in the same fixed arm pose during calibration and during
the perception triangulation phase. If the arm pose changes, the calibrated
Front-to-EEF transform is no longer valid.

Runtime can compare current TF against the calibration transform and warn when
the difference is large.

## Capture Image Pairs

Use a checkerboard with 9 x 11 inner corners and 20 mm square size.

```bash
ros2 run mp_control capture_front_eef_stereo_pairs.py \
  --front-topic /camera/color/image_raw \
  --eef-topic /eef_camera/image_raw \
  --output-dir mp_control/calibration/stereo/images \
  --manual
```

In manual mode, press `s` to save a pair and `q` or Esc to stop. Auto-save mode
is also available:

```bash
ros2 run mp_control capture_front_eef_stereo_pairs.py \
  --front-topic /camera/color/image_raw \
  --eef-topic /eef_camera/image_raw \
  --output-dir mp_control/calibration/stereo/images \
  --save-every-s 0.7 \
  --max-pairs 40
```

Move the board through different positions and tilts. Keep both cameras seeing
the same board at the same time.

## Generate Stereo NPZ

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

`board-cols` and `board-rows` are inner corner counts, not the number of black
and white squares.

To seed or fix existing intrinsics:

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

The `.npz` stores `K_front`, `D_front`, `K_eef`, `D_eef`,
`R_front_to_eef`, `T_front_to_eef`, image sizes, RMS error, board metadata,
camera names/frames, rectification outputs, and used image indices.

## Generate Runtime YAML

Generate runtime YAML from the canonical `.npz`:

```bash
python3 mp_control/scripts/stereo_npz_to_yaml.py \
  mp_control/calibration/stereo/front_eef_stereo_calib.npz \
  --output mp_control/calibration/stereo/front_eef_stereo_calib.yaml
```

## Offline Validation Without Running The Robot

Validate the `.npz` using saved checkerboard image pairs:

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

Outputs:

```bash
mp_control/calibration/stereo/validation/report.txt
mp_control/calibration/stereo/validation/report.csv
mp_control/calibration/stereo/validation/per_pair_errors.csv
mp_control/calibration/stereo/validation/overlays/
```

The report includes reprojection error, epipolar error, undistort shift,
positive depth ratio, reconstructed square size, plane fitting error, and
front/eef board distance.

## Real-Time Video Validation

For saved videos:

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

For directly connected cameras:

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

Useful stream options:

```bash
--max-frames 300
--frame-stride 5
--stream-log-every 10
--stream-overlay-every 30
--display
```

## Runtime Parameters

The default is the legacy TF/ray closest-point method:

```yaml
use_stereo_npz_calibration: false
```

Enable the calibrated stereo path only after validation passes:

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

If `input_images_are_rectified: false`, runtime uses OpenCV
`undistortPoints()` before triangulation. If the subscribed images are already
rectified, set it to `true`.

## Runtime Validation

After enabling the stereo path, place the target at known `base_link` x
distances such as:

```text
x = 0.50 m
x = 0.40 m
x = 0.30 m
```

Compare logged `triangulated point in base_link` or status object x against the
known distance.

Watch these values when `debug_stereo_triangulation: true`:

```text
front raw bbox center
eef raw bbox center
front undistorted normalized point
eef undistorted normalized point
triangulated point in front frame
triangulated point in base_link
front/eef reprojection error
current TF vs calibration transform warning
accepted/rejected reason
```

Common causes of bad triangulation:

- front bbox center and EEF bbox center are not the same physical object point
- distortion correction is disabled or wrong
- EEF pose differs from the calibration pose
- checkerboard calibration quality is poor
- `R/T` convention is reversed

## PASS / WARN / FAIL

Default validation thresholds:

```bash
--max-mean-reproj-error-px 2.0
--max-max-reproj-error-px 8.0
--max-mean-epipolar-error-px 2.0
--max-plane-error-m 0.005
--max-square-size-relative-error 0.15
```

PASS means the calibration is likely usable. WARN means inspect overlays and
CSV before trusting it. FAIL means do not enable runtime stereo calibration.

## Rollback

Set this parameter back to false:

```yaml
use_stereo_npz_calibration: false
```

The node will return to the existing TF-based two-ray closest-point
triangulation path.
