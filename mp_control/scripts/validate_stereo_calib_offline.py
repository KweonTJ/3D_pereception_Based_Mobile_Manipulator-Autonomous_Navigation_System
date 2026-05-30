#!/usr/bin/env python3
"""Offline stereo calibration validation for front/eef image pairs.

This script is intentionally ROS-free. It validates a saved stereo `.npz`
calibration using only the calibration arrays and stored checkerboard images.
"""

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


@dataclass
class CheckResult:
    status: str
    name: str
    message: str


@dataclass
class CalibData:
    path: Path
    K_front: np.ndarray
    D_front: np.ndarray
    K_eef: np.ndarray
    D_eef: np.ndarray
    R_front_to_eef: np.ndarray
    T_front_to_eef: np.ndarray
    image_size_front: Optional[Tuple[int, int]] = None
    image_size_eef: Optional[Tuple[int, int]] = None
    image_size_common: Optional[Tuple[int, int]] = None
    rms_error: Optional[float] = None
    key_notes: List[str] = field(default_factory=list)


@dataclass
class PairResult:
    pair_index: int
    front_image: str
    eef_image: str
    front_detected: bool
    eef_detected: bool
    front_reproj_mean_px: float = math.nan
    front_reproj_max_px: float = math.nan
    eef_reproj_mean_px: float = math.nan
    eef_reproj_max_px: float = math.nan
    total_reproj_mean_px: float = math.nan
    total_reproj_max_px: float = math.nan
    epipolar_mean_px: float = math.nan
    epipolar_max_px: float = math.nan
    triangulated_positive_depth_ratio: float = math.nan
    eef_positive_depth_ratio: float = math.nan
    front_board_center_x_m: float = math.nan
    front_board_center_y_m: float = math.nan
    front_board_center_z_m: float = math.nan
    front_board_distance_m: float = math.nan
    front_board_depth_z_m: float = math.nan
    eef_board_center_x_m: float = math.nan
    eef_board_center_y_m: float = math.nan
    eef_board_center_z_m: float = math.nan
    eef_board_distance_m: float = math.nan
    eef_board_depth_z_m: float = math.nan
    mean_square_size_m: float = math.nan
    square_size_relative_error: float = math.nan
    plane_error_m: float = math.nan
    front_undistort_mean_px: float = math.nan
    front_undistort_max_px: float = math.nan
    eef_undistort_mean_px: float = math.nan
    eef_undistort_max_px: float = math.nan
    failure_reason: str = ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate front/eef stereo calibration offline from npz and saved image pairs."
    )
    parser.add_argument("calib", nargs="?", help="Path to stereo calibration .npz")
    parser.add_argument("--calib", dest="calib_opt", help="Path to stereo calibration .npz")
    parser.add_argument("--front-dir", help="Directory containing front camera checkerboard images")
    parser.add_argument("--eef-dir", help="Directory containing EEF camera checkerboard images")
    parser.add_argument("--board-cols", type=int, default=9, help="Checkerboard inner corner columns")
    parser.add_argument("--board-rows", type=int, default=11, help="Checkerboard inner corner rows")
    parser.add_argument("--square-size", type=float, default=0.020, help="Checkerboard square size in meters")
    parser.add_argument(
        "--output-dir",
        default="calibration/stereo/validation",
        help="Directory for report CSV/TXT and overlay images",
    )
    parser.add_argument("--synthetic-test-only", action="store_true", help="Run synthetic triangulation test only")
    parser.add_argument("--manual-points", action="store_true", help="Triangulate one manually supplied point pair")
    parser.add_argument("--front-u", type=float, help="Manual front image u pixel")
    parser.add_argument("--front-v", type=float, help="Manual front image v pixel")
    parser.add_argument("--eef-u", type=float, help="Manual EEF image u pixel")
    parser.add_argument("--eef-v", type=float, help="Manual EEF image v pixel")
    parser.add_argument(
        "--stream-validation",
        action="store_true",
        help="Validate continuously from video files or camera indices",
    )
    parser.add_argument(
        "--front-source",
        help='Front video source for stream validation, e.g. "0" or path/to/front.mp4',
    )
    parser.add_argument(
        "--eef-source",
        help='EEF video source for stream validation, e.g. "2" or path/to/eef.mp4',
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Maximum processed frames for stream validation; 0 means until stream ends",
    )
    parser.add_argument(
        "--frame-stride",
        type=int,
        default=1,
        help="Process every Nth stream frame",
    )
    parser.add_argument(
        "--stream-log-every",
        type=int,
        default=30,
        help="Print live stream metrics every N processed frames; 0 disables periodic logs",
    )
    parser.add_argument(
        "--stream-overlay-every",
        type=int,
        default=30,
        help="Save stream overlay images every N processed frames; 0 disables overlay image saving",
    )
    parser.add_argument("--display", action="store_true", help="Show validation overlay window")
    parser.add_argument(
        "--display-wait-ms",
        type=int,
        default=1,
        help="OpenCV wait time for saved-image overlays; 0 waits for a key",
    )

    parser.add_argument("--max-mean-reproj-error-px", type=float, default=2.0)
    parser.add_argument("--max-max-reproj-error-px", type=float, default=8.0)
    parser.add_argument("--max-mean-epipolar-error-px", type=float, default=2.0)
    parser.add_argument("--max-plane-error-m", type=float, default=0.005)
    parser.add_argument("--max-square-size-relative-error", type=float, default=0.15)
    parser.add_argument("--min-valid-pairs", type=int, default=10)
    parser.add_argument("--min-positive-depth-ratio", type=float, default=0.95)
    parser.add_argument("--min-baseline-m", type=float, default=0.01)
    parser.add_argument("--max-baseline-m", type=float, default=2.0)
    parser.add_argument("--det-r-tolerance", type=float, default=0.02)
    parser.add_argument("--max-rotation-orthogonality-error", type=float, default=1.0e-2)
    parser.add_argument("--max-synthetic-error-m", type=float, default=0.005)
    return parser.parse_args()


def format_status(check: CheckResult) -> str:
    return f"[{check.status}] {check.name}: {check.message}"


def load_npz_array(
    npz: np.lib.npyio.NpzFile,
    canonical_key: str,
    aliases: Sequence[str],
    notes: List[str],
) -> np.ndarray:
    for key in (canonical_key, *aliases):
        if key in npz.files:
            if key != canonical_key:
                notes.append(f'{canonical_key} loaded from alias "{key}"')
            return np.asarray(npz[key], dtype=np.float64)
    expected = ", ".join((canonical_key, *aliases))
    raise KeyError(f"Missing required calibration key. Expected one of: {expected}")


def parse_image_size(value: np.ndarray) -> Optional[Tuple[int, int]]:
    flat = np.asarray(value).reshape(-1)
    if flat.size < 2:
        return None
    width = int(round(float(flat[0])))
    height = int(round(float(flat[1])))
    if width <= 0 or height <= 0:
        return None
    return width, height


def optional_image_size(npz: np.lib.npyio.NpzFile, keys: Sequence[str]) -> Optional[Tuple[int, int]]:
    for key in keys:
        if key in npz.files:
            return parse_image_size(npz[key])
    return None


def optional_float(npz: np.lib.npyio.NpzFile, keys: Sequence[str]) -> Optional[float]:
    for key in keys:
        if key in npz.files:
            flat = np.asarray(npz[key]).reshape(-1)
            if flat.size:
                return float(flat[0])
    return None


def load_calibration(path: Path) -> CalibData:
    if not path.exists():
        raise FileNotFoundError(f"Calibration file not found: {path}")

    notes: List[str] = []
    with np.load(str(path), allow_pickle=False) as npz:
        K_front = load_npz_array(
            npz,
            "K_front",
            ("front_K", "camera_matrix_front", "K1", "M1"),
            notes,
        )
        D_front = load_npz_array(
            npz,
            "D_front",
            ("front_D", "dist_coeffs_front", "D1", "d1"),
            notes,
        )
        K_eef = load_npz_array(
            npz,
            "K_eef",
            ("eef_K", "camera_matrix_eef", "K2", "M2"),
            notes,
        )
        D_eef = load_npz_array(
            npz,
            "D_eef",
            ("eef_D", "dist_coeffs_eef", "D2", "d2"),
            notes,
        )
        R_front_to_eef = load_npz_array(
            npz,
            "R_front_to_eef",
            ("R", "R_front_eef", "rotation", "stereo_R"),
            notes,
        )
        T_front_to_eef = load_npz_array(
            npz,
            "T_front_to_eef",
            ("T", "T_front_eef", "translation", "stereo_T"),
            notes,
        )
        image_size_common = optional_image_size(npz, ("image_size", "imageSize"))
        image_size_front = optional_image_size(
            npz,
            ("image_size_front", "front_image_size", "imageSize_front"),
        )
        image_size_eef = optional_image_size(
            npz,
            ("image_size_eef", "eef_image_size", "imageSize_eef"),
        )
        rms_error = optional_float(npz, ("rms", "rms_error", "calibration_rms", "stereo_rms"))

    return CalibData(
        path=path,
        K_front=K_front,
        D_front=D_front.reshape(-1),
        K_eef=K_eef,
        D_eef=D_eef.reshape(-1),
        R_front_to_eef=R_front_to_eef,
        T_front_to_eef=T_front_to_eef.reshape(-1),
        image_size_front=image_size_front or image_size_common,
        image_size_eef=image_size_eef or image_size_common,
        image_size_common=image_size_common,
        rms_error=rms_error,
        key_notes=notes,
    )


def validate_shape_and_pose(
    calib: CalibData,
    args: argparse.Namespace,
    front_input_size: Optional[Tuple[int, int]] = None,
    eef_input_size: Optional[Tuple[int, int]] = None,
) -> List[CheckResult]:
    checks: List[CheckResult] = []

    def check(condition: bool, status_name: str, name: str, ok_msg: str, bad_msg: str) -> None:
        checks.append(CheckResult("OK" if condition else status_name, name, ok_msg if condition else bad_msg))

    check(
        calib.K_front.shape == (3, 3),
        "FAIL",
        "K_front shape",
        f"{calib.K_front.shape}",
        f"K_front shape invalid: {calib.K_front.shape}",
    )
    check(
        calib.K_eef.shape == (3, 3),
        "FAIL",
        "K_eef shape",
        f"{calib.K_eef.shape}",
        f"K_eef shape invalid: {calib.K_eef.shape}",
    )
    check(
        calib.D_front.ndim == 1 and calib.D_front.size >= 4,
        "FAIL",
        "D_front length",
        f"{calib.D_front.size}",
        f"D_front shape invalid: {calib.D_front.shape}",
    )
    check(
        calib.D_eef.ndim == 1 and calib.D_eef.size >= 4,
        "FAIL",
        "D_eef length",
        f"{calib.D_eef.size}",
        f"D_eef shape invalid: {calib.D_eef.shape}",
    )
    check(
        calib.R_front_to_eef.shape == (3, 3),
        "FAIL",
        "R_front_to_eef shape",
        f"{calib.R_front_to_eef.shape}",
        f"R_front_to_eef shape invalid: {calib.R_front_to_eef.shape}",
    )
    check(
        calib.T_front_to_eef.size == 3,
        "FAIL",
        "T_front_to_eef shape",
        f"{calib.T_front_to_eef.shape}",
        f"T_front_to_eef shape invalid: {calib.T_front_to_eef.shape}",
    )

    if calib.R_front_to_eef.shape == (3, 3):
        det_r = float(np.linalg.det(calib.R_front_to_eef))
        ortho_error = float(np.linalg.norm(calib.R_front_to_eef.T @ calib.R_front_to_eef - np.eye(3), ord="fro"))
        checks.append(
            CheckResult(
                "OK" if abs(det_r - 1.0) <= args.det_r_tolerance else "FAIL",
                "det(R)",
                f"{det_r:.6f}",
            )
        )
        checks.append(
            CheckResult(
                "OK" if ortho_error <= args.max_rotation_orthogonality_error else "FAIL",
                "R^T R error",
                f"{ortho_error:.6g}",
            )
        )

    if calib.T_front_to_eef.size == 3:
        baseline = float(np.linalg.norm(calib.T_front_to_eef))
        baseline_ok = args.min_baseline_m <= baseline <= args.max_baseline_m
        checks.append(
            CheckResult(
                "OK" if baseline_ok else "FAIL",
                "baseline norm",
                f"{baseline:.6f} m",
            )
        )

    if calib.image_size_front:
        if front_input_size and calib.image_size_front != front_input_size:
            checks.append(
                CheckResult(
                    "WARN",
                    "front image size",
                    f"npz says {calib.image_size_front[0]}x{calib.image_size_front[1]}, input image is {front_input_size[0]}x{front_input_size[1]}",
                )
            )
        else:
            checks.append(
                CheckResult(
                    "OK",
                    "front image size",
                    f"{calib.image_size_front[0]}x{calib.image_size_front[1]}",
                )
            )
    else:
        checks.append(CheckResult("WARN", "front image size", "not stored in npz"))

    if calib.image_size_eef:
        if eef_input_size and calib.image_size_eef != eef_input_size:
            checks.append(
                CheckResult(
                    "WARN",
                    "eef image size",
                    (
                        f"npz says {calib.image_size_eef[0]}x{calib.image_size_eef[1]}, "
                        f"input image is {eef_input_size[0]}x{eef_input_size[1]}"
                    ),
                )
            )
        else:
            checks.append(
                CheckResult(
                    "OK",
                    "eef image size",
                    f"{calib.image_size_eef[0]}x{calib.image_size_eef[1]}",
                )
            )
    else:
        checks.append(CheckResult("WARN", "eef image size", "not stored in npz"))

    for note in calib.key_notes:
        checks.append(CheckResult("WARN", "key alias", note))

    return checks


def image_size(path: Path) -> Optional[Tuple[int, int]]:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        return None
    height, width = image.shape[:2]
    return width, height


def list_images(directory: Path) -> List[Path]:
    if not directory.exists():
        raise FileNotFoundError(f"Image directory not found: {directory}")
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def pair_images(front_dir: Path, eef_dir: Path) -> Tuple[List[Tuple[Path, Path]], List[str]]:
    front_images = list_images(front_dir)
    eef_images = list_images(eef_dir)
    warnings: List[str] = []

    if not front_images:
        raise RuntimeError(f"No front images found in {front_dir}")
    if not eef_images:
        raise RuntimeError(f"No EEF images found in {eef_dir}")

    front_by_stem = {path.stem: path for path in front_images}
    eef_by_stem = {path.stem: path for path in eef_images}
    common_stems = sorted(set(front_by_stem) & set(eef_by_stem))

    if common_stems:
        pairs = [(front_by_stem[stem], eef_by_stem[stem]) for stem in common_stems]
        missing_front = sorted(set(eef_by_stem) - set(front_by_stem))
        missing_eef = sorted(set(front_by_stem) - set(eef_by_stem))
        if missing_front:
            warnings.append(f"EEF images without matching front stem: {len(missing_front)}")
        if missing_eef:
            warnings.append(f"Front images without matching EEF stem: {len(missing_eef)}")
        return pairs, warnings

    pair_count = min(len(front_images), len(eef_images))
    if len(front_images) != len(eef_images):
        warnings.append(
            f"No common file stems; pairing by sorted index with count mismatch front={len(front_images)}, eef={len(eef_images)}"
        )
    else:
        warnings.append("No common file stems; pairing by sorted index")
    return list(zip(front_images[:pair_count], eef_images[:pair_count])), warnings


def detect_checkerboard_in_image(
    image: np.ndarray,
    pattern_size: Tuple[int, int],
) -> Tuple[bool, Optional[np.ndarray]]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    found = False
    corners = None
    if hasattr(cv2, "findChessboardCornersSB"):
        sb_flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=sb_flags)

    if not found:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if found:
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                40,
                0.001,
            )
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    if not found or corners is None:
        return False, None
    return True, corners.reshape(-1, 2).astype(np.float64)


def detect_checkerboard(
    image_path: Path,
    pattern_size: Tuple[int, int],
) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        return False, None, None
    found, corners = detect_checkerboard_in_image(image, pattern_size)
    return found, corners, image


def save_corner_overlay(
    image: np.ndarray,
    corners: Optional[np.ndarray],
    pattern_size: Tuple[int, int],
    found: bool,
    output_path: Path,
) -> None:
    overlay = image.copy()
    if corners is not None:
        cv2.drawChessboardCorners(overlay, pattern_size, corners.reshape(-1, 1, 2).astype(np.float32), found)
    else:
        cv2.putText(
            overlay,
            "corners not detected",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), overlay)


def undistort_to_normalized(points_px: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    points = np.asarray(points_px, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.undistortPoints(points, K, D).reshape(-1, 2)


def undistort_to_pixel(points_px: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    points = np.asarray(points_px, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.undistortPoints(points, K, D, P=K).reshape(-1, 2)


def undistort_shift_stats(points_px: np.ndarray, K: np.ndarray, D: np.ndarray) -> Tuple[float, float]:
    corrected = undistort_to_pixel(points_px, K, D)
    shifts = np.linalg.norm(corrected - points_px.reshape(-1, 2), axis=1)
    return float(np.mean(shifts)), float(np.max(shifts))


def sample_undistort_shift(
    image_size_value: Optional[Tuple[int, int]],
    K: np.ndarray,
    D: np.ndarray,
) -> Tuple[float, float]:
    if not image_size_value:
        return math.nan, math.nan
    width, height = image_size_value
    samples = np.array(
        [
            [0.5 * width, 0.5 * height],
            [0.02 * width, 0.02 * height],
            [0.98 * width, 0.02 * height],
            [0.02 * width, 0.98 * height],
            [0.98 * width, 0.98 * height],
        ],
        dtype=np.float64,
    )
    corrected = undistort_to_pixel(samples, K, D)
    shifts = np.linalg.norm(corrected - samples, axis=1)
    return float(shifts[0]), float(np.max(shifts[1:]))


def triangulate_normalized(
    front_norm: np.ndarray,
    eef_norm: np.ndarray,
    R_front_to_eef: np.ndarray,
    T_front_to_eef: np.ndarray,
) -> np.ndarray:
    P_front = np.hstack((np.eye(3), np.zeros((3, 1))))
    P_eef = np.hstack((R_front_to_eef, T_front_to_eef.reshape(3, 1)))
    homogeneous = cv2.triangulatePoints(P_front, P_eef, front_norm.T, eef_norm.T).T
    xyz = np.full((homogeneous.shape[0], 3), np.nan, dtype=np.float64)
    valid_w = np.abs(homogeneous[:, 3]) > 1.0e-12
    xyz[valid_w] = homogeneous[valid_w, :3] / homogeneous[valid_w, 3:4]
    return xyz


def transform_front_to_eef(points_front: np.ndarray, R: np.ndarray, T: np.ndarray) -> np.ndarray:
    return (R @ points_front.T + T.reshape(3, 1)).T


def project_points(points_camera: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    points = np.asarray(points_camera, dtype=np.float64).reshape(-1, 1, 3)
    projected, _ = cv2.projectPoints(points, np.zeros((3, 1)), np.zeros((3, 1)), K, D)
    return projected.reshape(-1, 2)


def reprojection_error(
    projected_px: np.ndarray,
    detected_px: np.ndarray,
) -> Tuple[float, float, np.ndarray]:
    errors = np.linalg.norm(projected_px.reshape(-1, 2) - detected_px.reshape(-1, 2), axis=1)
    return float(np.mean(errors)), float(np.max(errors)), errors


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector.reshape(3)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )


def point_line_distances(points_h: np.ndarray, lines: np.ndarray) -> np.ndarray:
    numerators = np.abs(np.sum(points_h * lines, axis=1))
    denominators = np.linalg.norm(lines[:, :2], axis=1)
    distances = np.full(points_h.shape[0], np.nan, dtype=np.float64)
    valid = denominators > 1.0e-12
    distances[valid] = numerators[valid] / denominators[valid]
    return distances


def epipolar_errors_px(
    front_points_px: np.ndarray,
    eef_points_px: np.ndarray,
    calib: CalibData,
) -> Tuple[float, float]:
    front_ideal_px = undistort_to_pixel(front_points_px, calib.K_front, calib.D_front)
    eef_ideal_px = undistort_to_pixel(eef_points_px, calib.K_eef, calib.D_eef)
    E = skew(calib.T_front_to_eef) @ calib.R_front_to_eef
    F = np.linalg.inv(calib.K_eef).T @ E @ np.linalg.inv(calib.K_front)

    front_h = np.column_stack((front_ideal_px, np.ones(front_ideal_px.shape[0])))
    eef_h = np.column_stack((eef_ideal_px, np.ones(eef_ideal_px.shape[0])))
    lines_in_eef = (F @ front_h.T).T
    lines_in_front = (F.T @ eef_h.T).T
    distance_eef = point_line_distances(eef_h, lines_in_eef)
    distance_front = point_line_distances(front_h, lines_in_front)
    symmetric = 0.5 * (distance_eef + distance_front)
    return float(np.nanmean(symmetric)), float(np.nanmax(symmetric))


def checkerboard_square_stats(
    points: np.ndarray,
    rows: int,
    cols: int,
    expected_square_size: float,
) -> Tuple[float, float]:
    grid = points.reshape(rows, cols, 3)
    distances: List[float] = []

    for row in range(rows):
        for col in range(cols - 1):
            a = grid[row, col]
            b = grid[row, col + 1]
            if np.all(np.isfinite(a)) and np.all(np.isfinite(b)):
                distances.append(float(np.linalg.norm(a - b)))

    for row in range(rows - 1):
        for col in range(cols):
            a = grid[row, col]
            b = grid[row + 1, col]
            if np.all(np.isfinite(a)) and np.all(np.isfinite(b)):
                distances.append(float(np.linalg.norm(a - b)))

    if not distances:
        return math.nan, math.nan
    mean_square = float(np.mean(distances))
    relative_error = abs(mean_square - expected_square_size) / expected_square_size
    return mean_square, float(relative_error)


def plane_fit_error(points: np.ndarray) -> Tuple[float, float]:
    finite_points = points[np.all(np.isfinite(points), axis=1)]
    if finite_points.shape[0] < 3:
        return math.nan, math.nan
    centroid = np.mean(finite_points, axis=0)
    centered = finite_points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    distances = np.abs(centered @ normal)
    return float(np.mean(distances)), float(np.max(distances))


def board_center_distance(points_camera: np.ndarray) -> Tuple[np.ndarray, float, float]:
    finite_points = points_camera[np.all(np.isfinite(points_camera), axis=1)]
    if finite_points.size == 0:
        return np.full(3, np.nan, dtype=np.float64), math.nan, math.nan
    center = np.mean(finite_points, axis=0)
    distance = float(np.linalg.norm(center))
    depth_z = float(center[2])
    return center, distance, depth_z


def compute_pair_metrics(
    pair_index: int,
    front_label: str,
    eef_label: str,
    front_found: bool,
    front_corners: Optional[np.ndarray],
    eef_found: bool,
    eef_corners: Optional[np.ndarray],
    calib: CalibData,
    board_rows: int,
    board_cols: int,
    square_size: float,
) -> PairResult:
    result = PairResult(
        pair_index=pair_index,
        front_image=front_label,
        eef_image=eef_label,
        front_detected=front_found,
        eef_detected=eef_found,
    )

    if not front_found or front_corners is None:
        result.failure_reason = "front checkerboard not detected"
        return result
    if not eef_found or eef_corners is None:
        result.failure_reason = "eef checkerboard not detected"
        return result
    if front_corners.shape != eef_corners.shape:
        result.failure_reason = f"corner count mismatch front={front_corners.shape[0]} eef={eef_corners.shape[0]}"
        return result

    result.front_undistort_mean_px, result.front_undistort_max_px = undistort_shift_stats(
        front_corners,
        calib.K_front,
        calib.D_front,
    )
    result.eef_undistort_mean_px, result.eef_undistort_max_px = undistort_shift_stats(
        eef_corners,
        calib.K_eef,
        calib.D_eef,
    )

    front_norm = undistort_to_normalized(front_corners, calib.K_front, calib.D_front)
    eef_norm = undistort_to_normalized(eef_corners, calib.K_eef, calib.D_eef)
    points_front = triangulate_normalized(
        front_norm,
        eef_norm,
        calib.R_front_to_eef,
        calib.T_front_to_eef,
    )

    finite = np.all(np.isfinite(points_front), axis=1)
    if not np.any(finite):
        result.failure_reason = "triangulation produced no finite points"
        return result

    points_eef = transform_front_to_eef(points_front, calib.R_front_to_eef, calib.T_front_to_eef)
    result.triangulated_positive_depth_ratio = float(np.mean(points_front[finite, 2] > 0.0))
    result.eef_positive_depth_ratio = float(np.mean(points_eef[finite, 2] > 0.0))

    points_front_finite = points_front[finite]
    points_eef_finite = points_eef[finite]
    front_board_center, result.front_board_distance_m, result.front_board_depth_z_m = board_center_distance(
        points_front_finite
    )
    eef_board_center, result.eef_board_distance_m, result.eef_board_depth_z_m = board_center_distance(
        points_eef_finite
    )
    (
        result.front_board_center_x_m,
        result.front_board_center_y_m,
        result.front_board_center_z_m,
    ) = front_board_center.tolist()
    (
        result.eef_board_center_x_m,
        result.eef_board_center_y_m,
        result.eef_board_center_z_m,
    ) = eef_board_center.tolist()

    result.mean_square_size_m, result.square_size_relative_error = checkerboard_square_stats(
        points_front,
        board_rows,
        board_cols,
        square_size,
    )
    result.plane_error_m, _ = plane_fit_error(points_front)

    front_corners_finite = front_corners[finite]
    eef_corners_finite = eef_corners[finite]

    front_projected = project_points(points_front_finite, calib.K_front, calib.D_front)
    eef_projected = project_points(points_eef_finite, calib.K_eef, calib.D_eef)
    result.front_reproj_mean_px, result.front_reproj_max_px, front_errors = reprojection_error(
        front_projected,
        front_corners_finite,
    )
    result.eef_reproj_mean_px, result.eef_reproj_max_px, eef_errors = reprojection_error(
        eef_projected,
        eef_corners_finite,
    )
    total_errors = np.concatenate((front_errors, eef_errors))
    result.total_reproj_mean_px = float(np.mean(total_errors))
    result.total_reproj_max_px = float(np.max(total_errors))
    result.epipolar_mean_px, result.epipolar_max_px = epipolar_errors_px(
        front_corners,
        eef_corners,
        calib,
    )
    return result


def process_pair(
    pair_index: int,
    front_path: Path,
    eef_path: Path,
    calib: CalibData,
    pattern_size: Tuple[int, int],
    board_rows: int,
    board_cols: int,
    square_size: float,
    overlay_dir: Path,
) -> PairResult:
    front_found, front_corners, front_image = detect_checkerboard(front_path, pattern_size)
    eef_found, eef_corners, eef_image = detect_checkerboard(eef_path, pattern_size)

    if front_image is not None:
        save_corner_overlay(
            front_image,
            front_corners,
            pattern_size,
            front_found,
            overlay_dir / f"front_{pair_index:03d}_corners.png",
        )
    if eef_image is not None:
        save_corner_overlay(
            eef_image,
            eef_corners,
            pattern_size,
            eef_found,
            overlay_dir / f"eef_{pair_index:03d}_corners.png",
        )

    return compute_pair_metrics(
        pair_index=pair_index,
        front_label=str(front_path),
        eef_label=str(eef_path),
        front_found=front_found,
        front_corners=front_corners,
        eef_found=eef_found,
        eef_corners=eef_corners,
        calib=calib,
        board_rows=board_rows,
        board_cols=board_cols,
        square_size=square_size,
    )


def show_saved_pair_overlays(
    pair_results: Sequence[PairResult],
    overlay_dir: Path,
    wait_ms: int,
) -> None:
    for result in pair_results:
        front_overlay_path = overlay_dir / f"front_{result.pair_index:03d}_corners.png"
        eef_overlay_path = overlay_dir / f"eef_{result.pair_index:03d}_corners.png"
        front_overlay = cv2.imread(str(front_overlay_path), cv2.IMREAD_COLOR)
        eef_overlay = cv2.imread(str(eef_overlay_path), cv2.IMREAD_COLOR)
        if front_overlay is None or eef_overlay is None:
            continue

        combined = stack_overlays(front_overlay, eef_overlay)
        lines = [
            f"pair {result.pair_index:03d}",
            f"valid: {'yes' if not result.failure_reason else 'no'}",
        ]
        if result.failure_reason:
            lines.append(result.failure_reason)
        else:
            lines.append(f"reproj mean/max: {result.total_reproj_mean_px:.3f}/{result.total_reproj_max_px:.3f}px")
            lines.append(f"epipolar mean: {result.epipolar_mean_px:.3f}px")
            lines.append(f"front range: {result.front_board_distance_m:.4f}m")
            lines.append(f"eef range: {result.eef_board_distance_m:.4f}m")
        cv2.rectangle(combined, (0, 0), (min(combined.shape[1], 620), 26 * len(lines) + 12), (0, 0, 0), -1)
        for index, line in enumerate(lines):
            cv2.putText(
                combined,
                line,
                (12, 26 + 26 * index),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if index == 1 and not result.failure_reason else (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        cv2.imshow("front/eef stereo validation", combined)
        key = cv2.waitKey(max(0, wait_ms)) & 0xFF
        if key in (ord("q"), 27):
            break
    cv2.destroyAllWindows()


def finite_mean(values: Iterable[float]) -> float:
    array = np.asarray([value for value in values if math.isfinite(value)], dtype=np.float64)
    return float(np.mean(array)) if array.size else math.nan


def finite_max(values: Iterable[float]) -> float:
    array = np.asarray([value for value in values if math.isfinite(value)], dtype=np.float64)
    return float(np.max(array)) if array.size else math.nan


def finite_min(values: Iterable[float]) -> float:
    array = np.asarray([value for value in values if math.isfinite(value)], dtype=np.float64)
    return float(np.min(array)) if array.size else math.nan


def aggregate_results(pair_results: Sequence[PairResult]) -> Dict[str, float]:
    valid = [result for result in pair_results if result.front_detected and result.eef_detected and not result.failure_reason]
    return {
        "pair_count": float(len(pair_results)),
        "valid_pair_count": float(len(valid)),
        "failed_pair_count": float(len(pair_results) - len(valid)),
        "front_reproj_mean_px": finite_mean(result.front_reproj_mean_px for result in valid),
        "front_reproj_max_px": finite_max(result.front_reproj_max_px for result in valid),
        "eef_reproj_mean_px": finite_mean(result.eef_reproj_mean_px for result in valid),
        "eef_reproj_max_px": finite_max(result.eef_reproj_max_px for result in valid),
        "total_reproj_mean_px": finite_mean(result.total_reproj_mean_px for result in valid),
        "total_reproj_max_px": finite_max(result.total_reproj_max_px for result in valid),
        "epipolar_mean_px": finite_mean(result.epipolar_mean_px for result in valid),
        "epipolar_max_px": finite_max(result.epipolar_max_px for result in valid),
        "positive_depth_ratio": finite_mean(result.triangulated_positive_depth_ratio for result in valid),
        "eef_positive_depth_ratio": finite_mean(result.eef_positive_depth_ratio for result in valid),
        "front_board_distance_mean_m": finite_mean(result.front_board_distance_m for result in valid),
        "front_board_distance_min_m": finite_min(result.front_board_distance_m for result in valid),
        "front_board_distance_max_m": finite_max(result.front_board_distance_m for result in valid),
        "front_board_depth_z_mean_m": finite_mean(result.front_board_depth_z_m for result in valid),
        "front_board_depth_z_min_m": finite_min(result.front_board_depth_z_m for result in valid),
        "front_board_depth_z_max_m": finite_max(result.front_board_depth_z_m for result in valid),
        "eef_board_distance_mean_m": finite_mean(result.eef_board_distance_m for result in valid),
        "eef_board_distance_min_m": finite_min(result.eef_board_distance_m for result in valid),
        "eef_board_distance_max_m": finite_max(result.eef_board_distance_m for result in valid),
        "eef_board_depth_z_mean_m": finite_mean(result.eef_board_depth_z_m for result in valid),
        "eef_board_depth_z_min_m": finite_min(result.eef_board_depth_z_m for result in valid),
        "eef_board_depth_z_max_m": finite_max(result.eef_board_depth_z_m for result in valid),
        "mean_square_size_m": finite_mean(result.mean_square_size_m for result in valid),
        "square_size_relative_error": finite_mean(result.square_size_relative_error for result in valid),
        "plane_error_m": finite_mean(result.plane_error_m for result in valid),
        "front_undistort_mean_px": finite_mean(result.front_undistort_mean_px for result in valid),
        "front_undistort_max_px": finite_max(result.front_undistort_max_px for result in valid),
        "eef_undistort_mean_px": finite_mean(result.eef_undistort_mean_px for result in valid),
        "eef_undistort_max_px": finite_max(result.eef_undistort_max_px for result in valid),
    }


def final_verdict(
    sanity_checks: Sequence[CheckResult],
    aggregate: Dict[str, float],
    args: argparse.Namespace,
) -> Tuple[str, List[str]]:
    reasons: List[str] = []
    fatal_sanity = [check for check in sanity_checks if check.status == "FAIL"]
    if fatal_sanity:
        return "FAIL", [f"{check.name}: {check.message}" for check in fatal_sanity]

    valid_pair_count = int(aggregate.get("valid_pair_count", 0.0))
    mean_reproj = aggregate.get("total_reproj_mean_px", math.nan)
    max_reproj = aggregate.get("total_reproj_max_px", math.nan)
    mean_epi = aggregate.get("epipolar_mean_px", math.nan)
    positive_ratio = aggregate.get("positive_depth_ratio", math.nan)
    square_rel = aggregate.get("square_size_relative_error", math.nan)
    plane_error = aggregate.get("plane_error_m", math.nan)

    if valid_pair_count == 0:
        return "FAIL", ["no valid checkerboard pairs"]
    if math.isfinite(positive_ratio) and positive_ratio < 0.50:
        return "FAIL", [f"most triangulated points have negative front depth: ratio={positive_ratio:.3f}"]
    if math.isfinite(mean_reproj) and mean_reproj > args.max_mean_reproj_error_px * 4.0:
        return "FAIL", [f"mean reprojection error is excessive: {mean_reproj:.3f} px"]
    if math.isfinite(max_reproj) and max_reproj > args.max_max_reproj_error_px * 4.0:
        return "FAIL", [f"max reprojection error is excessive: {max_reproj:.3f} px"]

    if valid_pair_count < args.min_valid_pairs:
        reasons.append(f"valid pair count {valid_pair_count} < {args.min_valid_pairs}")
    if math.isfinite(mean_reproj) and mean_reproj > args.max_mean_reproj_error_px:
        reasons.append(f"mean reprojection error {mean_reproj:.3f} px > {args.max_mean_reproj_error_px:.3f} px")
    if math.isfinite(max_reproj) and max_reproj > args.max_max_reproj_error_px:
        reasons.append(f"max reprojection error {max_reproj:.3f} px > {args.max_max_reproj_error_px:.3f} px")
    if math.isfinite(mean_epi) and mean_epi > args.max_mean_epipolar_error_px:
        reasons.append(f"mean epipolar error {mean_epi:.3f} px > {args.max_mean_epipolar_error_px:.3f} px")
    if math.isfinite(positive_ratio) and positive_ratio < args.min_positive_depth_ratio:
        reasons.append(f"positive depth ratio {positive_ratio:.3f} < {args.min_positive_depth_ratio:.3f}")
    if math.isfinite(square_rel) and square_rel > args.max_square_size_relative_error:
        reasons.append(f"square size relative error {square_rel:.3f} > {args.max_square_size_relative_error:.3f}")
    if math.isfinite(plane_error) and plane_error > args.max_plane_error_m:
        reasons.append(f"plane error {plane_error:.6f} m > {args.max_plane_error_m:.6f} m")

    return ("WARN", reasons) if reasons else ("PASS", ["all validation thresholds passed"])


def metric_status(value: float, threshold: Optional[float], lower_is_better: bool = True) -> str:
    if not math.isfinite(value) or threshold is None:
        return ""
    if lower_is_better:
        return "OK" if value <= threshold else "WARN"
    return "OK" if value >= threshold else "WARN"


def write_reports(
    output_dir: Path,
    calib: CalibData,
    sanity_checks: Sequence[CheckResult],
    pair_warnings: Sequence[str],
    pair_results: Sequence[PairResult],
    aggregate: Dict[str, float],
    verdict: str,
    verdict_reasons: Sequence[str],
    args: argparse.Namespace,
    front_input_size: Optional[Tuple[int, int]],
    eef_input_size: Optional[Tuple[int, int]],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    report_txt = output_dir / "report.txt"
    with report_txt.open("w", encoding="utf-8") as handle:
        handle.write("Offline stereo calibration validation report\n")
        handle.write("==========================================\n\n")
        handle.write(f"calibration file: {calib.path}\n")
        handle.write(f"front input image size: {front_input_size or 'unknown'}\n")
        handle.write(f"eef input image size: {eef_input_size or 'unknown'}\n")
        handle.write(f"RMS error from npz: {calib.rms_error if calib.rms_error is not None else 'not stored'}\n")
        handle.write(f"valid pair count: {int(aggregate['valid_pair_count'])}\n")
        handle.write(f"failed pair count: {int(aggregate['failed_pair_count'])}\n\n")

        handle.write("Sanity checks\n")
        for check in sanity_checks:
            handle.write(f"{format_status(check)}\n")
        for warning in pair_warnings:
            handle.write(f"[WARN] image pairing: {warning}\n")

        handle.write("\nAggregate metrics\n")
        for name, value in aggregate.items():
            handle.write(f"{name}: {value}\n")

        handle.write("\nCalibration board distance from triangulated corners\n")
        handle.write(
            "front range mean/min/max m: "
            f"{aggregate['front_board_distance_mean_m']} / "
            f"{aggregate['front_board_distance_min_m']} / "
            f"{aggregate['front_board_distance_max_m']}\n"
        )
        handle.write(
            "front optical-axis depth Z mean/min/max m: "
            f"{aggregate['front_board_depth_z_mean_m']} / "
            f"{aggregate['front_board_depth_z_min_m']} / "
            f"{aggregate['front_board_depth_z_max_m']}\n"
        )
        handle.write(
            "eef range mean/min/max m: "
            f"{aggregate['eef_board_distance_mean_m']} / "
            f"{aggregate['eef_board_distance_min_m']} / "
            f"{aggregate['eef_board_distance_max_m']}\n"
        )
        handle.write(
            "eef optical-axis depth Z mean/min/max m: "
            f"{aggregate['eef_board_depth_z_mean_m']} / "
            f"{aggregate['eef_board_depth_z_min_m']} / "
            f"{aggregate['eef_board_depth_z_max_m']}\n"
        )

        front_center_shift, front_edge_shift = sample_undistort_shift(
            calib.image_size_front or front_input_size,
            calib.K_front,
            calib.D_front,
        )
        eef_center_shift, eef_edge_shift = sample_undistort_shift(
            calib.image_size_eef or eef_input_size,
            calib.K_eef,
            calib.D_eef,
        )
        handle.write("\nUndistort sample shift\n")
        handle.write(f"front center shift px: {front_center_shift}\n")
        handle.write(f"front edge max shift px: {front_edge_shift}\n")
        handle.write(f"eef center shift px: {eef_center_shift}\n")
        handle.write(f"eef edge max shift px: {eef_edge_shift}\n")

        handle.write("\nThresholds\n")
        handle.write(f"min valid pairs: {args.min_valid_pairs}\n")
        handle.write(f"max mean reprojection error px: {args.max_mean_reproj_error_px}\n")
        handle.write(f"max max reprojection error px: {args.max_max_reproj_error_px}\n")
        handle.write(f"max mean epipolar error px: {args.max_mean_epipolar_error_px}\n")
        handle.write(f"max plane error m: {args.max_plane_error_m}\n")
        handle.write(f"max square size relative error: {args.max_square_size_relative_error}\n")
        handle.write(f"min positive depth ratio: {args.min_positive_depth_ratio}\n")

        handle.write(f"\nfinal verdict: {verdict}\n")
        for reason in verdict_reasons:
            handle.write(f"- {reason}\n")

        failed_pairs = [result for result in pair_results if result.failure_reason]
        if failed_pairs:
            handle.write("\nFailed pair list\n")
            for result in failed_pairs:
                handle.write(
                    f"{result.pair_index}: front={result.front_image}, eef={result.eef_image}, reason={result.failure_reason}\n"
                )

    report_csv = output_dir / "report.csv"
    with report_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["metric", "value", "status"])
        writer.writerow(["calibration_file", str(calib.path), ""])
        writer.writerow(["rms_error_from_npz", calib.rms_error if calib.rms_error is not None else "", ""])
        writer.writerow(["valid_pair_count", int(aggregate["valid_pair_count"]), metric_status(aggregate["valid_pair_count"], args.min_valid_pairs, False)])
        writer.writerow(["failed_pair_count", int(aggregate["failed_pair_count"]), ""])
        writer.writerow(["total_reproj_mean_px", aggregate["total_reproj_mean_px"], metric_status(aggregate["total_reproj_mean_px"], args.max_mean_reproj_error_px)])
        writer.writerow(["total_reproj_max_px", aggregate["total_reproj_max_px"], metric_status(aggregate["total_reproj_max_px"], args.max_max_reproj_error_px)])
        writer.writerow(["epipolar_mean_px", aggregate["epipolar_mean_px"], metric_status(aggregate["epipolar_mean_px"], args.max_mean_epipolar_error_px)])
        writer.writerow(["epipolar_max_px", aggregate["epipolar_max_px"], ""])
        writer.writerow(["positive_depth_ratio", aggregate["positive_depth_ratio"], metric_status(aggregate["positive_depth_ratio"], args.min_positive_depth_ratio, False)])
        writer.writerow(["front_board_distance_mean_m", aggregate["front_board_distance_mean_m"], ""])
        writer.writerow(["front_board_distance_min_m", aggregate["front_board_distance_min_m"], ""])
        writer.writerow(["front_board_distance_max_m", aggregate["front_board_distance_max_m"], ""])
        writer.writerow(["front_board_depth_z_mean_m", aggregate["front_board_depth_z_mean_m"], ""])
        writer.writerow(["front_board_depth_z_min_m", aggregate["front_board_depth_z_min_m"], ""])
        writer.writerow(["front_board_depth_z_max_m", aggregate["front_board_depth_z_max_m"], ""])
        writer.writerow(["eef_board_distance_mean_m", aggregate["eef_board_distance_mean_m"], ""])
        writer.writerow(["eef_board_distance_min_m", aggregate["eef_board_distance_min_m"], ""])
        writer.writerow(["eef_board_distance_max_m", aggregate["eef_board_distance_max_m"], ""])
        writer.writerow(["eef_board_depth_z_mean_m", aggregate["eef_board_depth_z_mean_m"], ""])
        writer.writerow(["eef_board_depth_z_min_m", aggregate["eef_board_depth_z_min_m"], ""])
        writer.writerow(["eef_board_depth_z_max_m", aggregate["eef_board_depth_z_max_m"], ""])
        writer.writerow(["mean_square_size_m", aggregate["mean_square_size_m"], ""])
        writer.writerow(["square_size_relative_error", aggregate["square_size_relative_error"], metric_status(aggregate["square_size_relative_error"], args.max_square_size_relative_error)])
        writer.writerow(["plane_error_m", aggregate["plane_error_m"], metric_status(aggregate["plane_error_m"], args.max_plane_error_m)])
        writer.writerow(["front_undistort_mean_px", aggregate["front_undistort_mean_px"], ""])
        writer.writerow(["front_undistort_max_px", aggregate["front_undistort_max_px"], ""])
        writer.writerow(["eef_undistort_mean_px", aggregate["eef_undistort_mean_px"], ""])
        writer.writerow(["eef_undistort_max_px", aggregate["eef_undistort_max_px"], ""])
        writer.writerow(["final_verdict", verdict, verdict])

    per_pair_csv = output_dir / "per_pair_errors.csv"
    with per_pair_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "pair_index",
                "front_image",
                "eef_image",
                "front_detected",
                "eef_detected",
                "front_reproj_mean_px",
                "front_reproj_max_px",
                "eef_reproj_mean_px",
                "eef_reproj_max_px",
                "total_reproj_mean_px",
                "total_reproj_max_px",
                "epipolar_mean_px",
                "epipolar_max_px",
                "triangulated_positive_depth_ratio",
                "eef_positive_depth_ratio",
                "front_board_center_x_m",
                "front_board_center_y_m",
                "front_board_center_z_m",
                "front_board_distance_m",
                "front_board_depth_z_m",
                "eef_board_center_x_m",
                "eef_board_center_y_m",
                "eef_board_center_z_m",
                "eef_board_distance_m",
                "eef_board_depth_z_m",
                "mean_square_size_m",
                "square_size_relative_error",
                "plane_error_m",
                "front_undistort_mean_px",
                "front_undistort_max_px",
                "eef_undistort_mean_px",
                "eef_undistort_max_px",
                "failure_reason",
            ],
        )
        writer.writeheader()
        for result in pair_results:
            writer.writerow(result.__dict__)


def print_summary(
    sanity_checks: Sequence[CheckResult],
    pair_warnings: Sequence[str],
    aggregate: Dict[str, float],
    verdict: str,
    verdict_reasons: Sequence[str],
) -> None:
    for check in sanity_checks:
        print(format_status(check))
    for warning in pair_warnings:
        print(f"[WARN] image pairing: {warning}")
    print("")
    print(f"valid pairs: {int(aggregate['valid_pair_count'])} / {int(aggregate['pair_count'])}")
    print(
        "front reproj mean/max: "
        f"{aggregate['front_reproj_mean_px']:.3f} / {aggregate['front_reproj_max_px']:.3f} px"
    )
    print(
        "eef reproj mean/max: "
        f"{aggregate['eef_reproj_mean_px']:.3f} / {aggregate['eef_reproj_max_px']:.3f} px"
    )
    print(
        "total reproj mean/max: "
        f"{aggregate['total_reproj_mean_px']:.3f} / {aggregate['total_reproj_max_px']:.3f} px"
    )
    print(
        "epipolar mean/max: "
        f"{aggregate['epipolar_mean_px']:.3f} / {aggregate['epipolar_max_px']:.3f} px"
    )
    print(f"triangulated positive depth ratio: {100.0 * aggregate['positive_depth_ratio']:.1f}%")
    print(
        "front board distance range mean/min/max: "
        f"{aggregate['front_board_distance_mean_m']:.4f} / "
        f"{aggregate['front_board_distance_min_m']:.4f} / "
        f"{aggregate['front_board_distance_max_m']:.4f} m"
    )
    print(
        "front board optical-axis depth Z mean/min/max: "
        f"{aggregate['front_board_depth_z_mean_m']:.4f} / "
        f"{aggregate['front_board_depth_z_min_m']:.4f} / "
        f"{aggregate['front_board_depth_z_max_m']:.4f} m"
    )
    print(
        "eef board distance range mean/min/max: "
        f"{aggregate['eef_board_distance_mean_m']:.4f} / "
        f"{aggregate['eef_board_distance_min_m']:.4f} / "
        f"{aggregate['eef_board_distance_max_m']:.4f} m"
    )
    print(
        "eef board optical-axis depth Z mean/min/max: "
        f"{aggregate['eef_board_depth_z_mean_m']:.4f} / "
        f"{aggregate['eef_board_depth_z_min_m']:.4f} / "
        f"{aggregate['eef_board_depth_z_max_m']:.4f} m"
    )
    print(f"mean adjacent corner distance: {aggregate['mean_square_size_m']:.6f} m")
    print(f"mean plane fitting error: {aggregate['plane_error_m']:.6f} m")
    print(
        "front mean/max undistort shift: "
        f"{aggregate['front_undistort_mean_px']:.3f} / {aggregate['front_undistort_max_px']:.3f} px"
    )
    print(
        "eef mean/max undistort shift: "
        f"{aggregate['eef_undistort_mean_px']:.3f} / {aggregate['eef_undistort_max_px']:.3f} px"
    )
    print("")
    print(f"final verdict: {verdict}")
    for reason in verdict_reasons:
        print(f"- {reason}")


def parse_video_source(source: str):
    stripped = source.strip()
    if stripped.isdigit():
        return int(stripped)
    return stripped


def open_video_capture(source: str, label: str) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(parse_video_source(source))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open {label} video source: {source}")
    return capture


def draw_stream_overlay(
    image: np.ndarray,
    corners: Optional[np.ndarray],
    pattern_size: Tuple[int, int],
    found: bool,
    label: str,
    result: PairResult,
    camera_kind: str,
) -> np.ndarray:
    overlay = image.copy()
    if corners is not None:
        cv2.drawChessboardCorners(overlay, pattern_size, corners.reshape(-1, 1, 2).astype(np.float32), found)

    status_color = (0, 180, 0) if found and not result.failure_reason else (0, 0, 255)
    lines = [label, f"detected: {'yes' if found else 'no'}"]
    if result.failure_reason:
        lines.append(result.failure_reason)
    else:
        if camera_kind == "front":
            lines.append(f"board range: {result.front_board_distance_m:.4f} m")
            lines.append(f"board z: {result.front_board_depth_z_m:.4f} m")
        else:
            lines.append(f"board range: {result.eef_board_distance_m:.4f} m")
            lines.append(f"board z: {result.eef_board_depth_z_m:.4f} m")
        lines.append(f"reproj mean: {result.total_reproj_mean_px:.3f} px")
        lines.append(f"epipolar mean: {result.epipolar_mean_px:.3f} px")
        lines.append(f"square: {result.mean_square_size_m:.4f} m")

    text_height = 22 * len(lines) + 10
    cv2.rectangle(overlay, (0, 0), (min(overlay.shape[1], 470), text_height), (0, 0, 0), -1)
    for index, line in enumerate(lines):
        cv2.putText(
            overlay,
            line,
            (10, 24 + 22 * index),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            status_color if index == 1 else (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return overlay


def stack_overlays(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    target_height = min(left.shape[0], right.shape[0])
    if left.shape[0] != target_height:
        left_width = int(round(left.shape[1] * target_height / left.shape[0]))
        left = cv2.resize(left, (left_width, target_height))
    if right.shape[0] != target_height:
        right_width = int(round(right.shape[1] * target_height / right.shape[0]))
        right = cv2.resize(right, (right_width, target_height))
    return np.hstack((left, right))


def print_stream_result(frame_index: int, result: PairResult) -> None:
    if result.failure_reason:
        print(f"[frame {frame_index}] {result.failure_reason}")
        return
    print(
        f"[frame {frame_index}] "
        f"front_z={result.front_board_depth_z_m:.4f}m "
        f"front_range={result.front_board_distance_m:.4f}m "
        f"eef_z={result.eef_board_depth_z_m:.4f}m "
        f"eef_range={result.eef_board_distance_m:.4f}m "
        f"reproj_mean={result.total_reproj_mean_px:.3f}px "
        f"epi_mean={result.epipolar_mean_px:.3f}px "
        f"square={result.mean_square_size_m:.4f}m"
    )


def run_stream_validation(calib: CalibData, args: argparse.Namespace) -> int:
    if not args.front_source or not args.eef_source:
        print("--front-source and --eef-source are required with --stream-validation", file=sys.stderr)
        return 2

    frame_stride = max(1, args.frame_stride)
    output_dir = Path(args.output_dir)
    overlay_dir = output_dir / "stream_overlays"
    pattern_size = (args.board_cols, args.board_rows)

    front_capture = open_video_capture(args.front_source, "front")
    eef_capture = open_video_capture(args.eef_source, "eef")
    pair_results: List[PairResult] = []
    pair_warnings = [
        f"stream front source: {args.front_source}",
        f"stream eef source: {args.eef_source}",
    ]
    sanity_checks: List[CheckResult] = []
    front_input_size: Optional[Tuple[int, int]] = None
    eef_input_size: Optional[Tuple[int, int]] = None

    frame_index = 0
    processed_count = 0
    try:
        while True:
            front_ok, front_frame = front_capture.read()
            eef_ok, eef_frame = eef_capture.read()
            if not front_ok or not eef_ok:
                if not front_ok:
                    pair_warnings.append(f"front stream ended at frame {frame_index}")
                if not eef_ok:
                    pair_warnings.append(f"eef stream ended at frame {frame_index}")
                break

            if frame_index % frame_stride != 0:
                frame_index += 1
                continue

            if front_input_size is None:
                front_input_size = (front_frame.shape[1], front_frame.shape[0])
                eef_input_size = (eef_frame.shape[1], eef_frame.shape[0])
                sanity_checks = validate_shape_and_pose(calib, args, front_input_size, eef_input_size)
                for check in sanity_checks:
                    print(format_status(check))
                if any(check.status == "FAIL" for check in sanity_checks):
                    pair_warnings.append("stream validation continued despite calibration sanity failure")

            front_found, front_corners = detect_checkerboard_in_image(front_frame, pattern_size)
            eef_found, eef_corners = detect_checkerboard_in_image(eef_frame, pattern_size)
            result = compute_pair_metrics(
                pair_index=frame_index,
                front_label=f"front_frame_{frame_index}",
                eef_label=f"eef_frame_{frame_index}",
                front_found=front_found,
                front_corners=front_corners,
                eef_found=eef_found,
                eef_corners=eef_corners,
                calib=calib,
                board_rows=args.board_rows,
                board_cols=args.board_cols,
                square_size=args.square_size,
            )
            pair_results.append(result)

            if args.stream_log_every > 0 and processed_count % args.stream_log_every == 0:
                print_stream_result(frame_index, result)

            should_save_overlay = args.stream_overlay_every > 0 and processed_count % args.stream_overlay_every == 0
            if should_save_overlay or args.display:
                front_overlay = draw_stream_overlay(
                    front_frame,
                    front_corners,
                    pattern_size,
                    front_found,
                    f"front frame {frame_index}",
                    result,
                    "front",
                )
                eef_overlay = draw_stream_overlay(
                    eef_frame,
                    eef_corners,
                    pattern_size,
                    eef_found,
                    f"eef frame {frame_index}",
                    result,
                    "eef",
                )
                if should_save_overlay:
                    overlay_dir.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(overlay_dir / f"front_{frame_index:06d}.png"), front_overlay)
                    cv2.imwrite(str(overlay_dir / f"eef_{frame_index:06d}.png"), eef_overlay)
                    cv2.imwrite(
                        str(overlay_dir / f"combined_{frame_index:06d}.png"),
                        stack_overlays(front_overlay, eef_overlay),
                    )
                if args.display:
                    cv2.imshow("stereo calibration stream validation", stack_overlays(front_overlay, eef_overlay))
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        pair_warnings.append(f"display stopped by user at frame {frame_index}")
                        break

            processed_count += 1
            frame_index += 1
            if args.max_frames > 0 and processed_count >= args.max_frames:
                pair_warnings.append(f"stopped after max processed frames: {args.max_frames}")
                break
    finally:
        front_capture.release()
        eef_capture.release()
        if args.display:
            cv2.destroyAllWindows()

    if not pair_results:
        print("[FAIL] no stream frames were processed", file=sys.stderr)
        return 1
    if not sanity_checks:
        sanity_checks = validate_shape_and_pose(calib, args, front_input_size, eef_input_size)

    aggregate = aggregate_results(pair_results)
    verdict, verdict_reasons = final_verdict(sanity_checks, aggregate, args)
    write_reports(
        output_dir,
        calib,
        sanity_checks,
        pair_warnings,
        pair_results,
        aggregate,
        verdict,
        verdict_reasons,
        args,
        front_input_size,
        eef_input_size,
    )
    print_summary(sanity_checks, pair_warnings, aggregate, verdict, verdict_reasons)
    print(f"\nstream report written to: {output_dir / 'report.txt'}")
    print(f"stream per-frame CSV written to: {output_dir / 'per_pair_errors.csv'}")
    if args.stream_overlay_every > 0:
        print(f"stream overlays written to: {overlay_dir}")
    return 1 if verdict == "FAIL" else 0


def run_synthetic_test(calib: CalibData, args: argparse.Namespace) -> int:
    checks = validate_shape_and_pose(calib, args)
    for check in checks:
        print(format_status(check))

    if any(check.status == "FAIL" for check in checks):
        print("synthetic test skipped because calibration sanity checks failed")
        return 1

    xs = np.linspace(-0.08, 0.08, 5)
    ys = np.linspace(-0.06, 0.06, 4)
    zs = np.linspace(0.35, 0.75, 3)
    points_front = np.array([[x, y, z] for z in zs for y in ys for x in xs], dtype=np.float64)
    points_eef = transform_front_to_eef(points_front, calib.R_front_to_eef, calib.T_front_to_eef)
    visible = points_front[:, 2] > 0.0
    visible &= points_eef[:, 2] > 0.0
    if not np.any(visible):
        print("[FAIL] synthetic triangulation: no generated points are in front of both cameras")
        return 1

    points_front = points_front[visible]
    points_eef = points_eef[visible]
    front_px = project_points(points_front, calib.K_front, calib.D_front)
    eef_px = project_points(points_eef, calib.K_eef, calib.D_eef)
    if calib.image_size_front and calib.image_size_eef:
        fw, fh = calib.image_size_front
        ew, eh = calib.image_size_eef
        in_image = (
            (front_px[:, 0] >= 0.0)
            & (front_px[:, 0] < fw)
            & (front_px[:, 1] >= 0.0)
            & (front_px[:, 1] < fh)
            & (eef_px[:, 0] >= 0.0)
            & (eef_px[:, 0] < ew)
            & (eef_px[:, 1] >= 0.0)
            & (eef_px[:, 1] < eh)
        )
        points_front = points_front[in_image]
        front_px = front_px[in_image]
        eef_px = eef_px[in_image]
        if points_front.size == 0:
            print("[FAIL] synthetic triangulation: no generated points project inside both images")
            return 1
    front_norm = undistort_to_normalized(front_px, calib.K_front, calib.D_front)
    eef_norm = undistort_to_normalized(eef_px, calib.K_eef, calib.D_eef)
    reconstructed = triangulate_normalized(
        front_norm,
        eef_norm,
        calib.R_front_to_eef,
        calib.T_front_to_eef,
    )
    errors = np.linalg.norm(reconstructed - points_front, axis=1)
    mean_error = float(np.mean(errors))
    max_error = float(np.max(errors))
    print(f"synthetic triangulation error mean: {mean_error:.6f} m")
    print(f"synthetic triangulation error max: {max_error:.6f} m")
    if max_error > args.max_synthetic_error_m:
        print(f"[FAIL] synthetic max error exceeds {args.max_synthetic_error_m:.6f} m")
        return 1
    print("[OK] synthetic triangulation")
    return 0


def run_manual_points(calib: CalibData, args: argparse.Namespace) -> int:
    required = {
        "front-u": args.front_u,
        "front-v": args.front_v,
        "eef-u": args.eef_u,
        "eef-v": args.eef_v,
    }
    missing = [name for name, value in required.items() if value is None]
    if missing:
        print(f"Missing manual point arguments: {', '.join(missing)}", file=sys.stderr)
        return 2

    checks = validate_shape_and_pose(calib, args)
    for check in checks:
        print(format_status(check))
    if any(check.status == "FAIL" for check in checks):
        print("manual point triangulation skipped because calibration sanity checks failed")
        return 1

    front_raw = np.array([[args.front_u, args.front_v]], dtype=np.float64)
    eef_raw = np.array([[args.eef_u, args.eef_v]], dtype=np.float64)
    front_norm = undistort_to_normalized(front_raw, calib.K_front, calib.D_front)
    eef_norm = undistort_to_normalized(eef_raw, calib.K_eef, calib.D_eef)
    point_front = triangulate_normalized(
        front_norm,
        eef_norm,
        calib.R_front_to_eef,
        calib.T_front_to_eef,
    )[0]
    point_eef = transform_front_to_eef(point_front.reshape(1, 3), calib.R_front_to_eef, calib.T_front_to_eef)[0]
    finite = bool(np.all(np.isfinite(point_front)))
    print(f"front raw point: {front_raw[0].tolist()}")
    print(f"eef raw point: {eef_raw[0].tolist()}")
    print(f"front undistorted normalized: {front_norm[0].tolist()}")
    print(f"eef undistorted normalized: {eef_norm[0].tolist()}")
    print(f"triangulated point in front frame: {point_front.tolist()}")
    print(f"triangulated point in eef frame: {point_eef.tolist()}")
    print(f"depth validity: finite={finite}, front_z_positive={point_front[2] > 0.0}, eef_z_positive={point_eef[2] > 0.0}")
    return 0 if finite and point_front[2] > 0.0 else 1


def run_image_validation(calib: CalibData, args: argparse.Namespace) -> int:
    if not args.front_dir or not args.eef_dir:
        print("--front-dir and --eef-dir are required unless using --synthetic-test-only or --manual-points", file=sys.stderr)
        return 2

    front_dir = Path(args.front_dir)
    eef_dir = Path(args.eef_dir)
    output_dir = Path(args.output_dir)
    overlay_dir = output_dir / "overlays"
    pairs, pair_warnings = pair_images(front_dir, eef_dir)
    front_input_size = image_size(pairs[0][0]) if pairs else None
    eef_input_size = image_size(pairs[0][1]) if pairs else None
    sanity_checks = validate_shape_and_pose(calib, args, front_input_size, eef_input_size)

    pattern_size = (args.board_cols, args.board_rows)
    pair_results = [
        process_pair(
            pair_index=index,
            front_path=front_path,
            eef_path=eef_path,
            calib=calib,
            pattern_size=pattern_size,
            board_rows=args.board_rows,
            board_cols=args.board_cols,
            square_size=args.square_size,
            overlay_dir=overlay_dir,
        )
        for index, (front_path, eef_path) in enumerate(pairs)
    ]

    aggregate = aggregate_results(pair_results)
    verdict, verdict_reasons = final_verdict(sanity_checks, aggregate, args)
    write_reports(
        output_dir,
        calib,
        sanity_checks,
        pair_warnings,
        pair_results,
        aggregate,
        verdict,
        verdict_reasons,
        args,
        front_input_size,
        eef_input_size,
    )
    print_summary(sanity_checks, pair_warnings, aggregate, verdict, verdict_reasons)
    print(f"\nreport written to: {output_dir / 'report.txt'}")
    print(f"per-pair CSV written to: {output_dir / 'per_pair_errors.csv'}")
    print(f"corner overlays written to: {overlay_dir}")
    return 1 if verdict == "FAIL" else 0


def main() -> int:
    args = parse_args()
    args.calib = args.calib_opt or args.calib
    if not args.calib:
        print("[FAIL] calibration npz path is required", file=sys.stderr)
        return 2
    try:
        calib = load_calibration(Path(args.calib))
        if args.synthetic_test_only:
            return run_synthetic_test(calib, args)
        if args.manual_points:
            return run_manual_points(calib, args)
        if args.stream_validation:
            return run_stream_validation(calib, args)
        return run_image_validation(calib, args)
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
