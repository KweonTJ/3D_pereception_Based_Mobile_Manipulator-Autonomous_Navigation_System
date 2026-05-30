#!/usr/bin/env python3
"""Calibrate Front Astra RGB + EEF USB camera as a stereo pair."""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def parse_args():
    parser = argparse.ArgumentParser(description="Create front/eef stereo calibration npz.")
    parser.add_argument("--front-dir", required=True)
    parser.add_argument("--eef-dir", required=True)
    parser.add_argument("--board-type", choices=("checkerboard", "charuco"), default="checkerboard")
    parser.add_argument("--board-cols", type=int, default=9, help="Checkerboard inner corner columns")
    parser.add_argument("--board-rows", type=int, default=11, help="Checkerboard inner corner rows")
    parser.add_argument("--square-size", type=float, default=0.020, help="Square size in meters")
    parser.add_argument("--output", required=True)
    parser.add_argument("--overlay-dir", default="")
    parser.add_argument("--front-intrinsics-json", default="")
    parser.add_argument("--eef-intrinsics-yaml", default="")
    parser.add_argument("--fix-intrinsics", action="store_true")
    parser.add_argument("--camera1-name", default="front_astra_rgb")
    parser.add_argument("--camera2-name", default="eef_usb_camera")
    parser.add_argument("--camera1-frame", default="camera_color_optical_frame")
    parser.add_argument("--camera2-frame", default="eef_usb_camera_optical_frame")
    parser.add_argument("--notes", default="")
    parser.add_argument("--run-validation", action="store_true")
    parser.add_argument("--validation-output-dir", default="calibration/stereo/validation")
    parser.add_argument("--display", action="store_true", help="Show detected front/eef corner overlays")
    parser.add_argument(
        "--display-wait-ms",
        type=int,
        default=1,
        help="OpenCV wait time for each displayed pair; 0 waits for a key",
    )

    parser.add_argument("--charuco-squares-x", type=int, default=10)
    parser.add_argument("--charuco-squares-y", type=int, default=12)
    parser.add_argument("--charuco-marker-size", type=float, default=0.014)
    parser.add_argument("--aruco-dict", default="DICT_4X4_50")
    return parser.parse_args()


def list_images(directory: Path) -> List[Path]:
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def pair_images(front_dir: Path, eef_dir: Path) -> Tuple[List[Tuple[Path, Path]], List[str]]:
    front_images = list_images(front_dir)
    eef_images = list_images(eef_dir)
    front_by_stem = {path.stem.replace("front_", ""): path for path in front_images}
    eef_by_stem = {path.stem.replace("eef_", ""): path for path in eef_images}
    common = sorted(set(front_by_stem) & set(eef_by_stem))
    warnings: List[str] = []
    if common:
        return [(front_by_stem[key], eef_by_stem[key]) for key in common], warnings
    count = min(len(front_images), len(eef_images))
    warnings.append("No matching stems; paired images by sorted order")
    return list(zip(front_images[:count], eef_images[:count])), warnings


def checkerboard_object_points(cols: int, rows: int, square_size: float) -> np.ndarray:
    obj = np.zeros((rows * cols, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj[:, :2] = grid * float(square_size)
    return obj


def detect_checkerboard(image: np.ndarray, cols: int, rows: int) -> Tuple[bool, Optional[np.ndarray]]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    pattern = (cols, rows)
    found = False
    corners = None
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, pattern, flags)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    if not found or corners is None:
        return False, None
    return True, corners.astype(np.float32)


def aruco_dictionary(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV aruco module is not available")
    if not hasattr(cv2.aruco, name):
        raise RuntimeError(f"Unknown aruco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_charuco_board(args):
    dictionary = aruco_dictionary(args.aruco_dict)
    if hasattr(cv2.aruco, "CharucoBoard"):
        return cv2.aruco.CharucoBoard(
            (args.charuco_squares_x, args.charuco_squares_y),
            args.square_size,
            args.charuco_marker_size,
            dictionary,
        ), dictionary
    return cv2.aruco.CharucoBoard_create(
        args.charuco_squares_x,
        args.charuco_squares_y,
        args.square_size,
        args.charuco_marker_size,
        dictionary,
    ), dictionary


def board_chessboard_corners(board) -> np.ndarray:
    if hasattr(board, "getChessboardCorners"):
        return np.asarray(board.getChessboardCorners(), dtype=np.float32)
    return np.asarray(board.chessboardCorners, dtype=np.float32)


def detect_charuco(image: np.ndarray, board, dictionary) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if marker_ids is None or len(marker_ids) == 0:
        return False, None, None
    count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
    )
    if charuco_ids is None or charuco_corners is None or count < 6:
        return False, None, None
    return True, charuco_corners.astype(np.float32), charuco_ids.reshape(-1).astype(np.int32)


def matching_charuco_points(
    front_corners: np.ndarray,
    front_ids: np.ndarray,
    eef_corners: np.ndarray,
    eef_ids: np.ndarray,
    charuco_object_points: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    front_map = {int(idx): corner for idx, corner in zip(front_ids, front_corners)}
    eef_map = {int(idx): corner for idx, corner in zip(eef_ids, eef_corners)}
    common_ids = sorted(set(front_map) & set(eef_map))
    object_points = []
    front_points = []
    eef_points = []
    for idx in common_ids:
        if idx < 0 or idx >= len(charuco_object_points):
            continue
        object_points.append(charuco_object_points[idx])
        front_points.append(front_map[idx])
        eef_points.append(eef_map[idx])
    return (
        np.asarray(object_points, dtype=np.float32).reshape(-1, 3),
        np.asarray(front_points, dtype=np.float32).reshape(-1, 1, 2),
        np.asarray(eef_points, dtype=np.float32).reshape(-1, 1, 2),
    )


def load_front_json(path: str):
    if not path:
        return None
    with Path(path).open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    info = payload.get("camera_info", payload)
    K = np.asarray(info["k"], dtype=np.float64).reshape(3, 3)
    D = np.asarray(info["d"], dtype=np.float64).reshape(-1, 1)
    size = (int(info["width"]), int(info["height"]))
    return K, D, size


def load_ros_camera_yaml(path: str):
    if not path:
        return None
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if fs.isOpened():
        K = fs.getNode("camera_matrix").mat()
        D = fs.getNode("distortion_coefficients").mat()
        width_node = fs.getNode("image_width")
        height_node = fs.getNode("image_height")
        width = int(width_node.real()) if not width_node.empty() else 0
        height = int(height_node.real()) if not height_node.empty() else 0
        fs.release()
        if K is not None and D is not None:
            return K.astype(np.float64), D.reshape(-1, 1).astype(np.float64), (width, height)

    import yaml

    with Path(path).open("r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle)
    K = np.asarray(payload["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.asarray(payload["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1, 1)
    return K, D, (int(payload["image_width"]), int(payload["image_height"]))


def draw_corner_overlay(
    image: np.ndarray,
    pattern_size: Tuple[int, int],
    corners: Optional[np.ndarray],
    found: bool,
    label: str,
) -> np.ndarray:
    overlay = image.copy()
    if corners is not None:
        cv2.drawChessboardCorners(overlay, pattern_size, corners, found)
    else:
        cv2.putText(overlay, "not detected", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    status = "detected" if found else "not detected"
    cv2.rectangle(overlay, (0, 0), (min(overlay.shape[1], 520), 34), (0, 0, 0), -1)
    cv2.putText(
        overlay,
        f"{label}: {status}",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0) if found else (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    return overlay


def stack_overlays(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    target_height = min(left.shape[0], right.shape[0], 720)
    if left.shape[0] != target_height:
        left_width = int(round(left.shape[1] * target_height / left.shape[0]))
        left = cv2.resize(left, (left_width, target_height))
    if right.shape[0] != target_height:
        right_width = int(round(right.shape[1] * target_height / right.shape[0]))
        right = cv2.resize(right, (right_width, target_height))
    return np.hstack((left, right))


def save_overlay(path: Path, image: np.ndarray, pattern_size: Tuple[int, int], corners: Optional[np.ndarray], found: bool):
    path.parent.mkdir(parents=True, exist_ok=True)
    overlay = draw_corner_overlay(image, pattern_size, corners, found, path.stem)
    cv2.imwrite(str(path), overlay)


def run_validation(output: Path, args):
    import subprocess
    import sys

    cmd = [
        sys.executable,
        str(Path(__file__).with_name("validate_stereo_calib_offline.py")),
        "--calib",
        str(output),
        "--front-dir",
        args.front_dir,
        "--eef-dir",
        args.eef_dir,
        "--board-cols",
        str(args.board_cols),
        "--board-rows",
        str(args.board_rows),
        "--square-size",
        str(args.square_size),
        "--output-dir",
        args.validation_output_dir,
    ]
    subprocess.run(cmd, check=False)


def main():
    args = parse_args()
    front_dir = Path(args.front_dir)
    eef_dir = Path(args.eef_dir)
    output = Path(args.output)
    pairs, warnings = pair_images(front_dir, eef_dir)
    for warning in warnings:
        print(f"[WARN] {warning}")
    if not pairs:
        raise SystemExit("No image pairs found")

    object_points: List[np.ndarray] = []
    front_points: List[np.ndarray] = []
    eef_points: List[np.ndarray] = []
    used_indices: List[int] = []
    failed_pairs: List[str] = []
    image_size_front = None
    image_size_eef = None

    checker_obj = checkerboard_object_points(args.board_cols, args.board_rows, args.square_size)
    charuco_board = None
    charuco_dictionary_value = None
    charuco_obj = None
    if args.board_type == "charuco":
        charuco_board, charuco_dictionary_value = make_charuco_board(args)
        charuco_obj = board_chessboard_corners(charuco_board)

    overlay_dir = Path(args.overlay_dir) if args.overlay_dir else None
    pattern_size = (args.board_cols, args.board_rows)

    for index, (front_path, eef_path) in enumerate(pairs):
        front_image = cv2.imread(str(front_path), cv2.IMREAD_COLOR)
        eef_image = cv2.imread(str(eef_path), cv2.IMREAD_COLOR)
        if front_image is None or eef_image is None:
            failed_pairs.append(f"{index}: unreadable image")
            continue
        image_size_front = (front_image.shape[1], front_image.shape[0])
        image_size_eef = (eef_image.shape[1], eef_image.shape[0])

        if args.board_type == "checkerboard":
            front_found, front_corners = detect_checkerboard(front_image, args.board_cols, args.board_rows)
            eef_found, eef_corners = detect_checkerboard(eef_image, args.board_cols, args.board_rows)
            if overlay_dir:
                save_overlay(overlay_dir / f"front_{index:03d}_corners.png", front_image, pattern_size, front_corners, front_found)
                save_overlay(overlay_dir / f"eef_{index:03d}_corners.png", eef_image, pattern_size, eef_corners, eef_found)
            if not front_found or not eef_found or front_corners is None or eef_corners is None:
                failed_pairs.append(f"{index}: checkerboard detection failed front={front_found} eef={eef_found}")
                continue
            object_points.append(checker_obj.copy())
            front_points.append(front_corners)
            eef_points.append(eef_corners)
        else:
            front_found, front_corners, front_ids = detect_charuco(front_image, charuco_board, charuco_dictionary_value)
            eef_found, eef_corners, eef_ids = detect_charuco(eef_image, charuco_board, charuco_dictionary_value)
            if not front_found or not eef_found:
                failed_pairs.append(f"{index}: charuco detection failed front={front_found} eef={eef_found}")
                continue
            obj, fpts, epts = matching_charuco_points(front_corners, front_ids, eef_corners, eef_ids, charuco_obj)
            if len(obj) < 6:
                failed_pairs.append(f"{index}: too few shared Charuco corners ({len(obj)})")
                continue
            object_points.append(obj)
            front_points.append(fpts)
            eef_points.append(epts)

        used_indices.append(index)

    if len(object_points) < 3:
        print("Failed pairs:")
        for failure in failed_pairs:
            print(f"  - {failure}")
        raise SystemExit(f"Need at least 3 valid pairs, got {len(object_points)}")

    front_init = load_front_json(args.front_intrinsics_json)
    eef_init = load_ros_camera_yaml(args.eef_intrinsics_yaml)

    if args.fix_intrinsics:
        if front_init is None or eef_init is None:
            raise SystemExit("--fix-intrinsics requires --front-intrinsics-json and --eef-intrinsics-yaml")
        K_front, D_front, _ = front_init
        K_eef, D_eef, _ = eef_init
    else:
        K_front = front_init[0].copy() if front_init else None
        D_front = front_init[1].copy() if front_init else None
        K_eef = eef_init[0].copy() if eef_init else None
        D_eef = eef_init[1].copy() if eef_init else None

        flags_front = cv2.CALIB_USE_INTRINSIC_GUESS if K_front is not None else 0
        flags_eef = cv2.CALIB_USE_INTRINSIC_GUESS if K_eef is not None else 0
        _, K_front, D_front, _, _ = cv2.calibrateCamera(
            object_points,
            front_points,
            image_size_front,
            K_front,
            D_front,
            flags=flags_front,
        )
        _, K_eef, D_eef, _, _ = cv2.calibrateCamera(
            object_points,
            eef_points,
            image_size_eef,
            K_eef,
            D_eef,
            flags=flags_eef,
        )

    flags = cv2.CALIB_FIX_INTRINSIC if args.fix_intrinsics else cv2.CALIB_USE_INTRINSIC_GUESS
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    per_view_errors = np.empty((0,), dtype=np.float64)
    try:
        result = cv2.stereoCalibrateExtended(
            object_points,
            front_points,
            eef_points,
            K_front,
            D_front,
            K_eef,
            D_eef,
            image_size_front,
            np.eye(3, dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            criteria=criteria,
            flags=flags,
        )
        rms, K_front, D_front, K_eef, D_eef, R, T, E, F, per_view_errors = result
    except AttributeError:
        rms, K_front, D_front, K_eef, D_eef, R, T, E, F = cv2.stereoCalibrate(
            object_points,
            front_points,
            eef_points,
            K_front,
            D_front,
            K_eef,
            D_eef,
            image_size_front,
            criteria=criteria,
            flags=flags,
        )

    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K_front,
        D_front,
        K_eef,
        D_eef,
        image_size_front,
        R,
        T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0.0,
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        str(output),
        K_front=K_front,
        D_front=D_front.reshape(-1),
        K_eef=K_eef,
        D_eef=D_eef.reshape(-1),
        R_front_to_eef=R,
        T_front_to_eef=T.reshape(3),
        image_size_front=np.asarray(image_size_front, dtype=np.int32),
        image_size_eef=np.asarray(image_size_eef, dtype=np.int32),
        rms_error=np.asarray([rms], dtype=np.float64),
        per_view_errors=np.asarray(per_view_errors, dtype=np.float64),
        board_type=np.asarray(args.board_type),
        board_rows=np.asarray([args.board_rows], dtype=np.int32),
        board_cols=np.asarray([args.board_cols], dtype=np.int32),
        square_size_m=np.asarray([args.square_size], dtype=np.float64),
        camera1_name=np.asarray(args.camera1_name),
        camera2_name=np.asarray(args.camera2_name),
        camera1_frame=np.asarray(args.camera1_frame),
        camera2_frame=np.asarray(args.camera2_frame),
        created_at=np.asarray(datetime.now(timezone.utc).isoformat()),
        calibration_notes=np.asarray(args.notes),
        R1=R1,
        R2=R2,
        P1=P1,
        P2=P2,
        Q=Q,
        valid_pair_count=np.asarray([len(object_points)], dtype=np.int32),
        used_image_indices=np.asarray(used_indices, dtype=np.int32),
    )

    print(f"Saved stereo calibration: {output}")
    print(f"valid pairs: {len(object_points)} / {len(pairs)}")
    print(f"front image size: {image_size_front}")
    print(f"eef image size: {image_size_eef}")
    print(f"RMS error: {rms:.6f}")
    print(f"baseline norm: {np.linalg.norm(T):.6f} m")
    print(f"det(R): {np.linalg.det(R):.9f}")
    if failed_pairs:
        print("Failed pairs:")
        for failure in failed_pairs:
            print(f"  - {failure}")

    if args.run_validation:
        run_validation(output, args)


if __name__ == "__main__":
    main()
