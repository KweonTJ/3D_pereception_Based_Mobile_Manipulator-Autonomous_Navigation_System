#!/usr/bin/env python3
"""Convert canonical stereo calibration npz into runtime OpenCV YAML."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description="Generate runtime YAML from front/eef stereo npz.")
    parser.add_argument("calib", nargs="?", help="Input stereo calibration .npz")
    parser.add_argument("--calib", dest="calib_opt", help="Input stereo calibration .npz")
    parser.add_argument(
        "--output",
        default="mp_control/calibration/stereo/front_eef_stereo_calib.yaml",
        help="Output OpenCV YAML path",
    )
    return parser.parse_args()


def scalar(npz, key, default=""):
    if key not in npz.files:
        return default
    value = np.asarray(npz[key]).reshape(-1)
    if value.size == 0:
        return default
    item = value[0]
    if isinstance(item, bytes):
        return item.decode("utf-8")
    return str(item)


def main():
    args = parse_args()
    calib_path = Path(args.calib_opt or args.calib or "")
    if not calib_path:
        raise SystemExit("calibration npz path is required")
    if not calib_path.exists():
        raise SystemExit(f"Calibration npz not found: {calib_path}")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    with np.load(str(calib_path), allow_pickle=False) as npz:
        required = [
            "K_front",
            "D_front",
            "K_eef",
            "D_eef",
            "R_front_to_eef",
            "T_front_to_eef",
            "image_size_front",
            "image_size_eef",
        ]
        missing = [key for key in required if key not in npz.files]
        if missing:
            raise SystemExit(f"Missing required npz keys: {', '.join(missing)}")

        fs = cv2.FileStorage(str(output), cv2.FILE_STORAGE_WRITE)
        fs.write("derived_from_npz", str(calib_path))
        fs.write("derived_file_warning", "auto-generated from npz; do not edit by hand")
        fs.write("K_front", np.asarray(npz["K_front"], dtype=np.float64))
        fs.write("D_front", np.asarray(npz["D_front"], dtype=np.float64).reshape(-1, 1))
        fs.write("K_eef", np.asarray(npz["K_eef"], dtype=np.float64))
        fs.write("D_eef", np.asarray(npz["D_eef"], dtype=np.float64).reshape(-1, 1))
        fs.write("R_front_to_eef", np.asarray(npz["R_front_to_eef"], dtype=np.float64))
        fs.write("T_front_to_eef", np.asarray(npz["T_front_to_eef"], dtype=np.float64).reshape(3, 1))
        fs.write("image_size_front", np.asarray(npz["image_size_front"], dtype=np.int32).reshape(1, -1))
        fs.write("image_size_eef", np.asarray(npz["image_size_eef"], dtype=np.int32).reshape(1, -1))
        fs.write("rms_error", float(np.asarray(npz.get("rms_error", [0.0])).reshape(-1)[0]))
        fs.write("camera1_frame", scalar(npz, "camera1_frame", "camera_color_optical_frame"))
        fs.write("camera2_frame", scalar(npz, "camera2_frame", "eef_usb_camera_optical_frame"))
        fs.write("camera1_name", scalar(npz, "camera1_name", "front_astra_rgb"))
        fs.write("camera2_name", scalar(npz, "camera2_name", "eef_usb_camera"))
        fs.release()

    text = output.read_text(encoding="utf-8")
    lines = text.splitlines()
    insert_at = 1 if lines and lines[0].startswith("%YAML") else 0
    lines.insert(insert_at, "# This file is derived from the canonical stereo calibration .npz.")
    lines.insert(insert_at + 1, "# Regenerate it with stereo_npz_to_yaml.py instead of editing it by hand.")
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote runtime YAML: {output}")


if __name__ == "__main__":
    main()
