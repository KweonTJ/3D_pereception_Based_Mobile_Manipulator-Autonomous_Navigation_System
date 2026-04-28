#!/usr/bin/env python3

import json
import os
import re
import time
from pathlib import Path

from camera_calibration.calibrator import Calibrator
from camera_calibration.nodes.cameracalibrator import main as cameracalibrator_main


def _sanitize_prefix(prefix: str) -> str:
    sanitized = re.sub(r'[^A-Za-z0-9_.-]+', '_', prefix).strip('._')
    return sanitized or 'astra_mini_color'


def _camera_info_to_dict(msg) -> dict:
    return {
        'width': msg.width,
        'height': msg.height,
        'distortion_model': msg.distortion_model,
        'd': list(msg.d),
        'k': list(msg.k),
        'r': list(msg.r),
        'p': list(msg.p),
        'binning_x': msg.binning_x,
        'binning_y': msg.binning_y,
        'roi': {
            'x_offset': msg.roi.x_offset,
            'y_offset': msg.roi.y_offset,
            'height': msg.roi.height,
            'width': msg.roi.width,
            'do_rectify': msg.roi.do_rectify,
        },
    }


def _board_to_dict(board) -> dict:
    return {
        'pattern': getattr(board, 'pattern', ''),
        'cols': getattr(board, 'n_cols', 0),
        'rows': getattr(board, 'n_rows', 0),
        'square_size_m': getattr(board, 'dim', 0.0),
        'marker_size_m': getattr(board, 'marker_size', 0.0),
    }


def _camera_model_name(calibrator) -> str:
    camera_model = getattr(calibrator, 'camera_model', None)
    return getattr(camera_model, 'name', str(camera_model))


def _compute_mono_linear_error_stats(calibrator) -> dict:
    errors = []
    for sample in getattr(calibrator, 'db', []):
        if len(sample) < 2:
            continue
        error = calibrator.linear_error_from_image(sample[1])
        if error is not None:
            errors.append(float(error))

    if not errors:
        return {}

    return {
        'linear_error_rms_px_mean': sum(errors) / len(errors),
        'linear_error_rms_px_min': min(errors),
        'linear_error_rms_px_max': max(errors),
        'linear_error_rms_px_last': errors[-1],
    }


def _build_payload(calibrator) -> dict:
    payload = {
        'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'camera_name': getattr(calibrator, 'name', ''),
        'is_mono': bool(getattr(calibrator, 'is_mono', True)),
        'camera_model': _camera_model_name(calibrator),
        'sample_count': len(getattr(calibrator, 'db', [])),
        'boards': [_board_to_dict(board) for board in getattr(calibrator, '_boards', [])],
        'ost': calibrator.ost(),
    }

    if calibrator.is_mono:
        payload['camera_info'] = _camera_info_to_dict(calibrator.as_message())
        payload.update(_compute_mono_linear_error_stats(calibrator))
    else:
        left_msg, right_msg = calibrator.as_message()
        payload['left_camera_info'] = _camera_info_to_dict(left_msg)
        payload['right_camera_info'] = _camera_info_to_dict(right_msg)
        payload['rotation'] = getattr(calibrator, 'R', []).tolist()
        payload['translation'] = getattr(calibrator, 'T', []).tolist()

    return payload


def _patched_do_save(self) -> None:
    output_dir = Path(os.environ.get('ASTRA_MINI_CALIB_OUTPUT_DIR', '/tmp'))
    prefix = _sanitize_prefix(os.environ.get('ASTRA_MINI_CALIB_OUTPUT_PREFIX', 'astra_mini_color'))

    output_dir.mkdir(parents=True, exist_ok=True)

    json_path = output_dir / f'{prefix}.json'
    payload = _build_payload(self)
    json_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding='utf-8',
    )

    print('Saved calibration file:')
    print(f'  - {json_path}')


def main() -> None:
    output_dir = Path(os.environ.get('ASTRA_MINI_CALIB_OUTPUT_DIR', '/tmp'))
    prefix = _sanitize_prefix(os.environ.get('ASTRA_MINI_CALIB_OUTPUT_PREFIX', 'astra_mini_color'))
    print(f'Calibration save directory: {output_dir}')
    print(f'Calibration file prefix: {prefix}')

    Calibrator.do_save = _patched_do_save
    cameracalibrator_main()


if __name__ == '__main__':
    main()
