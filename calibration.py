"""Validate the calibration fields consumed by the ODIN processing nodes."""
import math
from pathlib import Path

import yaml


def validate_calibration(path):
    """Raise ValueError for empty, partial, or invalid calibration files."""
    try:
        data = yaml.safe_load(Path(path).read_text())
        if not isinstance(data, dict) or not isinstance(data.get('cam_0'), dict):
            raise ValueError('缺少 cam_0 相机参数')
        camera = data['cam_0']
        for key in ('image_width', 'image_height'):
            value = camera.get(key)
            if type(value) is not int or value <= 0:
                raise ValueError(f'cam_0.{key} 必须为正整数')
        for key in ('A11', 'A12', 'A22', 'u0', 'v0',
                    'k2', 'k3', 'k4', 'k5', 'k6', 'k7'):
            value = camera.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'cam_0.{key} 缺失或无效')
        if camera['A11'] <= 0 or camera['A22'] <= 0:
            raise ValueError('相机焦距必须为正数')
        transform = data.get('Tcl_0')
        if not isinstance(transform, list) or len(transform) != 16:
            raise ValueError('Tcl_0 必须包含 16 个数值')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in transform):
            raise ValueError('Tcl_0 包含无效数值')
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'标定文件尚不可用: {exc}') from exc


class CalibrationGate:
    """Require valid content unchanged across polls before starting readers."""
    def __init__(self, path):
        self.path = Path(path)
        self.previous = None
        self.error = '等待设备写入标定文件'

    def ready(self):
        try:
            before = self.path.stat()
            validate_calibration(self.path)
            after = self.path.stat()
            signature = (after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns)
            if before != after:
                raise ValueError('标定文件正在写入')
        except (OSError, ValueError) as exc:
            self.previous = None
            self.error = str(exc)
            return False
        ready = signature == self.previous
        self.previous = signature
        self.error = '等待标定文件写入稳定'
        return ready
