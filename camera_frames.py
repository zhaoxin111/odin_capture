"""ODIN camera geometry shared by live metadata and offline bag completion."""
from pathlib import Path

import numpy as np
import yaml

from calibration import validate_calibration


CAMERA_FRAME = 'odin1_camera_optical_frame'
LIDAR_FRAME = 'odin1_base_link'
IMAGE_TOPICS = ('/odin1/image/compressed', '/odin1/image', '/odin1/image/undistorted')


def camera_geometry(calib):
    from scipy.spatial.transform import Rotation

    validate_calibration(calib)
    data = yaml.safe_load(Path(calib).read_text())
    # Despite reversed prose comments in parts of the vendor driver, its
    # pointcloud_depth_converter multiplies K * Tcl * p_lidar: p_camera = Tcl*p_lidar.
    t_cl = np.array(data['Tcl_0'], dtype=float).reshape(4, 4)
    if not np.allclose(t_cl[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError('Tcl_0 的最后一行必须为 [0, 0, 0, 1]')
    r = t_cl[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-3) or np.linalg.det(r) < 0.99:
        raise ValueError('Tcl_0 不是有效的刚体旋转')
    # Device YAML rounds rotation coefficients. Project to SO(3) before TF's
    # unit-quaternion representation; keep the original calibration unchanged.
    u, _, vt = np.linalg.svd(r)
    r_cl = u @ vt
    r_lc = r_cl.T
    t_lc = -r_lc @ t_cl[:3, 3]
    t_tf = np.eye(4)
    t_tf[:3, :3], t_tf[:3, 3] = r_lc, t_lc
    return {
        'camera_frame': CAMERA_FRAME,
        'lidar_frame': LIDAR_FRAME,
        'point_transform': 'p_camera = Tcl_0 @ p_lidar (homogeneous, metres)',
        'T_camera_lidar_calibration': t_cl.tolist(),
        'T_lidar_camera_tf': t_tf.tolist(),
        'translation_xyz_m': t_lc.tolist(),
        'rotation_xyzw': Rotation.from_matrix(r_lc).as_quat().tolist(),
        'rotation_rounding_correction_max': float(np.max(np.abs(r_cl - r))),
        'camera_model': data['cam_0'].get('cam_model', 'FishPoly'),
        'projection_note': 'Original JPEG uses vendor FishPoly k2..k7, not a pinhole/plumb_bob model.',
    }


def static_camera_transform(geometry):
    from geometry_msgs.msg import TransformStamped
    out = TransformStamped()
    # A static transform is valid at every sensor timestamp, including device uptime.
    out.header.frame_id = LIDAR_FRAME
    out.child_frame_id = CAMERA_FRAME
    t, q = out.transform.translation, out.transform.rotation
    t.x, t.y, t.z = geometry['translation_xyz_m']
    q.x, q.y, q.z, q.w = geometry['rotation_xyzw']
    return out


def complete_image_header(msg):
    if msg.header.frame_id not in ('', CAMERA_FRAME):
        raise ValueError(f'图像已有其他坐标系，拒绝覆盖: {msg.header.frame_id}')
    msg.header.frame_id = CAMERA_FRAME
    return msg
