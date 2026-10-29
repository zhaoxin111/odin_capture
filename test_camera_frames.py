"""Geometry direction and header preservation checks for fusion metadata."""
import tempfile
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CompressedImage
import yaml

from camera_frames import (CAMERA_FRAME, camera_geometry, complete_image_header,
                           static_camera_transform)
from test_calibration import complete_calibration


class CameraFrameTests(unittest.TestCase):
    def test_transform_direction_and_translation(self):
        data = complete_calibration()
        # FLU lidar to RDF optical: forward -> +Z, left -> -X, up -> -Y.
        t_cl = np.array([[0., -1., 0., .03], [0., 0., -1., .002],
                         [1., 0., 0., -.01], [0., 0., 0., 1.]])
        data['Tcl_0'] = t_cl.ravel().tolist()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'calib.yaml'
            path.write_text(yaml.safe_dump(data))
            geometry = camera_geometry(path)
        t_lc = np.array(geometry['T_lidar_camera_tf'])
        np.testing.assert_allclose(t_lc @ t_cl, np.eye(4), atol=1e-12)
        np.testing.assert_allclose(Rotation.from_quat(geometry['rotation_xyzw']).as_matrix(),
                                   t_lc[:3, :3], atol=1e-12)
        np.testing.assert_allclose(t_cl @ [1, 0, 0, 1], [.03, .002, .99, 1])
        tf = static_camera_transform(geometry)
        self.assertEqual(tf.header.frame_id, 'odin1_base_link')
        self.assertEqual(tf.child_frame_id, CAMERA_FRAME)

    def test_header_completion_preserves_image_and_time(self):
        msg = CompressedImage()
        msg.header.stamp.sec, msg.header.stamp.nanosec = 686, 123
        msg.format, msg.data = 'jpeg', b'unchanged JPEG bytes'
        complete_image_header(msg)
        self.assertEqual(msg.header.frame_id, CAMERA_FRAME)
        self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec), (686, 123))
        self.assertEqual(bytes(msg.data), b'unchanged JPEG bytes')
        self.assertEqual(msg.format, 'jpeg')
        msg.header.frame_id = 'another_camera'
        with self.assertRaises(ValueError):
            complete_image_header(msg)

    def test_reject_nonrigid_extrinsic(self):
        data = complete_calibration()
        data['Tcl_0'][0] = 2.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'calib.yaml'
            path.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError):
                camera_geometry(path)


if __name__ == '__main__':
    unittest.main()
