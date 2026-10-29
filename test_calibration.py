"""Offline regression checks; run with the ROS environment from env.sh."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from calibration import CalibrationGate, validate_calibration


def complete_calibration():
    return {'Tcl_0': [1., 0., 0., 0., 0., 1., 0., 0.,
                      0., 0., 1., 0., 0., 0., 0., 1.],
            'cam_0': dict(image_width=1600, image_height=1296,
                          A11=730., A12=0., A22=730., u0=800., v0=648.,
                          k2=0., k3=0., k4=0., k5=0., k6=0., k7=0.)}


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'calib.yaml'

    def test_missing_empty_partial_then_complete(self):
        gate = CalibrationGate(self.path)
        self.assertFalse(gate.ready())
        for content in ('', 'cam_0: [', 'cam_0:\n  image_width: 1600\n'):
            self.path.write_text(content)
            self.assertFalse(gate.ready())
        self.path.write_text(yaml.safe_dump(complete_calibration()))
        self.assertFalse(gate.ready())
        self.assertTrue(gate.ready())
        # A rewrite must settle again, even if the previous file was valid.
        self.path.write_text('')
        self.assertFalse(gate.ready())
        self.path.write_text(yaml.safe_dump(complete_calibration()))
        self.assertFalse(gate.ready())
        self.assertTrue(gate.ready())

    def test_bad_parameters(self):
        for field, value in [('image_width', 0), ('image_height', '1296'),
                             ('A11', float('nan')), ('k7', None)]:
            data = complete_calibration()
            data['cam_0'][field] = value
            self.path.write_text(yaml.safe_dump(data))
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_calibration(self.path)
        data = complete_calibration()
        data['Tcl_0'] = [0.] * 15
        self.path.write_text(yaml.safe_dump(data))
        with self.assertRaises(ValueError):
            validate_calibration(self.path)

    def test_launch_defers_readers_and_preserves_old_file(self):
        import start_nodes
        from launch.actions import EmitEvent, TimerAction
        tools = Path(self.temp.name)
        (tools / 'capture.yaml').write_text(yaml.safe_dump({'register_keys': {
            key: 1 for key in ('senddepth', 'senddtof', 'sendrgb', 'sendcloudslam',
                               'sendodom', 'sendreprojection', 'sendoverlay',
                               'sendrgbundistort')}}))
        self.path.write_text('')
        env = {'ODIN_TOOLS': str(tools), 'ODIN_CALIB_DIR': str(tools),
               'ROS_LOG_DIR': str(tools / 'logs')}
        with patch.dict(os.environ, env):
            description = start_nodes.make_description()
        self.assertFalse(self.path.exists())
        self.assertEqual(len(list(tools.glob('calib.previous.*.yaml'))), 1)
        # Five exit handlers (including camera metadata), driver, polling timer.
        self.assertEqual(len(description.entities), 7)
        timer = description.entities[-1]
        self.assertIsInstance(timer, TimerAction)
        callback = timer.actions[0]
        context = type('Context', (), {'is_shutdown': False})()
        self.assertIsInstance(callback.execute(context)[0], TimerAction)
        self.path.write_text(yaml.safe_dump(complete_calibration()))
        self.assertIsInstance(callback.execute(context)[0], TimerAction)
        self.assertEqual(len(callback.execute(context)), 4)
        self.path.write_text('')
        with patch.object(start_nodes.time, 'monotonic', return_value=float('inf')):
            self.assertIsInstance(callback.execute(context)[0], EmitEvent)
        context.is_shutdown = True
        self.assertEqual(callback.execute(context), [])


if __name__ == '__main__':
    unittest.main()
