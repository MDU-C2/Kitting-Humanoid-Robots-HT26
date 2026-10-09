"""Unittest works with stock Python: no rclpy, robot, XR, or ROS network."""
import unittest
from offline_feedback_contract import synthetic_arm_feedback, SyntheticFeedbackError


def status():
    return {'kind': 'status_ack', 'mode': 'sim', 'hardware_connected': False,
            'xr_loop_fresh': True, 'feedback_fresh': True,
            'synthetic_joint_positions': [0.0] * 15 + [0.01] * 14}


class OfflineSyntheticFeedbackTests(unittest.TestCase):
    def test_correct_joint_slice_and_tracking_difference(self):
        result = synthetic_arm_feedback(status(), [0.02] * 14)
        self.assertEqual(result[0], [0.01] * 14)
        self.assertEqual(result[1], [0.01] * 14)

    def test_simulator_is_not_rebranded_as_synthetic(self):
        s = status()
        del s['synthetic_joint_positions']
        s['sim_joint_positions'] = [0.0] * 29
        self.assertIsNone(synthetic_arm_feedback(s, [0.0] * 14))

    def test_reject_stale_or_untrusted_receiver(self):
        for key, value in [('xr_loop_fresh', False), ('feedback_fresh', False),
                           ('hardware_connected', True), ('hardware_connected', None),
                           ('mode', 'hardware'), ('kind', 'pong')]:
            s = status()
            s[key] = value
            with self.subTest(key=key, value=value):
                with self.assertRaises(SyntheticFeedbackError):
                    synthetic_arm_feedback(s, [0.0] * 14)

    def test_reject_invalid_shapes_values(self):
        for invalid in [[0.0] * 28, [0.0] * 30, [float('nan')] * 29,
                        [True] * 29, ['0'] * 29]:
            s = status()
            s['synthetic_joint_positions'] = invalid
            with self.subTest(invalid=str(invalid)[:30]):
                with self.assertRaises(SyntheticFeedbackError):
                    synthetic_arm_feedback(s, [0.0] * 14)
        with self.assertRaises(SyntheticFeedbackError):
            synthetic_arm_feedback(status(), [0.0] * 13)

    def test_original_bridge_does_not_create_actuation(self):
        from pathlib import Path
        source = Path(__file__).with_name('ros2_trajectory_inprocess_sim.py').read_text()
        self.assertIn("base.read_feedback_status = lambda: transact({'kind': 'status'}, SIM_SOCKET)", source)
        self.assertNotIn('unitree_sdk2py', source)


if __name__ == '__main__':
    unittest.main()
