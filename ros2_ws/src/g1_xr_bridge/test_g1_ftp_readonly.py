"""SDK-free regression tests: FTP read-only observation, no URDF mislabeling."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import g1_pipeline
from g1_ftp_state_contract import (FTP_NAMES, FtpFeedbackError,
                                   normalize_angle_act, validate_ftp_snapshot)
from g1_ftp_readonly_observer import FtpStateCache
from g1_ftp_preflight import verify_stream


def snapshot(left_age=.01, right_age=.01, *, now=100.):
    return {
        'kind': 'ftp_state', 'source': 'inspire_ftp_state',
        'hardware_connected': True, 'read_only': True, 'both_fresh': True,
        'actuator_names': list(FTP_NAMES), 'normalized_angle_act': [0., .1, .2, .3, .4, 1.] * 2,
        'left_age_s': left_age, 'right_age_s': right_age,
        'units': 'fraction_of_full_range_not_radians'
    }


class FtpStateContractTests(unittest.TestCase):
    def test_order_and_real_units(self):
        self.assertEqual(FTP_NAMES[0], 'left:pinky')
        self.assertEqual(FTP_NAMES[5], 'left:thumb_rotation')
        self.assertEqual(FTP_NAMES[6], 'right:pinky')
        self.assertEqual(FTP_NAMES[11], 'right:thumb_rotation')
        self.assertEqual(normalize_angle_act([0, 100, 200, 300, 500, 1000]),
                         [0, .1, .2, .3, .5, 1.])
        result = validate_ftp_snapshot(snapshot(), now=100.)
        self.assertEqual(len(result.normalized), 12)
        self.assertAlmostEqual(result.left_received_at, 99.99)

    def test_zero_actuators_are_valid_measurements(self):
        self.assertEqual(normalize_angle_act([0] * 6), [0.] * 6)
        raw = snapshot()
        raw['normalized_angle_act'] = [0.] * 12
        self.assertEqual(validate_ftp_snapshot(raw, now=100.).normalized, (0.,) * 12)

    def test_reject_malformed_raw_states(self):
        for data in ([], [1] * 5, [1] * 7, [float('nan')] * 6,
                     [True] + [0] * 5, [-1] + [0] * 5,
                     [1001] + [0] * 5, ['0'] + [0] * 5):
            with self.subTest(data=data):
                with self.assertRaises(FtpFeedbackError):
                    normalize_angle_act(data)

    def test_reject_stale_missing_swapped_or_synthetic_feedback(self):
        cases = (
            ('read_only', False), ('source', 'synthetic'),
            ('both_fresh', False), ('actuator_names', list(reversed(FTP_NAMES))),
            ('left_age_s', .3), ('right_age_s', .3),
            ('left_age_s', float('nan')), ('right_age_s', None),
            ('normalized_angle_act', [0.] * 11),
            ('normalized_angle_act', [float('inf')] + [0.] * 11),
            ('normalized_angle_act', [False] + [0.] * 11),
        )
        for field, value in cases:
            data = snapshot()
            data[field] = value
            with self.subTest(field=field):
                with self.assertRaises(FtpFeedbackError):
                    validate_ftp_snapshot(data, now=100.)


class FtpCacheTests(unittest.TestCase):
    def test_wait_both_and_never_fabricate_missing_hand(self):
        cache = FtpStateCache()
        self.assertFalse(cache.snapshot(now=100.)['both_fresh'])
        self.assertTrue(cache.update('left', SimpleNamespace(angle_act=[0] * 6), now=100.))
        self.assertFalse(cache.snapshot(now=100.01)['both_fresh'])
        self.assertIsNone(cache.snapshot(now=100.01)['normalized_angle_act'])
        self.assertTrue(cache.update('right', SimpleNamespace(angle_act=[1000] * 6), now=100.))
        state = cache.snapshot(now=100.01)
        self.assertTrue(state['both_fresh'])
        self.assertEqual(state['normalized_angle_act'], [0.] * 6 + [1.] * 6)
        self.assertFalse(cache.snapshot(now=100.30)['both_fresh'])
        self.assertIsNone(cache.snapshot(now=100.30)['normalized_angle_act'])

    def test_receive_vs_reject_counters_and_last_message_age(self):
        cache = FtpStateCache()
        baseline = cache.snapshot(now=100.)['diagnostics']
        self.assertEqual(baseline['received_messages']['left'], 0)
        self.assertIsNone(baseline['last_message_age_s']['left'])
        self.assertFalse(cache.update('left', object(), now=100.))
        self.assertTrue(cache.update('left', SimpleNamespace(angle_act=[250] * 6), now=101.))
        snap = cache.snapshot(now=101.1)
        self.assertEqual(snap['diagnostics']['received_messages']['left'], 2)
        self.assertEqual(snap['diagnostics']['rejected_messages']['left'], 1)
        self.assertEqual(snap['diagnostics']['received_messages']['right'], 0)
        self.assertIn('AttributeError', snap['diagnostics']['last_rejection']['left'])
        self.assertAlmostEqual(snap['diagnostics']['last_message_age_s']['left'], .1)
        self.assertAlmostEqual(snap['left_age_s'], .1)
        self.assertIsNone(snap['right_age_s'])

    def test_invalid_updates_cannot_overwrite_last_valid(self):
        cache = FtpStateCache()
        self.assertTrue(cache.update('left', SimpleNamespace(angle_act=[100] * 6), now=100.))
        self.assertFalse(cache.update('left', SimpleNamespace(angle_act=[-1] * 6), now=100.1))
        self.assertFalse(cache.update('left', object(), now=100.1))
        self.assertAlmostEqual(cache.snapshot(now=100.2)['left_age_s'], .2)


class FtpPreflightTests(unittest.TestCase):
    def test_both_receipt_clocks_advance(self):
        current = [100.]
        def now():
            return current[0]
        def sleep(dt):
            current[0] += dt
        def fetch():
            return snapshot(left_age=.01, right_age=.02)
        result = verify_stream(fetch, duration_s=.4, interval_s=.1,
                               clock=now, sleep=sleep)
        self.assertGreater(result['left_progress_s'], .35)
        self.assertGreater(result['right_progress_s'], .35)
        self.assertFalse(result['execution_enabled'])

    def test_freeze_one_hand_is_rejected(self):
        current = [100.]
        def now():
            return current[0]
        def sleep(dt):
            current[0] += dt
        def fetch():
            return snapshot(left_age=.01, right_age=current[0] - 99.98)
        with self.assertRaises(FtpFeedbackError):
            verify_stream(fetch, duration_s=.4, interval_s=.1,
                          clock=now, sleep=sleep)


class OptInLaunchTests(unittest.TestCase):
    def test_default_preflight_is_unchanged(self):
        plan = g1_pipeline.build_commands(g1_pipeline.get_args([
            '--mode', 'hardware-preflight', '--interface', 'eth0']))
        flat = '\n'.join(' '.join(cmd) for _, cmd, _ in plan)
        self.assertNotIn('g1_ftp_readonly_observer.py', flat)
        self.assertNotIn('g1_ftp_ros_relay.py', flat)
        self.assertIn('enable_execution:=false', flat)

    def test_opt_in_ftp_only_subscribes_and_never_enables_moveit_execution(self):
        plan = g1_pipeline.build_commands(g1_pipeline.get_args([
            '--mode', 'hardware-preflight', '--ftp-observe', '--interface', 'eth0']))
        flat = '\n'.join(' '.join(cmd) for _, cmd, _ in plan)
        self.assertIn('g1_readonly_observer.py', flat)
        self.assertIn('g1_hardware_preflight.py', flat)
        self.assertIn('g1_ftp_readonly_observer.py', flat)
        self.assertIn('g1_ftp_preflight.py', flat)
        self.assertIn('g1_ftp_ros_relay.py', flat)
        self.assertIn('enable_execution:=false', flat)
        self.assertNotIn('ros2_trajectory_inprocess_sim.py', flat)
        self.assertNotIn('--motion', flat)
        self.assertNotIn('teleop_hand_and_arm.py', flat)

    def test_no_ftp_on_live_motion_or_offline_modes(self):
        for mode in ('hardware', 'xr-sim', 'observe', 'offline'):
            args = ['--mode', mode, '--ftp-observe', '--interface', 'eth0']
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                g1_pipeline.build_commands(g1_pipeline.get_args(args))
        with self.assertRaisesRegex(ValueError, 'HARDWARE BLOCKED'):
            g1_pipeline.build_commands(g1_pipeline.get_args(['--mode', 'hardware']))

    def test_ftp_observer_has_no_dds_publishers_and_ros_relay_not_joint_states(self):
        directory = Path(__file__).parent
        source = (directory / 'g1_ftp_readonly_observer.py').read_text()
        module = ast.parse(source)
        self.assertNotIn('ChannelPublisher', source)
        self.assertNotIn('teleop_hand_and_arm', source)
        self.assertNotIn('arm_sdk', source)
        self.assertNotIn('inspire_hand/ctrl/', source)
        relay = (directory / 'g1_ftp_ros_relay.py').read_text()
        self.assertNotIn("'/joint_states'", relay)
        self.assertIn("'/g1_ftp/angle_act_normalized'", relay)


if __name__ == '__main__':
    unittest.main()
