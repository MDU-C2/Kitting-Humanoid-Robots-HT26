"""Offline tests for read-only XR limited-command diagnostics (no robot)."""
import csv
import importlib.util
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BR = ROOT / 'src/g1_xr_bridge'
TRACE = ROOT / 'scripts/trace_g1_tracking.py'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TrackingDiagnosticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = load_module('xr_headless_moveit_trace_test', BR / 'xr_headless_moveit.py')
        cls.trace = load_module('trace_g1_tracking_test', TRACE)

    def fake_status(self):
        now = time.monotonic()
        arm = types.SimpleNamespace(
            lowstate_rx_at=now,
            ctrl_lock=threading.Lock(),
            q_target=np.full(14, 0.10),
            _headless_command_q=np.full(14, 0.04),
            arm_velocity_limit=0.25,
            _headless_max_command_lead_rad=0.035,
            mode_machine=6,
            get_current_dual_arm_q=lambda: np.zeros(14),
            get_current_dual_arm_dq=lambda: np.zeros(14),
        )
        control = self.bridge.Control.__new__(self.bridge.Control)
        control.np = np
        control.arm = arm
        control.lock = threading.RLock()
        control.last_q = np.full(14, 0.10)
        control.source = 'trajectory'
        control.started = now - 10
        control.active = True
        control.worker = types.SimpleNamespace(is_alive=lambda: True)
        control.last_sample_at = now
        return control.status()

    def test_status_exposes_limited_command_without_actuation(self):
        status = self.fake_status()
        self.assertEqual(status['kind'], 'status_ack')
        self.assertEqual(len(status['xr_limited_command_q']), 14)
        self.assertAlmostEqual(status['xr_limited_command_q'][0], 0.04)
        self.assertAlmostEqual(status['selected_arm_target'][0], 0.10)
        self.assertAlmostEqual(status['measured_arm_positions'][0], 0.0)
        self.assertAlmostEqual(status['xr_headless_speed_limit_rad_s'], 0.25)
        self.assertAlmostEqual(status['xr_headless_max_lead_rad'], 0.035)

    def test_csv_contains_desired_limited_measured(self):
        status = self.fake_status()
        row = self.trace.status_to_row(status, 12345.0)
        self.assertEqual(len(row), len(self.trace.HEADER))
        self.assertEqual(row[2], 'trajectory')
        self.assertAlmostEqual(row[6], 0.06)
        self.assertAlmostEqual(row[7], 0.04)
        self.assertAlmostEqual(row[8], 0.10)
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'trace.csv'
            calls = []
            def fake_transact(msg, *args, **kwargs):
                calls.append(dict(msg))
                return self.fake_status()
            with mock.patch.object(self.trace, 'transact', side_effect=fake_transact):
                rc = self.trace.main(['--output', str(output), '--rate-hz', '20', '--duration', '0.11'])
            self.assertEqual(rc, 0)
            self.assertTrue(calls)
            self.assertTrue(all(call == {'kind': 'status'} for call in calls))
            with output.open(newline='') as f:
                rows = list(csv.DictReader(f))
            self.assertGreaterEqual(len(rows), 2)
            key = 'limited_command_' + self.trace.ARM_JOINT_NAMES[0]
            self.assertAlmostEqual(float(rows[0][key]), 0.04)

    def test_direct_mode_lead_zero(self):
        # G1_MOVEIT_DIRECT_TEST_TRACE_V1
        status = self.fake_status()
        status['xr_headless_max_lead_rad'] = 0.0
        row = self.trace.status_to_row(status, 12345.0)
        self.assertEqual(row[5], 0.0)

    def test_rejects_invalid_command_vector(self):
        status = self.fake_status()
        status['xr_limited_command_q'] = [0.0] * 13
        with self.assertRaisesRegex(ValueError, '14 joint values'):
            self.trace.status_to_row(status, 1.0)


if __name__ == '__main__':
    unittest.main()
