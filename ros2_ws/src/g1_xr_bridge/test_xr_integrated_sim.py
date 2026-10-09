"""Non-actuating regression checks for combined pinned-XR simulator overlay."""
import ast
import os
from pathlib import Path
import subprocess
import sys
import time
import unittest
from types import SimpleNamespace

import numpy as np

import xr_integrated_sim_overlay as overlay
from xr_sim_startup_patch import patch_controller_source, check_feedback_fresh


class SourceOverlayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xr_main = overlay.XR_ROOT / 'teleop' / 'teleop_hand_and_arm.py'
        cls.xr_arm = overlay.XR_ROOT / 'teleop' / 'robot_control' / 'robot_arm.py'
        if not cls.xr_main.is_file() or not cls.xr_arm.is_file():
            raise RuntimeError('Original pinned XR files missing')

    def test_main_source_and_controller_patch_compile_without_writing(self):
        orig_main = self.xr_main.read_bytes()
        orig_arm = self.xr_arm.read_bytes()
        patched_main = overlay.patch_main(orig_main.decode())
        patched_arm = patch_controller_source(orig_arm.decode())
        compile(patched_main, 'overlay main', 'exec')
        compile(patched_arm, 'overlay arm', 'exec')
        self.assertIn('g1_bridge.frame(', patched_main)
        self.assertIn('bridge_gravity_fn=g1_bridge._gravity', patched_main)
        self.assertIn('bridge_motion_guard.heartbeat()', patched_main)
        self.assertIn('self.mode_machine == self.msg.mode_machine else None', patched_arm)
        self.assertIn('self.bridge_last_lowstate_at = time.monotonic()', patched_arm)
        self.assertEqual(self.xr_main.read_bytes(), orig_main)
        self.assertEqual(self.xr_arm.read_bytes(), orig_arm)

    def test_only_one_original_motor_target_setter(self):
        main = overlay.patch_main(self.xr_main.read_text())
        self.assertEqual(main.count('arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)'), 1)
        self.assertIn("g1_bridge.mux.source.value != 'trajectory'", main)
        self.assertIn('g1_bridge.mux.enter_hold(initial_hold_at)', main)
        self.assertIn("g1_bridge.mux.source.value != 'trajectory'))", main)
        self.assertIn('sol_q = current_lr_arm_q.copy()', main)
        self.assertIn('install_sim_controller_patch(G1_29_ArmController)', main)
        self.assertIn('g1_bridge.stop()', main)
        self.assertIn('arm_ctrl.ctrl_dual_arm_go_home()', main)

    def test_pinned_source_drift_fails_closed(self):
        with self.assertRaises(RuntimeError):
            overlay.patch_main('not an XR source')
        with self.assertRaises(RuntimeError):
            patch_controller_source('class Foo: pass')

    def test_hardware_launch_refused_before_importing_sdk(self):
        script = str(Path(overlay.__file__))
        command = [sys.executable, script, '--motion', '--arm', 'G1_29']
        run = subprocess.run(command, capture_output=True, text=True, timeout=15)
        self.assertNotEqual(run.returncode, 0)
        self.assertIn('PHYSICAL BRIDGE BLOCKED', run.stderr + run.stdout)

    def test_simulation_refuses_unacknowledged_isolation(self):
        script = str(Path(overlay.__file__))
        command = [sys.executable, script, '--g1-bridge-sim', '--sim',
                   '--motion', '--arm', 'G1_29']
        env = os.environ.copy()
        env.pop('G1_XR_SIM_ACK', None)
        run = subprocess.run(command, capture_output=True, text=True,
                             timeout=5, env=env)
        self.assertNotEqual(run.returncode, 0)
        self.assertIn('not confirmed isolated', run.stderr + run.stdout)

    def test_arm_class_mock_first_command_uses_measured_and_gravity(self):
        from xr_controller_startup_review import mock_controller
        arm_source = self.xr_arm.read_text()
        candidate = patch_controller_source(arm_source)
        # Inject a mock-only gravity callback into the default of the temporary
        # source: no changes to the real controller or its runtime arguments.
        candidate = candidate.replace(
            '*, bridge_gravity_fn=None):',
            '*, bridge_gravity_fn=lambda q: np.full(14, 0.5)):', 1)
        initial = np.zeros(35)
        initial[18] = 0.4
        initial[25] = -0.2
        output = mock_controller(candidate, q=initial, patched=False)
        self.assertAlmostEqual(float(output.q[3]), 0.4)
        self.assertAlmostEqual(float(output.tau[3]), 0.5)
        self.assertEqual(output.topic, 'rt/arm_sdk')
        # Simulator patch must NOT engage blend before XR main-loop heartbeat.
        self.assertEqual(output.blend_weight, 0.0)

    def test_vr_path_keeps_original_ik_and_trajectory_bypasses_it(self):
        src = overlay.patch_main(self.xr_main.read_text())
        a = src.index('            # VR: use unchanged original XR IK')
        b = src.index('            # record data', a)
        block = src[a:b]
        # Compile the exact patched XR decision block inside a function.
        import textwrap
        chooser = ('def choose(arm_ik, arm_ctrl, g1_bridge, current_lr_arm_q, '
                   'current_lr_arm_dq, tele_data, time, logger_mp, g1_bridge_initial_hold=True):\n' +
                   textwrap.indent(textwrap.dedent(block), '    ') +
                   '    return sol_q, sol_tauff\n')
        module_globals = {}
        exec(compile(chooser, '<original XR switch decision block>', 'exec'),
             module_globals)
        choose = module_globals['choose']

        class FakeIK:
            calls = 0
            def solve_ik(self, *args):
                self.calls += 1
                return np.full(14, 0.1), np.full(14, 0.2)

        class FakeArm:
            bridge_last_lowstate_at = time.monotonic()
            bridge_lowstate_seq = 3
            calls = 0
            q = None
            bridge_motion_guard = SimpleNamespace(heartbeat=lambda: None)
            def get_current_motor_q(self):
                return np.zeros(35)
            def ctrl_dual_arm(self, q, tau):
                self.calls += 1
                self.q = np.asarray(q).copy()

        class FakeBridge:
            def __init__(self, mode):
                self.mux = SimpleNamespace(source=SimpleNamespace(value=mode))
            def update_sim_state(self, q):
                self.last_sim_state = q
            def _gravity(self, q):
                return np.full(14, 0.7)
            def frame(self, measured_q, q, tau, *, vr_ready):
                return SimpleNamespace(q=np.asarray(q).copy(), tau_ff=np.asarray(tau).copy())

        tele = SimpleNamespace(left_wrist_pose=None, right_wrist_pose=None,
                               motion_data_ready=True)
        ik = FakeIK()
        arm = FakeArm()
        logger = SimpleNamespace(debug=lambda msg: None)
        q = np.zeros(14)
        v = np.zeros(14)
        output_q, torque = choose(ik, arm, FakeBridge('vr'), q, v, tele, time, logger)
        self.assertEqual(ik.calls, 1)
        self.assertEqual(arm.calls, 1)
        np.testing.assert_allclose(output_q, 0.1)
        np.testing.assert_allclose(torque, 0.2)
        output_q, torque = choose(ik, arm, FakeBridge('trajectory'), q, v, tele, time, logger)
        self.assertEqual(ik.calls, 1, 'Trajectory must not trigger a second XR IK solve')
        self.assertEqual(arm.calls, 2)
        np.testing.assert_allclose(output_q, 0.0)
        np.testing.assert_allclose(torque, 0.7)


class FeedbackFreshTests(unittest.TestCase):
    def test_new_lowstate_receipt_required(self):
        clock = time.monotonic()
        ctrl = SimpleNamespace(bridge_lowstate_seq=4,
                               bridge_last_lowstate_at=clock)
        self.assertTrue(check_feedback_fresh(ctrl, now=clock + 0.01))
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            check_feedback_fresh(ctrl, now=clock + 0.30)
        ctrl.bridge_lowstate_seq = 0
        with self.assertRaises(RuntimeError):
            check_feedback_fresh(ctrl, now=clock)


if __name__ == '__main__':
    unittest.main()
