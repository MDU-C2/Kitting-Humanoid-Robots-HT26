"""No DDS/ROS: verify the actual guarded XR publisher gets installed."""
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

import xr_sim_startup_patch as sim


# Minimal source exercises exactly the in-memory AST installer, never imports
# or writes external XR files. A separate source-drift test covers pinned XR.
_SOURCE = """class G1_29_ArmController:
    def __init__(self):
        return 'original constructor'
    def _subscribe_motor_state(self):
        return 'original subscriber'
    def _ctrl_motor_state(self):
        return 'original unguarded publisher'
"""
_PATCHED = """class G1_29_ArmController:
    def __init__(self):
        return 'guarded constructor'
    def _subscribe_motor_state(self):
        return 'guarded subscriber'
    def _ctrl_motor_state(self):
        return 'guarded publisher'
"""


class PublisherInstallTests(unittest.TestCase):
    def test_all_three_methods_installed_without_external_changes(self):
        ns = {}
        exec(_SOURCE, ns)
        controller = ns['G1_29_ArmController']
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'robot_arm.py'
            source.write_text(_SOURCE)
            with patch.object(sim, 'original_arm_path', return_value=source), \
                 patch.object(sim, 'patch_controller_source', return_value=_PATCHED):
                result = sim.install_sim_controller_patch(controller)
            self.assertEqual(controller._ctrl_motor_state(None), 'guarded publisher')
            self.assertEqual(controller._subscribe_motor_state(None), 'guarded subscriber')
            self.assertEqual(controller.__init__(None), 'guarded constructor')
            self.assertEqual(source.read_text(), _SOURCE)
            self.assertFalse(result['hardware_authorized'])
            with self.assertRaisesRegex(RuntimeError, 'already installed'):
                sim.install_sim_controller_patch(controller)


if __name__ == '__main__':
    unittest.main()
