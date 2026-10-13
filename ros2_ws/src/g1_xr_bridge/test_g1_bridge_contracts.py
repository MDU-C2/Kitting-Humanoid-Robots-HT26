"""Pure dependency tests; do not initialize ROS, Unitree DDS, or XR control."""
import ast
from pathlib import Path
import unittest
from g1_ipc_paths import HEADLESS_CONTROL_SOCKET, READONLY_STATE_SOCKET, SIM_SOCKET, OBSERVER_SOCKET
from xr_moveit_goal_conversion import convert_goal


class BridgeRefactorTests(unittest.TestCase):
    def test_endpoints(self):
        self.assertEqual(HEADLESS_CONTROL_SOCKET, '/tmp/g1_xr_headless_moveit.sock')
        self.assertEqual(READONLY_STATE_SOCKET, '/tmp/g1_xr_readonly_state.sock')
        self.assertEqual(OBSERVER_SOCKET, READONLY_STATE_SOCKET)

    def test_physical_relay_imports_no_simulation_or_observer(self):
        tree = ast.parse(Path(__file__).with_name('g1_joint_state_relay.py').read_text())
        modules = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
        self.assertNotIn('xr_inprocess_bridge', modules)
        self.assertNotIn('g1_readonly_observer', modules)

    def test_physical_action_imports_pure_converter(self):
        tree = ast.parse(Path(__file__).with_name('ros2_headless_moveit_action.py').read_text())
        modules = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
        self.assertIn('xr_moveit_goal_conversion', modules)
        self.assertNotIn('ros2_trajectory_xr_ipc_dry_run', modules)
        self.assertTrue(callable(convert_goal))


if __name__ == '__main__':
    unittest.main()
