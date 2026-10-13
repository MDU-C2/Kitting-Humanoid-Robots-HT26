"""Contract tests for offline-only source relocation; no ROS/DDS/hardware."""
import ast
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
OFFLINE_DIR = HERE / 'experiments' / 'offline'


class OfflineLayoutTests(unittest.TestCase):
    def test_expected_offline_entrypoints(self):
        names = (
            'ros2_trajectory_xr_ipc_dry_run.py',
            'ros2_trajectory_inprocess_sim.py',
            'xr_inprocess_offline_harness.py',
            'run_offline_runtime_faults.py',
            'inprocess_sim_status.py',
            'send_inprocess_sim_goal.py',
        )
        for name in names:
            with self.subTest(name=name):
                self.assertTrue((OFFLINE_DIR / name).is_file())
                self.assertFalse((HERE / name).exists())

    def test_physical_runtime_has_no_offline_imports(self):
        for name in ('ros2_headless_moveit_action.py', 'xr_headless_moveit.py',
                     'g1_readonly_observer.py', 'g1_joint_state_relay.py'):
            tree = ast.parse((HERE / name).read_text())
            for node in ast.walk(tree):
                if isinstance(node, (ast.Import, ast.ImportFrom)):
                    imports = ([alias.name for alias in node.names]
                               if isinstance(node, ast.Import) else [node.module or ''])
                    self.assertFalse(any('experiments' in item or 'ros2_trajectory_inprocess_sim' in item
                                         or 'ros2_trajectory_xr_ipc_dry_run' in item
                                         for item in imports), name)

    def test_offline_run_script_points_at_new_path(self):
        source = (HERE / 'g1_pipeline.py').read_text()
        self.assertIn("OFFLINE_DIR = HERE / 'experiments' / 'offline'", source)
        source = (HERE / 'run_xr_harness.sh').read_text()
        self.assertIn('experiments/offline/xr_inprocess_offline_harness.py', source)


if __name__ == '__main__':
    unittest.main()
