"""Relocation contract: simulated XR overlay scripts remain isolated."""
import ast
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
SIM_DIR = HERE / 'experiments' / 'simulation'

class SimulationLayoutTests(unittest.TestCase):
    def test_simulation_overlays_live_only_in_experiments(self):
        for name in ('xr_integrated_sim_overlay.py',
                     'xr_minimal_sim_overlay.py', 'xr_runtime_overlay.py'):
            with self.subTest(name=name):
                self.assertTrue((SIM_DIR / name).is_file())
                self.assertFalse((HERE / name).exists())
                tree = ast.parse((SIM_DIR / name).read_text())
                root_def = next(n for n in tree.body if isinstance(n, ast.Assign)
                                and any(isinstance(t, ast.Name) and t.id == 'BRIDGE_ROOT'
                                        for t in n.targets))
                self.assertIn('parents[2]', ast.unparse(root_def.value))

    def test_pipelines_point_to_new_location(self):
        pipeline = (HERE / 'g1_pipeline.py').read_text()
        self.assertIn("SIMULATION_DIR = HERE / 'experiments' / 'simulation'", pipeline)
        self.assertIn('str(SIMULATION_DIR/adapter_script)', pipeline)
        script = (HERE / 'run_xr_integrated_sim.sh').read_text()
        self.assertIn('experiments/simulation/xr_integrated_sim_overlay.py', script)

    def test_physical_headless_runtime_does_not_depend_on_sim(self):
        for name in ('xr_headless_moveit.py', 'ros2_headless_moveit_action.py',
                     'g1_readonly_observer.py', 'g1_joint_state_relay.py'):
            tree = ast.parse((HERE / name).read_text())
            modules = []
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    modules.extend(x.name for x in node.names)
                elif isinstance(node, ast.ImportFrom):
                    modules.append(node.module or '')
            self.assertFalse(any('experiments.simulation' in mod or
                                 'xr_integrated_sim_overlay' in mod or
                                 'xr_minimal_sim_overlay' in mod or
                                 'xr_runtime_overlay' in mod
                                 for mod in modules), name)

if __name__ == '__main__':
    unittest.main()
