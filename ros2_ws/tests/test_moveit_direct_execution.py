"""Offline source-contract tests for opt-in G1 direct MoveIt execution."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
BR = ROOT / 'src/g1_xr_bridge'

class MoveItDirectExecutionTests(unittest.TestCase):
    def test_auto_install_and_startup(self):
        self.assertIn('G1_MOVEIT_DIRECT_INSTALL_V1', (ROOT/'scripts/install.sh').read_text())
        self.assertIn('verify_and_apply_xr.sh" --apply',
                      (BR/'launch_g1_moveit_xr.sh').read_text())
        self.assertIn('G1_MOVEIT_DIRECT_PATCH_CHECK_V1',
                      (BR/'verify_and_apply_xr.sh').read_text())
    def test_overlay_does_not_change_vr(self):
        patch = (BR/'patches/xr_moveit_direct_commands.patch').read_text()
        self.assertIn('G1_MOVEIT_DIRECT_COMMANDS_V1', patch)
        self.assertIn('cliped_arm_q_target = arm_q_target.copy()', patch)
        self.assertNotIn('+                cliped_arm_q_target = self.clip_arm_q_target', patch)
    def test_bridge_fails_closed_without_overlay(self):
        bridge = (BR/'xr_headless_moveit.py').read_text()
        self.assertIn('G1_MOVEIT_DIRECT_BRIDGE_V1', bridge)
    def test_moveit_yaml_scaling_remains(self):
        launch = (ROOT/'src/g1_moveit_config/launch/xr_pipeline_moveit.launch.py').read_text()
        self.assertIn('joint_limits.yaml', launch)
        self.assertNotIn('0.90 * speed', launch)

if __name__ == '__main__':
    unittest.main()
