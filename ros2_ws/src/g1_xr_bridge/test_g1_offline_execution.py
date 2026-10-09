"""Protect explicit offline-only MoveIt execution opt-in, without robot SDK/DDS."""
import unittest

import g1_pipeline


class OfflineExecutionLaunchTests(unittest.TestCase):
    def launch_command(self, *flags):
        commands = g1_pipeline.build_commands(g1_pipeline.get_args(list(flags)))
        matches = [cmd for name, cmd, _ in commands if 'MoveIt 2 + robot_state_publisher' in name]
        self.assertEqual(len(matches), 1)
        return ' '.join(matches[0])

    def test_offline_default_is_plan_only(self):
        cmd = self.launch_command('--mode', 'offline')
        self.assertIn('enable_execution:=false', cmd)
        self.assertNotIn('enable_execution:=true', cmd)

    def test_explicit_optin_only_in_offline(self):
        cmd = self.launch_command('--mode', 'offline', '--enable-offline-execution')
        self.assertIn('enable_execution:=true', cmd)
        self.assertIn('xr_pipeline_moveit.launch.py', cmd)
        all_commands = g1_pipeline.build_commands(g1_pipeline.get_args(['--mode', 'offline', '--enable-offline-execution']))
        flattened = ' '.join(' '.join(c) for _, c, _ in all_commands)
        self.assertIn('ros2_trajectory_inprocess_sim.py', flattened)
        self.assertIn('xr_inprocess_offline_harness.py', flattened)
        self.assertNotIn('g1_readonly_observer.py', flattened)
        self.assertNotIn('--network-interface', flattened)

    def test_reject_optin_in_every_nonoffline_mode(self):
        for mode in ('observe', 'hardware-preflight', 'xr-sim', 'hardware'):
            with self.subTest(mode=mode):
                with self.assertRaisesRegex(ValueError, 'ONLY permitted'):
                    g1_pipeline.build_commands(g1_pipeline.get_args([
                        '--mode', mode, '--enable-offline-execution']))

    def test_cannot_enable_without_moveit(self):
        with self.assertRaisesRegex(ValueError, 'requires MoveIt'):
            g1_pipeline.build_commands(g1_pipeline.get_args([
                '--mode', 'offline', '--enable-offline-execution', '--no-moveit']))

    def test_readonly_observation_still_disables_execution(self):
        for mode in ('observe', 'hardware-preflight'):
            with self.subTest(mode=mode):
                self.assertIn('enable_execution:=false', self.launch_command(
                    '--mode', mode, '--interface', 'eth0'))

    def test_existing_isolated_xr_sim_behavior_unchanged(self):
        cmd = self.launch_command('--mode', 'xr-sim',
                                  '--isolation-ack', 'ISOLATED_SIMULATOR',
                                  '--sim-interface', 'lo')
        self.assertIn('enable_execution:=true', cmd)


if __name__ == '__main__':
    unittest.main()
