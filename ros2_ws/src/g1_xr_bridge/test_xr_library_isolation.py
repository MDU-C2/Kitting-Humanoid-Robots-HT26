"""Regression checks for XR/ROS native-library isolation; no SDK or G1 needed."""
import unittest

import g1_pipeline


class XrLibraryIsolationTests(unittest.TestCase):
    def test_readonly_g1_and_ftp_observers_use_xr_openssl(self):
        args = g1_pipeline.get_args([
            '--mode', 'hardware-preflight', '--ftp-observe', '--interface', 'eth0',
        ])
        commands = g1_pipeline.build_commands(args)
        xr = [(name, env) for name, cmd, env in commands
              if cmd and cmd[0] == str(g1_pipeline.XR_PY)]
        self.assertEqual(len(xr), 2)
        for name, env in xr:
            with self.subTest(name=name):
                self.assertNotIn('PYTHONPATH', env)
                self.assertEqual(env.get('LD_LIBRARY_PATH'), '/opt/mamba/envs/xr/lib')

    def test_ros_state_relay_keeps_original_environment(self):
        args = g1_pipeline.get_args([
            '--mode', 'hardware-preflight', '--ftp-observe', '--interface', 'eth0',
        ])
        commands = g1_pipeline.build_commands(args)
        for name, cmd, env in commands:
            if name.startswith('ROS '):
                self.assertNotEqual(env.get('LD_LIBRARY_PATH'), '/opt/mamba/envs/xr/lib')

    def test_hardware_command_launch_still_refused(self):
        with self.assertRaisesRegex(ValueError, 'HARDWARE BLOCKED'):
            g1_pipeline.build_commands(g1_pipeline.get_args(['--mode', 'hardware']))


if __name__ == '__main__':
    unittest.main()
