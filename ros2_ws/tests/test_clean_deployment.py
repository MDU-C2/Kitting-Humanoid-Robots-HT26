"""Non-actuating contract tests for the minimal deployment layout."""
import ast
import importlib.util
import os
import tempfile
import re
import xml.etree.ElementTree as ET
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
BR = ROOT / 'src' / 'g1_xr_bridge'
FILES = {
    'xr_headless_moveit.py', 'ros2_headless_moveit_action.py',
    'g1_readonly_observer.py', 'g1_joint_state_relay.py',
    'xr_moveit_goal_conversion.py', 'xr_arm_trajectory.py',
    'xr_ipc_protocol.py', 'g1_state_contract.py', 'g1_ipc_paths.py',
}

class CleanDeploymentTests(unittest.TestCase):
    def test_only_nine_runtime_python_modules(self):
        self.assertEqual({p.name for p in BR.glob('*.py')}, FILES)
        self.assertFalse((BR / 'experiments').exists())

    def test_runtime_local_imports_are_closed(self):
        for source in FILES:
            tree = ast.parse((BR / source).read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom) and node.module:
                    head = node.module.split('.')[0]
                    if (BR / (head + '.py')).exists():
                        self.assertIn(head + '.py', FILES)
                elif isinstance(node, ast.Import):
                    for item in node.names:
                        head = item.name.split('.')[0]
                        if (BR / (head + '.py')).exists():
                            self.assertIn(head + '.py', FILES)

    def test_original_hardware_launcher_and_pins(self):
        source = (BR / 'launch_g1_moveit_xr.sh').read_text()
        self.assertIn('G1_REGULAR_MODE_SUPERVISED', source)
        self.assertIn('verify_and_apply_xr.sh', source)
        self.assertTrue((BR / 'patches/xr_headless_moveit.patch').is_file())
        self.assertTrue((BR / 'patches/dependency_revisions.txt').is_file())
        subprocess.run(['bash', '-n', str(BR / 'launch_g1_moveit_xr.sh')], check=True)

    def test_robot_model_and_controller_config(self):
        self.assertTrue((ROOT / 'src/g1_description/urdf/g1_29dof_with_inspire_hand_ftp.urdf.xacro').is_file())
        cfg = ROOT / 'src/g1_moveit_config/config/moveit_xr_controllers.yaml'
        self.assertIn('action_ns: follow_joint_trajectory', cfg.read_text())
        self.assertNotIn('g1_control', (ROOT / 'src/g1_moveit_config/package.xml').read_text())

    def test_description_has_no_legacy_arm_sdk_plugin(self):
        description = ROOT / 'src/g1_description'
        wrapper = description / 'urdf/g1_29dof_with_inspire_hand_ftp.urdf.xacro'
        mechanical = description / 'urdf/g1_29dof_with_inspire_hand_ftp.urdf'
        wrapper_root = ET.parse(wrapper).getroot()
        srdf = ROOT / 'src/g1_moveit_config/config/g1.srdf'
        self.assertEqual(wrapper_root.attrib['name'], ET.parse(srdf).getroot().attrib['name'])
        self.assertEqual(len(list(wrapper_root)), 1)
        include = list(wrapper_root)[0]
        self.assertEqual(include.tag, '{http://www.ros.org/wiki/xacro}include')
        self.assertEqual(include.attrib['filename'], mechanical.name)
        self.assertTrue(mechanical.is_file())
        self.assertNotIn('xacro.load_yaml', wrapper.read_text())
        self.assertNotIn('<ros2_control', wrapper.read_text())
        self.assertNotIn('g1_hardware_interface/', wrapper.read_text())

    def test_description_cmake_installs_only_existing_directories(self):
        description = ROOT / 'src/g1_description'
        cmake = (description / 'CMakeLists.txt').read_text()
        match = re.search(r'install\s*\(\s*DIRECTORY\s+([^\n]+)', cmake)
        self.assertIsNotNone(match, 'Expected a CMake install(DIRECTORY ...) rule')
        for directory in match.group(1).split():
            self.assertTrue((description / directory).is_dir(),
                            f'Missing directory referenced by CMake: {directory}')

    def test_robot_xacro_expands_without_removed_hardware_package(self):
        # xacro is part of the ROS 2 Docker environment. Outside ROS, skip.
        if importlib.util.find_spec('xacro') is None:
            self.skipTest('ROS 2 xacro not installed in this Python environment')
        import xacro
        path = ROOT / 'src/g1_description/urdf/g1_29dof_with_inspire_hand_ftp.urdf.xacro'
        rendered = ET.fromstring(xacro.process_file(str(path)).toxml())
        self.assertEqual(rendered.attrib['name'],
                         'g1_29dof_with_inspire_hand_ftp')
        self.assertFalse(list(rendered.iter('ros2_control')))
        all_joint_names = {j.attrib['name'] for j in rendered.iter('joint')}
        self.assertIn('left_shoulder_pitch_joint', all_joint_names)
        self.assertIn('right_wrist_yaw_joint', all_joint_names)

    def test_start_never_stops_or_removes_container(self):
        script = (ROOT / 'scripts/start.sh').read_text()
        self.assertNotIn('docker rm', script)
        self.assertNotIn('docker stop', script)
        self.assertNotIn('docker run --rm', script)
        self.assertIn('sleep infinity', script)
        self.assertIn('docker exec -it "$NAME" bash', script)
        self.assertNotIn('CYCLONEDDS_URI', script)
        self.assertNotIn('G1_ROBOT_IP', script)
        for name in ('install.sh', 'start.sh', 'run_g1.sh'):
            subprocess.run(['bash', '-n', str(ROOT / 'scripts' / name)], check=True)

    def test_start_reattaches_without_destroying_container(self):
        # Simulate Docker CLI responses. No real Docker or robot involvement.
        with tempfile.TemporaryDirectory() as tmp:
            fake = Path(tmp) / 'docker'
            fake.write_text("""#!/bin/bash
printf '%s\\n' "$*" >> "$DOCKER_MOCK_LOG"
case "$1 $2" in
  'ps --format') exit 0 ;;
  'container inspect') exit 0 ;;
  'inspect -f') echo true; exit 0 ;;
  'exec -it') echo 'MOCK ATTACHED'; exit 0 ;;
esac
echo "Unexpected Docker call: $*" >&2
exit 94
""")
            fake.chmod(0o755)
            log = Path(tmp) / 'docker.log'
            env = os.environ.copy()
            env['DOCKER_MOCK_LOG'] = str(log)
            env['PATH'] = tmp + os.pathsep + env.get('PATH', '')
            result = subprocess.run(['bash', str(ROOT / 'scripts/start.sh')],
                                    capture_output=True, text=True, env=env)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('Reusing existing container', result.stdout)
            calls = log.read_text().splitlines()
            self.assertTrue(any(x.startswith('exec -it g1_xr_clean_humble') for x in calls))
            self.assertFalse(any(x.startswith(('stop ', 'rm ', 'run ', 'start ')) for x in calls))

    def test_physical_launch_is_separate_and_read_only_preflight_required(self):
        script = (ROOT / 'scripts/run_g1.sh').read_text()
        self.assertFalse((ROOT / 'scripts/run_physical.sh').exists())
        self.assertIn('CYCLONEDDS_URI', script)
        self.assertIn('check_g1_dds.py', script)
        self.assertIn('G1_REGULAR_MODE_SUPERVISED', script)
        self.assertIn('launch_g1_moveit_xr.sh', script)
        self.assertIn('READ-ONLY CHECK PASSED', script)
        preflight = (ROOT / 'scripts/check_g1_dds.py').read_text()
        self.assertIn("ChannelSubscriber('rt/lowstate'", preflight)
        self.assertNotIn('ChannelPublisher', preflight)
        ast.parse(preflight)

if __name__ == '__main__':
    unittest.main()
