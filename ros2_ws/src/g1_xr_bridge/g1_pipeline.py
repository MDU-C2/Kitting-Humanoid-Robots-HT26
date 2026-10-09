#!/usr/bin/env python3
"""One supervisor for MoveIt -> XR G1 integration.

OFFLINE (default): synthetic XR/Pinocchio + ROS + RViz, NO Unitree DDS.
OBSERVE: actual G1 LowState subscription ONLY; MoveIt execution disabled.
HARDWARE-PREFLIGHT: observe + continuous receipt/feedback validation; NO motor commands.
XR-SIM: XR VR + original G1_29 controller in externally isolated Isaac sim.
HARDWARE: deliberately not implemented. Do not bypass interlocks.
"""
import argparse
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
SRC = HERE.parent
XR = SRC / 'xr_teleoperate'
XR_PY = Path('/opt/mamba/envs/xr/bin/python')
ROS_PY = Path('/usr/bin/python3')
ROS_BASE = Path('/opt/ros/humble/setup.bash')
ROS_INSTALL = Path('/workspace/install/setup.bash')


def command_for_ros(argv):
    """Source ROS in each process, don't change Micromamba's environment."""
    import shlex
    setup = f'source {shlex.quote(str(ROS_BASE))}; source {shlex.quote(str(ROS_INSTALL))}; '
    return ['bash', '-lc', setup + 'exec ' + shlex.join([str(x) for x in argv])]


def build_commands(args):
    """Pure planner for inspection/tests; does not run or import robot SDK."""
    if args.mode == 'hardware':
        raise ValueError('HARDWARE BLOCKED: watchdog, blending, measured action tolerances and physical stop are unvalidated')
    if args.moveit_plan_shadow and args.mode != 'hardware-preflight':
        raise ValueError('--moveit-plan-shadow requires READ-ONLY hardware-preflight')
    if args.moveit_plan_shadow and args.no_moveit:
        raise ValueError('--moveit-plan-shadow needs MoveIt planning; remove --no-moveit')
    if args.arm_selector_probe and args.mode != 'hardware-preflight':
        raise ValueError('--arm-selector-probe is only allowed in read-only hardware-preflight')
    if args.ftp_observe and args.mode != 'hardware-preflight':
        raise ValueError('--ftp-observe is only allowed in read-only hardware-preflight')
    if args.mode in ('observe', 'hardware-preflight') and not args.interface:
        raise ValueError('--interface is required for read-only G1 observation')
    if args.mode == 'xr-sim' and (args.isolation_ack != 'ISOLATED_SIMULATOR' or not args.sim_interface):
        raise ValueError('XR simulator requires --isolation-ack ISOLATED_SIMULATOR and --sim-interface')
    sim = args.mode in ('offline', 'xr-sim')
    env = os.environ.copy()
    env['G1_XR_SOURCE_ROOT'] = str(XR)
    env.pop('G1_XR_SIM_ACK', None)
    # Deliberate separation: XR Python is Micromamba, ROS Python is Ubuntu.
    xr_env = dict(env)
    xr_env.pop('PYTHONPATH', None)
    # Use XR's matching conda-forge OpenSSL/native libraries, not Ubuntu's
    # older libcrypto from the ROS-sourced shell. Do not modify ROS env.
    xr_env['LD_LIBRARY_PATH'] = '/opt/mamba/envs/xr/lib'
    commands = []
    if args.mode == 'offline':
        commands.append(('XR offline Pinocchio/IK, zero DDS',
                         [str(XR_PY), str(HERE/'xr_inprocess_offline_harness.py')], xr_env))
    elif args.mode in ('observe', 'hardware-preflight'):
        commands.append(('G1 read-only LowState observer',
                         [str(XR_PY), str(HERE/'g1_readonly_observer.py'),
                          '--interface', args.interface], xr_env))
    else:
        xr_env['G1_XR_SIM_ACK'] = 'ISOLATED_SIMULATOR'
        adapter_script = ('xr_minimal_sim_overlay.py' if args.xr_adapter == 'minimal'
                          else 'xr_integrated_sim_overlay.py')
        adapter_flag = ('--g1-bridge-minimal-sim' if args.xr_adapter == 'minimal'
                        else '--g1-bridge-sim')
        command = [str(XR_PY), str(HERE/adapter_script),
                   adapter_flag, '--sim', '--motion', '--arm', 'G1_29',
                   '--ee', 'inspire_ftp', '--network-interface', args.sim_interface,
                   '--img-server-ip', args.image_server_ip]
        if args.xr_ipc:
            command.append('--ipc')
        commands.append(('ORIGINAL XR + VR (ISOLATED SIMULATOR ONLY)', command, xr_env))

    if args.mode == 'hardware-preflight' and args.ftp_observe:
        commands.append(('Read-only Inspire FTP DDS state observer (NO commands)',
                         [str(XR_PY), str(HERE/'g1_ftp_readonly_observer.py'),
                          '--interface', args.interface], xr_env))
        commands.append(('FTP real read-only feedback preflight (NO commands)',
                         [str(ROS_PY), str(HERE/'g1_ftp_preflight.py'), '--watch'], env))
        commands.append(('ROS FTP raw normalized diagnostics (NOT /joint_states)',
                         command_for_ros([str(ROS_PY), str(HERE/'g1_ftp_ros_relay.py')]), env))
    if args.mode == 'hardware-preflight' and args.arm_selector_probe:
        commands.append(('G1 read-only measured arm selector probe (NO commands)',
                         [str(ROS_PY), str(HERE/'g1_measured_arm_probe.py'), '--watch'], env))
    if args.mode == 'hardware-preflight' and args.moveit_plan_shadow:
        commands.append(('G1 MoveIt displayed-plan shadow (NO commands)',
                         command_for_ros([str(ROS_PY), str(HERE/'g1_moveit_shadow_monitor.py')]), env))
    if args.mode == 'hardware-preflight':
        commands.append(('G1 real read-only feedback preflight (NO commands)',
                         [str(ROS_PY), str(HERE/'g1_hardware_preflight.py'), '--watch'], env))
    if sim:
        commands.append(('MoveIt -> XR offline/sim FollowJointTrajectory action',
                         command_for_ros([str(ROS_PY), str(HERE/'ros2_trajectory_inprocess_sim.py')]), env))
    commands.append(('ROS measured/synthetic joint-state relay',
                     command_for_ros([str(ROS_PY), str(HERE/'g1_joint_state_relay.py'),
                                      '--mode', 'observe' if args.mode == 'hardware-preflight' else args.mode]), env))
    if not args.no_moveit:
        commands.append(('MoveIt 2 + robot_state_publisher' + ('' if args.no_rviz else ' + RViz'),
                         command_for_ros(['ros2', 'launch', 'g1_moveit_config',
                                          'xr_pipeline_moveit.launch.py',
                                          'enable_execution:=' + ('true' if sim else 'false'),
                                          'with_rviz:=' + ('false' if args.no_rviz else 'true')]), env))
    return commands


def preflight(args):
    if not XR.is_dir():
        raise RuntimeError(f'Missing pinned XR source: {XR}')
    if not XR_PY.is_file():
        raise RuntimeError(f'Missing Docker-provisioned Micromamba Python: {XR_PY}')
    if not ROS_PY.is_file() or not ROS_BASE.is_file() or not ROS_INSTALL.is_file():
        raise RuntimeError('ROS 2 Humble /workspace/install not available. Run colcon build and source ROS inside Docker.')
    paths = ['/tmp/g1_xr_readonly_state.sock' if args.mode in ('observe', 'hardware-preflight')
             else '/tmp/g1_xr_bridge_inprocess_sim.sock']
    if args.ftp_observe:
        paths.append('/tmp/g1_xr_ftp_readonly.sock')
    if any(os.path.lexists(p) for p in paths):
        raise RuntimeError('IPC socket already exists; another controller/observer may own it. Refusing to replace it.')
    if args.mode in ('observe', 'hardware-preflight', 'xr-sim'):
        iface = args.interface if args.mode in ('observe', 'hardware-preflight') else args.sim_interface
        if not (Path('/sys/class/net') / iface).exists():
            raise RuntimeError(f'Network interface missing: {iface}')


def supervise(commands):
    processes = []
    shutting_down = False

    def stop_all(*_):
        nonlocal shutting_down
        if shutting_down:
            return
        shutting_down = True
        print('\nStopping pipeline children...', flush=True)
        for _name, p in reversed(processes):
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass

    signal.signal(signal.SIGINT, stop_all)
    signal.signal(signal.SIGTERM, stop_all)
    try:
        for name, cmd, env in commands:
            if shutting_down:
                break
            print('Starting: ' + name, flush=True)
            p = subprocess.Popen(cmd, env=env, cwd=str(HERE), start_new_session=True)
            processes.append((name, p))
            time.sleep(0.15)
            if p.poll() is not None:
                raise RuntimeError(f'{name} exited immediately with code {p.returncode}')
        while not shutting_down:
            for name, p in processes:
                status = p.poll()
                if status is not None:
                    raise RuntimeError(f'{name} exited with code {status}')
            time.sleep(0.2)
    finally:
        stop_all()
        for _, p in reversed(processes):
            try:
                p.wait(timeout=4)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    p.wait()


def get_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--mode', choices=['offline', 'observe', 'hardware-preflight', 'xr-sim', 'hardware'], default='offline')
    parser.add_argument('--interface', help='Real G1 NIC; observe/hardware-preflight READ ONLY')
    parser.add_argument('--arm-selector-probe', action='store_true',
                        help='Hardware-preflight ONLY: dry-run arm selector against measured G1 LowState; no commands')
    parser.add_argument('--moveit-plan-shadow', action='store_true',
                        help='Hardware-preflight ONLY: inspect MoveIt Plan visualization against real arm pose; NEVER execute')
    parser.add_argument('--ftp-observe', action='store_true',
                        help='Hardware-preflight ONLY: subscribe to measured Inspire FTP hands read-only')
    parser.add_argument('--sim-interface', help='Isolated Isaac simulator DDS NIC (not connected to G1)')
    parser.add_argument('--xr-adapter', choices=['existing', 'minimal'], default='existing',
                        help='XR-SIM only: existing guarded overlay or minimal untouched-XR adapter')
    parser.add_argument('--isolation-ack', help='Must equal ISOLATED_SIMULATOR for xr-sim')
    parser.add_argument('--image-server-ip', default='127.0.0.1', help='XR camera server for isolated sim')
    parser.add_argument('--xr-ipc', action='store_true', help='Use upstream XR IPC instead of keyboard r/q')
    parser.add_argument('--no-moveit', action='store_true', help='Run state/ROS backend only')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--print-plan', action='store_true', help='Show child commands without launching or importing SDK')
    parser.add_argument('--verify', action='store_true', help='Run offline regression checks only')
    return parser.parse_args(argv)


def main(argv=None):
    args = get_args(argv)
    if args.verify:
        env = os.environ.copy()
        env['G1_XR_SOURCE_ROOT'] = str(XR)
        return subprocess.call(['bash', str(HERE/'run_xr_integrated_checks.sh')], cwd=str(HERE), env=env)
    try:
        commands = build_commands(args)
        if args.print_plan:
            for name, cmd, _ in commands:
                print(f'{name}: {cmd}')
            return 0
        preflight(args)
        print(f'G1 pipeline mode: {args.mode}. '
              + ('NO ACTUATORS' if args.mode != 'xr-sim'
                 else 'SIMULATOR ACTUATORS: confirm physically isolated network!'), flush=True)
        supervise(commands)
        return 0
    except (RuntimeError, ValueError, OSError) as exc:
        print(f'PIPELINE REFUSED/STOPPED: {exc}', file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
