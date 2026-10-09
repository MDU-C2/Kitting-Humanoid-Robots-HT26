#!/usr/bin/env python3
"""End-to-end ROS 2 + ORIGINAL XR offline action fault tests (NO ACTUATORS).

Run INSIDE the project's ROS Humble Docker, with all other G1 pipelines stopped:
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  python3 /workspace/src/g1_xr_bridge/run_offline_runtime_faults.py

Starts exclusively owned offline XR + action processes (no MoveIt, no G1 DDS),
checks cancellation/HOLD, concurrent-goal rejection, and XR child death/abort.
Does not modify upstream xr_teleoperate. Not a physical stopping test.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

# Require ROS 2: this is intentionally a separate integration test, NOT picked
# up by the generic unittest discovery (it would launch subprocesses).
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from xr_arm_trajectory import ARM_JOINT_NAMES
from xr_inprocess_bridge import SIM_SOCKET
from xr_ipc_protocol import IpcError, ping, transact

HERE = Path(__file__).resolve().parent
PIPELINE = HERE / 'g1_pipeline.py'
ACTION = '/g1_xr_bridge/inprocess_sim/follow_joint_trajectory'


def wait_future(node, future, seconds, label):
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
    if not future.done():
        raise AssertionError(f'timed out: {label} after {seconds}s')
    return future.result()


def goal(distance=0.25, seconds=12):
    message = FollowJointTrajectory.Goal()
    message.trajectory.joint_names = list(ARM_JOINT_NAMES)
    zero = JointTrajectoryPoint()
    zero.positions = [0.0] * 14
    finish = JointTrajectoryPoint()
    finish.positions = [0.0] * 14
    finish.positions[3] = distance
    finish.time_from_start.sec = seconds
    message.trajectory.points = [zero, finish]
    return message


def wait_ready(node, client, process, seconds=70):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if any(proc.poll() is not None for proc in process):
            raise AssertionError('owned XR or ROS process exited before readiness')
        if client.wait_for_server(timeout_sec=0.2):
            try:
                answer = ping(SIM_SOCKET)
                if answer.get('xr_loop_fresh') is True:
                    return
            except (IpcError, OSError):
                pass
        time.sleep(0.2)
    raise AssertionError('offline XR loop / ROS action not ready')


def status_source():
    reply = transact({'kind': 'status'}, SIM_SOCKET)
    if reply.get('kind') != 'status_ack' or reply.get('hardware_connected') is not False:
        raise AssertionError('status is not confirmed OFFLINE')
    return reply.get('source')


def wait_source(expected, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if status_source() == expected:
            return
        time.sleep(0.05)
    raise AssertionError(f'selector did not enter {expected}: got {status_source()}')


def scenario(name, check, log_directory):
    """Start only OUR synthetic XR harness and ROS action process, independently.

    Separate process ownership is essential for the XR-death scenario: the
    action server must stay alive long enough to explicitly report ABORTED.
    """
    if os.path.lexists(SIM_SOCKET):
        raise RuntimeError(f'XR IPC socket exists: {SIM_SOCKET}. Stop other pipelines FIRST.')
    if os.path.lexists('/tmp/g1_xr_readonly_state.sock'):
        raise RuntimeError('Real G1 observation socket exists. Refusing runtime test.')
    log_dir = Path(log_directory)
    xr_env = os.environ.copy()
    xr_env.pop('PYTHONPATH', None)
    xr_env.pop('G1_XR_SIM_ACK', None)
    xr_env['LD_LIBRARY_PATH'] = '/opt/mamba/envs/xr/lib'
    xr_env['G1_XR_SOURCE_ROOT'] = str(HERE.parent / 'xr_teleoperate')
    ros_env = os.environ.copy()
    ros_env.pop('G1_XR_SIM_ACK', None)
    xr_command = ['/opt/mamba/envs/xr/bin/python', str(HERE / 'xr_inprocess_offline_harness.py')]
    ros_command = ['/usr/bin/python3', str(HERE / 'ros2_trajectory_inprocess_sim.py')]
    processes = []
    owned_socket_identity = None
    try:
        with (log_dir / f'{name}_xr.log').open('wb') as xr_log, \
                (log_dir / f'{name}_ros.log').open('wb') as ros_log:
            xr = subprocess.Popen(xr_command, cwd=str(HERE), env=xr_env,
                                  stdout=xr_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
            processes.append(xr)
            ros = subprocess.Popen(ros_command, cwd=str(HERE), env=ros_env,
                                   stdout=ros_log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
            processes.append(ros)
            print(f'[{name}] spawned isolated XR pid={xr.pid}, ROS action pid={ros.pid}', flush=True)
            node = Node(f'g1_offline_fault_{name}')
            try:
                client = ActionClient(node, FollowJointTrajectory, ACTION)
                wait_ready(node, client, (xr, ros))
                # Save the identity of the socket created by OUR isolated XR.
                # Never unlink an unrelated path that may appear later.
                socket_stat = os.lstat(SIM_SOCKET)
                owned_socket_identity = (socket_stat.st_dev, socket_stat.st_ino)
                check(node, client, xr)
            finally:
                node.destroy_node()
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in reversed(processes):
            try:
                proc.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
        for _ in range(40):
            if not os.path.lexists(SIM_SOCKET):
                break
            time.sleep(0.1)
        if os.path.lexists(SIM_SOCKET):
            # SIGTERM intentionally simulates an abrupt XR failure. Such a
            # failure can leave a Unix socket *pathname* even though the
            # listening process is gone. This is not evidence of a live XR.
            # Do not hide a live listener or delete another process's socket.
            if not processes or processes[0].poll() is None:
                raise AssertionError('XR still alive while IPC socket remains')
            current_stat = os.lstat(SIM_SOCKET)
            identity = (current_stat.st_dev, current_stat.st_ino)
            if owned_socket_identity is None or identity != owned_socket_identity:
                raise AssertionError('IPC socket changed ownership; refusing cleanup')
            try:
                ping(SIM_SOCKET)
            except (IpcError, OSError):
                pass  # no responding offline XR receiver
            else:
                raise AssertionError('IPC socket still responds after XR exit')
            os.unlink(SIM_SOCKET)
            print(f'[{name}] removed stale socket pathname from terminated owned XR',
                  flush=True)
    print(f'[{name}] PASS', flush=True)

def accepted(node, client, seconds=12):
    handle = wait_future(node, client.send_goal_async(goal(seconds=seconds)),
                         8, 'goal response')
    if not handle.accepted:
        raise AssertionError('offline trajectory unexpectedly rejected')
    return handle


def cancel_test(node, client, proc):
    handle = accepted(node, client)
    wait_source('trajectory')
    cancel = wait_future(node, handle.cancel_goal_async(), 5, 'cancel response')
    if not cancel.goals_canceling:
        raise AssertionError('action server refused cancellation')
    result = wait_future(node, handle.get_result_async(), 7, 'cancel result')
    if result.status != GoalStatus.STATUS_CANCELED:
        raise AssertionError(f'cancel reported status {result.status} instead of CANCELED')
    wait_source('hold')
    time.sleep(0.5)
    if status_source() != 'hold':
        raise AssertionError('unsafe automatic VR resume after cancellation')


def simultaneous_test(node, client, proc):
    first = accepted(node, client)
    wait_source('trajectory')
    second = wait_future(node, client.send_goal_async(goal(-0.1, 12)),
                         8, 'concurrent goal response')
    if second.accepted:
        raise AssertionError('simultaneous trajectory ownership erroneously accepted')
    wait_future(node, first.cancel_goal_async(), 5, 'cleanup cancellation')
    result = wait_future(node, first.get_result_async(), 7, 'cleanup result')
    if result.status != GoalStatus.STATUS_CANCELED:
        raise AssertionError('cleanup cancellation failed')
    wait_source('hold')


def xr_death_test(node, client, proc):
    handle = accepted(node, client)
    wait_source('trajectory')
    # Send TERM only to the XR process WE spawned, never an arbitrary PID.
    os.killpg(proc.pid, signal.SIGTERM)
    result = wait_future(node, handle.get_result_async(), 8, 'XR process death action result')
    if result.status != GoalStatus.STATUS_ABORTED:
        raise AssertionError(f'XR death returned {result.status}, expected ABORTED')
    if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
        raise AssertionError('XR death falsely reported trajectory SUCCESSFUL')
    # Action server stays alive, independently, and must actively report ABORTED.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=['all', 'cancel', 'simultaneous', 'xr-death'],
                        default='all')
    parser.add_argument('--log-dir', default=None)
    args = parser.parse_args()
    if not PIPELINE.is_file():
        parser.error('g1_pipeline.py missing')
    log_dir = args.log_dir or tempfile.mkdtemp(prefix='g1_offline_faults_')
    Path(log_dir).mkdir(parents=True, exist_ok=True)
    rclpy.init()
    checks = [('cancel', cancel_test), ('simultaneous', simultaneous_test),
              ('xr-death', xr_death_test)]
    try:
        for name, check in checks:
            if args.scenario in ('all', name):
                scenario(name.replace('-', '_'), check, log_dir)
        print(f'PASS: offline ROS 2 fault scenarios; logs: {log_dir}', flush=True)
        return 0
    except Exception as exc:
        print(f'FAIL: {exc}; logs: {log_dir}', file=sys.stderr, flush=True)
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
