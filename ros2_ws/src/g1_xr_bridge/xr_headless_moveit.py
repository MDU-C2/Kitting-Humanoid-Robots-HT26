#!/usr/bin/env python3
"""Headless MoveIt -> ORIGINAL XR G1_29_ArmController, motion mode only.

Two-process boundary: ROS Humble action server -> local Unix socket -> Micromamba XR.
No VR, camera, hand control, ros2_control or second arm motor publisher.
Live actuation requires explicit operator confirmation AND the opt-in XR class patch.
Not hardware-validated. Supervise physical activation and release.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socketserver
import sys
import threading
import time

XR_ROOT = Path(os.environ.get('G1_XR_SOURCE_ROOT', '/workspace/src/xr_teleoperate'))
BRIDGE_ROOT = Path(__file__).resolve().parent
SOCKET = '/tmp/g1_xr_headless_moveit.sock'
PROTOCOL = 1
MAX_LINE = 16384


def verify():
    main = XR_ROOT / 'teleop' / 'teleop_hand_and_arm.py'
    arm = XR_ROOT / 'teleop' / 'robot_control' / 'robot_arm.py'
    for path in (main, arm):
        if not path.exists():
            raise RuntimeError(f'Missing original XR source: {path}')
    source = arm.read_text()
    if 'G1_HEADLESS_MOVEIT_PATCH_V1' not in source:
        raise RuntimeError('Opt-in headless safety patch not installed in original XR arm class')
    # G1_MOVEIT_DIRECT_BRIDGE_V1: launch only when the automatic direct-command
    # overlay is installed; never fall back to silently filtered trajectories.
    if 'G1_MOVEIT_DIRECT_COMMANDS_V1' not in source or 'def _headless_slew_step(' in source:
        raise RuntimeError('XR direct MoveIt command overlay absent or old limiter active')
    for anchor in ('class G1_29_ArmController:', 'def release_headless(self)',
                   'self.lowcmd_publisher.Write(self.msg)', 'def ctrl_dual_arm('):
        if anchor not in source:
            raise RuntimeError(f'XR source anchor missing: {anchor}')
    print('PASS: original XR motor publisher with opt-in headless startup/release patch')
    print('Mode: XR --motion; source: MoveIt via local socket; headset: not used')
    print('G1 headless execution: direct MoveIt time-parameterized targets (no extra XR slew or feedback-lead clamp)')
    print('No hardware or DDS initialized during --verify')


# G1_INSPIRE_FTP_GRAVITY_V1: headless-only gravity model; never edits original XR or VR.
def ftp_gravity_urdf(workspace):
    return workspace / 'src' / 'g1_description' / 'urdf' / 'g1_29dof_with_inspire_hand_ftp.urdf'


def build_inspire_ftp_reduced_model(pin, urdf_path, joint_names):
    """Use the MoveIt mechanical URDF and fix all non-arm DOFs at neutral.

    Frames, masses and inertias of attached Inspire FTP hands are retained
    in the reduced arm model. Pinocchio's joint ordering is strictly checked.
    No mesh geometry or ROS runtime is needed for dynamics.
    """
    import numpy as np
    if not urdf_path.is_file():
        raise RuntimeError(f'Missing Inspire FTP URDF: {urdf_path}')
    full = pin.buildModelFromUrdf(str(urdf_path))
    expected = tuple(joint_names)
    for name in expected:
        if not full.existJointName(name):
            raise RuntimeError(f'Inspire FTP URDF missing arm joint: {name}')
    to_lock = [i for i in range(1, full.njoints) if full.names[i] not in expected]
    # Pinocchio neutral() is 0 for the 1-DOF revolute joints of this G1 URDF.
    reduced = pin.buildReducedModel(full, to_lock, pin.neutral(full))
    actual = tuple(reduced.names[1:])
    if reduced.nq != 14 or reduced.nv != 14 or actual != expected:
        raise RuntimeError('Inspire FTP dynamics order does not match XR arm motor order: '
                           f'nq={reduced.nq} nv={reduced.nv} joints={actual}')
    data = reduced.createData()
    zeros = np.zeros(14)
    for q in (zeros, np.array([-0.7, 0.2, 0, 0.3, 0, 0, 0,
                              0.3, -0.2, 0, 1.0, 0, 0, 0])):
        torque = np.asarray(pin.rnea(reduced, data, q, zeros, zeros), dtype=float)
        if torque.shape != (14,) or not np.isfinite(torque).all():
            raise RuntimeError('Inspire FTP gravity model produced invalid torque')
    return reduced


def check_inspire_ftp_gravity(pin, urdf_path, joint_names):
    """Offline, no-DDS model check. Prints sample torques for review."""
    import numpy as np
    model = build_inspire_ftp_reduced_model(pin, urdf_path, joint_names)
    data = model.createData()
    zero = np.zeros(14)
    for description, q in (
        ('neutral', zero),
        ('example left-shoulder-down', np.array([-0.7, 0.2, 0, 0.3, 0, 0, 0,
                                               0.3, -0.2, 0, 1.0, 0, 0, 0])),
    ):
        tau = np.asarray(pin.rnea(model, data, q, zero, zero), dtype=float)
        print(f'FTP gravity model check: {description}; '
              f'left_shoulder_pitch={tau[0]:+.3f} Nm; left_elbow={tau[3]:+.3f} Nm; '
              f'max |tau|={np.max(np.abs(tau)):.3f} Nm', flush=True)
    print(f'PASS: Pinocchio 14-arm Inspire FTP gravity model: {urdf_path}', flush=True)
    print('No DDS or motor controller initialized; URDF parameters not hardware-verified.', flush=True)


class Control:
    def __init__(self, arm, gravity):
        import numpy as np
        self.np = np
        self.arm = arm
        self.gravity = gravity
        self.lock = threading.RLock()
        self.last_q = arm.get_current_dual_arm_q().copy()
        self.source = 'hold'
        self.last_sample_at = None
        self.last_seq = -1
        self.started = time.monotonic()
        self.active = True
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self._tick, daemon=True)
        self.worker.start()

    def measured(self):
        now = time.monotonic()
        if self.arm.lowstate_rx_at is None or now - self.arm.lowstate_rx_at > 0.15:
            raise RuntimeError('Physical G1 LowState stale >150 ms')
        q = self.arm.get_current_dual_arm_q()
        dq = self.arm.get_current_dual_arm_dq()
        if q.shape != (14,) or dq.shape != (14,) or not self.np.isfinite(q).all() or not self.np.isfinite(dq).all():
            raise RuntimeError('Invalid physical 14-arm feedback')
        return q, dq

    def status(self):
        with self.lock:
            q, dq = self.measured()
            now = time.monotonic()
            with self.arm.ctrl_lock:
                motor_target = self.np.asarray(self.arm.q_target, dtype=float).copy()
            # XR_LIMITED_COMMAND_STATUS_V1: read-only snapshot of the latest
            # slew/lead-clipped q. This is not a motor-drive acknowledgement.
            # Publisher assigns a fresh NumPy array rather than mutating it.
            limited_command = self.np.asarray(self.arm._headless_command_q, dtype=float).copy()
            if (limited_command.shape != (14,) or
                    not self.np.isfinite(limited_command).all()):
                raise RuntimeError('Invalid XR limited-command diagnostic state')
            selected_error = float(self.np.max(self.np.abs(self.last_q-q)))
            # G1_READONLY_LOW_CMD_TELEMETRY_V1: inspect the outgoing command
            # buffer only; no motor writes or control decisions here.
            arm_motor_ids = range(15, 29)
            outgoing_q = [float(self.arm.msg.motor_cmd[i].q) for i in arm_motor_ids]
            outgoing_tau = [float(self.arm.msg.motor_cmd[i].tau) for i in arm_motor_ids]
            outgoing_kp = [float(self.arm.msg.motor_cmd[i].kp) for i in arm_motor_ids]
            return dict(kind='status_ack', hardware_connected=True, mode='hardware',
                        selected_arm_target=self.last_q.tolist(),
                        xr_motor_target=motor_target.tolist(),
                        xr_limited_command_q=limited_command.tolist(),
                        xr_lowcmd_q=outgoing_q, xr_lowcmd_tau=outgoing_tau,
                        xr_lowcmd_kp=outgoing_kp,
                        xr_execution_mode='direct_moveit_trajectory',
                        xr_extra_speed_shaping_active=False,
                        xr_headless_speed_limit_rad_s=float(self.arm.arm_velocity_limit),
                        xr_headless_max_lead_rad=float(self.arm._headless_max_command_lead_rad),
                        status_monotonic_s=now,
                        selected_target_delta_rad=round(selected_error, 5),
                        motor_target_delta_rad=round(float(self.np.max(self.np.abs(motor_target-q))), 5),
                        xr_loop_fresh=self.active and self.worker.is_alive(),
                        feedback_fresh=True, mode_machine=self.arm.mode_machine,
                        measured_arm_positions=q.tolist(), measured_arm_velocities=dq.tolist(),
                        source=self.source, weight_ramp_complete=(now-self.started >= 3.5),
                        heartbeat_age_s=None if self.last_sample_at is None else now-self.last_sample_at)

    def handle(self, msg):
        if not isinstance(msg, dict) or msg.get('protocol') != PROTOCOL:
            raise ValueError('Invalid protocol')
        kind = msg.get('kind')
        if kind == 'ping':
            self.measured()
            return dict(kind='pong', hardware_connected=True, mode='hardware')
        if kind == 'status':
            return self.status()
        if kind != 'sample':
            raise ValueError('Only samples and read-only status accepted')
        q = self.np.asarray(msg.get('q'), dtype=float)
        source = msg.get('source')
        seq = msg.get('seq')
        if q.shape != (14,) or not self.np.isfinite(q).all() or source not in ('trajectory', 'hold'):
            raise ValueError('Invalid 14 arm targets/source')
        # G1_BOUNDED_WRIST_CORRECTION_V1: retain an approved final target
        # *only* for wrist correction in hardware-supervised opt-in mode.
        retain_target = msg.get('retain_final_target', False)
        if type(retain_target) is not bool:
            raise ValueError('retain_final_target must be boolean')
        if retain_target and (source != 'hold' or
                os.environ.get('G1_WRIST_CORRECTION_MODE', 'preview') != 'execute' or
                os.environ.get('G1_WRIST_CORRECTION_OPERATOR_ACK') != 'G1_WRIST_CORRECTION_SUPERVISED'):
            raise ValueError('Retained HOLD requires explicitly approved wrist correction mode')
        if type(seq) is not int or seq < 0:
            raise ValueError('Invalid sequence')
        with self.lock:
            if not self.active:
                raise RuntimeError('Controller no longer active')
            measured_q, _ = self.measured()
            if time.monotonic() - self.started < 3.5:
                raise RuntimeError('Motion-mode authority ramp not yet complete')
            if source == 'trajectory':
                if self.source != 'trajectory' and self.np.max(self.np.abs(q-measured_q)) > 0.05:
                    raise ValueError('First trajectory target >0.05rad from actual arms')
                if self.source == 'trajectory' and seq <= self.last_seq:
                    raise ValueError('Out-of-order trajectory sample')
                if self.np.max(self.np.abs(q-measured_q)) > 0.15:
                    raise ValueError('Physical tracking deviation >0.15rad')
            else:
                if retain_target:
                    if self.source != 'trajectory':
                        raise ValueError('Retained HOLD only allowed after controlled trajectory')
                    if self.np.max(self.np.abs(q-measured_q)) > 0.05:
                        raise ValueError('Retained HOLD target farther than 0.05rad from measured')
                    if self.np.max(self.np.abs(q-self.last_q)) > 0.003:
                        raise ValueError('Retained HOLD cannot jump from the last trajectory target')
                    # The action server validates URDF bounds and per-goal
                    # correction offset. XR itself rejects retained hold jumps.
                else:
                    # Original XR behavior for ordinary success, cancel, abort:
                    # HOLD captures current physical joint positions.
                    q = measured_q.copy()
            self.source = source
            self.last_q = q.copy()
            self.last_seq = seq
            self.last_sample_at = time.monotonic()
            return dict(kind='sample_ack', source=source, seq=seq, tau_ff=self.gravity(q).tolist(),
                        hardware_connected=True, selected_source=source, executed=False)

    def _tick(self):
        while not self.stop_event.is_set():
            try:
                with self.lock:
                    actual, _ = self.measured()
                    now = time.monotonic()
                    if self.source == 'trajectory' and (self.last_sample_at is None or
                                                         now-self.last_sample_at > 0.25):
                        self.last_q = actual.copy()
                        self.source = 'hold'
                    q = self.last_q.copy()
                    if self.source == 'hold':
                        # A hold captures measured pose once, rather than chasing gravity/noise.
                        pass
                    tau = self.gravity(q)
                    self.arm.ctrl_dual_arm(q, tau)
            except Exception as exc:
                print('HEADLESS CONTROL FAULT:', exc, file=sys.stderr, flush=True)
                self.active = False
                self.stop_event.set()
                try:
                    self.arm.release_headless()
                except Exception as release_exc:
                    print('HEADLESS RELEASE FAULT:', release_exc, file=sys.stderr, flush=True)
                break
            self.stop_event.wait(0.02)

    def shutdown(self):
        self.active = False
        self.stop_event.set()
        self.worker.join(timeout=1)
        self.arm.release_headless()


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            raw = self.rfile.readline(MAX_LINE+1)
            if not raw.endswith(b'\n') or len(raw) > MAX_LINE:
                raise ValueError('Malformed IPC request')
            result = self.server.control.handle(json.loads(raw))
            reply = dict(protocol=PROTOCOL, ok=True, **result)
        except Exception as exc:
            reply = dict(protocol=PROTOCOL, ok=False, error=str(exc))
        self.wfile.write((json.dumps(reply, allow_nan=False)+'\n').encode())


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    allow_reuse_address = False


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--verify', action='store_true')
    p.add_argument('--hardware', action='store_true')
    p.add_argument('--interface', default='enx50a03000b0dc')
    p.add_argument('--operator-ack', default='')
    # G1_MOVEIT_YAML_AUTHORITY_V1: speed comes from MoveIt joint_limits.yaml.
    p.add_argument('--arm-speed-rad-s', type=float, default=0.25)
    p.add_argument('--gravity-model', choices=('xr', 'inspire_ftp'), default='xr',
                   help='Headless arm gravity model; XR remains the default until explicitly selected')
    p.add_argument('--gravity-check', action='store_true',
                   help='Validate Inspire FTP Pinocchio dynamics offline (no DDS/motor publisher)')
    args = p.parse_args()
    if not (math.isfinite(args.arm_speed_rad_s) and args.arm_speed_rad_s > 0):
        p.error('--arm-speed-rad-s must be positive and finite')
    verify()
    if args.verify:
        return
    if args.gravity_check:
        # No hardware/network initialization; do this before any physical activation.
        import pinocchio as pin
        from xr_arm_trajectory import ARM_JOINT_NAMES
        workspace = Path(os.environ.get('G1_WORKSPACE', str(BRIDGE_ROOT.parent.parent)))
        check_inspire_ftp_gravity(pin, ftp_gravity_urdf(workspace), ARM_JOINT_NAMES)
        return
    if not args.hardware or args.operator_ack != 'G1_REGULAR_MODE_SUPERVISED':
        raise SystemExit('Hardware startup requires --hardware --operator-ack G1_REGULAR_MODE_SUPERVISED')
    if os.path.lexists(SOCKET):
        raise SystemExit(f'Socket exists: {SOCKET}; do not overwrite an existing controller')
    if not Path('/sys/class/net', args.interface).exists():
        raise SystemExit(f'Missing interface {args.interface}')
    sys.path.insert(0, str(XR_ROOT))
    # Uses original XR Python env (including Pinocchio+Unitree). Does not import ROS.
    import numpy as np
    import pinocchio as pin
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from teleop.robot_control.robot_arm import G1_29_ArmController

    # This joint ordering is required, otherwise force feedforward would be wrong.
    from xr_arm_trajectory import ARM_JOINT_NAMES
    if args.gravity_model == 'inspire_ftp':
        # Only headless MoveIt uses this new model. Original XR headset IK
        # and its serialized model cache are never touched.
        workspace = Path(os.environ.get('G1_WORKSPACE', str(BRIDGE_ROOT.parent.parent)))
        urdf = ftp_gravity_urdf(workspace)
        model = build_inspire_ftp_reduced_model(pin, urdf, ARM_JOINT_NAMES)
        print(f'GRAVITY MODEL SELECTED: inspire_ftp; URDF={urdf}', flush=True)
    else:
        from teleop.robot_control.robot_arm_ik import G1_29_ArmIK
        ik = G1_29_ArmIK()
        model = ik.reduced_robot.model
        print('GRAVITY MODEL SELECTED: original XR (unchanged)', flush=True)
    if model.nq != 14 or model.nv != 14 or tuple(model.names[1:]) != ARM_JOINT_NAMES:
        raise RuntimeError('XR gravity model joint order does not match MoveIt 14-arm order')
    data = model.createData()
    zeros = np.zeros(14)
    gravity_lock = threading.Lock()

    def gravity(q):
        with gravity_lock:
            return pin.rnea(model, data, np.asarray(q), zeros, zeros).copy()

    # No XR camera, TeleVuer, VR input, or Inspire hand control is instantiated.
    # Explicit regular/motion mode: original SDK arm_sdk path, not lowcmd debug.
    ChannelFactoryInitialize(0, networkInterface=args.interface)
    ctrl = G1_29_ArmController(motion_mode=True, simulation_mode=False,
                               headless_moveit=True, gravity_fn=gravity)
    control = None
    server = None
    try:
        # This sets the legacy XR metadata only; direct headless commands are NOT clipped.
        ctrl.set_arm_velocity_limit(args.arm_speed_rad_s)
        print(f'XR direct MoveIt trajectory mode; YAML max velocity: {args.arm_speed_rad_s:.3f} rad/s (no XR headless shaping)', flush=True)
        control = Control(ctrl, gravity)
        server = Server(SOCKET, Handler)
        os.chmod(SOCKET, 0o600)
        server.control = control
        terminate = threading.Event()
        def shutdown_signal(signum, frame):
            terminate.set()
        signal.signal(signal.SIGTERM, shutdown_signal)
        signal.signal(signal.SIGINT, shutdown_signal)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print('PHYSICAL ARM SDK ACTIVE: headless MoveIt bridge; no VR; Ctrl-C requests controlled release', flush=True)
        while not terminate.wait(0.1):
            if not control.active or not ctrl.publish_thread.is_alive():
                raise RuntimeError('XR motor publisher or control heartbeat stopped')
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        if control is not None:
            control.shutdown()
        else:
            ctrl.release_headless()
        if os.path.lexists(SOCKET):
            os.unlink(SOCKET)
        print('Headless XR: shutdown completed; motor release is not a guaranteed physical emergency stop', flush=True)


if __name__ == '__main__':
    main()
