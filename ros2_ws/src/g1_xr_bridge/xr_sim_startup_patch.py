"""In-memory, simulation-only startup instrumentation for original XR G1 arm.

It reuses XR's ORIGINAL G1_29_ArmController. The upstream checkout is unchanged.
A tested mock verifies measured q and tau; sim guard starts weight slot 29 at zero.
Do not use this patch to enable physical actuation.
"""
import ast
import copy
import os
from pathlib import Path

STARTUP_ANCHOR = '        self.all_motor_q = self.get_current_motor_q()\n'
INIT_FEEDBACK_ANCHOR = '        self.lowstate_sub_ready = False\n'
RX_FEEDBACK_ANCHOR = '                self.lowstate_sub_ready = True\n'


def original_arm_path():
    return (Path(os.environ.get('G1_XR_SOURCE_ROOT', '/workspace/src/xr_teleoperate'))
            / 'teleop' / 'robot_control' / 'robot_arm.py')


def _class_span(source):
    tree = ast.parse(source)
    matches = [n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'G1_29_ArmController']
    if len(matches) != 1:
        raise RuntimeError('Expected exactly one pinned G1_29_ArmController')
    return matches[0]


def _once(segment, anchor, replacement, name):
    count = segment.count(anchor)
    if count != 1:
        raise RuntimeError(f'XR source drift at {name}: {count} anchors')
    return segment.replace(anchor, replacement, 1)


def patch_controller_source(source):
    """Patch original controller *source text in memory* (SIM ONLY)."""
    cls = _class_span(source)
    lines = source.splitlines(keepends=True)
    segment = ''.join(lines[cls.lineno - 1:cls.end_lineno])
    segment = _once(segment,
                    '    def __init__(self, motion_mode = False, simulation_mode = False):\n',
                    '    def __init__(self, motion_mode = False, simulation_mode = False, *, bridge_gravity_fn=None):\n',
                    'constructor signature')
    segment = _once(segment, INIT_FEEDBACK_ANCHOR,
                    INIT_FEEDBACK_ANCHOR +
                    '        self.bridge_last_lowstate_at = None\n'
                    '        self.bridge_lowstate_seq = 0\n', 'feedback fields')
    segment = _once(segment, RX_FEEDBACK_ANCHOR,
                    RX_FEEDBACK_ANCHOR +
                    '                self.bridge_last_lowstate_at = time.monotonic()\n'
                    '                self.bridge_lowstate_seq += 1\n', 'new LowState receipt')
    segment = _once(segment, STARTUP_ANCHOR, STARTUP_ANCHOR +
                    '        # SIMULATION ONLY: seed from actual startup state BEFORE publish thread.\n'
                    '        if bridge_gravity_fn is not None:\n'
                    '            measured_arm_q = self.get_current_dual_arm_q()\n'
                    '            measured_arm_dq = self.get_current_dual_arm_dq()\n'
                    '            if (measured_arm_q.shape != (14,) or\n'
                    '                    measured_arm_dq.shape != (14,) or\n'
                    '                    not np.isfinite(measured_arm_q).all() or\n'
                    '                    not np.isfinite(measured_arm_dq).all()):\n'
                    '                raise RuntimeError("Invalid XR startup feedback")\n'
                    '            gravity_tau = np.asarray(bridge_gravity_fn(measured_arm_q.copy()),\n'
                    '                                     dtype=float)\n'
                    '            if gravity_tau.shape != (14,) or not np.isfinite(gravity_tau).all():\n'
                    '                raise RuntimeError("Invalid XR initial gravity feedforward")\n'
                    '            self.q_target = measured_arm_q.copy()\n'
                    '            self.tauff_target = gravity_tau.copy()\n',
                    'initial q and gravity torque')
    segment = _once(segment,
                    '        self.tauff_target = gravity_tau.copy()\n',
                    '        self.tauff_target = gravity_tau.copy()\n'
                    '            from xr_motion_guard import XrMotionGuard\n'
                    '            self.bridge_motion_guard = XrMotionGuard()\n',
                    'initialize simulator weight guard')
    segment = _once(segment,
                    '            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0;\n',
                    '            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = ('
                    '0.0 if hasattr(self, "bridge_motion_guard") else 1.0)\n',
                    'block immediate weight=1')
    segment = _once(segment,
                    '            self.lowcmd_publisher.Write(self.msg)\n',
                    '            if hasattr(self, "bridge_motion_guard"):\n'
                    '                self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = (\n'
                    '                    self.bridge_motion_guard.step(\n'
                    '                        self.bridge_last_lowstate_at if\n'
                    '                        self.mode_machine == self.msg.mode_machine else None))\n'
                    '                self.msg.crc = self.crc.Crc(self.msg)\n'
                    '            self.lowcmd_publisher.Write(self.msg)\n',
                    'publisher blend ramp & watchdog')
    if not (segment.index('self.tauff_target = gravity_tau.copy()') <
            segment.index('self.publish_thread.start()')):
        raise RuntimeError('Seed must execute before XR publish thread')
    result = ''.join(lines[:cls.lineno - 1]) + segment + ''.join(lines[cls.end_lineno:])
    compile(result, 'XR robot_arm.py (in-memory simulation copy)', 'exec')
    return result


def install_sim_controller_patch(controller_cls, gravity_fn=None):
    """Patch original controller startup, subscription and publisher methods in memory.

    The original class remains the SDK publisher and controller. Must only be
    called by guarded XR --sim --motion path, before controller construction.
    """
    if getattr(controller_cls, '_g1_integrated_sim_patch', False):
        raise RuntimeError('G1 sim controller patch already installed')
    path = original_arm_path()
    if not path.is_file():
        raise RuntimeError(f'Original XR robot_arm.py missing: {path}')
    original = path.read_text(encoding='utf-8')
    patched = patch_controller_source(original)
    cls = _class_span(patched)
    funcs = {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
    wanted = ('__init__', '_subscribe_motor_state', '_ctrl_motor_state')
    if any(name not in funcs for name in wanted):
        raise RuntimeError('Original XR controller methods not found')
    namespace = {}
    methods = ast.Module(body=[copy.deepcopy(funcs[n]) for n in wanted], type_ignores=[])
    ast.fix_missing_locations(methods)
    # Correct original XR module globals, including DDS/SDK objects. No new
    # controller, no file mutation. SDK import has already occurred upstream.
    exec(compile(methods, str(path) + ' (sim-only)', 'exec'),
         controller_cls.__init__.__globals__, namespace)
    controller_cls.__init__ = namespace['__init__']
    controller_cls._subscribe_motor_state = namespace['_subscribe_motor_state']
    # The guarded publisher is essential: without this assignment the old
    # XR method still writes weight slot 29 at its default full blend.
    controller_cls._ctrl_motor_state = namespace['_ctrl_motor_state']
    controller_cls._g1_integrated_sim_patch = True
    return {'controller': controller_cls.__name__, 'original_file_unchanged': True,
            'hardware_authorized': False}


def check_feedback_fresh(controller, *, now=None, max_age_s=0.25):
    """Simulation fail-closed freshness check based on actual DDS read events.

    A getter being called is NOT evidence of new LowState feedback.
    """
    import math
    import time
    current = time.monotonic() if now is None else float(now)
    received = getattr(controller, 'bridge_last_lowstate_at', None)
    sequence = getattr(controller, 'bridge_lowstate_seq', 0)
    if (type(sequence) is not int or sequence < 1 or received is None or
            not math.isfinite(current) or not math.isfinite(float(received)) or
            current < received or current - received > max_age_s):
        raise RuntimeError('New XR LowState feedback missing or stale; SIM stopped')
    return True
