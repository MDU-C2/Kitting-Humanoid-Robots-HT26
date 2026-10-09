"""OFFLINE source-accurate startup review for pinned XR G1_29_ArmController.

Loads the original class from the existing XR checkout, in a restricted fake
namespace. NEVER imports Unitree DDS, creates real publishers, or touches hardware.
The candidate patch is IN MEMORY ONLY and never written to xr_teleoperate.

This demonstrates an actual initial-command discontinuity and reviews a possible
way to initialize q and gravity feedforward from freshly observed arm state.
It DOES NOT solve motion-mode blend ramp, ownership, state age or stop safety.
"""
import ast
from pathlib import Path
from types import SimpleNamespace
import os
import threading
import numpy as np
from enum import IntEnum

DEFAULT_PATH = '/workspace/src/xr_teleoperate/teleop/robot_control/robot_arm.py'


def original_path():
    return Path(os.environ.get('G1_XR_ROBOT_ARM_PATH', DEFAULT_PATH))


def load_source():
    path = original_path()
    if not path.is_file():
        raise RuntimeError(f'Pinned XR robot_arm.py missing: {path}')
    return path.read_text(encoding='utf-8')


def _class_segment(source):
    tree = ast.parse(source)
    klass = next((n for n in tree.body if isinstance(n, ast.ClassDef)
                  and n.name == 'G1_29_ArmController'), None)
    if klass is None:
        raise RuntimeError('Original XR G1_29_ArmController missing')
    lines = source.splitlines(keepends=True)
    return ''.join(lines[klass.lineno-1:klass.end_lineno]), klass.lineno, klass.end_lineno


def patch_source_for_review(source):
    """Opt-in candidate initialization; unchanged XR default; memory only."""
    segment, start, end = _class_segment(source)
    before = '    def __init__(self, motion_mode = False, simulation_mode = False):\n'
    after = ('    def __init__(self, motion_mode = False, simulation_mode = False, '
             '*, bridge_gravity_fn=None):\n')
    seed = '        self.all_motor_q = self.get_current_motor_q()\n'
    replacement = (seed +
        '        # REVIEW ONLY: optional candidate seed before publish thread starts.\n'
        '        if bridge_gravity_fn is not None:\n'
        '            measured_arm_q = self.get_current_dual_arm_q()\n'
        '            measured_arm_dq = self.get_current_dual_arm_dq()\n'
        '            if (measured_arm_q.shape != (14,) or\n'
        '                    measured_arm_dq.shape != (14,) or\n'
        '                    not np.isfinite(measured_arm_q).all() or\n'
        '                    not np.isfinite(measured_arm_dq).all()):\n'
        '                raise RuntimeError("Invalid initial XR arm feedback")\n'
        '            gravity_tau = np.asarray(\n'
        '                bridge_gravity_fn(measured_arm_q.copy()), dtype=float)\n'
        '            if gravity_tau.shape != (14,) or not np.isfinite(gravity_tau).all():\n'
        '                raise RuntimeError("Invalid initial XR gravity feedforward")\n'
        '            self.q_target = measured_arm_q.copy()\n'
        '            self.tauff_target = gravity_tau.copy()\n')
    if segment.count(before) != 1 or segment.count(seed) != 1:
        raise RuntimeError('Pinned XR source changed; refusing candidate patch')
    segment = segment.replace(before, after, 1).replace(seed, replacement, 1)
    if segment.index('self.tauff_target = gravity_tau.copy()') > segment.index('self.publish_thread.start()'):
        raise RuntimeError('Initialization must precede publisher thread')
    lines = source.splitlines(keepends=True)
    patched = ''.join(lines[:start-1]) + segment + ''.join(lines[end:])
    compile(patched, str(original_path()), 'exec')
    return patched


class _FinishedOnePublish(Exception):
    pass


class _FakePublisher:
    def __init__(self, topic, _msg_type):
        self.topic = topic
        self.commands = []
    def Init(self):
        pass
    def Write(self, msg):
        # Snapshot current command, the real publisher is NEVER imported.
        self.commands.append([(motor.q, motor.tau) for motor in msg.motor_cmd])
        self.last_weight = msg.motor_cmd[29].q


class _FakeSubscriber:
    def __init__(self, topic, _msg_type):
        self.topic = topic
    def Init(self):
        pass


class _FakeCRC:
    def Crc(self, _msg):
        return 0


class _FakeCmd:
    def __init__(self):
        self.q = 0.0
        self.tau = 0.0
        self.dq = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.mode = 0


class _FakeLowCmd:
    def __init__(self):
        self.motor_cmd = [_FakeCmd() for _ in range(35)]
        self.mode_pr = 0
        self.mode_machine = 0
        self.crc = 0


class _FakeThread:
    """Never schedules a real background thread; preloads synthetic LowState."""
    def __init__(self, target, *args, **kwargs):
        self.target = target
        self.daemon = False
    def start(self):
        if self.target.__name__ != '_subscribe_motor_state':
            return  # motor thread NEVER runs automatically
        instance = self.target.__self__
        q, dq = instance._offline_feedback
        lowstate = instance._offline_lowstate_type()
        for idx in range(35):
            lowstate.motor_state[idx].q = q[idx]
            lowstate.motor_state[idx].dq = dq[idx]
        instance.lowstate_buffer.SetData(lowstate)
        instance.mode_machine = 1
        instance.lowstate_sub_ready = True


class _FakeClock:
    @staticmethod
    def time():
        return 1.0
    @staticmethod
    def sleep(_):
        raise _FinishedOnePublish()


def mock_controller(source, *, q, dq=None, gravity_fn=None, patched=False):
    """Execute one ORIGINAL 250Hz publisher iteration with fake DDS objects."""
    q = np.asarray(q, dtype=float)
    dq = np.zeros(35) if dq is None else np.asarray(dq, dtype=float)
    if q.shape != (35,) or dq.shape != (35,):
        raise ValueError('Expected 35 fake motor states')
    if patched:
        source = patch_source_for_review(source)
    root = ast.parse(source)
    names = {'MotorState', 'G1_29_LowState', 'DataBuffer',
             'G1_29_ArmController', 'G1_29_JointArmIndex', 'G1_29_JointIndex'}
    nodes = [node for node in root.body if isinstance(node, ast.ClassDef)
             and node.name in names]
    if len(nodes) != len(names):
        raise RuntimeError('Pinned XR class definitions have changed')
    class _ThreadFactory:
        def __call__(self, target, *args, **kwargs):
            thread = _FakeThread(target, *args, **kwargs)
            if target.__name__ == '_subscribe_motor_state':
                # Attach the synthetic feedback before starting the mock thread.
                target.__self__._offline_feedback = (q, dq)
                target.__self__._offline_lowstate_type = namespace['G1_29_LowState']
            return thread

    namespace = {
        'np': np, 'IntEnum': IntEnum,
        'threading': SimpleNamespace(Lock=threading.Lock, Thread=_ThreadFactory()),
        'time': _FakeClock,
        'ChannelPublisher': _FakePublisher,
        'ChannelSubscriber': _FakeSubscriber,
        'hg_LowCmd': object, 'hg_LowState': object,
        'unitree_hg_msg_dds__LowCmd_': _FakeLowCmd,
        'CRC': _FakeCRC,
        'kTopicLowCommand_Motion': 'rt/arm_sdk',
        'kTopicLowCommand_Debug': 'rt/lowcmd',
        'kTopicLowState': 'rt/lowstate',
        'G1_29_Num_Motors': 35,
        'wait_for_dds': lambda predicate, _name: (
            None if predicate() else (_ for _ in ()).throw(RuntimeError('Fake DDS missing'))),
        'logger_mp': SimpleNamespace(info=lambda *a, **k: None,
                                     debug=lambda *a, **k: None),
    }
    isolated = ast.Module(body=nodes, type_ignores=[])
    isolated = ast.fix_missing_locations(isolated)
    exec(compile(isolated, '<isolated pinned XR classes / fake SDK>', 'exec'), namespace)
    kwargs = {'motion_mode': True, 'simulation_mode': False}
    if patched:
        kwargs['bridge_gravity_fn'] = gravity_fn
    ctrl = namespace['G1_29_ArmController'](**kwargs)
    try:
        ctrl._ctrl_motor_state()
    except _FinishedOnePublish:
        pass
    else:
        raise AssertionError('Fake publisher did not finish one iteration')
    pub = ctrl.lowcmd_publisher
    if len(pub.commands) != 1:
        raise AssertionError('Expected exactly one fake publish')
    return SimpleNamespace(controller=ctrl, topic=pub.topic,
                           q=np.asarray([cmd[0] for cmd in pub.commands[0][15:29]]),
                           tau=np.asarray([cmd[1] for cmd in pub.commands[0][15:29]]),
                           blend_weight=pub.last_weight)


def main():
    source = load_source()
    fake_q = np.zeros(35)
    fake_q[18] = .4
    fake_q[25] = -.2
    gravity = lambda q: np.full(14, 0.5)  # mock, not actual Pinocchio
    original = mock_controller(source, q=fake_q)
    candidate = mock_controller(source, q=fake_q, gravity_fn=gravity, patched=True)
    print('READ-ONLY ORIGINAL XR CONTROLLER EMULATION (fake SDK, fake LowState)')
    print(f'Original left elbow initial command: {original.q[3]:+.4f} rad '
          f'(feedback: {fake_q[18]:+.4f} rad)')
    print(f'Candidate left elbow initial command: {candidate.q[3]:+.4f} rad '
          f'(feedback: {fake_q[18]:+.4f} rad)')
    print(f'Candidate initial torque (MOCK): {candidate.tau[3]:+.4f} Nm')
    print(f'Both use motion topic: {original.topic} / {candidate.topic}')
    print(f'Motion blending weight: {original.blend_weight:.3f} / '
          f'{candidate.blend_weight:.3f} (STILL IMMEDIATE 1.0: NOT SAFE FOR HARDWARE)')
    print('Original XR source modified: NO')
    print('Hardware commands: NONE. Fake publisher captures in-memory motor commands.')


if __name__ == '__main__':
    main()
