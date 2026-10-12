#!/usr/bin/env python3
"""Idempotent, pinned patch: add opt-in safe-start/headless controls to XR G1_29 class.

Stock VR control remains default (headless_moveit=False). Does NOT run motors.
"""
import argparse
from pathlib import Path
import shutil
import sys


def patch(source):
    if 'G1_HEADLESS_MOVEIT_PATCH_V1' in source:
        return source
    before_class, separator, remainder = source.partition('class G1_29_JointArmIndex')
    if not separator:
        raise RuntimeError('Missing G1_29 class terminator')
    source = before_class
    anchors = [
        ('    def __init__(self, motion_mode = False, simulation_mode = False):\n'
         '        logger_mp.info("Initialize G1_29_ArmController...")',
         '''    def __init__(self, motion_mode = False, simulation_mode = False, *, headless_moveit=False, gravity_fn=None):
        # G1_HEADLESS_MOVEIT_PATCH_V1. Default XR/Quest behavior is unchanged.
        if headless_moveit and (not motion_mode or simulation_mode or gravity_fn is None):
            raise ValueError('Headless physical controller requires motion mode and gravity callback')
        self.headless_moveit = bool(headless_moveit)
        self._headless_gravity = gravity_fn
        self._headless_running = True
        self._headless_release_at = None
        self._headless_release_weight = 0.0
        self._headless_last_target_at = None
        self._headless_initial_at = None
        self._headless_target_timeout_s = 0.35
        self.lowstate_rx_at = None
        logger_mp.info("Initialize G1_29_ArmController...")'''),
        ('                self.lowstate_buffer.SetData(lowstate)\n                self.mode_machine = msg.mode_machine',
         '''                self.lowstate_buffer.SetData(lowstate)
                self.lowstate_rx_at = time.monotonic()
                self.mode_machine = msg.mode_machine'''),
        ('        # initialize publish thread\n        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)',
         '''        # Headless: initialize BOTH desired joint state and gravity BEFORE publisher begins.
        # This opt-in branch is not executed by normal Quest/XR teleoperation.
        if self.headless_moveit:
            initial_q = self.get_current_dual_arm_q()
            initial_tau = np.asarray(self._headless_gravity(initial_q), dtype=float)
            if (initial_q.shape != (14,) or initial_tau.shape != (14,) or
                    not np.isfinite(initial_q).all() or not np.isfinite(initial_tau).all()):
                raise RuntimeError('Unsafe initial XR feedback/gravity values')
            if self.lowstate_rx_at is None or time.monotonic() - self.lowstate_rx_at > 0.15:
                raise RuntimeError('LowState stale before XR headless publisher startup')
            self.q_target = initial_q.copy()
            self.tauff_target = initial_tau.copy()
            self.set_arm_velocity_limit(0.25)
            self._headless_initial_at = time.monotonic()
            self._headless_last_target_at = self._headless_initial_at
        # initialize publish thread
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)'''),
        ('        if self.motion_mode:\n            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0;\n\n        while True:',
         '''        if self.motion_mode and not self.headless_moveit:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0;

        while not (self.headless_moveit and not self._headless_running):'''),
        ('            with self.ctrl_lock:\n                arm_q_target     = self.q_target\n                arm_tauff_target = self.tauff_target',
         '''            if self.headless_moveit:
                now = time.monotonic()
                # A crashed/lost ROS input MUST NOT leave an old moving target active.
                # Hold measured q. LowState loss releases control rather than driving blind.
                if (self.lowstate_rx_at is None or now - self.lowstate_rx_at > 0.15):
                    if self._headless_release_at is None:
                        self._headless_release_weight = min(1.0, max(0.0, (now-self._headless_initial_at)/3.0))
                        self._headless_release_at = now
                elif (self._headless_release_at is None and
                      now - self._headless_last_target_at > self._headless_target_timeout_s):
                    measured_q = self.get_current_dual_arm_q()
                    with self.ctrl_lock:
                        self.q_target = measured_q.copy()
                        self.tauff_target = np.asarray(self._headless_gravity(measured_q), dtype=float).copy()
                    self._headless_last_target_at = now
                if self._headless_release_at is None:
                    weight = min(1.0, max(0.0, (now - self._headless_initial_at) / 3.0))
                else:
                    weight = max(0.0, self._headless_release_weight *
                                 (1.0 - (now - self._headless_release_at) / 2.0))
                self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = weight
            with self.ctrl_lock:
                arm_q_target     = self.q_target
                arm_tauff_target = self.tauff_target'''),
        ('            self.lowcmd_publisher.Write(self.msg)\n\n            current_time = time.time()',
         '''            self.lowcmd_publisher.Write(self.msg)
            if self.headless_moveit and self._headless_release_at is not None and weight <= 0.0:
                self._headless_running = False

            current_time = time.time()'''),
        ("    def ctrl_dual_arm(self, q_target, tauff_target):\n        '''Set control target values q & tau of the left and right arm motors.'''\n        with self.ctrl_lock:\n            self.q_target = q_target\n            self.tauff_target = tauff_target",
         '''    def ctrl_dual_arm(self, q_target, tauff_target):
        \'''Set control target values q & tau of the left and right arm motors.\'''
        if self.headless_moveit:
            q_target = np.asarray(q_target, dtype=float)
            tauff_target = np.asarray(tauff_target, dtype=float)
            if (q_target.shape != (14,) or tauff_target.shape != (14,) or
                    not np.isfinite(q_target).all() or not np.isfinite(tauff_target).all()):
                raise ValueError('Invalid headless arm target')
            if self._headless_release_at is not None or not self._headless_running:
                raise RuntimeError('Headless arm control is releasing/closed')
            if self.lowstate_rx_at is None or time.monotonic() - self.lowstate_rx_at > 0.15:
                raise RuntimeError('Headless LowState is stale')
        with self.ctrl_lock:
            self.q_target = q_target.copy() if self.headless_moveit else q_target
            self.tauff_target = tauff_target.copy() if self.headless_moveit else tauff_target
            if self.headless_moveit:
                self._headless_last_target_at = time.monotonic()

    def release_headless(self):
        \'''Ramp out arm SDK authority without commanding home or zero targets.\'''
        if not self.headless_moveit:
            raise RuntimeError('This method is headless only')
        if self._headless_release_at is None:
            now = time.monotonic()
            self._headless_release_weight = min(1.0, max(0.0, (now-self._headless_initial_at)/3.0))
            self._headless_release_at = now
        self.publish_thread.join(timeout=4.0)
        if self.publish_thread.is_alive():
            raise RuntimeError('XR publishing thread did not finish release')'''),
    ]
    for before, after in anchors:
        matches = source.count(before)
        if matches != 1:
            raise RuntimeError(f'Pinned original XR source mismatch for {before[:80]!r}: matches={matches}')
        source = source.replace(before, after, 1)
    source = source + separator + remainder
    compile(source, '<xr_headless_patched>', 'exec')
    return source


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--source', default='/workspace/src/xr_teleoperate/teleop/robot_control/robot_arm.py')
    p.add_argument('--apply', action='store_true')
    args = p.parse_args()
    file = Path(args.source)
    original = file.read_text(encoding='utf-8')
    result = patch(original)
    if result == original:
        print('Already patched; no change')
    elif args.apply:
        backup = file.with_name(file.name + '.before_headless_moveit.bak')
        if backup.exists():
            raise RuntimeError(f'Refusing to overwrite existing backup: {backup}')
        shutil.copy2(file, backup)
        file.write_text(result, encoding='utf-8')
        print(f'Patched {file}; original preserved at {backup}')
    else:
        print('PASS: pinned anchors, patch compiles. Dry run only; use --apply to write.')
    print('Existing XR/Quest constructor behavior remains unchanged by default')


if __name__ == '__main__':
    main()
