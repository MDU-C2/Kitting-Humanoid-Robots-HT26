"""Opt-in, in-memory XR/MoveIt adapter. ISOLATED SIMULATION ONLY.

Leaves original xr_teleoperate files untouched. Unlike earlier experimental
sim overlays, this does NOT patch original G1_29_ArmController initialization,
publisher, torque model, ramp, shutdown, XR IK, or Inspire FTP hands.

The single arm-target setter is selected just before ctrl_dual_arm().
Any invocation without --sim --motion and an explicit simulator interlock
is rejected BEFORE importing XR/Unitree SDK modules.
"""
import argparse
import os
from pathlib import Path
import sys

XR_ROOT = Path(os.environ.get('G1_XR_SOURCE_ROOT', '/workspace/src/xr_teleoperate'))
BRIDGE_ROOT = Path(__file__).resolve().parent


def replace_once(source, before, after, label):
    count = source.count(before)
    if count != 1:
        raise RuntimeError(f'Pinned original XR {label} anchor matched {count} times; refusing')
    return source.replace(before, after, 1)


def patch_main(source):
    source = replace_once(
        source,
        "    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')",
        "    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')\n"
        "    parser.add_argument('--g1-bridge-minimal-sim', action='store_true',\n"
        "                        help='XR/MoveIt command selector, isolated simulator only')",
        'simulator flag')
    source = replace_once(
        source,
        '    args = parser.parse_args()\n',
        '    args = parser.parse_args()\n'
        "    if args.g1_bridge_minimal_sim and (not args.sim or not args.motion or\n"
        "            args.arm != 'G1_29' or args.ee != 'inspire_ftp'):\n"
        "        parser.error('Minimal adapter requires --sim --motion --arm G1_29 --ee inspire_ftp')\n",
        'simulator argument guard')
    source = replace_once(
        source, '        # arm\n',
        '        # Optional external adapter; original XR controller is unchanged.\n'
        '        g1_bridge = None\n'
        '        # arm\n', 'adapter variable')
    source = replace_once(
        source, '        # end-effector\n',
        "        if args.g1_bridge_minimal_sim:\n"
        "            from xr_vr_passthrough_bridge import VrFirstBridge\n"
        "            g1_bridge = VrFirstBridge(arm_ik)\n"
        "            g1_bridge.start()\n"
        "            logger_mp.warning('Minimal XR adapter active: ISOLATED SIMULATOR ONLY; '\n"
        "                              'original XR startup and FTP hand controller unchanged')\n"
        "        # end-effector\n", 'start adapter after original controller')
    source = replace_once(
        source,
        '            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)\n',
        "            if g1_bridge is not None:\n"
        "                selected = g1_bridge.frame(\n"
        "                    current_lr_arm_q, sol_q, sol_tauff,\n"
        "                    vr_ready=bool(tele_data.motion_data_ready))\n"
        "                sol_q, sol_tauff = selected.q, selected.tau_ff\n"
        "            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)\n",
        'single arm command selection')
    source = replace_once(
        source,
        '    finally:\n        try:\n            arm_ctrl.ctrl_dual_arm_go_home()\n',
        "    finally:\n"
        "        if 'g1_bridge' in locals() and g1_bridge is not None:\n"
        "            try:\n"
        "                g1_bridge.stop()\n"
        "            except Exception as exc:\n"
        "                logger_mp.error(f'Unable to close XR adapter socket: {exc}')\n"
        "        try:\n"
        "            arm_ctrl.ctrl_dual_arm_go_home()\n",
        'IPC cleanup with original shutdown')
    compile(source, '<XR minimal in-memory simulator adapter>', 'exec')
    if source.count('arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)') != 1:
        raise RuntimeError('Unexpected additional controller command setter')
    return source


def verify_only():
    original = XR_ROOT / 'teleop' / 'teleop_hand_and_arm.py'
    if not original.is_file():
        raise RuntimeError(f'Pinned XR main not found: {original}')
    before = original.read_bytes()
    patched = patch_main(before.decode('utf-8'))
    compile(patched, str(original), 'exec')
    if original.read_bytes() != before:
        raise RuntimeError('Original XR source was unexpectedly modified')
    print('PASS: pinned XR anchors; memory-only overlay; original XR bytes untouched')
    print('PASS: original arm constructor, zero start, XR IK, FTP hands, motor publisher, home')
    print('PHYSICAL HARDWARE: HARD-BLOCKED (no physical actuator path)')


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--verify', action='store_true')
    args, remaining = parser.parse_known_args(argv)
    if args.verify:
        verify_only()
        return 0
    if not {'--g1-bridge-minimal-sim', '--sim', '--motion'}.issubset(set(remaining)):
        raise SystemExit('HARDWARE BLOCKED: isolated simulator --sim --motion required')
    def has_pair(flag, value):
        return any(remaining[i] == flag and remaining[i+1] == value
                   for i in range(len(remaining) - 1))
    if not has_pair('--arm', 'G1_29') or not has_pair('--ee', 'inspire_ftp'):
        raise SystemExit('HARDWARE BLOCKED: only G1_29 with inspire_ftp simulator allowed')
    if os.environ.get('G1_XR_SIM_ACK') != 'ISOLATED_SIMULATOR':
        raise SystemExit('Simulator isolation not acknowledged; robot connection is forbidden')
    original = XR_ROOT / 'teleop' / 'teleop_hand_and_arm.py'
    patched = patch_main(original.read_text(encoding='utf-8'))
    sys.path.insert(0, str(XR_ROOT))
    sys.path.insert(0, str(BRIDGE_ROOT))
    os.chdir(XR_ROOT / 'teleop')
    sys.argv = [str(original), *remaining]
    exec(compile(patched, str(original), 'exec'),
         {'__name__': '__main__', '__file__': str(original), '__package__': None})
    return 0


if __name__ == '__main__':
    sys.exit(main())
