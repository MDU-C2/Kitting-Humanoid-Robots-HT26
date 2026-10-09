"""Apply a fail-closed, memory-only overlay to pinned XR's main entry point.

Never writes into external xr_teleoperate repository. Defaults to unchanged XR.
The bridge is only permitted when --sim AND --motion are both selected.
"""
import argparse
from pathlib import Path
import os
import sys


XR_ROOT = Path(os.environ.get('G1_XR_SOURCE_ROOT', '/workspace/src/xr_teleoperate'))
BRIDGE_ROOT = Path(__file__).resolve().parent


def _replace_once(text, before, after, label):
    count = text.count(before)
    if count != 1:
        raise RuntimeError(f'Pinned XR anchor {label!r} matched {count} times; refusing patch')
    return text.replace(before, after, 1)


def patch_source(source):
    source = _replace_once(
        source,
        "    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')",
        "    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')\n"
        "    parser.add_argument('--g1-bridge-sim', action='store_true',\n"
        "                        help='Trajectory IPC integration with Isaac SIMULATION ONLY')",
        'simulation flag')
    source = _replace_once(
        source,
        '    args = parser.parse_args()\n',
        "    args = parser.parse_args()\n"
        "    if args.g1_bridge_sim and (not args.sim or not args.motion\n"
        "             or args.arm != 'G1_29' or args.ee == 'dex1_internal'):\n"
        "        parser.error('--g1-bridge-sim requires --sim --motion --arm G1_29 '\n"
        "                     'and non-internal end effector. Real hardware is BLOCKED.')\n",
        'real-hardware guard')
    source = _replace_once(
        source,
        '        # end-effector\n',
        "        # In-process trajectory selector: simulator ONLY, original arm controller retained.\n"
        "        g1_bridge = None\n"
        "        if args.g1_bridge_sim:\n"
        "            from xr_inprocess_bridge import XrInProcessBridge\n"
        "            g1_bridge = XrInProcessBridge(arm_ik)\n"
        "            g1_bridge.start()\n"
        "            logger_mp.warning('G1 XR bridge IPC enabled for SIMULATION ONLY.')\n\n"
        '        # end-effector\n',
        'bridge initialisation')
    source = _replace_once(
        source,
        '            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)\n',
        "            if g1_bridge is not None:\n"
        "                selected = g1_bridge.frame(\n"
        "                    current_lr_arm_q, sol_q, sol_tauff,\n"
        "                    vr_ready=bool(tele_data.motion_data_ready))\n"
        "                sol_q, sol_tauff = selected.q, selected.tau_ff\n"
        "            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)\n",
        'single original motor-command point')
    source = _replace_once(
        source,
        '    finally:\n        try:\n            arm_ctrl.ctrl_dual_arm_go_home()\n',
        "    finally:\n"
        "        try:\n"
        "            if 'g1_bridge' in locals() and g1_bridge is not None:\n"
        "                g1_bridge.stop()\n"
        "        except Exception as e:\n"
        "            logger_mp.error(f'Bridge IPC shutdown error: {e}')\n"
        "        try:\n"
        "            arm_ctrl.ctrl_dual_arm_go_home()\n",
        'XR shutdown')
    return source


def main():
    cli = argparse.ArgumentParser(add_help=False)
    cli.add_argument('--verify-overlay', action='store_true')
    opts, passthrough = cli.parse_known_args()
    xr_script = XR_ROOT / 'teleop' / 'teleop_hand_and_arm.py'
    if not xr_script.is_file():
        raise SystemExit(f'Pinned XR source missing: {xr_script}')
    patched = patch_source(xr_script.read_text(encoding='utf-8'))
    compile(patched, str(xr_script), 'exec')
    if opts.verify_overlay:
        print('XR overlay: all source anchors found and compiled.')
        print('External XR checkout unchanged. Physical trajectory bridge BLOCKED.')
        return
    # Never use this experimental launcher to run hardware. The unchanged
    # original XR app has a separate entry point for its normal VR workflow.
    if ('--g1-bridge-sim' not in passthrough or '--sim' not in passthrough
            or '--motion' not in passthrough):
        raise SystemExit('Bridge runtime is SIMULATION ONLY: requires '
                         '--g1-bridge-sim --sim --motion. No hardware launch.')
    # Absolute paths allow invocation from any working directory. The XR
    # original script uses relative URDF paths from xr_teleoperate/teleop.
    os.chdir(XR_ROOT / 'teleop')
    sys.path.insert(0, str(BRIDGE_ROOT))
    sys.path.insert(0, str(XR_ROOT))
    sys.argv = [str(xr_script), *passthrough]
    runtime_globals = {'__name__': '__main__', '__file__': str(xr_script),
                       '__package__': None}
    exec(compile(patched, str(xr_script), 'exec'), runtime_globals)


if __name__ == '__main__':
    main()
