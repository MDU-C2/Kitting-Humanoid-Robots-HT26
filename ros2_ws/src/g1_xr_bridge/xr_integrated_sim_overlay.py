"""Sim-only, memory-only overlay into pinned Unitree XR + ROS trajectory bridge.

XR's original arm IK/Pinocchio and G1_29_ArmController remain the algorithms.
NO PHYSICAL ROBOT BRIDGE PATH: requires --sim --motion and explicit sim flag.
No writes to the pinned xr_teleoperate checkout.
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
        raise RuntimeError(f'Pinned XR {label} anchor matched {count} times; refusing overlay')
    return source.replace(before, after, 1)


def patch_main(source):
    source = replace_once(source,
        "    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')",
        "    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')\n"
        "    parser.add_argument('--g1-bridge-sim', action='store_true',\n"
        "                        help='Use MoveIt trajectory bridge with XR in simulation ONLY')",
        'simulation CLI')
    source = replace_once(source, '    args = parser.parse_args()\n',
        '    args = parser.parse_args()\n'
        "    if args.g1_bridge_sim and (not args.sim or not args.motion or\n"
        "            args.arm != 'G1_29' or args.ee == 'dex1_internal'):\n"
        "        parser.error('BRIDGE IS SIMULATION ONLY: requires '\n"
        "                     '--g1-bridge-sim --sim --motion --arm G1_29')\n",
        'simulation hardware interlock')
    source = replace_once(source, '        # arm\n',
        "        # Original XR controller still owns the single motor-command path.\n"
        "        g1_bridge = None\n"
        "        # arm\n", 'bridge initial declaration')
    source = replace_once(source,
        '                arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)\n',
        "                if args.g1_bridge_sim:\n"
        "                    from xr_inprocess_bridge import XrInProcessBridge\n"
        "                    from xr_sim_startup_patch import install_sim_controller_patch\n"
        "                    g1_bridge = XrInProcessBridge(arm_ik)\n"
        "                    install_sim_controller_patch(G1_29_ArmController)\n"
        "                    arm_ctrl = G1_29_ArmController(\n"
        "                        motion_mode=True, simulation_mode=True,\n"
        "                        bridge_gravity_fn=g1_bridge._gravity)\n"
        "                else:\n"
        "                    arm_ctrl = G1_29_ArmController(\n"
        "                        motion_mode=args.motion, simulation_mode=args.sim)\n",
        'measured-position seeded original XR controller')
    source = replace_once(source, '        # end-effector\n',
        "        if g1_bridge is not None:\n"
        "            g1_bridge.start()\n"
        "            g1_bridge_initial_hold = False\n"
        "            logger_mp.warning('G1 integrated bridge: SIMULATION ONLY, '\n"
        "                                'physical bridge disabled.')\n\n"
        "        # end-effector\n", 'starting XR IPC bridge')
    before = (
        "            # solve ik using motor data and wrist pose, then use ik results to control arms.\n"
        "            time_ik_start = time.time()\n"
        "            sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_wrist_pose, tele_data.right_wrist_pose, current_lr_arm_q, current_lr_arm_dq)\n"
        "            time_ik_end = time.time()\n"
        "            logger_mp.debug(f\"ik:\\t{round(time_ik_end - time_ik_start, 6)}\")\n"
        "            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)\n")
    after = (
        "            # VR: use unchanged original XR IK (and its RNEA torque).\n"
        "            # Trajectory/Hold: MoveIt already supplied joints; do not solve IK again.\n"
        "            if g1_bridge is not None:\n"
        "                from xr_sim_startup_patch import check_feedback_fresh\n"
        "                check_feedback_fresh(arm_ctrl)  # real DDS read time, not cached getter\n"
        "                g1_bridge.update_sim_state(arm_ctrl.get_current_motor_q())\n"
        "                if not g1_bridge_initial_hold:\n"
        "                    initial_hold_at = time.monotonic()\n"
        "                    g1_bridge.mux.update_measured(current_lr_arm_q, initial_hold_at)\n"
        "                    g1_bridge.mux.enter_hold(initial_hold_at)\n"
        "                    g1_bridge_initial_hold = True\n"
        "            # In HOLD, solve VR IK only to prepare a possible explicit resume.\n"
        "            # During TRAJECTORY, skip VR IK entirely.\n"
        "            if (g1_bridge is None or\n"
        "                    (g1_bridge.mux.source.value != 'trajectory' and\n"
        "                     bool(tele_data.motion_data_ready))):\n"
        "                time_ik_start = time.time()\n"
        "                sol_q, sol_tauff = arm_ik.solve_ik(\n"
        "                    tele_data.left_wrist_pose, tele_data.right_wrist_pose,\n"
        "                    current_lr_arm_q, current_lr_arm_dq)\n"
        "                time_ik_end = time.time()\n"
        "                logger_mp.debug(f'ik: {round(time_ik_end - time_ik_start, 6)}')\n"
        "            else:\n"
        "                sol_q = current_lr_arm_q.copy()\n"
        "                sol_tauff = g1_bridge._gravity(sol_q)\n"
        "            if g1_bridge is not None:\n"
        "                selected = g1_bridge.frame(\n"
        "                    current_lr_arm_q, sol_q, sol_tauff,\n"
        "                    vr_ready=(bool(tele_data.motion_data_ready) and\n"
        "                              g1_bridge.mux.source.value != 'trajectory'))\n"
        "                sol_q, sol_tauff = selected.q, selected.tau_ff\n"
        "            # Exactly one original XR motor target setter, after selection.\n"
        "            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)\n"
        "            if g1_bridge is not None:\n"
        "                arm_ctrl.bridge_motion_guard.heartbeat()\n")
    source = replace_once(source, before, after, 'single original XR arm command point')
    source = replace_once(source,
        '    finally:\n        try:\n            arm_ctrl.ctrl_dual_arm_go_home()\n',
        "    finally:\n"
        "        try:\n"
        "            if 'g1_bridge' in locals() and g1_bridge is not None:\n"
        "                arm_ctrl.bridge_motion_guard.disarm()\n"
        "                # Keep last measured-position target during blend-down.\n"
        "                time.sleep(0.5)\n"
        "                g1_bridge.stop()\n"
        "        except Exception as e:\n"
        "            logger_mp.error(f'Bridge IPC shutdown error: {e}')\n"
        "        try:\n"
        "            if 'g1_bridge' not in locals() or g1_bridge is None:\n"
        "                arm_ctrl.ctrl_dual_arm_go_home()\n",
        'preserve normal XR shutdown; avoid zeroing targets in integrated sim')
    compile(source, 'XR main in-memory simulation overlay', 'exec')
    if source.count('arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)') != 1:
        raise RuntimeError('Expected exactly one XR arm target setter')
    return source


def verify_only():
    xr_script = XR_ROOT / 'teleop' / 'teleop_hand_and_arm.py'
    robot_arm = XR_ROOT / 'teleop' / 'robot_control' / 'robot_arm.py'
    from xr_sim_startup_patch import patch_controller_source
    patched_main = patch_main(xr_script.read_text(encoding='utf-8'))
    patched_arm = patch_controller_source(robot_arm.read_text(encoding='utf-8'))
    compile(patched_main, str(xr_script), 'exec')
    compile(patched_arm, str(robot_arm), 'exec')
    print('Pinned XR main + controller: source anchors verified and compiled.')
    print('XR arm controller: measured q + XR RNEA initial torque; simulator overlay only.')
    print('MoveIt: joint trajectories bypass XR IK; VR: original XR IK unchanged.')
    print('DDS age check: based on new LowState receive timestamps.')
    print('Original XR checkout modified: NO; hardware execution supported: NO.')
    print('Simulator: gradual blend + heartbeat guard installed; NOT hardware validated.')


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--verify', action='store_true')
    args, remaining = parser.parse_known_args()
    if args.verify:
        verify_only()
        return
    required = {'--g1-bridge-sim', '--sim', '--motion', '--arm'}
    if not required.issubset(set(remaining)) or not any(
            remaining[i] == '--arm' and i + 1 < len(remaining) and remaining[i+1] == 'G1_29'
            for i in range(len(remaining))):
        raise SystemExit('PHYSICAL BRIDGE BLOCKED. Actual simulator only: '
                         '--g1-bridge-sim --sim --motion --arm G1_29')
    if os.environ.get('G1_XR_SIM_ACK') != 'ISOLATED_SIMULATOR':
        raise SystemExit('Simulator is not confirmed isolated: set '
                         'G1_XR_SIM_ACK=ISOLATED_SIMULATOR only for a disconnected simulator. '
                         'The bridge is NOT approved for the physical G1.')
    xr_script = XR_ROOT / 'teleop' / 'teleop_hand_and_arm.py'
    patched = patch_main(xr_script.read_text(encoding='utf-8'))
    sys.path.insert(0, str(XR_ROOT))
    sys.path.insert(0, str(BRIDGE_ROOT))
    os.chdir(XR_ROOT / 'teleop')
    sys.argv = [str(xr_script), *remaining]
    exec(compile(patched, str(xr_script), 'exec'),
         {'__name__': '__main__', '__file__': str(xr_script), '__package__': None})


if __name__ == '__main__':
    main()
