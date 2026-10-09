"""Exercise THE SAME XR in-process bridge used by the XR runtime overlay.

Only synthetic VR and idealized measured joints. Original XR Pinocchio RNEA.
NO Unitree controller import; NO DDS; NO hardware actuation.
"""
import os
import sys
import time
from pathlib import Path

import numpy as np

from xr_inprocess_bridge import XrInProcessBridge


def main():
    root = Path(os.environ.get('G1_XR_SOURCE_ROOT', '/workspace/src/xr_teleoperate'))
    if not (root / 'teleop/robot_control/robot_arm_ik.py').exists():
        raise RuntimeError(f'Pinned XR source not found: {root}')
    sys.path.insert(0, str(root))
    os.chdir(root / 'teleop')  # Original XR model uses paths relative to teleop
    from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

    print('Loading ORIGINAL XR IK dynamics. NO DDS, NO MOTOR COMMANDS.', flush=True)
    arm_ik = G1_29_ArmIK(Unit_Test=False, Visualization=False)
    bridge = XrInProcessBridge(arm_ik)
    measured = np.zeros(14)
    vr_q = np.zeros(14)
    vr_tau = bridge._gravity(vr_q)
    last_source = None
    bridge.start()
    print('XR in-process offline harness ready at '
          '/tmp/g1_xr_bridge_inprocess_sim.sock', flush=True)
    try:
        while True:
            selected = bridge.frame(measured, vr_q, vr_tau, vr_ready=True)
            # FAKE feedback: ideal instantaneous trajectory tracking.
            measured = selected.q.copy()
            bridge.update_synthetic_state(measured)
            if selected.source.value != last_source:
                print(f'XR in-process mode={selected.source.value}, '
                      f'left_elbow={selected.q[3]:+.4f} rad, '
                      f'RNEA={selected.tau_ff[3]:+.4f} Nm; NO ACTUATION', flush=True)
                last_source = selected.source.value
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        print('Stopping XR in-process offline harness.', flush=True)
    finally:
        bridge.stop()


if __name__ == '__main__':
    main()
