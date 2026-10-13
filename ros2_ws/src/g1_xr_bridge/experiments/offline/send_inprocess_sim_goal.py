"""Existing synthetic ROS client, pointed at sim-only XR loop endpoint."""
# Standalone tools live two levels below the shared bridge contracts.
# Keep ROS and XR Python environments separate; only expose pure bridge modules.
from pathlib import Path
import sys
BRIDGE_ROOT = Path(__file__).resolve().parents[2]
if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))

import send_offline_ipc_goal as base

base.ACTION_NAME = '/g1_xr_bridge/inprocess_sim/follow_joint_trajectory'

if __name__ == '__main__':
    base.main()
