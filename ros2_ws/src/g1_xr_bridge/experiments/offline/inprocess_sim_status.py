"""Operator-only status and guarded VR resume for XR simulation bridge."""
import argparse
# Standalone tools live two levels below the shared bridge contracts.
# Keep ROS and XR Python environments separate; only expose pure bridge modules.
from pathlib import Path
import sys
BRIDGE_ROOT = Path(__file__).resolve().parents[2]
if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))

from xr_ipc_protocol import transact
from xr_inprocess_bridge import SIM_SOCKET

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--resume-vr', action='store_true')
    opts = p.parse_args()
    if opts.resume_vr:
        print('VR resume:', transact({'kind': 'resume_vr'}, SIM_SOCKET))
    print('Status:', transact({'kind': 'status'}, SIM_SOCKET))
