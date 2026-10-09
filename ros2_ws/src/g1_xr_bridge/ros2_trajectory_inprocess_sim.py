"""Offline-only ROS trajectory action pointed at XR's in-process SIM bridge."""
import ros2_trajectory_xr_ipc_dry_run as base
from xr_ipc_protocol import ping, send_sample, transact
from xr_inprocess_bridge import SIM_SOCKET

base.ACTION_NAME = '/g1_xr_bridge/inprocess_sim/follow_joint_trajectory'
def require_active_xr():
    status = ping(SIM_SOCKET)
    if not status.get('xr_loop_fresh'):
        raise base.IpcError('XR main loop not started; press r in XR first')
    return status

base.ping = require_active_xr


def require_measured_baseline(self):
    state = transact({'kind': 'status'}, SIM_SOCKET)
    if (state.get('kind') != 'status_ack' or not state.get('feedback_fresh')
            or not state.get('xr_loop_fresh')):
        raise base.IpcError('No fresh XR arm measurement for single-arm goal')
    return state.get('measured_arm_positions')


base.OfflineIpcServer.get_baseline = require_measured_baseline
base.send_sample = lambda q, source, seq: send_sample(q, source, seq, SIM_SOCKET)

if __name__ == '__main__':
    base.main()
