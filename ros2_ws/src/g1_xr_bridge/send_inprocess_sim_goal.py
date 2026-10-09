"""Existing synthetic ROS client, pointed at sim-only XR loop endpoint."""
import send_offline_ipc_goal as base

base.ACTION_NAME = '/g1_xr_bridge/inprocess_sim/follow_joint_trajectory'

if __name__ == '__main__':
    base.main()
