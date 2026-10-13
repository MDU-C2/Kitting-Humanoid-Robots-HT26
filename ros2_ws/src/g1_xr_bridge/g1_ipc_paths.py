"""Local G1 bridge Unix socket endpoints; no DDS, ROS, or side effects."""

HEADLESS_CONTROL_SOCKET = '/tmp/g1_xr_headless_moveit.sock'
READONLY_STATE_SOCKET = '/tmp/g1_xr_readonly_state.sock'
SIMULATION_SOCKET = '/tmp/g1_xr_bridge_inprocess_sim.sock'
OFFLINE_SOCKET = '/tmp/g1_xr_bridge_offline.sock'

# Compatibility aliases for existing simulator/relay consumers.
SIM_SOCKET = SIMULATION_SOCKET
OBSERVER_SOCKET = READONLY_STATE_SOCKET
