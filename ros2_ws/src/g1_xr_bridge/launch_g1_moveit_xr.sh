#!/usr/bin/env bash
# Single container-side launch supervisor. Physical actuation is strictly opt-in.
# Not an emergency stop. Supervised regular-mode operation only.
set -Eeuo pipefail
WS="${G1_WORKSPACE:-/workspace}"
BR="$WS/src/g1_xr_bridge"
MODE="${1:-}"
IFACE="${G1_INTERFACE:-enx50a03000b0dc}"
if [[ "$MODE" != '--hardware' || "${2:-}" != 'G1_REGULAR_MODE_SUPERVISED' ]]; then
  echo "Usage: bash $0 --hardware G1_REGULAR_MODE_SUPERVISED" >&2
  echo 'Physical activation requires a verified independent stop, clear area, robot operator, regular mode.' >&2
  exit 2
fi
[[ -d "$WS" && -f "$BR/xr_headless_moveit.py" ]] || { echo 'Missing workspace/bridge' >&2; exit 1; }
[[ -x /opt/mamba/envs/xr/bin/python ]] || { echo 'XR Micromamba environment missing' >&2; exit 1; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'ROS Humble missing' >&2; exit 1; }
# Prevent confusing a down robot link with a configuration error.
[[ -d "/sys/class/net/$IFACE" ]] || { echo "Interface not found: $IFACE" >&2; exit 1; }
[[ "$(cat "/sys/class/net/$IFACE/operstate")" == up ]] || { echo "Interface $IFACE is not UP; start robot/verify Ethernet first" >&2; exit 1; }
[[ -f "$WS/src/g1_moveit_config/config/moveit_xr_controllers.yaml" ]] || { echo 'Missing XR MoveIt controller YAML' >&2; exit 1; }
grep -Eq '^\s*action_ns:\s*follow_joint_trajectory\s*$' "$WS/src/g1_moveit_config/config/moveit_xr_controllers.yaml" || { echo 'XR MoveIt action namespace is not the hardware action. Abort.' >&2; exit 1; }
# No duplicate XR publisher launch when this supervisor is run twice.
if pgrep -af '[p]ython.*xr_headless_moveit.py.*--hardware' >/dev/null; then echo 'Headless XR already running. Refusing duplicate publisher.' >&2; exit 1; fi
if pgrep -af '[p]ython.*ros2_headless_moveit_action.py' >/dev/null; then echo 'XR action server already running. Refusing duplicate.' >&2; exit 1; fi
bash "$BR/verify_and_apply_xr.sh" || { echo 'Run offline preparation and apply patch first.' >&2; exit 1; }
# verify in XR environment (does not open DDS)
(cd "$WS/src/xr_teleoperate/teleop" && env -u PYTHONPATH LD_LIBRARY_PATH="/opt/mamba/envs/xr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" G1_XR_SOURCE_ROOT="$WS/src/xr_teleoperate" /opt/mamba/envs/xr/bin/python "$BR/xr_headless_moveit.py" --verify)
LOGDIR="$WS/g1_headless_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"
echo "Logs: $LOGDIR"
# Run supervised controller first and only start ROS components after XR indicates it has fresh feedback.
( cd "$WS/src/xr_teleoperate/teleop"; unset PYTHONPATH; export LD_LIBRARY_PATH="/opt/mamba/envs/xr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; exec /opt/mamba/envs/xr/bin/python "$BR/xr_headless_moveit.py" --hardware --interface "$IFACE" --operator-ack G1_REGULAR_MODE_SUPERVISED ) > "$LOGDIR/xr.log" 2>&1 & XR_PID=$!
CHILDREN=()
stop_children() {
  trap - INT TERM EXIT
  echo 'Stopping MoveIt, action server, relay, and observer; XR must release through its supervised procedure.' >&2
  for pid in "${CHILDREN[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  # XR has a dedicated release implementation, but this is not an emergency stop.
  kill -INT "$XR_PID" 2>/dev/null || true
  echo "Review XR shutdown and logs: $LOGDIR/xr.log" >&2
}
trap stop_children INT TERM EXIT
# Do not start planner until live IPC is healthy (maximum ~12 s)
ready=0
for ((i=0;i<24;i++)); do
  if ! kill -0 "$XR_PID" 2>/dev/null; then echo 'XR exited unexpectedly; see xr.log' >&2; exit 1; fi
  if (cd "$BR" && /usr/bin/python3 - <<'PY' >/dev/null 2>&1
from xr_ipc_protocol import transact
s=transact({'kind':'status'}, '/tmp/g1_xr_headless_moveit.sock', timeout=0.3)
assert s.get('hardware_connected') and s.get('feedback_fresh') and s.get('xr_loop_fresh') and s.get('weight_ramp_complete') and s.get('source') == 'hold'
PY
  ); then ready=1; break; fi
  sleep 0.5
done
[[ "$ready" == 1 ]] || { echo 'XR not ready in time; see xr.log. Check robot safety before further actions.' >&2; exit 1; }
# ROS setup scripts expect unset environment variables.
# Temporarily disable nounset while sourcing them.
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$WS/install/setup.bash" ]]; then
    source "$WS/install/setup.bash"
fi
set -u
/usr/bin/python3 "$BR/ros2_headless_moveit_action.py" > "$LOGDIR/action.log" 2>&1 & CHILDREN+=("$!")
( unset PYTHONPATH; export LD_LIBRARY_PATH="/opt/mamba/envs/xr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; exec /opt/mamba/envs/xr/bin/python "$BR/g1_readonly_observer.py" --interface "$IFACE" ) > "$LOGDIR/observer.log" 2>&1 & CHILDREN+=("$!")
/usr/bin/python3 "$BR/g1_joint_state_relay.py" --mode observe > "$LOGDIR/relay.log" 2>&1 & CHILDREN+=("$!")
ros2 launch g1_moveit_config xr_pipeline_moveit.launch.py enable_execution:=true > "$LOGDIR/moveit_rviz.log" 2>&1 & CHILDREN+=("$!")
echo 'Started XR + action + observer + relay + MoveIt/RViz. Check RViz and logs; no trajectory is sent automatically.'
echo 'This terminal is the supervisor. Do NOT close it or use Ctrl+C as an emergency stop.'
echo 'Physical stopping must follow the robot operator’s verified procedure.'
while true; do
  sleep 1
  for pid in "$XR_PID" "${CHILDREN[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then echo "A component exited (PID $pid). Consult $LOGDIR logs." >&2; exit 1; fi
  done
done
