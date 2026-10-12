#!/usr/bin/env bash
# Single container-side launch supervisor. Physical actuation is strictly opt-in.
# Not an emergency stop. Supervised regular-mode operation only.
set -Eeuo pipefail
WS="${G1_WORKSPACE:-/workspace}"
BR="$WS/src/g1_xr_bridge"
MODE="${1:-}"
IFACE="${G1_INTERFACE:-enx50a03000b0dc}"
# 0 disables only the extra per-goal movement cap. URDF position bounds,
# trajectory speed limits, and physical feedback watchdogs stay enabled.
MAX_DISPLACEMENT="${G1_MAX_DISPLACEMENT_RAD:-0.0}"
if ! MAX_DISPLACEMENT="$(/usr/bin/python3 - "$MAX_DISPLACEMENT" <<'PYVALIDATE'
import math
import sys
try:
    displacement = float(sys.argv[1])
    assert math.isfinite(displacement) and 0.0 <= displacement <= 0.30
    print(repr(displacement))  # ROS 2 must receive 0.0 (DOUBLE), not 0 (INTEGER).
except (ValueError, AssertionError):
    sys.exit(1)
PYVALIDATE
)"; then
  echo 'G1_MAX_DISPLACEMENT_RAD must be 0 (disabled) or a finite number <= 0.30 rad.' >&2
  exit 2
fi
if [[ "$MODE" != '--hardware' || "${2:-}" != 'G1_REGULAR_MODE_SUPERVISED' ]]; then
  echo "Usage: bash $0 --hardware G1_REGULAR_MODE_SUPERVISED" >&2
  echo 'Physical activation requires a verified independent stop, clear area, robot operator, regular mode.' >&2
  exit 2
fi
[[ -d "$WS" && -f "$BR/xr_headless_moveit.py" ]] || { echo 'Missing workspace/bridge' >&2; exit 1; }
[[ -x /opt/mamba/envs/xr/bin/python ]] || { echo 'XR Micromamba environment missing' >&2; exit 1; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'ROS Humble missing' >&2; exit 1; }
command -v setsid >/dev/null || { echo 'setsid is required for isolated MoveIt process cleanup' >&2; exit 1; }
command -v flock >/dev/null || { echo 'flock is required for single-supervisor ownership' >&2; exit 1; }
# The lock prevents two launchers racing through preflight and socket checks.
# Child processes close fd 9, so a failed launcher cannot leave a stale lock.
exec 9>/tmp/g1_moveit_xr_launcher.lock
flock -n 9 || { echo 'A G1 MoveIt-XR launcher already owns the supervisor lock.' >&2; exit 1; }
# Prevent confusing a down robot link with a configuration error.
[[ -d "/sys/class/net/$IFACE" ]] || { echo "Interface not found: $IFACE" >&2; exit 1; }
[[ "$(cat "/sys/class/net/$IFACE/operstate")" == up ]] || { echo "Interface $IFACE is not UP; start robot/verify Ethernet first" >&2; exit 1; }
[[ -f "$WS/src/g1_moveit_config/config/moveit_xr_controllers.yaml" ]] || { echo 'Missing XR MoveIt controller YAML' >&2; exit 1; }
grep -Eq '^\s*action_ns:\s*follow_joint_trajectory\s*$' "$WS/src/g1_moveit_config/config/moveit_xr_controllers.yaml" || { echo 'XR MoveIt action namespace is not the hardware action. Abort.' >&2; exit 1; }
# No duplicate XR publisher launch when this supervisor is run twice.
if pgrep -af '[p]ython.*xr_headless_moveit.py.*--hardware' >/dev/null; then echo 'Headless XR already running. Refusing duplicate publisher.' >&2; exit 1; fi
if pgrep -af '[p]ython.*ros2_headless_moveit_action.py' >/dev/null; then echo 'XR action server already running. Refusing duplicate.' >&2; exit 1; fi
# Avoid starting hardware while leftover ROS/observer instances can produce conflicting
# /tf, planning services and /joint_states. Never remove an occupied socket automatically.
existing="$(pgrep -af '[g]1_readonly_observer.py|[g]1_joint_state_relay.py|[x]r_pipeline_moveit.launch.py|[m]oveit_ros_move_group/move_group|[r]obot_state_publisher/robot_state_publisher' || true)"
if [[ -n "$existing" ]]; then
  echo 'Existing observer/relay/MoveIt processes detected. Shut down old pipeline before starting:' >&2
  printf '%s\n' "$existing" >&2
  exit 1
fi
# A previous observer can leave a Unix socket pathname after its process dies.
# Remove it only after checking for an existing observer and proving there is no
# listening peer. Never remove an active socket, non-socket, or replaced inode.
cleanup_stale_observer_socket() {
  if pgrep -af '[p]ython.*g1_readonly_observer.py' >/dev/null; then
    echo 'Read-only observer process is still running; refusing socket cleanup.' >&2
    return 1
  fi
  /usr/bin/python3 - <<'PYSOCKET'
import errno
import os
import socket
import stat
import sys

path = '/tmp/g1_xr_readonly_state.sock'
try:
    before = os.lstat(path)
except FileNotFoundError:
    sys.exit(0)
if not stat.S_ISSOCK(before.st_mode):
    sys.exit(f'Observer socket path exists but is not a socket: {path}')
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(0.3)
try:
    try:
        s.connect(path)
    except OSError as exc:
        if exc.errno == errno.ENOENT:
            sys.exit(0)
        if exc.errno != errno.ECONNREFUSED:
            sys.exit(f'Observer socket may be active; refusing to unlink: {exc}')
    else:
        sys.exit('Observer socket accepts connections; refusing to unlink')
finally:
    s.close()
try:
    after = os.lstat(path)
except FileNotFoundError:
    sys.exit(0)
if (before.st_dev, before.st_ino) != (after.st_dev, after.st_ino):
    sys.exit('Observer socket changed ownership during check; refusing to unlink')
os.unlink(path)
print('Removed abandoned read-only observer socket (no listener).', flush=True)
PYSOCKET
}
cleanup_stale_observer_socket || exit 1
bash "$BR/verify_and_apply_xr.sh" || { echo 'Run offline preparation and apply patch first.' >&2; exit 1; }
# verify in XR environment (does not open DDS)
(cd "$WS/src/xr_teleoperate/teleop" && env -u PYTHONPATH LD_LIBRARY_PATH="/opt/mamba/envs/xr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" G1_XR_SOURCE_ROOT="$WS/src/xr_teleoperate" /opt/mamba/envs/xr/bin/python "$BR/xr_headless_moveit.py" --verify)
LOGDIR="$WS/g1_headless_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"
echo "Logs: $LOGDIR"
# Run supervised controller first and only start ROS components after XR indicates it has fresh feedback.
( cd "$WS/src/xr_teleoperate/teleop"; unset PYTHONPATH; export LD_LIBRARY_PATH="/opt/mamba/envs/xr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; exec /opt/mamba/envs/xr/bin/python "$BR/xr_headless_moveit.py" --hardware --interface "$IFACE" --operator-ack G1_REGULAR_MODE_SUPERVISED ) 9>&- > "$LOGDIR/xr.log" 2>&1 & XR_PID=$!
ACTION_PID=""
OBSERVER_PID=""
RELAY_PID=""
MOVEIT_PID=""
MOVEIT_ISOLATED=0

pid_active() {
  local pid="$1" state
  kill -0 "$pid" 2>/dev/null || return 1
  state="$(ps -o stat= -p "$pid" 2>/dev/null || true)"
  [[ -n "$state" && "$state" != Z* ]]
}
wait_process() {
  local pid="$1" timeout="$2" label="$3" started=$SECONDS
  [[ -n "$pid" ]] || return 0
  while pid_active "$pid"; do
    if (( SECONDS - started >= timeout )); then
      echo "TIMEOUT: $label (PID $pid) did not exit in ${timeout}s." >&2
      return 1
    fi
    sleep 0.2
  done
  wait "$pid" 2>/dev/null || true
  return 0
}
# ROS components never publish arm motor commands. If one becomes unresponsive
# after XR has released, it can be force-stopped without touching XR itself.
stop_ros_process() {
  local pid="$1" label="$2"
  [[ -n "$pid" ]] || return 0
  if ! wait_process "$pid" 5 "$label"; then
    echo "Force-stopping unresponsive $label (PID $pid); not the XR motor controller." >&2
    kill -KILL "$pid" 2>/dev/null || true
    wait_process "$pid" 2 "$label"
  fi
}
moveit_group_active() {
  (( MOVEIT_ISOLATED == 1 )) || return 1
  # Only processes in our own setsid-created group; exclude zombies.
  ps -e -o pgid=,stat= | awk -v pg="$MOVEIT_PID"     '$1 == pg && $2 !~ /^Z/ { alive=1; exit } END { exit !alive }'
}
cleanup() {
  local code="$1" failed=0 started
  trap - EXIT INT TERM
  set +e
  echo 'Orderly shutdown: stopping trajectory sources, requesting XR release, then closing observers.' >&2
  # No new MoveIt goals or action commands. The XR watchdog remains active.
  [[ -n "$ACTION_PID" ]] && kill -TERM "$ACTION_PID" 2>/dev/null
  if [[ -n "$MOVEIT_PID" ]]; then
    if (( MOVEIT_ISOLATED == 1 )); then
      kill -TERM -- "-$MOVEIT_PID" 2>/dev/null
    else
      kill -TERM "$MOVEIT_PID" 2>/dev/null
      echo 'WARNING: MoveIt process group was not confirmed isolated; check for descendants.' >&2
      failed=1
    fi
  fi
  # Never force-kill XR. Its original headless release may take several seconds.
  if [[ -n "${XR_PID:-}" ]]; then
    kill -INT "$XR_PID" 2>/dev/null
    if wait_process "$XR_PID" 12 'XR controlled motor release'; then
      if grep -q 'Headless XR: shutdown completed' "$LOGDIR/xr.log"; then
        echo 'XR release completed according to xr.log.' >&2
      else
        echo 'WARNING: XR exited without its release-completed log; operator must verify release.' >&2
        failed=1
      fi
    else
      echo 'DANGER: XR may still be controlling the G1. Do not relaunch or close Docker.' >&2
      failed=1
    fi
  fi
  [[ -n "$RELAY_PID" ]] && kill -TERM "$RELAY_PID" 2>/dev/null
  [[ -n "$OBSERVER_PID" ]] && kill -TERM "$OBSERVER_PID" 2>/dev/null
  if ! stop_ros_process "$ACTION_PID" 'ROS action server'; then failed=1; fi
  if ! stop_ros_process "$RELAY_PID" 'ROS joint-state relay'; then failed=1; fi
  if ! stop_ros_process "$OBSERVER_PID" 'read-only observer'; then failed=1; fi
  if [[ -n "$MOVEIT_PID" ]]; then
    started=$SECONDS
    while moveit_group_active && (( SECONDS - started < 5 )); do sleep 0.2; done
    if moveit_group_active; then
      echo 'MoveIt/RViz group did not exit after SIGTERM; force-stopping ONLY the isolated ROS group.' >&2
      kill -KILL -- "-$MOVEIT_PID" 2>/dev/null || true
      sleep 0.3
      if moveit_group_active; then failed=1; fi
    fi
    wait "$MOVEIT_PID" 2>/dev/null || true
  fi
  # The observer should remove its own socket. Fallback requires it to be gone,
  # and checks actual socket reachability and inode before unlinking.
  if [[ -n "$OBSERVER_PID" ]] && ! pid_active "$OBSERVER_PID"; then
    cleanup_stale_observer_socket || failed=1
  fi
  if (( failed )); then
    echo "INCOMPLETE shutdown; inspect $LOGDIR and verify robot state before restart." >&2
    exit 1
  fi
  echo 'Shutdown complete: no supervised XR or ROS processes remain.' >&2
  exit "$code"
}
# Ctrl+C is a normal orderly-shutdown request, not an emergency stop.
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'cleanup "$?"' EXIT
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
echo "MoveIt extra per-joint displacement cap: ${MAX_DISPLACEMENT} rad (0=off; joint bounds and 0.20 rad/s speed guard stay enabled)"
/usr/bin/python3 "$BR/ros2_headless_moveit_action.py" --ros-args -p "max_displacement_rad:=${MAX_DISPLACEMENT}" 9>&- > "$LOGDIR/action.log" 2>&1 & ACTION_PID=$!
( unset PYTHONPATH; export LD_LIBRARY_PATH="/opt/mamba/envs/xr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; exec /opt/mamba/envs/xr/bin/python "$BR/g1_readonly_observer.py" --interface "$IFACE" ) 9>&- > "$LOGDIR/observer.log" 2>&1 & OBSERVER_PID=$!
/usr/bin/python3 "$BR/g1_joint_state_relay.py" --mode observe 9>&- > "$LOGDIR/relay.log" 2>&1 & RELAY_PID=$!
# setsid isolates MoveIt/RViz from XR so termination cannot signal the motor publisher.
setsid ros2 launch g1_moveit_config xr_pipeline_moveit.launch.py enable_execution:=true 9>&- > "$LOGDIR/moveit_rviz.log" 2>&1 & MOVEIT_PID=$!
for ((i=0; i<20; i++)); do
  pgid="$(ps -o pgid= -p "$MOVEIT_PID" 2>/dev/null | tr -d '[:space:]')"
  if [[ "$pgid" == "$MOVEIT_PID" ]]; then MOVEIT_ISOLATED=1; break; fi
  pid_active "$MOVEIT_PID" || break
  sleep 0.05
done
if (( MOVEIT_ISOLATED != 1 )); then
  echo 'MoveIt launcher failed to enter its own process group; refusing to continue.' >&2
  exit 1
fi
echo 'Started XR + action + observer + relay + MoveIt/RViz. Check RViz and logs; no trajectory is sent automatically.'
echo 'This terminal is the supervisor. Do NOT close it or use Ctrl+C as an emergency stop.'
echo 'Physical stopping must follow the robot operator’s verified procedure.'
while true; do
  sleep 1
  if ! kill -0 "$XR_PID" 2>/dev/null; then
    echo "XR motor controller exited (PID $XR_PID). Consult $LOGDIR/xr.log." >&2; exit 1
  fi
  for name in ACTION_PID OBSERVER_PID RELAY_PID MOVEIT_PID; do
    pid="${!name}"
    if ! pid_active "$pid"; then
      echo "$name exited (PID $pid). Consult $LOGDIR logs." >&2
      exit 1
    fi
  done
done
