#!/usr/bin/env bash
# READ-ONLY G1 robot model for perception tests: TF pelvis -> ... -> d435_link from the real
# joint angles, WITHOUT the XR arm controller or MoveIt (nothing is commanded, the arms are
# not taken over). Runs inside the team's MoveIt container (ros2_ws/scripts/start.sh).
#
#     bash /workspace/src/karis_bringup/scripts/observe_g1_tf.sh            # cable: real joints
#     bash /workspace/src/karis_bringup/scripts/observe_g1_tf.sh --fake     # no /lowstate (WiFi): all joints 0
#     G1_INTERFACE=eth1 bash .../observe_g1_tf.sh                            # pick the interface
#
# Uses the team's own pieces: g1_readonly_observer.py (reads rt/lowstate, no publishers) ->
# g1_joint_state_relay.py --mode observe (/joint_states) -> robot_state_publisher with the
# same URDF (g1_description xacro) as MoveIt. The perception launch adds the camera mount
# (d435_link -> head_camera_color_optical_frame), giving /karis/object_poses_base.
#
# Do NOT run this together with ros2_ws/scripts/run_g1.sh: that already starts the observer,
# the relay and robot_state_publisher (two copies would give TF two sources). Ctrl+C stops it.
set -Eeuo pipefail

FAKE=0
case "${1:-}" in
  '') ;;
  --fake) FAKE=1 ;;
  -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
  *) echo "Usage: bash $0 [--fake]" >&2; exit 2 ;;
esac

WS=/workspace
BR="$WS/src/g1_xr_bridge"
XACRO="$WS/src/g1_description/urdf/g1_29dof_with_inspire_hand_ftp.urdf.xacro"
[[ -f "$BR/g1_readonly_observer.py" && -f "$XACRO" ]] || {
  echo 'Run this INSIDE the team MoveIt container (ros2_ws/scripts/start.sh); /workspace/src is missing.' >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo 'Workspace not built: run ros2_ws/scripts/install.sh (on the host) first.' >&2; exit 1; }
if pgrep -af '[g]1_readonly_observer.py|[g]1_joint_state_relay.py|[r]obot_state_publisher/robot_state_publisher' >/dev/null; then
  echo 'An observer / relay / robot_state_publisher is already running (run_g1.sh or another copy):' >&2
  pgrep -af '[g]1_readonly_observer.py|[g]1_joint_state_relay.py|[r]obot_state_publisher/robot_state_publisher' >&2
  echo 'It already publishes the robot model; do not start a second one.' >&2
  exit 1
fi

# The robot-network interface: G1_INTERFACE, else the one interface with a 192.168.123.x address
IFACE="${G1_INTERFACE:-}"
if [[ -z "$IFACE" ]]; then
  mapfile -t found < <(ip -4 -o addr show | awk '$4 ~ /^192\.168\.123\./ {print $2}' | sort -u)
  if (( ${#found[@]} == 1 )); then IFACE="${found[0]}"; fi
fi
[[ -n "$IFACE" && -d "/sys/class/net/$IFACE" ]] || {
  echo 'Cannot find exactly one interface on 192.168.123.x. Set it: G1_INTERFACE=<name> bash ...' >&2
  ip -4 -o addr show | awk '{print "  " $2, $4}' >&2; exit 1; }
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ -n "${KARIS_DDS_PEERS:-}" ]]; then
  peers=""; for p in $KARIS_DDS_PEERS 127.0.0.1; do peers="$peers<Peer Address=\"$p\"/>"; done
  disc="<Discovery><Peers>$peers</Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery>"
else
  disc=""
fi
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General>${disc}</Domain></CycloneDDS>"
echo "ROS 2 on $IFACE ($(ip -4 -o addr show "$IFACE" | awk '{print $4}')), ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
  if [[ -n "${TMP:-}" ]]; then rm -rf -- "$TMP"; fi
  echo 'Stopped (read-only robot model).'
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# URDF into a file + a params file (an XML string on the command line is not reliably parsed)
TMP="$(mktemp -d /tmp/karis_observe.XXXXXX)"
xacro "$XACRO" -o "$TMP/g1.urdf"
/usr/bin/python3 - "$TMP" <<'PY'
import sys, yaml, pathlib
d = pathlib.Path(sys.argv[1])
desc = (d / "g1.urdf").read_text()
(d / "rsp.yaml").write_text(yaml.safe_dump(
    {"robot_state_publisher": {"ros__parameters": {"robot_description": desc}}}))
PY
if (( FAKE )); then
  echo 'Joint states: all joints at 0 (waist assumed straight); no robot data used.'
  ros2 run joint_state_publisher joint_state_publisher "$TMP/g1.urdf" &
  PIDS+=($!)
else
  [[ -x /opt/mamba/envs/xr/bin/python ]] || { echo 'XR Python missing: run ros2_ws/scripts/install.sh.' >&2; exit 1; }
  SOCK=/tmp/g1_xr_readonly_state.sock
  if [[ -S "$SOCK" ]] && ! pgrep -af '[p]ython.*g1_readonly_observer.py' >/dev/null; then
    echo "Removing a stale observer socket ($SOCK, no observer running)."; rm -f -- "$SOCK"
  fi
  echo 'Joint states: READ-ONLY from rt/lowstate (g1_readonly_observer.py, no command publishers).'
  ( unset PYTHONPATH; export LD_LIBRARY_PATH="/opt/mamba/envs/xr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    exec /opt/mamba/envs/xr/bin/python "$BR/g1_readonly_observer.py" --interface "$IFACE" ) &
  PIDS+=($!)
  sleep 2
  /usr/bin/python3 "$BR/g1_joint_state_relay.py" --mode observe &
  PIDS+=($!)
fi
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$TMP/rsp.yaml" &
PIDS+=($!)
echo 'Robot model running: TF pelvis -> ... -> d435_link. Check: ros2 run tf2_ros tf2_echo pelvis d435_link'
while true; do
  for pid in "${PIDS[@]}"; do
    kill -0 "$pid" 2>/dev/null || { echo "A process exited (PID $pid); stopping." >&2; exit 1; }
  done
  sleep 1
done
