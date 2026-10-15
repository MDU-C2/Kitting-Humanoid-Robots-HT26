# KARIS ROS 2 environment (ROS 2 Humble). SOURCE it in every terminal of a KARIS container
# (the perception and Isaac containers do it automatically; the team MoveIt container too):
#     source <repo>/ros2_ws/karis_ros_env.sh
#
# KARIS_ROLE picks the workspace build of this container (each container builds into its
# own folders, so they never mix):  perception -> install_perception,
# isaac -> install_isaac, anything else (MoveIt container) -> install.
#
# Optional (export before sourcing; docker/compose.yaml passes them from docker/.env):
#   ROS_DOMAIN_ID     default 0 (the G1 uses 0; every computer must use the same)
#   KARIS_NET_IF      network interface on the robot network (192.168.123.x); default: found automatically
#   KARIS_DDS_PEERS   extra unicast peers, e.g. "192.168.123.164" if multicast discovery does not work
#   KARIS_DIR         the local data folder (models, recordings), default <repo>/local_data

_karis_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "karis_ros_env: ROS 2 Humble not found in /opt/ros/humble" >&2; return 1 2>/dev/null || exit 1
fi
source /opt/ros/humble/setup.bash
case "${KARIS_ROLE:-}" in
    perception) _karis_install="$_karis_ws/install_perception" ;;
    isaac)      _karis_install="$_karis_ws/install_isaac" ;;
    *)          _karis_install="$_karis_ws/install" ;;
esac
if [ -f "$_karis_install/setup.bash" ]; then
    source "$_karis_install/setup.bash"
else
    echo "karis_ros_env: workspace not built yet for role '${KARIS_ROLE:-motion}': bash $_karis_ws/build.sh" >&2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export KARIS_DIR="${KARIS_DIR:-$(dirname "$_karis_ws")/local_data}"    # default: local_data/ next to ros2_ws
export KARIS_REPO="${KARIS_REPO:-$(dirname "$_karis_ws")}"

# CycloneDDS on the robot-network interface (the one with a 192.168.123.x address)
if [ -z "$KARIS_NET_IF" ]; then
    KARIS_NET_IF="$(ip -4 -o addr show 2>/dev/null | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}')"
fi
_karis_peers=""
for _p in $KARIS_DDS_PEERS; do _karis_peers="$_karis_peers<Peer Address=\"$_p\"/>"; done
_karis_disc=""
if [ -n "$_karis_peers" ]; then
    _karis_disc="<Discovery><Peers>$_karis_peers<Peer Address=\"127.0.0.1\"/></Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery>"
fi
if [ -n "$KARIS_NET_IF" ]; then
    # Interface by name; the same config works on the G1 (Unitree's CycloneDDS 0.10 build)
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$KARIS_NET_IF\" multicast=\"default\"/></Interfaces></General>$_karis_disc</Domain></CycloneDDS>"
    _karis_if_msg="$KARIS_NET_IF ($(ip -4 -o addr show "$KARIS_NET_IF" 2>/dev/null | awk '{print $4}'))"
else
    unset CYCLONEDDS_URI
    [ -n "$_karis_disc" ] && export CYCLONEDDS_URI="<CycloneDDS><Domain>$_karis_disc</Domain></CycloneDDS>"
    _karis_if_msg="none on 192.168.123.x found: DDS picks one (only tests on this computer will work)"
fi
echo "KARIS ROS 2 Humble [${KARIS_ROLE:-motion}]: ROS_DOMAIN_ID=$ROS_DOMAIN_ID, CycloneDDS on $_karis_if_msg"
if [ "$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)" -lt 8388608 ]; then
    echo "  Note: small network receive buffers; camera frames may be dropped. On the host (not in a container):"
    echo "        sudo sysctl -w net.core.rmem_max=33554432 net.core.rmem_default=8388608"
fi
unset _karis_ws _karis_install _karis_peers _karis_disc _karis_if_msg _p
