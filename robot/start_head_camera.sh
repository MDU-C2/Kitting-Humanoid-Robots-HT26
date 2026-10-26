#!/usr/bin/env bash
# KARIS: start the G1 head camera as ROS 2 topics. Run ON THE G1 (unitree@192.168.123.164):
#     bash start_head_camera.sh                                  cable (eth0)
#     bash start_head_camera.sh --wifi --laptop 192.168.123.184  WiFi; --laptop = the laptop's address
#     bash start_head_camera.sh --wifi --laptop 192.168.123.184 --ros-args -p publish_rate:=10.0
#     bash start_head_camera.sh --wifi --laptop 192.168.123.184 --tf   + robot model TF (pelvis)
#
# Options (or the same as environment variables):
#   --wifi            use the robot's WiFi (wlan0) instead of its cable network (eth0)  [ROBOT_IF=wlan0]
#   --if NAME         any other interface                                               [ROBOT_IF=NAME]
#   --laptop IP       send to this computer directly (needed over WiFi when multicast  [LAPTOP_IP="IP IP2"]
#                     does not get through); repeat for several computers
#   --env NAME        conda env with pyrealsense2 + opencv                              [CONDA_ENV=NAME]
#   --tf              also publish TF pelvis -> torso_link -> d435_link from the real waist  [ROBOT_TF=1]
#                     joints (g1_tf_ros2.py, read only). NOT while the motion laptop runs its
#                     robot model (run_g1.sh / observe_g1_tf.sh): TF would have two sources.
#   --tf-rate HZ      TF rate (default 50)                                               [TF_RATE=HZ]
#   --sdk-python PATH a Python with unitree_sdk2py (default: found automatically)       [SDK_PY=PATH]
# Everything after them (e.g. --ros-args -p publish_rate:=10.0) goes to g1_camera_ros2.py.
#
# Sets up ROS 2 Foxy with CycloneDDS, frees the camera, and runs g1_camera_ros2.py with a
# Python that has pyrealsense2.

# ---- settings (defaults; options above override) -----------------------------
CONDA_ENV="${CONDA_ENV:-kitting38}"       # conda env with pyrealsense2 + opencv (Python 3.8, like Foxy)
ROBOT_IF="${ROBOT_IF:-eth0}"              # PC2's interface on 192.168.123.x
LAPTOP_IP="${LAPTOP_IP:-}"                # computers to send to directly (unicast peers)
ROBOT_TF="${ROBOT_TF:-0}"                 # 1 = also run g1_tf_ros2.py (robot model TF)
TF_RATE="${TF_RATE:-50}"
SDK_PY="${SDK_PY:-}"                      # Python with unitree_sdk2py, for --tf
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" # must be the same on every computer
# -----------------------------------------------------------------------------

while [ $# -gt 0 ]; do
    case "$1" in
        --wifi)   ROBOT_IF=wlan0; shift ;;
        --if)     ROBOT_IF="$2"; shift 2 ;;
        --laptop) LAPTOP_IP="$LAPTOP_IP $2"; shift 2 ;;
        --env)    CONDA_ENV="$2"; shift 2 ;;
        --tf)     ROBOT_TF=1; shift ;;
        --tf-rate) TF_RATE="$2"; shift 2 ;;
        --sdk-python) SDK_PY="$2"; shift 2 ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) break ;;
    esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. ROS 2 Foxy (+ Unitree's CycloneDDS 0.10 build, if it is there)
source /opt/ros/foxy/setup.bash
if [ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
    source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
_peers=""
for _p in $LAPTOP_IP; do _peers="$_peers<Peer Address=\"$_p\"/>"; done
_disc=""
[ -n "$_peers" ] && _disc="<Discovery><Peers>$_peers</Peers></Discovery>"
if [ -n "$CYCLONEDDS_URI" ]; then
    echo "Using CYCLONEDDS_URI from this shell (unset CYCLONEDDS_URI to let --wifi/--laptop set it)"
else
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${ROBOT_IF}</NetworkInterfaceAddress></General>$_disc</Domain></CycloneDDS>"
fi
if ! ip -4 -o addr show "$ROBOT_IF" 2>/dev/null | grep -q inet; then
    echo "WARNING: interface $ROBOT_IF has no IPv4 address (cable: eth0, WiFi: --wifi)"
fi

# 2. Python with pyrealsense2 (conda env), still able to import Foxy's rclpy (both Python 3.8)
if [ -f "$HOME/miniconda3/bin/activate" ] && [ -d "$HOME/miniconda3/envs/$CONDA_ENV" ]; then
    source "$HOME/miniconda3/bin/activate" "$CONDA_ENV"
fi
PY=python3
if ! $PY -c "import pyrealsense2, cv2, rclpy" 2>/dev/null; then
    echo "ERROR: '$($PY -c 'import sys; print(sys.executable)')' cannot import pyrealsense2, cv2 and rclpy."
    $PY -c "import pyrealsense2" 2>&1 | tail -1
    $PY -c "import cv2" 2>&1 | tail -1
    $PY -c "import rclpy" 2>&1 | tail -1
    echo "Set CONDA_ENV to an env with Python 3.8 + pyrealsense2 + opencv, e.g.  CONDA_ENV=teleimager bash $0"
    exit 1
fi

# 3. Free the camera (only one program can use it)
if systemctl is-active --quiet teleimager.service 2>/dev/null; then
    echo "Stopping teleimager.service (it holds the camera)..."
    sudo systemctl stop teleimager.service
fi

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  interface=$ROBOT_IF ($(ip -4 -o addr show "$ROBOT_IF" 2>/dev/null | awk '{print $4}'))  laptop peers:${LAPTOP_IP:- none}  python=$(command -v $PY)"

# 4. Robot model TF (optional, own processes: its own rate, and a camera restart does not stop it).
#    g1_waist_reader.py (Unitree SDK, rt/lowstate on eth0, read only) | g1_tf_ros2.py (ROS 2, TF)
TF_PID=""
if [ "$ROBOT_TF" = 1 ]; then
    if [ -z "$SDK_PY" ]; then
        for _c in "$HOME/motiontest/bin/python" "$HOME/miniconda3/envs/tv/bin/python" \
                  "$HOME/miniconda3/envs/inspire/bin/python"; do
            if [ -x "$_c" ] && env -u PYTHONPATH "$_c" -c "import unitree_sdk2py" 2>/dev/null; then SDK_PY="$_c"; break; fi
        done
    fi
    if [ -z "$SDK_PY" ] || ! env -u PYTHONPATH "$SDK_PY" -c "import unitree_sdk2py" 2>/dev/null; then
        echo "ERROR: --tf needs a Python with unitree_sdk2py; give one with --sdk-python PATH."
        exit 1
    fi
    echo "Robot model TF: pelvis -> torso_link -> d435_link at ${TF_RATE} Hz (read only, rt/lowstate via eth0, SDK python $SDK_PY)"
    ( env -u PYTHONPATH -u CYCLONEDDS_URI "$SDK_PY" -u "$HERE/g1_waist_reader.py" --if eth0 \
        | /usr/bin/python3 "$HERE/g1_tf_ros2.py" --ros-args -p publish_rate:="$TF_RATE" ) &
    TF_PID=$!
    trap '[ -n "$TF_PID" ] && pkill -TERM -P "$TF_PID" 2>/dev/null; kill "$TF_PID" 2>/dev/null; wait 2>/dev/null' EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    $PY "$HERE/g1_camera_ros2.py" "$@"
else
    exec $PY "$HERE/g1_camera_ros2.py" "$@"
fi
