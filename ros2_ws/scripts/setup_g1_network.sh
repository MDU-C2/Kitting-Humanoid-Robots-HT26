#!/usr/bin/env bash

# Configure the current shell for ROS 2 communication with the Unitree G1.
#
# IMPORTANT:
# This script must be sourced, not executed:
#
#   source /workspace/scripts/setup_g1_network.sh
#
# This is necessary because the RMW_IMPLEMENTATION and CYCLONEDDS_URI
# environment variables must be exported into the current shell.

G1_ROBOT_IP="192.168.123.161"

# Ask Linux which network interface it would use to reach the G1.
G1_NETWORK_INTERFACE="$(
    ip route get "${G1_ROBOT_IP}" 2>/dev/null |
    awk '{for (i = 1; i <= NF; i++) if ($i == "dev") {print $(i+1); exit}}'
)"

# A route alone is not enough. Verify that the G1 actually responds
# through the selected interface.
if [ -z "${G1_NETWORK_INTERFACE}" ] ||
   ! ping -I "${G1_NETWORK_INTERFACE}" -c 1 -W 1 "${G1_ROBOT_IP}" >/dev/null 2>&1; then

    echo "G1 network not detected."
    echo "Could not reach ${G1_ROBOT_IP}."
    echo "ROS 2 DDS configuration was not changed."

    unset G1_NETWORK_INTERFACE
    return 1 2>/dev/null || exit 1
fi

# Use CycloneDDS and explicitly bind it to the interface that reaches the G1.
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${G1_NETWORK_INTERFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

echo "G1 network detected."
echo "Robot IP: ${G1_ROBOT_IP}"
echo "Network interface: ${G1_NETWORK_INTERFACE}"
echo "RMW implementation: ${RMW_IMPLEMENTATION}"
echo "CycloneDDS configured for ${G1_NETWORK_INTERFACE}."
