#!/usr/bin/env bash

set -e

echo "=== G1 ROS 2 Control Startup ==="

echo
echo "[1/3] Starting ros2_control..."
ros2 launch g1_control g1_control.launch.py &
LAUNCH_PID=$!

echo "ros2_control launch PID: ${LAUNCH_PID}"

echo
echo "[2/3] Waiting for controller_manager..."

until ros2 control list_hardware_components >/dev/null 2>&1; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "ERROR: g1_control launch exited while waiting for controller_manager."
        exit 1
    fi

    sleep 1
done

echo "controller_manager is available."

echo
echo "[3/3] Activating G1ArmSdkSystem..."
echo "This requires fresh /lowstate data from the G1."

ros2 control set_hardware_component_state G1ArmSdkSystem active

echo
echo "=== Hardware state ==="
ros2 control list_hardware_components

echo
echo "=== Controller state ==="
ros2 control list_controllers

echo
echo "G1 hardware interface is active."
echo "arm_trajectory_controller has NOT been activated automatically."
echo "Validate the robot state before activating the trajectory controller."
