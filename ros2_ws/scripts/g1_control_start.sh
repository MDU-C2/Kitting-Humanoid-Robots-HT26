#!/usr/bin/env bash

set -e

echo "=== G1 ROS 2 Control Startup ==="

echo
echo "[1/4] Starting ros2_control..."
ros2 launch g1_control g1_control.launch.py &
LAUNCH_PID=$!

echo "ros2_control launch PID: ${LAUNCH_PID}"

echo
echo "[2/4] Waiting for controller_manager..."

until ros2 control list_hardware_components >/dev/null 2>&1; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "ERROR: g1_control launch exited while waiting for controller_manager."
        exit 1
    fi

    sleep 1
done

echo "controller_manager is available."

echo
echo "[3/4] Configuring G1ArmSdkSystem..."
ros2 control set_hardware_component_state G1ArmSdkSystem inactive

echo
echo "Waiting for fresh /lowstate data..."

if ! timeout 10 ros2 topic echo /lowstate --once >/dev/null 2>&1; then
    echo "ERROR: No /lowstate message received within 10 seconds."
    echo "G1ArmSdkSystem will remain inactive."
    exit 1
fi

echo "Fresh /lowstate received."

echo
echo "[4/4] Activating G1ArmSdkSystem..."
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
