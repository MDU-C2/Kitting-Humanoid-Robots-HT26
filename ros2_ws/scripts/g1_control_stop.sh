#!/usr/bin/env bash

set -e

echo "=== G1 ROS 2 Control Shutdown ==="

echo
echo "[1/2] Deactivating arm trajectory controller..."
ros2 control set_controller_state arm_trajectory_controller inactive

echo
echo "[2/2] Deactivating G1 arm hardware interface..."
echo "Waiting for controlled arm SDK ramp-down..."
ros2 control set_hardware_component_state G1ArmSdkSystem inactive

echo
echo "=== Controller state ==="
ros2 control list_controllers

echo
echo "=== Hardware state ==="
ros2 control list_hardware_components

echo
echo "G1 ROS 2 arm control shutdown complete."
