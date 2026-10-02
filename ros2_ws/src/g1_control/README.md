# G1 Control

This package contains the ROS 2 control configuration and launch integration
for controlling the Unitree G1 arms through `ros2_control`.

## Hardware Interface Lifecycle

The `G1ArmSdkSystem` hardware interface is intentionally initialized in the
`unconfigured` state by `controller_manager`.

The lifecycle is:

```text
UNCONFIGURED
     |
     | on_configure()
     v
 INACTIVE
     |
     | on_activate()
     v
  ACTIVE
```

## Unconfigured State

The hardware interface starts in the `unconfigured` state as a safe lockout
state. In this state, the arm command interfaces are unavailable and cannot be
claimed by the trajectory controller.

Communication with the G1 is not initialized until the hardware interface is
explicitly configured. Transitioning to `inactive` therefore acts as an
explicit preparation step before robot communication and state feedback are
enabled.

## Inactive State

During `on_configure()`, the hardware interface initializes communication with
the G1 by:

- creating the internal ROS 2 node,
- subscribing to `/lowstate`,
- creating the `/arm_sdk` publisher, and
- starting the internal executor.

The hardware interface does not publish arm commands while inactive.

This allows communication and robot-state reception to be established before
the hardware interface is explicitly given command authority.

## Controller Startup

Both `joint_state_broadcaster` and `arm_trajectory_controller` are loaded and
configured in the `inactive` state during startup.

This prevents the controllers from being activated while the hardware
interface is still `unconfigured`. After the hardware has been explicitly
configured, the required controllers can be activated as part of the
controlled startup sequence.

## Activation

Activation must be performed explicitly.

During `on_activate()`, the hardware interface:

1. verifies that recent `/lowstate` data is available,
2. initializes the arm command positions from the currently measured joint
   positions,
3. stores the current waist position for hold control, and
4. enables the active command mode.

If the `/lowstate` data is stale, activation is rejected.

This startup sequence is intentional so that starting the ROS 2 control system
does not immediately enable arm command output.

## References

See the [ROS 2 Control Controller Manager documentation](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)
for information about hardware component lifecycle management.
