# robot_hardware_control

ROS 2 hardware-control baseline extracted from
`BK-Robotics/ROBOCON_2026` at commit
`574b1e5f241ee15131a3eb143241b0941b65fb75`.

This repository intentionally preserves the original package names,
namespaces, targets, IPC layout, topics, concurrency, safety behavior, and
launch assets. It is an extraction baseline only; no refactoring is included.

## Included source

- `hardware_system_handler`: HSH node and tinyfsm-based state machine.
- `odrive_can_interface`: ros2_control hardware plugin, SocketCAN backend,
  ODrive protocol implementation, and shared-memory IPC types.
- `tinyfsm`: minimal internal dependency pinned from upstream commit
  `01908cab0397fcdadb0a14e9a3187c308e2708ca`.

## External dependencies

The ROS dependencies remain declared by each package's `package.xml`.
Existing ODrive launch and URDF assets also reference full-robot packages such
as `swerve_controller`, `swerve_base_description`, and
`sensor_can_interface`. Those packages are intentionally not vendored here
because they are not build dependencies of HSH or the ODrive hardware plugin.

## Build

From the repository root in a sourced ROS 2 environment:

```bash
colcon build --symlink-install
```

Build verification was not performed during extraction because ROS 2 and
`colcon` were unavailable in the Windows environment used for the split.
