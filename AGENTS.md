# AGENTS.md

## Scope

These instructions apply to the entire `robot_hardware_control` repository.
This is a ROS 2 Humble workspace containing a safety-relevant motor-control
path. Treat changes to shared memory, HSH state transitions, command gating,
ODrive actions, watchdogs, homing, and unit conversion as high risk.

## Repository map

- `src/OdriveCanHardwareInterface/odrive_can_interface`: ros2_control hardware
  plugin, SocketCAN transport, and ODrive protocol.
- `src/hardware_ipc_common`: shared-memory names, layouts, snapshot protocol,
  and POSIX transport used by HWI, HSH, sensors, simulators, and viewers.
- `src/HardwareSystemHandler`: HSH ROS node and tinyfsm state machine. HSH is
  the authority that decides whether motion is allowed.
- `src/vehicle_controller`: ros2_control vehicle controller.
- `src/vehicle_keyboard_teleop`: operator command utility.
- `src/vehicle_msgs`: ROS message definitions used by the vehicle layer.
- `src/tinyfsm`: vendored dependency; avoid modifying it unless the task
  explicitly requires an upstream fix.

The intended command path is:

```text
controller -> ros2_control HWI command snapshot -> HSH safety/FSM
           -> HSH control snapshot -> ODrive CAN backend -> hardware
```

## Build and verification

Run commands from the workspace root, one directory above this repository:

```bash
cd /home/ddon/robot_hardware_ws
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

For a focused iteration, use `--packages-select <package>`. Build every
dependent package after changing exported headers, message definitions, IPC
layouts, or plugin interfaces. Do not claim runtime or hardware verification
from a compile-only result. State explicitly when SocketCAN/ODrive hardware was
not available.

There are currently no meaningful automated tests in the baseline. New IPC,
FSM, safety-policy, or conversion behavior must include tests where practical;
do not use a successful zero-test `colcon test` run as evidence of correctness.

## Shared-memory rules

- Preserve a single documented writer for each block. Readers must consume a
  copied, validated snapshot; do not retain or make decisions from a raw mmap
  pointer while another process can write it.
- Latest-only data (targets and feedback) belongs in snapshots. One-shot
  operations (clear errors, homing, idle, enable, disable, calibration, reset)
  require monotonic request IDs and acknowledgement IDs; do not encode them as
  a transient value in the same field as cyclic targets.
- Safety overrides such as emergency stop, critical fault, offline state, and
  command timeout bypass queues and must force a safe output deterministically.
- Do not use `msync()` as inter-process synchronization. IPC changes should use
  a reviewed snapshot protocol such as a seqlock or double buffer, with explicit
  memory ordering and retry/failure behavior.
- IPC layout changes require a magic value, schema version, payload size, fixed
  width fields, alignment checks, and `static_assert` checks for the supported
  ABI. Update all producers, consumers, viewers, simulators, and tests together.
- Shared-memory payloads must be trivially copyable and must not contain owning
  pointers, ROS objects, mutexes, or non-trivial lifetime state.
- Opening one process must not silently erase live data owned by another
  process. Any initialization/reset policy must identify the creator and be
  safe under different startup orders.

## Control and safety invariants

- HWI lifecycle `ACTIVE` does not imply that motion is permitted. Only an HSH
  decision based on fresh, valid inputs may enable wheel motion.
- The fast command path must fail safe on stale/invalid state, invalid axis
  counts, offline axes, critical faults, sensor/watchdog timeout, or invalid
  numeric commands. Clear stale targets when disabling an axis.
- Keep units explicit at every boundary. ROS position/velocity values are in
  rad and rad/s; ODrive targets may use turns and turns/s. Test conversions in
  both directions and do not rely on comments alone.
- Brake/position-axis behavior is hardware-profile-specific. Preserve the
  distinction between brake position axes and wheel velocity axes, and require
  an explicit safe configuration before bypassing homing.
- A software ACK only means that the CAN backend accepted/sent a request. It is
  not proof that ODrive completed the operation; completion must come from fresh
  hardware feedback.
- Avoid logging, allocation, blocking I/O, or synchronous filesystem flushes in
  the 5 ms command/CAN paths. Profiling in fast paths should use bounded,
  preallocated storage.

## Design direction

Prefer these boundaries for new work:

- `hardware_ipc_common`: IPC headers, layouts, snapshot primitives, versioning,
  and ownership documentation shared by all producers and consumers.
- A pure C++ HSH core with a deterministic `step(input, now) -> output` API.
- A thin ROS/SHM adapter responsible only for timers, snapshot I/O, parameters,
  and diagnostics.
- A fast safety gate evaluated at the command cadence, with the slower FSM used
  for sequencing, recovery, and diagnostics.

Do not merge HSH into the controller process or add a priority queue merely to
work around action overwrites. First fix IPC consistency and one-shot ownership,
then measure latency and jitter before changing process topology.

## Change discipline

- The worktree may contain active hardware-profile changes. Inspect `git status`
  and `git diff` first, preserve unrelated edits, and never discard them to make
  a patch easier.
- Keep refactors staged in reviewable slices: tests/contract, IPC primitive and
  layout, producer migration, consumer migration, then removal of the legacy
  path. Do not mix a broad FSM rewrite with an untested IPC ABI migration.
- For safety-relevant behavior, document the safe default, timeout, retry limit,
  latching/reset rule, and evidence used to choose tunable values.
- Do not run calibration, homing, closed-loop motion, or raw CAN transmit tests
  on physical hardware unless the user explicitly authorizes that operation and
  the mechanism is secured for the test.
