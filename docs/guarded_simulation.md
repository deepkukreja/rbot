# Guarded warehouse simulation

This checkpoint runs the Robot Buildworks differential-drive simulator with
AMCL, Nav2, and a local lease-based motion supervisor. The supervised
navigation adapter checks current localization and sensor health, owns the
public `NavigateToPose` action, and forwards fresh commands through the
supervisor. The simulator fixture exercises this path on one host.

## Build

Use Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic. The repository's
[`scripts/install_deps.sh`](../scripts/install_deps.sh) documents the native
dependency setup. The loopback fixture uses Fast DDS
(`ros-jazzy-rmw-fastrtps-cpp`). The unit tests use `python3-pytest`.
The checkpoint was built against Nav2 1.3.12, ros2_controllers 4.40.1,
and rmw_fastrtps 8.4.4.

Run these commands from the repository root in a fresh Bash shell.

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --packages-up-to rlai_bringup --executor sequential \
  --allow-overriding nav2_mppi_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

The workspace builds the [local MPPI patch](../src/navigation/nav2_mppi_controller/LOCAL_PATCH.md)
on Navigation2 1.3.12. The [coupled wheel limiter](../experiments/coupled_drive/README.md)
can be built and enabled separately in the same checkout.

## Verify in a private simulation

```bash
python3 -m pytest -q tests experiments/coupled_drive/test_profile.py \
  src/control/rlai_motion_supervisor/test \
  src/localization/rlai_localization/test src/navigation/rlai_navigation/test
```

Use a free domain in the fixture's supported range of 1–101. The command
below launches its own Gazebo and ROS processes, requests simulated motion,
and injects a frozen scan after motion begins. Run it only in its own simulator
session. The fixture chooses its Gazebo partition and writes local reports and
traces to a fresh directory.

```bash
run_dir="$(mktemp -d)"
python3 src/control/rlai_motion_supervisor/scripts/verify_route_scenario.py \
  --scenario nominal --domain 96 \
  --map "$(pwd -P)/maps/guarded_small_warehouse/nav2_map.yaml" \
  --out "$run_dir/frozen-scan" \
  --loopback-udp --scan-occlusion --scan-fault frozen
```

Read `report.json` in the output directory for the scenario result, individual
checks, stopping latencies, and owned-process cleanup. For interactive
launches and the command contract, see the
[guarded navigation guide](../src/control/rlai_motion_supervisor/GUARDED_NAVIGATION.md).
Source this checkout's install prefix in each shell and use
`ros2 run rlai_localization ros_loopback_udp --` for commands that join
the loopback session.

## Checkpoint results

The build selected 15 packages. The frozen-scan run passed all 12 scenario
checks. The supervisor inhibited motion with `SCAN_STALE` and zero output
was observed after 0.629 s (limit 0.85 s). Measured wheel standstill took
1.469 s (limit 1.5 s). Nav2 reported the goal aborted. Restoring the scan
did not reacquire the lease or resume the goal, and the fixture verified
cleanup of its own processes.

The first clean-build launch exposed a missing `rlai_meshes` runtime
dependency before the robot spawned. Both mesh consumers now declare it.
The passing run used the rebuilt workspace and the original motion limits.

## Drive startup readiness

Guarded simulation waits for controller-manager discovery and at least
50 ms of fresh forward `/clock` progress before requesting one grouped
activation of the joint-state broadcaster and differential-drive
controller. The clock gate uses a 20 s wall-clock budget. The controller
switch retains its 5 s deadline. A gate failure, spawner failure, or
35 s overall control-startup timeout fails the enclosing launch with
`GUARDED_CONTROL_STARTUP_FAILED`. Normal launch-process shutdown grace
follows the failure. Actuator supervision and stopping limits stay in
place.

The historical transition attempt timed out at a drive switch before
the first observed Gazebo ground-truth message. Those logs establish
the order of the events. A ROS clock trace was unavailable for that
attempt.

Five isolated startup checks used ROS 2 Jazzy, headless Gazebo Harmonic
where applicable, and the coupled drive profile. They requested no
leases or goals and observed no nonzero drive commands.

| Check | Observation |
| --- | --- |
| Original normal launch | Both controllers active with joint state and odometry at 12.82 wall s |
| Original invalid drive plugin | Joint-state broadcaster active. Launch still running at the end of 42 s observation |
| Guarded normal launch | One successful grouped switch after clock readiness. Both controllers active with joint state and odometry at 11.02 wall s |
| Guarded invalid drive plugin | Launch exited with status 1 and a controller startup failure |
| Guarded frozen clock with discoverable manager service | Launch exited with status 1. Controller spawner never started |

The healthy guarded launch advanced at about 0.50
simulated seconds per wall second in the two-second observation window
after readiness. This is a startup measurement from one run. See the
[stopping timing report](../experiments/coupled_drive/STOPPING_TIMING.md)
for the separate motion and command-expiry measurements.

## Map

[`nav2_map.yaml`](../maps/guarded_small_warehouse/nav2_map.yaml) and its
adjacent image were generated in Gazebo's `small_warehouse` world. The map
resolution is 0.05 m/pixel. The fixture uses world spawn (1 m, 1 m) for map pose
(0 m, 0 m), with the same yaw. Keep the YAML and image together.

This checkpoint's motion evidence covers the simulated 2D scan and warehouse
configuration. Hardware stopping and personnel safety need separate
qualification. The optional coupled limiter's
[published trial results](../experiments/coupled_drive/simulation_validation.json)
retain a 1.623 s standstill against the 1.5 s deadline.
