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
