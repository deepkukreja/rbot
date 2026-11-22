# Optional coupled wheel limits in supervised Gazebo

The driver and MPPI use a shared wheel-speed envelope and advance the linear and angular components together within existing axis rate limits. The Gazebo actuator stays behind the local motion supervisor. The maximum wheel speed comes from the generated robot description, and the planner and driver receive identical effective wheel geometry. Profile generation rejects unknown combinations of actuator, planner, wheel limits, and acceleration settings.

For the development simulator, the prior test configuration used 5 rad/s per wheel, 0.0625 m wheel radii and 0.3101 m effective wheel separation. The original turn-to-straight limiter demanded 7.21 rad/s. The coupled driver remained within 5 rad/s during the same transition. Previous simulation evidence includes 52 passing checks across fixed-input motion, obstacle appearance and reentry, and frozen scan handling. The frozen-scan run stopped the wheels in 1.432 s against a 1.5 s deadline. One attempted run timed out before motion when joint-state broadcaster activation failed.

The pinned reconstruction reproduced the wheel and acceleration bounds and retained a stopping failure. In the rebuilt transition run, output reached zero after 0.319 s and wheel standstill took 1.623 s, exceeding the 1.5 s wall-clock deadline. The earlier matching transition took 1.422 s. From the wheel sample nearest zero output to standstill, simulation advanced by 0.410 s in the rebuilt run and 0.400 s earlier. Their simulated-to-wall clock rates were approximately 0.314 and 0.373. The separate rebuilt frozen-scan run passed all 12 checks and reached wheel standstill in 1.160 s. See [the aggregate results](simulation_validation.json) for the per-scenario outcomes and acquisition timing. Loaded stopping remains open, with priority on simulator scheduling and clock progress at the original deadline.

## Reproduce the overlay

Use ROS 2 Jazzy, Gazebo Harmonic and a built **guarded rbot development workspace** containing `rlai_motion_supervisor` and `supervised_navigation.launch.py`. A stock upstream rbot checkout does not include that guarded runtime. Install the upstream packages' build dependencies in the ROS environment. This directory carries pinned source inputs and a profile builder. It is excluded from ordinary colcon discovery.

From the repository root in Bash, source ROS and the guarded workspace, then run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rbot/install/setup.bash
profile="$(pwd -P)/experiments/coupled_drive"
runtime="$HOME/rbot-coupled-runtime"
mkdir -p "$runtime"
python3 "$profile/profile_overlay.py" "$runtime/profiles"
python3 "$profile/source_overlay.py" "$runtime/src" --archive-cache "$runtime/archives"
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon --log-base "$runtime/log" build \
  --base-paths "$runtime/src" --build-base "$runtime/build" --install-base "$runtime/install" \
  --executor sequential --allow-overriding diff_drive_controller nav2_mppi_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

Use an empty runtime directory for a new configuration. You can prefill the archive cache with the files named in `sources.json` for offline preparation. Both pinned upstream archives and every modified source file are SHA256-checked. The builder retains upstream licenses and applies the checked-in patches to the exact releases.

In each launch shell source the normal guarded workspace first. Use a free, nonzero ROS domain and a unique Gazebo partition for this simulation. Use the same values and activation script in both the bringup and navigation shells:

```bash
export ROS_DOMAIN_ID=70 GZ_PARTITION=rbot_coupled_70
source "$profile/activate.bash" "$runtime"
# Bringup shell, with the path to your validated map:
ros2 run rlai_localization ros_loopback_udp -- ros2 launch rlai_bringup simulation.launch.py \
  headless:=true teleop_enabled:=false motion_supervision_enabled:=true \
  motion_health_enabled:=true use_amcl:=true map_yaml_file:=/path/to/validated-map.yaml
# Separate navigation shell, after sourcing the same profile:
ros2 run rlai_localization ros_loopback_udp -- ros2 launch rlai_navigation supervised_navigation.launch.py
```

Closing those shells discards the opt-in environment. Begin from a fresh shell to generate profiles after changing the base robot or navigation configuration. The manual launch requires the same supervised motion topology, exclusive driver input, map and clock assumptions as the guarded rbot runtime. The results cover Gazebo. Loaded scheduling margins and physical stopping have not been qualified.

## Review and verify

Run `python3 -m pytest -q experiments/coupled_drive/test_profile.py` to check paired parameters and rejection of invalid configurations. In a shell with the activated overlay and a private ROS domain, build and run the two native probes:

```bash
for probe in driver mppi; do
  cmake -S "$profile/${probe}_probe" -B "$runtime/${probe}_probe"
  cmake --build "$runtime/${probe}_probe" --parallel 1
  ros2 run rlai_localization ros_loopback_udp -- "$runtime/${probe}_probe/${probe}_probe"
done
```

The shared header's regression is available using `BUILD_TESTING=ON` for `rlai_drive_constraints`.

The driver change is based on [ros2_controllers 4.40.1](https://github.com/ros-controls/ros2_controllers/tree/4.40.1/diff_drive_controller). The planner change builds on the [Navigation2 1.3.12 rollout](https://github.com/ros-navigation/navigation2/tree/1.3.12/nav2_mppi_controller). The [Jazzy controller interface](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) identifies task-space limits and the body-twist-to-wheel boundary. Separate upstream axis limiters leave the coupled envelope to this profile. Newer [Nav2 motion-model plugins](https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/README.md) are a possible migration path when upgrading the stack. The current build uses pinned Jazzy versions.
