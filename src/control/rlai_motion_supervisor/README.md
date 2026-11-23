# Local motion ownership — development profile

This adds exclusive local ownership, bounded stamped commands, monotonic expiry and a latched **software** stop. It is not an independent emergency stop, obstacle/clearance supervisor, authenticated remote-control system or physical safety validation.

The legacy profile remains the default. Select the tested supervised profile explicitly:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rbot/install/setup.bash
ros2 launch rlai_bringup simulation.launch.py motion_supervision_enabled:=true \
  headless:=true teleop_enabled:=false camera_processing_enabled:=false
```

In this profile the existing velocity smoother/lifecycle manager are disconnected. The only configured publisher to `/diff_drive_controller/cmd_vel` is `motion_supervisor`. Controller velocity/acceleration limits and its own command timeout still apply. No raw `/cmd_vel` subscriber grants ownership. Legacy joystick/toggle-stop launch is rejected before Gazebo starts. Nav2/joystick adapters must become lease-aware before they can drive in this profile; there is no automatic acquisition based on arriving velocity packets.

## Contract

Endpoints are relative to the ROS namespace:

| Endpoint | Interface | Behavior |
|---|---|---|
| `motion_supervisor/acquire` | `AcquireControl` | Owner label, source `AUTONOMY` or `TELEOP`, lease duration 1..2000 ms. One owner, no automatic preemption; returns a new capability token. |
| `motion_supervisor/renew` | `RenewControl` | Requires the current token; extends the lease, never the command deadline. |
| `motion_supervisor/release` | `ReleaseControl` | Current token only; immediately publishes zero and revokes ownership. |
| `motion_supervisor/command` | `LeasedTwist` | Current token, increasing uint64 sequence, host SYSTEM/UTC issuance time, lifetime 1..300 ms and planar twist. |
| `motion_supervisor/set_stop` | `std_srvs/SetBool` | True latches zero/revokes ownership; repeated true is idempotent. False clears the latch without resuming or restoring an old lease. |
| `motion_supervisor/status` | `SupervisorStatus` | State/owner/source/reason, remaining lease, accepted sequence; no token. |
| `motion_supervisor/drive` | `TwistStamped` | Standalone unconnected output; control launch explicitly remaps it to the simulated controller. |

Command issuance uses the **local host's system UTC**, not `/clock`. A local adapter must validate any future remote request's original issuance, expiry, operator authorization and session before forming this envelope; blindly restamping delayed remote input would defeat its age contract. Commands over 50 ms in the future, expired commands, old sequences and wrong tokens are rejected and do not extend deadlines. Accepted command expiry includes transit age. Host-clock rollback relative to steady elapsed time latches a clock fault until process restart.

No first command within 300 ms of acquisition revokes the lease. Subsequent command/lease expiry revokes it and publishes zero. Commands cannot reacquire after loss or restart; clients must explicitly acquire a new lease and send a fresh command. A steady-clock timer checks at 20 Hz even when ROS simulation time is paused. This is best-effort Python scheduling, not a certified deadline guarantee. Release/stop/invalid owned velocity also publish zero in their callback.

Finite planar bounds mirror the inspected simulator: forward 0.5 m/s, reverse 0.35 m/s, yaw 1.9 rad/s. Unsupported axes, nonfinite values and out-of-range owned commands revoke ownership immediately; they are not silently clamped. These are existing simulator settings, not measured physical vehicle limits. There is no new acceleration smoother; existing controller acceleration limits remain active.

Lease IDs are unguessable per-acquisition process-memory capabilities and are excluded from status/logs/evidence. Owner/source labels are diagnostic claims, **not authenticated identities**. The ROS domain must remain local/trusted. DDS/SROS2 access control, authenticated clients, exclusive deployment and restrictions on direct driver-topic publishing are required before exposing control to other machines. A node able to publish directly to the controller topic can bypass this application graph; this milestone does not claim a security fence around unrestricted ROS.

The existing diff-drive controller's command timeout is retained (installed default 0.5 s). It provides a separate software fallback if the supervisor process stops publishing. It is not independent emergency-stop hardware and depends on controller scheduling/clock behavior. The ownership-only profile leaves health admission disabled. The guarded-navigation extension requires direct sensor/localization health; neither profile establishes obstacle clearance from `OWNED` status.

## Reproduce

```bash
cd ~/rbot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rlai_motion_interfaces rlai_motion_supervisor rlai_control rlai_gazebo rlai_bringup
source install/setup.bash
python3 -m pytest -q src/control/rlai_motion_supervisor/test tests
python3 src/control/rlai_motion_supervisor/scripts/verify_ros.py --domain 73 --out /tmp/motion-ros-check-01
python3 src/control/rlai_motion_supervisor/scripts/verify_gazebo.py --domain 74 --out /tmp/motion-gazebo-check-01
```

Choose unused nonzero ROS domains and new output directories. Both runners clean up only owned children and never use the live domain 0. The ROS runner uses an unconnected sink/frozen clock; the Gazebo runner moves a small isolated simulated robot briefly, tests stop/loss, deliberately kills only its own supervisor child and verifies downstream timeout. These commands do not operate physical hardware.

Next: lease-aware Nav2/manual adapters tied to goal/operator lifecycle, local sensor/localization inhibits, bounded deployment/authentication and then mission/remote-assistance requests. No fleet write endpoint, WebRTC path or cloud motion command was added.


## Guarded navigation extension

See [GUARDED_NAVIGATION.md](GUARDED_NAVIGATION.md) for the tested lease-aware NavigateToPose proxy, direct local health admission, lifecycle checks, cancellation quarantine and reproduction scripts. Public status messages gained health fields in interface version 0.2.0; rebuild and restart affected nodes together. Legacy defaults remain unchanged, and the guarded proxy refuses the ownership-only profile.
