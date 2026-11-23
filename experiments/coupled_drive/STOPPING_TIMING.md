# Turn-to-straight command expiry in Gazebo

The original **1.623 s** standstill miss is reproducible with this profile.
Two new four-CPU trials missed the same **1.5 s** wall-clock criterion.
A six-CPU trial passed at **1.192 s**. All three completed trials used the
same command sequence, met the supervisor zero-output deadline, and stayed
within the wheel-speed and configured acceleration limits.

| New trial | CPUs | Added workers | Supervisor zero (s) | Direct joints (s) | Gazebo body (s) | Scored odometry (s) | Result |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| First baseline | 4 | 0 | — | — | — | — | Drive activation timed out before motion |
| Loaded | 4 | 2 | 0.322 | 1.329 | 1.312 | 1.593 | Missed standstill limit |
| Post-load baseline | 4 | 0 | 0.326 | 1.296 | 1.346 | 1.548 | Missed standstill limit |
| Wider baseline | 6 | 0 | 0.327 | 1.028 | 1.031 | 1.192 | Passed all 10 checks |

All times are wall seconds from the final command. The supervisor had
0.85 s to reach zero. The original 1.5 s check uses the controller's
position-derived, rolling-mean odometry twist. The direct-joint column
converts simulated joint velocities to linear and yaw speeds using the
effective wheel geometry. All three standstill indicators use limits of
0.005 m/s and 0.02 rad/s, with samples required to remain below their
thresholds through the observed tail. The two diagnostic channels provide
additional observations. The original odometry score determines pass/fail.

The historical rebuilt run crossed the same direct-joint threshold at
1.350 s and the Gazebo body threshold at 1.382 s. Its scored odometry
crossed at 1.623 s. The 10-sample rolling velocity estimator and separate
odometry publication cadence contribute to the difference. The samples do
not identify an exact delay for either. See the
[diff-drive odometry source at 4.40.1](https://github.com/ros-controls/ros2_controllers/blob/4.40.1/diff_drive_controller/src/odometry.cpp).

## Simulation time and scheduling

From supervisor zero to the scored threshold, paired odometry samples
advanced by 0.402, 0.381 and 0.402 simulation seconds across the three new
completed runs. The corresponding simulation-to-wall rates were 0.313,
0.316 and 0.463. Gazebo ground-truth timestamps independently gave rates
of 0.307, 0.316 and 0.460 over their closest matching sample pairs.
The historical 1.623 s run advanced by 0.410 simulation seconds at a
rate of 0.314. These are local estimates over the braking and odometry
settling window.

Limited-drive command timestamps reached zero 0.310–0.321 simulation
seconds after the raw joint stamp nearest supervisor zero across the
three new trials. The p95 interval between received limited commands
was 53.0, 44.6 and 30.2 wall ms. For their source timestamps the
corresponding intervals were 15.8, 13.0 and 13.0 simulation ms. Gazebo
drives controller updates with its simulation time. The longer wall
intervals are consistent with slower simulation progress. These message
samples cannot count every controller update or attribute scheduling delay
to a specific thread. See the
[gz_ros2_control 1.2.20 update path](https://github.com/ros-controls/gz_ros2_control/blob/1.2.20/gz_ros2_control/src/gz_ros2_control_plugin.cpp).

The added workers received about 0.69 core-equivalents combined during
the measured interval. The two four-CPU motion trials had nearly the
same simulation clock progress despite the different load. The wider
allocation is one passing observation on a shared host. The sample
does not establish a causal effect size, a minimum CPU requirement, or a
worst-case stopping margin.

## Trial inputs

- [Guarded workspace and warehouse map](../../docs/guarded_simulation.md)
  at commit `38df273`, with the pinned coupled controller and planner
  overlay. Gazebo Harmonic, ROS 2 Jazzy, gz_ros2_control 1.2.20. The private
  observation profile enables limited-velocity publication.
- A health-admitted lease sent 121 freshly timestamped commands at
  20 ms simulation offsets. The first 61 asked for 0 m/s and 1.9 rad/s.
  The last 60 asked for 0.3125 m/s and 0 rad/s. Each expires in 300 ms.
  The fixture sent no further commands, renewal, or goal after expiry.
- The first three trials allowed CPUs 8, 10, 12 and 14. The loaded trial
  started two bounded integer-loop workers on CPUs 8 and 10 after startup.
  The follow-up added CPUs 4 and 6 to the simulator allocation with no
  added workers. Other host workload was uncontrolled throughout.
- Each completed trial observed all 121 command sequences and matched
  the active controller and planner libraries. The peak limited wheel
  command was 5 rad/s. The tests observed lease expiry and confirmed no
  reacquisition. All four fixture domains were empty after owned cleanup.

[Aggregate measurements](stopping_timing_results.json) include the startup
failure, both historical runs, the timing windows, and available checks.
Raw traces and bags are local. These results cover the Gazebo
command-expiry fixture. The separate frozen-scan scenario has its own
[validation results](simulation_validation.json).
