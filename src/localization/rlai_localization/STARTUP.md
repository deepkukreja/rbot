# Bounded localization startup

Both localization launches retain the stock Nav2 lifecycle manager and bonds.
A local `bounded_lifecycle` relay owns the public map_server/AMCL change_state
and get_state services. The actual services are remapped under the relative
`_localization_control/<node>/` prefix. Other node interfaces and names stay
the same. Do not send lifecycle requests directly to those internal services.

Before advertising the public lifecycle services, the relay requires both
actual private services to be discoverable and both actual states to be
UNCONFIGURED. These read-only discovery probes may repeat under the launch
deadline; no change request is sent during this admission gate.

For each supported change the relay checks a fresh initial primary state,
dispatches at most one request, and verifies a fresh target state. If the
change reply times out but that target is confirmed, it reports success with
reason RECONCILED. A denial, unknown state, wrong state, unsupported transition,
or unsent request cannot establish success. Concurrent changes for one node
are refused. No change-state retries or automatic stop clearing are performed.

Client availability and response share a steady-clock budget: 3 seconds for
change_state and 1.5 seconds per state query. Late pending responses are
removed. Public get_state returns the actual state when available, otherwise
UNKNOWN (id 0, label unavailable); it never returns a cached primary state.
This lets stock hard-reset processing continue past a missing node.

A separate launch-process wall timer requires confirmation of manager-active
and both nodes ACTIVE within `localization_startup_timeout` (default 25 seconds,
finite permitted range 5 to 120). It does not depend on ROS time or the relay
executor. If readiness is not confirmed, launch reports
LOCALIZATION_STARTUP_TIMEOUT, exits unsuccessfully, and terminates the owned
launch tree using normal launch signal escalation. Termination adds the
configured process shutdown grace to the readiness deadline. An unexpected
relay exit also fails the launch. AMCL-disabled launches start neither this
relay nor its deadline.

Readiness is communicated through a dedicated stdout protocol line only after
real service responses; launch handles split output chunks. The relay remains
running to serve lifecycle requests after startup. The startup deadline is
then satisfied; stock bonds and the independent motion-health guard handle
runtime faults. This is not a new runtime recovery manager or a guarantee
against arbitrary middleware loss after startup.

`config/localization_service_replies.xml` gives service reply writers a 2 second
reader-discovery wait in map_server, AMCL and the relay. Topic writer defaults
and transports are unchanged. No RMW_FASTRTPS_USE_QOS_FROM_XML override is
required. Other middleware implementations ignore the Fast DDS XML setting.

Use `localization_fastdds_profile:=/absolute/path/profile.xml` for custom XML.
An inherited FASTRTPS_DEFAULT_PROFILES_FILE is preserved by default. Merge
the bundled service profile if that setting is wanted. The launch deadline
still bounds failed startup independently of custom reply settings; process
shutdown grace and OS scheduling also contribute to total exit time.

Neither node-state reconciliation nor startup readiness is a motion command.
These components never request a pose reset, acquire a drive lease, clear a
software stop, or send a navigation goal. Existing local motion admission
and explicit localization preflight remain required.

Verification: run the localization and supervisor tests plus the existing
Gazebo asset tests. Real service-reply-loss, paused-clock startup timeout,
invalid-map, AMCL crash and full guarded-route evidence is retained with
the startup_recovery milestone. Fault interposers are diagnostic-only and
are never installed in the runtime package.


## Explicit loopback UDP mode for local development

On this WSL/Jazzy environment, native stacks showed map_server and AMCL waiting
inside Fast DDS shared-memory input-channel creation. LOCALHOST discovery in
rmw_fastrtps 8.4.4 explicitly adds SHM and UDP transports, so setting only
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 does not remove SHM.

Use the installed ros_loopback_udp wrapper for a fresh local session:

    ros2 run rlai_localization ros_loopback_udp -- ros2 launch rlai_localization localization_amcl.launch.py map_yaml_file:=/absolute/path/map.yaml

The wrapper replaces only its child process environment. It selects the
bundled loopback_udp.xml and SYSTEM_DEFAULT discovery so the middleware leaves
XML discovery/transport configuration intact. That XML disables built-in
transports, supplies one UDPv4 transport restricted to 127.0.0.1, and uses
localhost unicast initial peers. The service reply budget remains 2 seconds.
There is no shared-memory transport in this mode. Network interfaces, shell
startup files and other running ROS sessions are not reconfigured.

Use the same wrapper for all processes in the local session, including
simulation, navigation, observation and recording. A process that still uses
LOCALHOST discovery can reintroduce its own shared-memory waits. This mode is
for one host; multi-host robots need a separately reviewed network profile.
UDP may have different CPU and bandwidth costs; dense camera/3D data performance
is not qualified by the warehouse route. Existing motion-health limits still
apply.

An inherited custom FASTRTPS_DEFAULT_PROFILES_FILE is refused rather than
silently replaced. Merge its settings with a loopback transport configuration
and select it explicitly with --profile /absolute/path/merged.xml. An existing
non-Fast-DDS middleware choice is also refused. The wrapper preserves domain
and Gazebo partition, and removes obsolete ROS_LOCALHOST_ONLY and built-in
transport overrides in its child environment.

The existing isolated route runner accepts --loopback-udp. It applies this
configuration before creating its ROS observer and passes it to both owned
launchers. The ordinary route mode is unchanged. Never use simulated test
motion requests in an operator session or on a physical robot.

No /dev/shm cleanup or existing-session restart is required to select the mode.
The stock lifecycle bonds and independent startup deadline remain enabled.


## AMCL cadence investigation — 2026-11-15

The current default remains `update_min_d=0.20` m and `update_min_a=0.20` rad.
Slow final approaches can remain below those motion thresholds while the
existing 5 s AMCL freshness limit expires. The `outputs/amcl_cadence/` evidence
bundle in the orchestration workspace preserves a recorded-input reproduction.

Actual AMCL replay with the same 48 scan packets, odometry and clock inputs
produced no subsequent pose with 0.20 m and two updates with a 0.05 m
translation threshold. Peak pose age changed from 5.570 s to 2.171 s in that
window. Gaussian initialization and anchor-scan reuse do not reconstruct the
historical particle state; installed AMCL randomness is time-seeded.

The 0.05 m candidate passed one full private appearance/removal/reentry trial.
A subsequent default-launch trial aborted on raw Nav2 angular commands above
the existing 1.9 rad/s admission limit (peak observed 1.958 rad/s). A frozen-scan
trial reached zero output in 0.522 s but wheels settled in 1.878 s, beyond the
unchanged 1.5 s requirement. Those failures remain failed. Their exact causes
are separate investigations; they do not establish that the AMCL threshold
caused either failure.

The candidate was archived and the runtime YAML restored byte-for-byte.
No health, velocity, goal, stopping or ownership limit was relaxed, no idle
localization-refresh loop was introduced, and the original demo was retained.
The cadence result is useful evidence for a future qualified change, not a
claim that all low-motion trajectories are now supported. The historical
intermittent scan-occlusion receive fault also remains unresolved.
