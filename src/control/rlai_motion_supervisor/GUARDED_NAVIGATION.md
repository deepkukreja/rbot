# Guarded Nav2 integration — development simulation

This extension routes the existing Nav2 stack through explicit local ownership and direct sensor/localization admission. It adds no browser/cloud motion API and does not establish physical personnel safety.

## Run

Inspect current ROS processes first. Do not start another simulator in a domain already occupied by the owned demo. Use the actual verified map for this fixture; map YAML and image must stay together.

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rbot/install/setup.bash
ros2 launch rlai_bringup simulation.launch.py motion_supervision_enabled:=true \
  motion_health_enabled:=true use_amcl:=true map_yaml_file:=/absolute/path/to/nav2_map.yaml \
  headless:=true world:=small_warehouse teleop_enabled:=false \
  camera_processing_enabled:=false depth_camera_enabled:=false
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rbot/install/setup.bash
ros2 launch rlai_navigation supervised_navigation.launch.py
```

Inspect `/motion_supervisor/status` and `/nav2_adapter/status`. At admission, navigation must say `IDLE / READY`; health, downstream action cleanup and lifecycle activity must all be current. While stationary, AMCL normally stops publishing poses and the five-second pose TTL expires to `AMCL_STALE`. This is an honest loss of current admission, not a request to drive or repeatedly refresh localization. To request a real simulated goal explicitly:

```bash
python3 ~/rbot/src/control/rlai_motion_supervisor/scripts/send_guarded_goal.py --x 1 --y 0
```

The helper first requests exactly one `request_nomotion_update` and waits within a bounded three-second deadline for a genuinely subsequent, freshly stamped AMCL pose and healthy local status. It requires no owner, no software stop and no quarantine; invalid localization or other health faults block preflight. Failure sends no goal and never retries the update automatically. It then uses the fresh simulation timestamp and sends `NavigateToPose`. An omitted/old timestamp, a non-map frame, invalid geometry or caller-selected behavior-tree path is rejected by the proxy. `--cancel-after 2` requests cancellation after two wall seconds. Do not send goals merely to test a live operator session; the isolated validation runners are provided for that purpose.

## Command and result contract

The public `navigate_to_pose` action belongs to the lease adapter. Nav2's navigator is remapped to `nav2_internal/navigate_to_pose`, and controller/behavior velocity outputs are remapped to `nav2_adapter/cmd_vel_input`. All actuator-directed commands still come from `motion_supervisor`. Raw velocity arrival alone never acquires ownership. Legacy navigation defaults are preserved unless `use_motion_supervisor:=true` is selected; the wrapper sets that flag.

Only one public goal is admitted. A source label such as `nav2_<id>` is a diagnostic owner, not authenticated identity. A competing owner or pending/quarantined goal blocks new admission. Planner, controller, behavior server and navigator must each report ACTIVE through fresh lifecycle queries. An action server merely existing is not proof it is active.

Cancellation/inhibit/loss closes velocity forwarding immediately, then requests zero/release and downstream cancellation. A result need not arrive before local forwarding closes. Startup/restart drains orphaned navigation goals; controller `follow_path` and motion recovery actions `spin`, `backup`, `drive_on_heading` must also acknowledge cleanup and show no active goals. Unknown downstream outcome quarantines new navigation instead of automatically reacquiring. Successful completion requires confirmed motion-action cleanup; otherwise error 60007 (`MOTION_CLEANUP_UNCONFIRMED`) aborts and retains quarantine.

Original Nav2 velocity stamps must increase, follow the current goal's activation and be less than 200 ms old (future tolerance 50 ms). The adapter preserves transit age in the local leased envelope and forwards each sample once. It never re-stamps cached nonzero velocity on a timer. During initial planning it sends explicit zeros for at most three seconds; after velocity starts, silence beyond the bounded stream budget ends the session. This conservative policy can abort long recovery/wait/replanning gaps and is not an automatic retry policy.

Lease renewal does not replace velocity input. Stop/reconnection/health restoration does not resume a goal: a new explicit goal is required. Public status values follow ROS actions: SUCCEEDED=4, CANCELED=5, ABORTED=6. A zero error code alone does not prove success. Wrapper errors 60001..60007 identify ownership, local inhibit/status loss, input loss, downstream failure, deadline, adapter failure and unconfirmed cleanup. Cancellation confirms the request/forwarding outcome, not independently measured physical standstill.

## Direct local health admission

`motion_health_enabled:=true` makes the supervisor require all of these for any lease:

- Advancing ROS clock plus fresh stamped scan and filtered odometry.
- Dynamic `map -> odom` and `odom -> base_footprint` transforms with valid geometry; scan mounting transform to the base must exist.
- A real AMCL pose in map coordinates with finite covariance and valid quaternion. The supervisor only subscribes and validates: it never requests localization updates on a timer. The explicit goal helper makes one bounded update request before seeking admission. Direct action clients must supply their own explicit localization preflight when the pose is stale.

Development limits are 0.6 s source/transform receipt age, 5 s AMCL pose age, XY variance <=0.25 m² and yaw variance <=0.25 rad². AMCL's future-dated transform is allowed up to 1.1 s; other future observations are bounded to 0.1 s. Scan metadata must be finite/consistent, invalid numeric returns are rejected, and at least half the rays must be finite in-range returns. Positive infinity is treated as no return/unknown, not free space. These are conservative indoor development policies, not calibrated hardware thresholds or evidence that nearby space is clear.

Repeated stationary refreshes were removed after an isolated 1,200-scan comparison produced 201 over-limit covariance reports with forced updates and no new passive poses beyond initialization. The passive case is not proof of fresh confidence. Idle stale localization remains inhibited; thresholds were not relaxed. Slow motion that fails to produce a fresh AMCL pose within the existing TTL may also abort conservatively.

Missing, invalid, repeated/backward or stale input revokes ownership. Clearing a transient inhibit does not restore a token. Clock rewind remains inhibited until restart. The legacy ownership-only profile remains available with health disabled; the Nav2 proxy refuses goals against that profile. Guarded simulation requires both supervision and AMCL with an explicit map. Topic `motion_health_scan_topic` defaults to the real scan; the integration fixture uses an unmodified relay only to interrupt one consumer hop while Nav2's original scans remain running.

This is not obstacle/person detection, a safety-rated collision monitor, authenticated remote driving, a motor/BMS fault interface or independent emergency-stop hardware. Ordinary ROS participants remain trusted: unrestricted DDS access could still call private actions or publish directly to a driver topic. SROS2/network isolation, exclusive deployment, operator/robot authorization and hardware stopping verification remain required before remote/physical operation.

## Verify

After building and sourcing the workspace:

```bash
python3 -m pytest -q ~/rbot/src/control/rlai_motion_supervisor/test ~/rbot/tests
python3 ~/rbot/src/control/rlai_motion_supervisor/scripts/verify_guarded_proxy.py --domain 75 --out /tmp/guard-proxy-01
python3 ~/rbot/src/control/rlai_motion_supervisor/scripts/verify_guarded_nav2.py --domain 76 \
  --map /absolute/path/to/nav2_map.yaml --out /tmp/guard-nav2-01
```

Use unused nonzero domains and fresh directories. The proxy fixture uses synthetic sensors/Nav2 with real ROS services/actions and no actuator. The full fixture launches one isolated Gazebo/Nav2 robot, sends bounded goals, cancels motion and drops only the supervisor's real scan-consumer relay. Both stop only owned children. Tests do not prove physical braking or personnel safety.


## Explicit localization recovery under a latched software stop

`reset_sim_localization` supplies a known pose prior and nonzero uncertainty to AMCL. It is a simulation tool: it requires an advancing simulation clock, health admission enabled, current STOPPED supervisor reports, no owner, idle/nonquarantined navigation with no goal, and at least four fresh stationary odometry samples spanning 0.3 seconds. Other sensor faults block reset. It never acquires a lease, publishes velocity, clears a stop or resumes a goal. It does not cancel or reconcile an unsettled navigation session on your behalf.

First engage the existing software stop and wait for local navigation cleanup and standstill:

```bash
ros2 service call /motion_supervisor/set_stop std_srvs/srv/SetBool "{data: true}"
```

For the validated warehouse fixture only, world spawn (1,1) corresponds to map pose (0,0). An example declared prior is 0.1 m XY standard deviation and 5 degrees yaw standard deviation:

```bash
ros2 run rlai_motion_supervisor reset_sim_localization \
  --x 0 --y 0 --yaw 0 --xy-std 0.1 --yaw-std 0.0872664626
```

All five values are required; yaw and yaw standard deviation are in radians. Supply the actual known pose and justified uncertainty for the selected map/spawn. These example values are simulation assumptions, not hardware calibration or a measured posterior. Zero, nonfinite and unrepresentable uncertainty is rejected before a reset request.

The tool makes one SetInitialPose request within a six-second overall deadline. It requires a subsequent fresh AMCL pose, compatibility with the supplied prior (per-axis three standard deviations plus 0.05 m, analogous yaw allowance), and accepted local health under the unchanged covariance limits. Compatibility is a sanity bound, not proof of accuracy. If AMCL first publishes an otherwise valid pose from a buffered pre-request scan, the tool may make at most one explicit no-motion update request to obtain a post-request acquisition. There is no timer or automatic retry. Service acknowledgement alone cannot establish localization success. Lost preconditions abort confirmation.

Success prints `ESTIMATE_CONFIRMED_STOP_LATCHED`, pose/scan request counts and reported covariance. The software stop stays latched, and goals remain rejected. Any unconfirmed result must be inspected locally; the command never clears the stop. Only an operator's separate stop-clear request and a new explicit goal can resume motion:

```bash
ros2 service call /motion_supervisor/set_stop std_srvs/srv/SetBool "{data: false}"
python3 ~/rbot/src/control/rlai_motion_supervisor/scripts/send_guarded_goal.py --x 1 --y 0
```

Do not clear a stop solely because a service responded. Current source health, settled navigation and the intended route must be checked. A stopped robot's AMCL pose can still expire after five seconds; the separate goal helper performs its existing bounded preflight. Unrestricted ROS participants remain trusted, so concurrent external stop-clearing or private action calls require deployment isolation and authorization outside this helper.

The moving covariance investigation directly traced global recovery particles widening the whole population while its dominant position estimate remained close to Gazebo truth. Explicit uncertain initialization improved the tested short routes, but that is not evidence of reliable operation across all routes or seeds. Recovery alphas, covariance thresholds and map were retained. The reset tool does not suppress multimodal uncertainty or automatically recover and retry aborted motion.

The real AMCL/Nav2 verification runner tests no-stop rejection, an out-of-map acknowledgement with no confirmed new estimate, stop retention and goal rejection, then explicitly clears its own isolated stop and tests goal/cancel/scan-loss/recovery:

```bash
python3 ~/rbot/src/control/rlai_motion_supervisor/scripts/verify_localization_reset.py \
  --domain 88 --map /absolute/path/to/nav2_map.yaml --out /tmp/reset-nav2-01
```

Choose an unused domain and a fresh output directory. The runner sends simulated navigation goals; never point it at an operator session.


## Varied-start warehouse route validation

`scripts/verify_route_scenario.py` runs one bounded warehouse loop in an unused ROS domain and Gazebo partition. It uses the existing stopped-only localization reset, separately clears its own software stop, and requests five explicit goals through the public guarded proxy. It never retries a failed goal or performs a pose reset during the route. On normal completion or failure it records its launcher descendants before shutdown, then rechecks PID/start identity, UID and exact domain/partition markers before signaling any survivor. It verifies their exit and preserves the reports and samples. A surviving or mismatched process prevents a passing result; unrelated processes are never swept by name.

```bash
python3 ~/rbot/src/control/rlai_motion_supervisor/scripts/verify_route_scenario.py \
  --scenario nominal --domain 91 --map /absolute/path/to/nav2_map.yaml --out /tmp/route-nominal-01
```

Choices are `nominal`, `translated`, `rotated` and `biased_prior`. The latter cases vary the actual Gazebo spawn and the declared pose prior independently. Initialization uses explicit 0.1 m XY / 5 degree yaw standard deviations. Use the previously validated small-warehouse map: this benchmark's world-to-map conversion subtracts (1,1) from Gazebo position and preserves yaw. It is not a generic evaluator for arbitrary worlds/maps or physical robots.

The nominal requested loop is about eight metres, with 90-degree turns and a final heading change. Executed distance can be longer because Nav2 chooses the path and approach orientation. Static-map centreline screening does not prove executed-path collision clearance.

Before reset, the runner requires a stable half-second history without ownership or nonlocalization sensor faults; after its explicit stop clear it waits for actual Nav2 READY before invoking the existing one-request localization preflight. Each preflight and the existing motion limits retain their original bounds. Missing/invalid sensor inputs cause failure rather than admission.

Every goal must report SUCCEEDED and independently finish within 0.35 m / 0.20 rad of Gazebo truth. During the route, all AMCL poses must pair with truth without extrapolation or gaps above 0.1 s, and maximum position error must stay within 0.20 m. These are declared free-space simulation benchmark criteria, separate from Nav2's existing 0.25 m / 0.1 rad estimated-pose goal checker. They do not change production movement limits or establish docking accuracy.

`navigation_metrics.py` interpolates ground-truth position and wrapped yaw at the acquisition timestamp. Missing truth stays unknown and cannot make an action pass; an aborted action near the target still fails. Reports retain the action result before cleanup/standstill checks, preserve partial failures and distinguish pairing gaps from localization error. Raw truth, AMCL, command, odometry, health and navigation samples are saved; the final runner also records laser statistics and its source hash.

The four completed validation loops passed 20 explicit goals and traversed approximately 41.2 m in the tested simulation. This is a small varied-start coverage sample, not a reliability rate across arbitrary routes, repeated seeds, sensor faults or hardware. Earlier blocked startup attempts remain in the milestone evidence.


## Passage and scan-fault regression scenarios

The isolated verify_route_scenario.py runner supports corridor_a and corridor_b
in the existing small_warehouse world and its validated saved map. Both request
a four-metre one-way traversal through a 1.2 m passage. They use the configured
Nav2 rectangle plus footprint_padding, check sampled ground-truth side-wall
clearance of at least 0.10 m, and reject truth gaps above 0.1 s. The regular
goal and localization-error criteria still apply. These sampled planar checks
are not continuous swept-volume or contact-sensor certification.

Use --scenario nominal --scan-fault drop or --scan-fault frozen for the isolated
scan transport tests. The bridge publishes to scan_raw in that fixture, and
the observer forwards the original messages to scan. AMCL, both costmaps and
the motion supervisor subscribe to the faulted public topic. The fixture
checks that its relay is the sole public scan publisher and the bridge is the
sole raw publisher. Default launches continue to publish scan directly.

The fault begins only while a goal is forwarding nonzero drive commands and
wheel odometry shows motion. Drop suppresses forwarding; frozen repeats the
last message with its original timestamp while raw acquisition continues.
Clock and wheel odometry must continue. No software stop, pose reset or new
goal is requested during the outage or restored-input hold.

Declared criteria are sustained zero drive output within 0.85 seconds,
wheel velocities within 0.005 m/s and 0.02 rad/s within 1.5 seconds, a goal
terminal status other than success, and no owner, goal or forwarding during
a two-second hold after input restoration. The scorer rejects empty output,
nonfinite samples and receipt gaps above 0.15 seconds. Stopping-path length is
reported from sampled independent truth, not qualified as physical braking.

Each scenario requires a fresh directory, unused nonzero ROS domain and the
validated warehouse map. Use --loopback-udp in this WSL setup. The runner sends
simulated stop/reset/goal requests and is not an operator-session command or
a physical robot safety qualification. Runtime health limits, footprint,
inflation, AMCL settings and drive limits are unchanged.


## Guarded unmapped-obstacle appearance

Use verify_route_scenario.py --scenario nominal --dynamic-obstacle with an
unused nonzero domain, fresh output directory, validated warehouse map and
--loopback-udp in this WSL environment. The developer fixture also requires
the installed python3-gz-transport13 and python3-gz-msgs10 bindings. It sends
simulated stop/reset/goal requests; never attach it to the live demo.

The fixture requests map (4,0,0), then inserts a static 0.8 x 0.8 x 1.0 m box
at world (3,1,0.5) only after guarded drive output and wheel odometry show
motion. The box is absent from the saved map. Its initial map/costmap ROI
must be known free, and the pre-insertion path must intersect its future
footprint. Creation acknowledgement and a separate Gazebo pose observation
are required. The observer keeps spinning while a bounded external create
command runs. It does not feed obstacle truth into navigation or drive.

The declared criteria require live costmap marking within 3 simulated
seconds, a new published path whose enclosing-circle footprint bound clears
the complete known box by at least 0.10 m within 5 simulated seconds, and at
least 0.10 m conservative executed clearance with truth gaps at most 0.1 s.
The existing action/truth and 0.20 m localization-error bounds still apply.
The radius is computed from the actual padded Nav2 footprint dimensions.
A negative circle bound is inconclusive about actual rectangle contact.

The first completed run reached the goal, maintained 0.178 m conservative
sampled clearance and changed its plan after 0.911 simulated seconds. The
strict complete-box circle-bound forecast was first satisfied at 7.289 s,
so the overall scenario remains FAILED against its unchanged 5 s deadline.
Early plans overlap the actual box's partly unobserved side volume; the
local controller executed a wider detour. Full-box oriented-rectangle
clearance reached 0.10 m at 4.645 s in a separate recorded-path diagnostic.
These different metrics must not be substituted to turn the strict test
green. No Nav2, health, footprint, inflation or velocity limits were changed.

Raw plans, ROI cell values, obstacle poses, truth, localization, commands,
wheel and guard status are retained. No goal is automatically retried and
no pose reset occurs while navigating. The owned fixture is stopped and
its descendants verified after the measurements, including on failure.
This is sudden obstacle appearance followed by a stationary object, not
continuous pedestrian tracking, contact certification or hardware safety.


## Bounded scan-occlusion planning profile

The opt-in use_scan_occlusion:=true navigation argument starts scan_occlusion
and adds its PointCloud2 hypotheses as a marking-only source in the global
rlai_costmap_plugins::RayObstacleLayer. Default launches keep the profile disabled. Existing
AMCL, local costmap, footprint, inflation, velocity and health settings remain
unchanged. The profile needs no Gazebo or ground-truth input at runtime.

For each valid finite scan return, the node reserves at most 1.0 m behind the
return along that beam, sampled at 0.05 m and limited to the sensor range and
the existing 0.25-8 m marking horizon. It does not join missing beams or put
points in front of the measured return. These points represent a bounded
occlusion hypothesis; they are not sensor measurements or a reconstructed
object bounding box. Objects deeper than this bound are not fully modeled.

The output keeps the original acquisition stamp and sensor frame. Duplicate,
zero or backwards stamps cannot refresh it. Invalid scan geometry is rejected.
An empty valid hypothesis cloud replaces the previous observation, including
when no finite return qualifies. It does not erase costmap cells. Fractional-coordinate raw
scan raytracing provides measured free-space clearing; existing Nav2 footprint clearing
and recovery mechanisms are retained. There is no timed forgetting or new
clear-costmap request. A 0.6 s expected observation interval makes a missing
derived source non-current in the global costmap.

This profile addresses the previously failed appearance fixture while keeping
its complete-box conservative 0.10 m / 5 simulated second criterion unchanged.
The fixture option --scan-occlusion enables it explicitly. Add
--remove-dynamic-obstacle to remove the box only after arrival, confirm its
Gazebo entity deletion and require the extended region to clear within 5
simulated seconds. At least three subsequent costmap snapshots must span at
least two simulated seconds without renewed high-cost cells. This avoids
mistaking a short wall-clock hold with no new costmap frames for evidence.

Example in a fresh directory and unused nonzero domain:

python3 ~/rbot/src/control/rlai_motion_supervisor/scripts/verify_route_scenario.py \
  --scenario nominal --dynamic-obstacle --remove-dynamic-obstacle \
  --scan-occlusion --loopback-udp --domain 98 \
  --map /absolute/path/to/validated/nav2_map.yaml --out /tmp/occlusion-check-01

The runner starts its own guarded simulator; do not aim it at the live demo.
The profile has isolated simulation coverage for the appearance/removal case
and both 1.2 m warehouse passages. It is not a general occlusion reconstruction,
moving-person tracker or physical collision/braking qualification. Sensor-only
marking is shared by the planner; independent truth is used only by the test
observer and can fail/cancel a test that violates the declared clearance.


## Required occlusion-source liveness

When guarded navigation enables use_scan_occlusion:=true, its adapter now
requires fresh scan_occlusion/points observations. The immutable startup
parameter require_scan_occlusion is wired automatically by navigation.launch.
With the profile disabled this dependency is absent. Raw sensor health and
the generic motion supervisor remain unchanged; this is a navigation-specific
precondition, not a global software-stop latch.

The adapter accepts the producer's little-endian unordered XYZ32 layout with
finite planar points; an empty observation is valid. It checks both advancing
acquisition stamps and a 0.6 s steady receipt deadline, plus ROS acquisition
age/future bounds. Repeated stamps never refresh or clear an invalid input.
Missing, stale, invalid or unusably timed observations reject new goals and
close velocity forwarding. The steady timer invokes the existing zero,
lease-release and backend-cancellation path with result code 60008 and a
SCAN_OCCLUSION_* reason. Restoring input never resumes the closed session.

The isolated fixture options --pause-occlusion and --restart-occlusion use
the nominal open-space goal with --scan-occlusion. They recheck UID, process
start identity, domain and partition before signaling only the owned source
process. Pause uses SIGSTOP/SIGCONT; restart uses SIGKILL followed by an
explicit new producer process. They require raw scan/clock/health to continue,
zero output within 0.85 s, wheel standstill within 1.5 s, an aborted goal,
rejection of a fresh probe goal while the source is absent, and no automatic
resume during a two-second restored-input hold.

Idle AMCL estimates still expire. Component recovery can leave navigation at
HEALTH_UNAVAILABLE with AMCL_STALE; the fixture preserves this block and
never refreshes localization merely to make a recovery check report READY.
Other raw-source faults are not accepted as component-only recovery. A future
operator goal still needs the existing explicit localization preflight.

This liveness check validates availability and wire format of the required
derived stream; it does not prove that every hypothesis was correctly fused
into the costmap. The earlier deletion failure with eight retained lethal cells is preserved.
A full-input capture reproduced twelve retained cells: measured free rays
crossed every cell, with no subsequent raw or derived marking there. Native
Nav2 1.3.12 integer-endpoint clearing retained all twelve under replay. The
optional global layer now uses the existing fractional-coordinate grid walker
to clear actual positive-length intersections. It retains configured range,
finite measured endpoints, marking, TF, source liveness and lifecycle behavior.
Default navigation and the local layer are unchanged. Unobserved occluded
space remains occupied; no age-based forgetting or forced map clear was added.

For diagnosis, add --record-clearing to the appearance/removal invocation.
The observer records raw scans, derived clouds, TF, clock, raw/published global
costmaps and independent truth only after arrival. It requires subscriptions
on all eight topics and holds 1.5 simulated seconds before physical deletion.
Recording adds no controller behavior or relaxed scoring limits. The bag is
stopped and flushed by the existing identity-scoped fixture cleanup.


## Isolated removal viewpoints

The observer's --dynamic-obstacle mode also accepts the existing translated,
rotated and biased_prior open-space starts. --obstacle-view chooses straight
(4,0,0), northeast (4,0.6,pi/4), southeast (4,-0.6,-pi/4), or turned
(4,0,pi/2), in map metres/radians. Straight remains the default. Alternate
views require the dynamic-box fixture; passage and raw-fault restrictions stay
in place. This configures an isolated test goal, not a runtime planning policy.

All cases insert the same 0.8 x 0.8 x 1 m stationary box at map (2,0) only
while moving. They must first show that their actual initial plan crosses its
known-free footprint. Full-box 0.10 m conservative clearance, 5 s replan and
5 s removal-clearing limits, acquisition/health gates and truth scoring stay
unchanged. Physical deletion follows successful arrival; clearing must remain
observed for at least three costmap snapshots spanning two simulated seconds.
Keep --record-clearing for viewpoint investigations, and preserve failures
instead of clearing the map or ignoring cells merely because they remain.


For an alternate destination whose initial path already avoids the box,
--staged-obstacle-view first sends the unchanged (4,0,0) appearance goal,
then explicitly sends the selected alternate goal with the box still present.
Both actions must succeed against independent truth; insertion happens once.
Deletion and recording follow the final viewpoint. This separates appearance
coverage from removal viewpoint without weakening either score. It adds a
declared second goal, not an automatic retry. Direct mode remains the default.


## Explicit use of cleared space

The observer option --reenter-cleared-space requires dynamic appearance/removal
with the scan-occlusion profile. After the unchanged removal/clearing checks,
it waits for idle AMCL_STALE and verifies sustained zero drive output,
stationary wheels, idle navigation and no owner. It then records one explicit
request, invokes the existing one-goal localization preflight and sends a goal
at map (2,0,pi), inside the deleted box footprint. No idle refresh loop is added.

The new goal must succeed against independent truth. A fresh plan and the
measured arrival centre must enter the former footprint; acquisition-paired
localization error stays <=0.20 m and truth gaps <=0.10 s. The physical box
deletion and original 0.10 m appearance clearance / 5 s replan and removal
deadlines must already have passed. Reentry has its own stage so the deleted
box is not treated as a still-present obstacle by the appearance scorer.
Failure cancels through existing owned cleanup; no automatic goal retry occurs.
The post-arrival bag continues through this explicit goal when recording is on.


## Token-free control timing trace

--record-control-trace adds observer receipt times for raw Nav2 velocities and
leased envelopes, including source stamp, sequence and validity duration. It
never records the lease capability. Health samples retain health reason in
the existing reason field and add control_reason, accepted_sequence, source
and lease_remaining_ms, so command timeout is distinguishable from lease loss.

When --record-clearing is also active, the bag additionally records raw input,
supervisor/navigation status, actuator output, wheel odometry and AMCL. The
leased-command topic is deliberately excluded from bags because its message
contains the capability; only the selected non-secret fields enter JSON.
Receipt times are observer times, not exact supervisor callback times. Actual
supervisor reason and accepted sequence remain authoritative for expiry.


## Ownership-loss reason

Result code 60001 retains the original abort, zero/release/cancel and drain
behavior. When a fresh supervisor status is ownerless IDLE, its authoritative
COMMAND_TIMEOUT, LEASE_EXPIRED or INVALID_COMMAND reason is now preserved as
OWNERSHIP_LOST_<reason>. Other ownership changes keep OWNERSHIP_LOST. No
deadline, validity duration, lease renewal or motion-admission rule changes.
An expiry reason identifies the local decision, not by itself the upstream
packet-delay cause. Inspect the token-free trace to locate that boundary.

--pause-controller with --record-control-trace is a bounded nominal simulation
fault. It rechecks owned PID start identity, UID, ROS domain and Gazebo partition
before SIGSTOP, holds 0.30 wall seconds, and guarantees SIGCONT before cleanup.
It requires command expiry with health/scans continuing, an aborted goal,
zero output <=0.85 s, wheels settled <=1.5 s, and no automatic resume. It
cannot be combined with the other dynamic-box or raw/derived-source faults.


--record-control-bag starts the same token-free 14-topic MCAP before the first
goal, rather than after arrival. It requires --record-control-trace and
--scan-occlusion and replaces --record-clearing. This captures startup and
early-action failures that cannot reach the post-arrival recording boundary.
The recorder still verifies all subscriptions and holds 1.5 simulated seconds
before dispatch; each explicit goal still performs its existing preflight.


The added raw/envelope diagnostic subscriptions use best-effort keep-last-50
delivery, as do the six additional high-rate recorder subscriptions. The
supervisor's actual command subscriber and existing observation/control
policies are unchanged. Diagnostic samples may be dropped; accepted sequence
and supervisor reason are authoritative. Source timing cannot be inferred
from observer receipt timing alone, and recording can still perturb scheduling.


## Native occlusion receive diagnostics

`trace_occlusion_receiver:=true` is an opt-in launch argument for the supervised
navigation launch and requires `use_scan_occlusion:=true`. The default is false.
It publishes best-effort JSON String events on `/nav2_adapter/occlusion_trace`.
Cloud events record callback entry time on steady/wall clocks, acquisition
nanoseconds, payload size, validation duration, validity and whether the native
gate advanced. Gate events record timer gaps, accepted acquisition/receipt,
current ROS time and the existing freshness reason. No ownership capability,
command contents or alternate motion authority is included.

`verify_route_scenario.py --trace-occlusion-receiver --scan-occlusion` captures
those events in `occlusion_receive_trace.json`. When the existing control bag
is also requested, it adds this diagnostic topic (15 topics total). Capture
starts before the first goal when `--record-control-bag` is used. The leased
command topic remains excluded. Observer arrival is recorded separately from
native callback entry; use the latter when examining adapter receive silence.

Diagnostic publication is default-off and best effort. `diagnostic_drops`
counts caught publication errors only; it cannot count undelivered DDS samples.
Trace event counts do not prove complete delivery. Tracing and recording can
affect scheduling, so preserve their settings with each result. Duplicate or
invalid acquisitions retain the existing gate behavior. The 0.6 s steady
receipt timeout and acquisition-clock limits, action cancellation, ownership,
velocity expiry and stopping deadlines are unchanged.

The 2026-11-15 delivery investigation retained one AMCL_STALE reentry failure
and one successful appearance/removal/reentry run. Neither reproduced the
historical SCAN_OCCLUSION_STALE fault. These results do not establish a QoS or
socket-buffer cure. Read-only per-process UDP accounting showed startup
discovery drops, while the observed adapter data sockets had no recorded
drops. The first run's adapter socket monitor attached late because its
executable name was initially misidentified; native callback capture covered
that run from startup. All negative results and this coverage limitation are
retained in the occlusion_delivery evidence bundle.


## Publisher and middleware receive boundary

The same default-off receive trace now records `matched_publishers` in each
gate event. A failed count query is represented as null and cannot change
motion admission. A matched publisher may be alive but silent; matching is
diagnostic context, not a replacement for the 0.6 s receipt watchdog.

Cloud events also carry `callback_wall_ns`, `middleware_source_wall_ns`,
`middleware_received_wall_ns`, `publication_sequence` and `reception_sequence`.
The traced subscription uses rclpy's required two-argument MessageInfo callback
form. The untraced subscription retains its original message-only callback,
QoS, validation and gate observation. The action and velocity paths are unchanged.

These metadata timestamps are middleware wall-clock values, not scan
acquisition time or hardware timestamps. Compare them to callback wall time,
not simulated ROS time. A source-to-receive interval covers the middleware
path, and receive-to-callback includes delivery/dispatch work; neither alone
isolates a kernel or executor bottleneck. Clock discontinuities can invalidate
their subtraction. Preserve raw values and flag negative intervals.

Installed Fast DDS supplies publication sequence but reports reception
sequence as unsupported; rclpy exposes unsupported sequence values as null.
Retain that null rather than fabricating a sequence. Sequence gaps can also
come from lost diagnostic events, writer restart, or wraparound. The trace
omits publisher GIDs and lease capabilities. Attribute gaps only within an
identity-verified unchanged producer, and compare the native callback counter
in gate events when its accepted stamp matches a cloud record.

The receiver_boundary evidence bundle contains the fixed three-trial
investigation, source delta, raw JSON, socket observations and a recorded
mission. Successful trials do not close the historical intermittent fault.
All negative outcomes remain part of the evidence and no automatic retries
or deadline/tolerance adjustments are added by this diagnostic extension.


## MPPI post-filter command constraints

The rbot workspace includes a local `nav2_mppi_controller` overlay based on
upstream 1.3.12, with original attribution and a LOCAL_PATCH.md. New launches
after sourcing `rbot/install/setup.bash` resolve that package ahead of the apt
package. Installed apt libraries are retained; an already running controller
does not hot-reload this change. The original read-only demo was not restarted.

Native regression showed that Savitzky-Golay smoothing after optimization
can turn a bounded 1.9 rad/s control into 1.9121 rad/s, and 0.5 m/s into
0.5121 m/s. Optimizer finalization now rejects non-finite results, reapplies
its existing current constraints after smoothing, and stores the bounded
emitted control in its history. Dynamic speed reductions are retained. A
bitwise IEEE float check keeps rejection meaningful under the package's
existing fast-math flags. No supervisor limit or ownership rule is changed.

The native command_bounds_probe isolates actual finalization using zero
optimizer iterations in the fixture only. It covers signed velocity bounds,
shifted/unshifted output, an interior control, reduced speed, history and
non-finite input. Run native tests under a nonzero private ROS domain with
the loopback UDP wrapper. The stock library passed 3/10 cases; the patch
passed 10/10. Existing optimizer unit/smoke tests passed 12 and 3 cases;
the full Python suite passed 222.

The private domain-55 integration verified the actual mapped patched library
and passed appearance/removal/reentry with 0.257 m reentry truth error.
That fixture used the separate, stopped-only 0.05 m AMCL experiment. The
repository AMCL default remains 0.20 m; this does not qualify that earlier
candidate or prove end-to-end reliability with every default profile.
Its 1468 recorded raw packets stayed within admission bounds; the largest
turn was 1.036 rad/s, so physical saturation was not exercised in that trial.
The native tests, not the different live trajectory, establish the limit fix.

Untrusted raw publishers remain subject to the unchanged adapter gate. The
old historical packet's exact optimizer sequence was not recorded, so the
native reproduction demonstrates a concrete mechanism without claiming sole
attribution. The earlier 1.878 s wheel-settling failure and historical unforced
scan-occlusion loss remain unresolved. No stopping, covariance, freshness,
velocity or goal limit was relaxed. See orchestration outputs/mppi_bounds/
for source provenance, patch, native tests, bag and measured limitations.


## Stopping-time clock investigation — 2026-11-16

The retained domain-54 frozen-scan failure remains a failure of the original
1.5 s steady-clock wheel-standstill limit. Recomputed supervisor zero latency
is 0.522 s and wheel-odometry latency is 1.878 s. Recorded body truth crosses
the same speed tolerance at 1.685 s, independently bracketed by native timer
events at 1.674–1.724 s. Odometry smoothing alone cannot dismiss the failure.

After the zero command stamped at simulation time 6.55, body speed reaches
tolerance at 6.94 and wheel odometry at 7.02. Only 0.47 simulation seconds pass
over the next 1.355 steady seconds. An offline probe executes the installed
diff-drive-controller 4.40.1 SpeedLimiter header: the existing 1.0 m/s²
acceleration/symmetric-deceleration limit takes 0.39 simulation seconds to
reduce the preceding 0.383895 m/s internal command to zero. The 5 rad/s wheel
interface and 0.0625 m radius explain the measured 0.3125 m/s speed plateau.
The controller-manager/Gazebo update period is in simulation time. Historical
CPU scheduling and the exact controller command adoption instant were not
recorded, so no particular load source or delivery delay is established.

A new isolated domain-56 frozen-scan diagnostic enabled only limited-velocity
publication in a workspace-only share overlay. It retained the original
default AMCL profile, installed MPPI fix, control limits and scoring. All 12
checks passed: supervisor zero 0.536 s, limited output zero 1.115 s, body
tolerance 1.127 s and wheel-odometry tolerance 1.212 s. Direct limited-command
telemetry measured 1.0 m/s² deceleration in simulation time. Raw joint states
and limited output were captured in addition to the existing token-free bag.
No leased-command topic was recorded and no production settings were changed.

The new run advanced simulation time faster and used a different trajectory
and AMCL profile from the retained failure. It is not a same-input performance
fix or loaded stopping qualification. The original failed trial, physical
braking qualification, and intermittent source-delivery issue remain open.
Before changing braking or asserting a bound under load, measure clock rate,
controller update/CPU timing and stopping distance with a controlled command
trajectory. The 0.5 m/s chassis configuration versus 5 rad/s wheel-interface
limit needs an explicit feasibility review; neither limit was changed here.
Evidence, native probe, original-failure analysis and the new diagnostic are
in orchestration outputs/stopping_timing/.


## Fixed-command stopping under bounded CPU load — 2026-11-16

One predeclared baseline/load/post-baseline sequence used fresh isolated
domains 57–59, the same four-CPU affinity, original AMCL defaults, active direct
health admission and real supervisor leases. Each case issued the same 61-point
0.3 m/s trajectory over 1.2 simulation seconds, then stopped sending commands.
The command validity remained 300 ms, lease duration 2 s, and existing braking
limits and stop criteria were unchanged. No Nav2 goal was sent. Commands were
freshly issued in each session; no capability or leased-command topic was saved.

All 10 checks passed in each case. Supervisor zero/wheel standstill times were
0.347/1.087 s (baseline), 0.330/1.151 s (two CPU workers), and 0.345/1.130 s
(baseline after), measured from the last transmitted command. Body speed was
0.3 m/s before loss in all cases. Active deceleration was 1.0 m/s² in simulation
time. The CPU workers shared CPUs 0–1 inside the same four-CPU pool and actually
received about 0.62 core-equivalents combined during the measured window; this
is a specific contention workload, not two dedicated saturated cores.

The loaded wheel time was 22–64 ms above the two baselines; baseline variation
was 42 ms. This single sequence does not establish an effect distribution or a
worst-case stopping bound. Sampled distance after loss was 0.110/0.094/0.097 m;
distance is reported separately and no distance criterion was invented. This
tests command-source loss at 0.3 m/s, not the earlier frozen-scan case or the
0.5 m/s chassis limit. The earlier 1.878 s failure remains failed.

CPU and main-thread scheduler counters plus limited-output observations were
retained. Output timestamp gaps are not proof of missed controller updates;
they can also include publication/observation loss. The last Gazebo process
needed identity-scoped cleanup escalation through SIGKILL after stopped output
was confirmed; no marked private processes remained. Root control settings,
driver/model limits and acceptance criteria were not changed. Reproduction,
three MCAPs, CPU traces, timing/distance plots and limitations are in orchestration
outputs/loaded_stopping/. Physical stopping remains unqualified.


## Read-only wheel feasibility audit

`ros2 run rlai_motion_supervisor audit_drive_limits` reads the installed
controller YAML and expanded Gazebo URDF without creating a ROS node or
publishing a command. It checks the four corners of the configured chassis
velocity box against the per-wheel interface/URDF velocity intersection.
It exits 2 when the configuration box is infeasible or metadata is unknown.
The current 0.5/−0.35 m/s and ±1.9 rad/s box is infeasible for the configured
5 rad/s wheels and 0.0625 m radius. No runtime limit is changed by this tool.

Use `--linear 0.3 --angular 0` to check one endpoint (exit 0 when admitted by
both chassis and wheel limits), or `--linear 0.3 --angular 1` to reproduce an
axis-admitted command whose right-wheel demand exceeds its limit (exit 2).
`--controllers PATH --urdf PATH` accepts explicit YAML and expanded URDF or
xacro files for offline use. Unknown or nonfinite geometry/bounds are errors.
The default inputs require the robot's description/control packages already
installed in the sourced workspace; no ROS master or running robot is needed.

The JSON common scale describes a single endpoint and preserves the requested
wheel-speed ratio. It is a diagnostic suggestion, not an installed governor
or physical safety guarantee. Independent downstream acceleration limits can
leave the wheel-feasible region during transitions between feasible endpoints;
endpoint scaling alone does not solve that problem. Keep the existing wheel
limits and review trajectory constraints, post-acceleration feasibility and
emitted-command history together before adopting a motion correction.


### Measured coupled-wheel mismatch — 2026-11-17

Two isolated leased-command trials (domains 60/61) retained direct health,
original limits, default AMCL and zero/standstill checks. The command 0.3 m/s
plus 1 rad/s passed the existing axis admission but requested left/right wheel
rates 2.3192/7.2808 rad/s. Measured wheel rates were 2.3192/5.0000: independent
clipping changed body curvature from the requested 3.333 to 2.363 per metre
(about 29.1% loss). The audit's common endpoint factor 0.686737721 gave a
private comparison command 0.206021316 m/s plus 0.686737721 rad/s; measured
wheel rates were 1.592682/5.0000 and body curvature 3.334 per metre in the
short observed window. Both original command-loss check sets passed and both
private launches cleaned up without fallback signals. This does not qualify
closed-loop navigation or an installed motion correction.

Endpoint scaling alone is insufficient. Executing the installed native linear
and angular SpeedLimiter headers from (v=0,w=1.9) toward (v=0.3125,w=0) gives
7.212566 rad/s peak wheel demand even though both endpoints are feasible.
The axes change at different rates. Before adopting a correction, review MPPI
predictions and final driver output after acceleration limiting together, and
retain emitted-command history consistent with what the wheels can receive.
Do not raise wheel limits or silently reclassify the earlier stopping failure.

The nominal 0.35 m no-slip model disagreed with measured body rotation by
12.9% in the scaled trial, failing the initial 10% diagnostic guard. This
negative comparison is retained. Pose-derived yaw agrees with the ground-truth
twist. The existing effective controller separation 0.35×0.886=0.3101 m predicts
these observations more closely; its physical origin is not established here
and no calibration was changed. See orchestration outputs/wheel_feasibility/
for the failed assumption, source patch, native transition and both full bags.
