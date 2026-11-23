# rbot measured-ray costmap clearing

`rlai_costmap_plugins::RayObstacleLayer` inherits Nav2's 2D ObstacleLayer and
replaces only its free-space ray traversal. It is selected only by the opt-in
`use_scan_occlusion:=true` global costmap overlay. Default 2D navigation and the
local costmap keep their existing plugins. Original raw scans clear; the
derived occlusion source remains marking-only. Observation buffers, TF,
marking, footprint, map combination and lifecycle still come from Nav2.

An isolated box-removal capture retained twelve lethal cells. None was marked
again in 125 paired post-removal frames, while each had positive-length free
ray intersections. Replaying all 90,000 measured rays through the installed
Nav2 1.3.12 ObstacleLayer retained all twelve; RayObstacleLayer cleared all
twelve using the same inputs. Integer endpoint quantization can therefore
lose actual ray/cell intersections. The shared fractional grid walker avoids
that loss. Update bounds include the sensor and clipped ray endpoints.

The 2D layer clips each finite measured segment to its configured minimum and
maximum range in metres, then to the grid. It preserves ObstacleLayer's
measured endpoint policy; the inherited marking pass restores the return cell.
It does not use VoxelLayer's two-cell endpoint margin. There is no beam widening,
interpolation, timed obstacle expiry or clearing beyond a measured return.
Cells without positive-length measured free-space evidence remain occupied.

`test_obstacle_ray` exercises the real 2D override with a recorded counterexample,
range limits, adjacent unobserved cells, occluded cells behind a return, grid
clipping, invalid rays and update bounds. Expected cells use literal independently
checked fixtures, not expectations computed by the traversal under test.

`rlai_costmap_plugins::RayVoxelLayer` inherits Nav2's VoxelLayer and replaces only
its `raytraceFreespace` implementation. Observation buffers, transforms, voxel
marking, height/range configuration, lifecycle, footprint handling and map
combination continue to use Nav2. This is used only by the optional 3D profile.

Nav2 Jazzy 1.3.12 integer Bresenham stepping can miss voxels crossed by an actual
fractional-coordinate ray. The recorded rbot low-box trial reproduced ground-level
marks after removal despite hundreds of free-space ray intersections per voxel.
The plugin uses fractional-coordinate Amanatides-Woo grid traversal instead.

For the 3D layer, only measured finite ray segments are cleared. The existing two-horizontal-cell
endpoint margin is retained. The configured minimum/maximum ray range is applied
in metres, and segments are clipped to the voxel grid. Simultaneous edge/corner
crossings do not clear cells that are touched for zero segment length. There is no
automatic expiry, artificial beam interpolation, or clearing behind a return.
The standard `clearing_endpoints` diagnostic topic remains available.

Tests compare traversal against an independent segment/AABB oracle for 2,025
geometric cases. A native-library regression includes the actual recorded ray
missed by the installed Nav2 implementation and verifies that an obstacle behind
the measured endpoint stays marked. That comparison also accepts a future
upstream implementation that has already corrected the example.

Build and run the tests from the rbot workspace:

```bash
colcon build --packages-select rlai_costmap_plugins --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select rlai_costmap_plugins --event-handlers console_direct+
colcon test-result --verbose
```

The plugin was compiled against ROS 2 Jazzy nav2_costmap_2d 1.3.12. Retained marks
in volumes genuinely not observed by the sensor remain expected; this correction
does not establish arbitrary dynamic-obstacle or hardware-safety performance.

An unspecified single-configuration build defaults to Release because this
per-point traversal executes under the costmap mutex. Explicit Debug and
RelWithDebInfo configurations remain available. An unoptimized build can hold
that mutex long enough to make the controller miss its 50 ms cycle budget.
