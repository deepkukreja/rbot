# Local post-filter command bounds patch

Based on ros-navigation/navigation2 tag 1.3.12. Retain upstream attribution.
Optimizer::evalControl now rejects non-finite smoothed controls, reapplies its
existing current constraints after Savitzky-Golay smoothing, and stores the
bounded emitted control in history. This preserves its existing speed-limit
settings, command stamp, shift and public ABI. It does not change supervisor
admission, velocity ceilings or command ownership.

The integer float classifier is intentional: this package enables fast math,
which can remove std::isfinite checks. The native command_bounds_probe covers
shifted/unshifted output, signed linear/turn limits, a dynamic reduction,
interior controls, history consistency and non-finite rejection. It disables
optimizer iterations only in the fixture, to isolate real native finalization.
Run the test in a private ROS domain with the project's loopback UDP wrapper.
This test has an unconfigured costmap node and no drive publisher.

The earlier historical packet's full optimizer sequence was not captured.
This reproduces and fixes a concrete native overshoot mechanism, not every
possible source of invalid Nav2 velocity. Physical stopping and source-health
failures remain separate evidence and are not fixed by command saturation.
