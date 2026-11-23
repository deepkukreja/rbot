"""Wait for the controller manager and fresh simulation-clock progress."""
import time


class ClockProgress:
    MIN_PROGRESS_NS = 50_000_000

    def __init__(self):
        self.first = None
        self.last = None
        self.ready = False

    def observe(self, stamp_ns):
        if stamp_ns <= 0:
            self.first = self.last = None
            self.ready = False
            return False
        if self.last is None or stamp_ns < self.last:
            self.first = stamp_ns
        self.last = stamp_ns
        self.ready = self.last - self.first >= self.MIN_PROGRESS_NS
        return self.ready


def main():
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node("guarded_control_clock_gate")
    manager = node.create_client(ListControllers, "controller_manager/list_controllers")
    progress = ClockProgress()
    saw_manager = False

    def on_clock(message):
        nonlocal saw_manager
        if manager.service_is_ready():
            saw_manager = True
            progress.observe(message.clock.sec * 1_000_000_000 + message.clock.nanosec)
        else:
            progress.observe(0)

    node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.1, deadline - time.monotonic())))
            if progress.ready and time.monotonic() <= deadline:
                node.get_logger().info("GUARDED_CONTROL_CLOCK_READY")
                return 0
        reason = "CLOCK_NOT_ADVANCING" if saw_manager or manager.service_is_ready() else "CONTROLLER_MANAGER_UNAVAILABLE"
        node.get_logger().error("GUARDED_CONTROL_CLOCK_TIMEOUT: " + reason)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
