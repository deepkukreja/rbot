"""Bounded planning hypotheses behind finite laser returns, never free-space evidence."""
import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32


def occlusion_points(ranges, *, angle_min, angle_increment, range_min,
                     range_max, depth=1., spacing=.05, max_range=8., min_range=.25):
    """Reserve at most depth metres behind each valid return along its own beam.

    No points are inserted in front of a return, between missing beams or
    beyond the sensor/marking horizon. These are hypotheses, not measured hits.
    """
    values = [angle_min, angle_increment, range_min, range_max,
              depth, spacing, max_range, min_range]
    if (not all(math.isfinite(v) for v in values) or angle_increment == 0.
            or not 0. < depth <= 2. or not .025 <= spacing <= .5
            or not 0. <= range_min < range_max or not 0. <= min_range < max_range):
        raise ValueError('Invalid scan geometry or bounded occlusion configuration')
    lower, upper = max(range_min, min_range), min(range_max, max_range)
    points = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not lower <= distance <= upper:
            continue
        angle = angle_min + index * angle_increment
        c, s = math.cos(angle), math.sin(angle)
        length = min(depth, upper - distance)
        steps = max(1, math.ceil(length / spacing))
        for step in range(steps + 1 if length else 1):
            radius = distance + length * step / steps
            points.append((radius * c, radius * s, 0.))
    return points


class ScanOcclusion(Node):
    """Publish acquisition-stamped hypotheses for a stock marking-only source."""

    def __init__(self, **kwargs):
        super().__init__('scan_occlusion', **kwargs)
        descriptor = ParameterDescriptor(read_only=True)
        self.settings = {
            'depth': self.declare_parameter('depth_m', 1., descriptor).value,
            'spacing': self.declare_parameter('spacing_m', .05, descriptor).value,
            'max_range': self.declare_parameter('max_range_m', 8., descriptor).value,
            'min_range': self.declare_parameter('min_range_m', .25, descriptor).value,
        }
        occlusion_points([], angle_min=0., angle_increment=1., range_min=.12,
                         range_max=12., **self.settings)
        self.last_stamp = 0
        self.output = self.create_publisher(PointCloud2, 'scan_occlusion/points', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp <= self.last_stamp or not message.header.frame_id:
            return
        try:
            points = occlusion_points(message.ranges, angle_min=message.angle_min,
                                      angle_increment=message.angle_increment,
                                      range_min=message.range_min, range_max=message.range_max,
                                      **self.settings)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self.output.publish(create_cloud_xyz32(message.header, points))
        self.last_stamp = stamp


def main(args=None):
    rclpy.init(args=args)
    node = ScanOcclusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
