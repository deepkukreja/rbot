"""Direct sensor/TF subscriptions for the local supervisor; no motion publishers."""
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from .health import HealthMonitor, valid_scan, valid_covariance, valid_geometry


def stamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec/1e9


class HealthInputs:
    def __init__(self,node):
        self.node=node
        self.monitor=HealthMonitor()
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,node,spin_thread=False)
        self.subscriptions=[
            node.create_subscription(LaserScan,node.get_parameter('health_scan_topic').value,self.scan,qos_profile_sensor_data),
            node.create_subscription(Odometry,'odometry/filtered',self.odom,qos_profile_sensor_data),
            node.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.pose,qos_profile_sensor_data),
            node.create_subscription(TFMessage,'tf',self.transforms,qos_profile_sensor_data)]

    def scan(self,message):
        valid=valid_scan(message.ranges,message.range_min,message.range_max,message.angle_increment)
        try:
            transform=self.buffer.lookup_transform('base_footprint',message.header.frame_id,Time())
            t,q=transform.transform.translation,transform.transform.rotation
            valid=valid and valid_geometry([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
        except Exception:
            valid=False
        self.monitor.observe('scan',stamp(message),time.monotonic(),valid=valid)

    def odom(self,message):
        p,q=message.pose.pose.position,message.pose.pose.orientation
        t=message.twist.twist
        valid=message.header.frame_id=='odom' and message.child_frame_id=='base_footprint' and valid_geometry(
            [p.x,p.y,p.z,t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z],[q.x,q.y,q.z,q.w])
        self.monitor.observe('odom',stamp(message),time.monotonic(),valid=valid)

    def pose(self,message):
        p,q=message.pose.pose.position,message.pose.pose.orientation
        valid=message.header.frame_id=='map' and valid_covariance(message.pose.covariance) and valid_geometry([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        if not valid and time.monotonic()-getattr(self,'last_pose_warning',-math.inf)>5.:
            self.last_pose_warning=time.monotonic()
            diagonal=[message.pose.covariance[i] for i in [0,7,35]]
            self.node.get_logger().warning(f'AMCL health rejection: frame={message.header.frame_id}, covariance_xy_yaw={diagonal}, quaternion_norm_squared={q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w}')
        self.monitor.observe('amcl',stamp(message),time.monotonic(),valid=valid)

    def transforms(self,message):
        for item in message.transforms:
            name={('map','odom'):'map_tf',('odom','base_footprint'):'odom_tf'}.get((item.header.frame_id,item.child_frame_id))
            if name:
                t,q=item.transform.translation,item.transform.rotation
                self.monitor.observe(name,stamp(item),time.monotonic(),valid=valid_geometry([t.x,t.y,t.z],[q.x,q.y,q.z,q.w]))

    def reason(self):
        now=time.monotonic()
        ros_now=self.node.get_clock().now().nanoseconds/1e9
        if ros_now>0:
            self.monitor.clock(ros_now,now)
        return self.monitor.reason(now,ros_now)
