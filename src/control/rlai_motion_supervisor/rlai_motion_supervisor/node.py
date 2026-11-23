"""Serialized ROS callbacks around the locally timed supervisor. No raw velocity input."""
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
from rlai_motion_interfaces.msg import LeasedTwist, SupervisorStatus
from rlai_motion_interfaces.srv import AcquireControl, RenewControl, ReleaseControl
from .core import Supervisor


class MotionSupervisor(Node):
    def __init__(self):
        super().__init__('motion_supervisor')
        self.core=Supervisor()
        self.declare_parameter('health_required',False)
        self.declare_parameter('health_scan_topic','scan')
        self.health=None
        if self.get_parameter('health_required').value:
            from .health_ros import HealthInputs
            self.health=HealthInputs(self)
            self.core.set_inhibit('HEALTH_STARTUP',time.monotonic())
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        # Relative endpoints support ROS namespaces. Standalone output is an unconnected sink.
        self.drive=self.create_publisher(TwistStamped,'motion_supervisor/drive',qos)
        self.status=self.create_publisher(SupervisorStatus,'motion_supervisor/status',qos)
        self.subscription=self.create_subscription(LeasedTwist,'motion_supervisor/command',self._command,qos)
        self._control_services=[self.create_service(AcquireControl,'motion_supervisor/acquire',self._acquire),
            self.create_service(RenewControl,'motion_supervisor/renew',self._renew),
            self.create_service(ReleaseControl,'motion_supervisor/release',self._release),
            self.create_service(SetBool,'motion_supervisor/set_stop',self._stop)]
        # The default mutually-exclusive callback group serializes all state changes.
        # Steady time keeps expiry active when /clock is paused or absent.
        self.steady_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.timer=self.create_timer(.05,self._publish,clock=self.steady_clock)
        self.get_logger().info('Local motion supervisor ready: explicit lease required; raw cmd_vel is ignored. Software stop only.')

    def _acquire(self,request,response):
        self._update_health()
        grant=self.core.acquire(owner=request.owner,source=request.source,duration=request.duration_ms/1000.,now=time.monotonic())
        response.granted,response.lease_id,response.reason=grant.ok,grant.token,grant.reason
        self._publish()
        return response

    def _renew(self,request,response):
        response.success=self.core.renew(request.lease_id,duration=request.duration_ms/1000.,now=time.monotonic())
        response.reason='RENEWED' if response.success else 'REJECTED'
        self._publish()
        return response

    def _release(self,request,response):
        response.success=self.core.release(request.lease_id,time.monotonic())
        response.reason='RELEASED' if response.success else 'REJECTED'
        self._publish()
        return response

    def _stop(self,request,response):
        self.core.set_stop(request.data,time.monotonic())
        self._publish()
        response.success=not self.core.clock_fault
        response.message='STOP_LATCHED' if self.core.stopped else 'STOP_CLEARED_REACQUIRE_REQUIRED'
        return response

    def _command(self,message):
        t=message.twist
        self.core.command(token=message.lease_id,sequence=message.sequence,
            twist=(t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z),
            issued_at=message.issued_at.sec+message.issued_at.nanosec/1e9,
            valid_for=message.valid_for_ms/1000.,now=time.monotonic(),wall_now=time.time())
        self._publish()

    def _update_health(self):
        if self.health:
            self.core.set_inhibit(self.health.reason(),time.monotonic())

    def _publish(self):
        self._update_health()
        result=self.core.tick(time.monotonic())
        message=TwistStamped()
        message.header.stamp=self.get_clock().now().to_msg()
        message.header.frame_id='base_footprint'
        message.twist.linear.x=result.linear
        message.twist.angular.z=result.angular
        self.drive.publish(message)
        status=SupervisorStatus(state=result.state,owner=result.owner,source=result.source,reason=result.reason,
            lease_remaining_ms=int(result.lease_remaining*1000),accepted_sequence=result.sequence,has_command=result.has_command,
            health_required=self.health is not None,inhibited=bool(self.core.inhibit_reason),health_reason=self.core.inhibit_reason)
        self.status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node=MotionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.core.set_stop(True,time.monotonic())
            node._publish()  # Best effort only; the downstream driver still needs its own watchdog.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
