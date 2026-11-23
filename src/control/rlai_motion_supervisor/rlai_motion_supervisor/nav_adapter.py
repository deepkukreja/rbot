"""One-goal Nav2 action proxy; leases never follow raw velocity arrival alone."""
from dataclasses import dataclass, field
import math
import json
import time
import uuid

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from rclpy.qos import qos_profile_action_status_default,qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rlai_motion_interfaces.msg import LeasedTwist, SupervisorStatus, NavigationStatus
from rlai_motion_interfaces.srv import AcquireControl, RenewControl, ReleaseControl
from .nav_gate import VelocityGate, LifecycleGate, OcclusionGate, valid_occlusion_cloud, ownership_loss_reason


@dataclass
class Session:
    public: object
    owner: str
    created: float
    done: Future = field(default_factory=Future)
    phase: str = 'ACQUIRING'
    token: str = ''
    sequence: int = 0
    gate: VelocityGate = field(default_factory=VelocityGate)
    acquire_future: object = None
    send_future: object = None
    backend: object = None
    backend_terminal: bool = False
    release_future: object = None
    released: bool = False
    renew_future: object = None
    confirmed: bool = False
    granted_at: float = 0.
    backend_at: float = 0.
    last_renew: float = 0.
    last_velocity: float | None = None
    stop_at: float | None = None
    reason: str = ''
    outcome: str = 'ABORTED'
    code: int = 60001
    cancelled: bool = False
    cancel_sent: bool = False
    public_finished: bool = False


class NavigationAdapter(Node):
    def __init__(self,**kwargs):
        super().__init__('nav2_lease_adapter',**kwargs)
        required=self.declare_parameter('require_scan_occlusion',False,ParameterDescriptor(read_only=True)).value
        self.occlusion=OcclusionGate() if required else None
        trace=self.declare_parameter('trace_occlusion_receiver',False,ParameterDescriptor(read_only=True)).value
        if trace and not required:raise ValueError('Occlusion receive tracing requires the occlusion source')
        self.occlusion_trace=self.create_publisher(String,'nav2_adapter/occlusion_trace',qos_profile_sensor_data) if trace else None
        self.occlusion_trace_last_tick=None
        self.occlusion_trace_callbacks=0
        self.occlusion_trace_dropped=0
        if required:
            self.occlusion_input=self.create_subscription(PointCloud2,'scan_occlusion/points',
                self._occlusion_message_trace if trace else self._occlusion_message,qos_profile_sensor_data)
        # Callbacks may suspend on futures, but this executable uses one executor
        # thread so every state transition and packet admission stays serialized.
        self.group=ReentrantCallbackGroup()
        self.lifecycle=LifecycleGate()
        self.lifecycle_clients={name:self.create_client(GetState,name+'/get_state',callback_group=self.group) for name in self.lifecycle.required}
        self.lifecycle_pending={}
        self.last_lifecycle_poll=-math.inf
        self.nav_active=set()
        self.nav_quiet=time.monotonic()
        self.drain_ack=False
        self.drain_future=None
        self.last_drain=-math.inf
        self.motion_endpoints=('follow_path','spin','backup','drive_on_heading')
        self.motion_active={name:set() for name in self.motion_endpoints}
        self.motion_ack=set()
        self.motion_clients={name:self.create_client(CancelGoal,name+'/_action/cancel_goal',callback_group=self.group) for name in self.motion_endpoints}
        self.motion_pending={}
        self.motion_last={name:-math.inf for name in self.motion_endpoints}
        self.motion_subscriptions=[self.create_subscription(GoalStatusArray,name+'/_action/status',
            lambda m,key=name:self._motion_status(key,m),qos_profile_action_status_default,callback_group=self.group) for name in self.motion_endpoints]
        self.ctx=None
        self.reserved=False
        self.reserved_at=0.
        self.supervisor=None
        self.supervisor_at=-math.inf
        self.declare_parameter('max_goal_seconds',120.)
        self.commands=self.create_publisher(LeasedTwist,'motion_supervisor/command',1)
        self.diagnostics=self.create_publisher(NavigationStatus,'nav2_adapter/status',1)
        self.create_subscription(SupervisorStatus,'motion_supervisor/status',self._status,1,callback_group=self.group)
        self.create_subscription(TwistStamped,'nav2_adapter/cmd_vel_input',self._velocity,1,callback_group=self.group)
        self.acquire=self.create_client(AcquireControl,'motion_supervisor/acquire',callback_group=self.group)
        self.renew=self.create_client(RenewControl,'motion_supervisor/renew',callback_group=self.group)
        self.release=self.create_client(ReleaseControl,'motion_supervisor/release',callback_group=self.group)
        self.backend=ActionClient(self,NavigateToPose,'nav2_internal/navigate_to_pose',callback_group=self.group)
        self.cancel_all=self.create_client(CancelGoal,'nav2_internal/navigate_to_pose/_action/cancel_goal',callback_group=self.group)
        self.create_subscription(GoalStatusArray,'nav2_internal/navigate_to_pose/_action/status',self._nav_status,
            qos_profile_action_status_default,callback_group=self.group)
        self.server=ActionServer(self,NavigateToPose,'navigate_to_pose',self._execute,
            goal_callback=self._goal,cancel_callback=self._cancel,handle_accepted_callback=self._accepted,
            callback_group=self.group,result_timeout=60)
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.timer=self.create_timer(.05,self._tick,clock=self.steady,callback_group=self.group)
        self.get_logger().info('Guarded Nav2 proxy ready; goals require healthy supervised ownership. No raw-topic auto-acquire.')

    def _occlusion_trace_emit(self,value):
        try:
            self.occlusion_trace.publish(String(data=json.dumps(value,allow_nan=False)))
        except Exception:
            # Diagnostics cannot change command admission or freshness.
            self.occlusion_trace_dropped+=1

    def _occlusion_message(self,message):
        self.occlusion.observe(message.header.stamp.sec+message.header.stamp.nanosec/1e9,time.monotonic(),valid=valid_occlusion_cloud(message))

    def _occlusion_message_trace(self,message,info):
        # A required second argument selects rclpy's MessageInfo callback form.
        received=time.monotonic();wall_ns=time.time_ns();wall=wall_ns/1e9
        previous=self.occlusion.stamp
        valid=valid_occlusion_cloud(message);validated=time.monotonic()
        acquisition=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        self.occlusion.observe(acquisition,received,valid=valid)
        self.occlusion_trace_callbacks+=1
        self._occlusion_trace_emit({'kind':'cloud','received_steady':received,'received_wall':wall,
            'acquisition_ns':message.header.stamp.sec*10**9+message.header.stamp.nanosec,
            'payload_bytes':len(message.data),'points':message.width*message.height,'valid':valid,
            'validation_s':validated-received,'advanced':self.occlusion.stamp!=previous,
            'accepted_stamp':self.occlusion.stamp,'accepted_received_steady':self.occlusion.received,
            'callback_wall_ns':wall_ns,'middleware_source_wall_ns':info.get('source_timestamp'),
            'middleware_received_wall_ns':info.get('received_timestamp'),
            'publication_sequence':info.get('publication_sequence_number'),
            'reception_sequence':info.get('reception_sequence_number')})

    def _occlusion_trace_tick(self,now):
        previous=self.occlusion_trace_last_tick;self.occlusion_trace_last_tick=now
        try:matched=self.occlusion_input.get_publisher_count()
        except Exception:matched=None  # Unavailable diagnostics must not change admission.
        self._occlusion_trace_emit({'kind':'gate','steady':now,'wall':time.time(),
            'callback_gap_s':None if previous is None else now-previous,
            'accepted_stamp':self.occlusion.stamp,'accepted_received_steady':self.occlusion.received,
            'cloud_callbacks':self.occlusion_trace_callbacks,'diagnostic_drops':self.occlusion_trace_dropped,
            'ros_now':self.get_clock().now().nanoseconds/1e9,'reason':self._occlusion_reason(now),'matched_publishers':matched})

    def _occlusion_reason(self,now):
        return self.occlusion.reason(now,self.get_clock().now().nanoseconds/1e9) if self.occlusion is not None else ''

    def _status(self,message):
        self.supervisor,self.supervisor_at=message,time.monotonic()

    def _healthy(self,now):
        s=self.supervisor
        return s is not None and now-self.supervisor_at<.3 and s.health_required and not s.inhibited and s.state!='STOPPED' and not self.lifecycle.reason(now) and not self._occlusion_reason(now)

    def _lifecycle_result(self,name,future):
        try:
            state=future.result().current_state.id
        except Exception:
            state=0
        self.lifecycle.observe(name,state,time.monotonic())

    def _poll_lifecycle(self,now):
        if now-self.last_lifecycle_poll<.3:
            return
        self.last_lifecycle_poll=now
        for name,client in self.lifecycle_clients.items():
            pending=self.lifecycle_pending.get(name)
            if pending and not pending[0].done():
                if now-pending[1]<.8:
                    continue
                client.remove_pending_request(pending[0])
                pending[0].cancel()
            if client.service_is_ready():
                future=client.call_async(GetState.Request())
                self.lifecycle_pending[name]=(future,now)
                future.add_done_callback(lambda f,key=name:self._lifecycle_result(key,f))
            else:
                self.lifecycle.observe(name,0,now)

    def _nav_status(self,message):
        active={bytes(s.goal_info.goal_id.uuid) for s in message.status_list if s.status in
                {GoalStatus.STATUS_ACCEPTED,GoalStatus.STATUS_EXECUTING,GoalStatus.STATUS_CANCELING}}
        if active!=self.nav_active:
            self.nav_quiet=time.monotonic()
        self.nav_active=active
        if active and self.ctx is None:
            self.drain_ack=False

    def _drained(self,now):
        return self.drain_ack and not self.nav_active and not any(self.motion_active.values()) and len(self.motion_ack)==len(self.motion_endpoints) and now-self.nav_quiet>=.2

    def _motion_status(self,name,message):
        active={bytes(s.goal_info.goal_id.uuid) for s in message.status_list if s.status in
                {GoalStatus.STATUS_ACCEPTED,GoalStatus.STATUS_EXECUTING,GoalStatus.STATUS_CANCELING}}
        if active!=self.motion_active[name]:
            self.nav_quiet=time.monotonic()
        self.motion_active[name]=active
        if active:
            self.motion_ack.discard(name)

    def _motion_canceled(self,name,future):
        try:
            if future.result().return_code==CancelGoal.Response.ERROR_NONE:
                self.motion_ack.add(name)
        except Exception:
            pass

    def _drain_motion(self,now):
        for name,client in self.motion_clients.items():
            pending=self.motion_pending.get(name)
            if (name not in self.motion_ack or self.motion_active[name]) and now-self.motion_last[name]>=.5 and client.service_is_ready() and (pending is None or pending.done()):
                self.motion_last[name]=now
                future=client.call_async(CancelGoal.Request())
                self.motion_pending[name]=future
                future.add_done_callback(lambda f,key=name:self._motion_canceled(key,f))

    def _drain_result(self,future):
        try:
            self.drain_ack=future.result().return_code==CancelGoal.Response.ERROR_NONE
            if self.drain_ack and not self.nav_active:
                self.nav_quiet=time.monotonic()
        except Exception:
            self.drain_ack=False

    def _drain_startup(self,now):
        self._drain_motion(now)
        # A restarted proxy knows no previous goal outcome. Cancel/drain the
        # private Nav2 action first; an ignored cancellation keeps it quarantined.
        if not self.drain_ack and now-self.last_drain>=1. and self.backend.server_is_ready() and self.cancel_all.service_is_ready() and (self.drain_future is None or self.drain_future.done()):
            self.last_drain=now
            self.drain_future=self.cancel_all.call_async(CancelGoal.Request())
            self.drain_future.add_done_callback(self._drain_result)

    def _goal(self,request):
        now=time.monotonic()
        p,q=request.pose.pose.position,request.pose.pose.orientation
        stamp=request.pose.header.stamp.sec+request.pose.header.stamp.nanosec/1e9
        age=self.get_clock().now().nanoseconds/1e9-stamp
        valid=request.pose.header.frame_id=='map' and not request.behavior_tree and all(math.isfinite(x) for x in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]) and abs(sum(x*x for x in [q.x,q.y,q.z,q.w])-1.)<.01 and -.1<=age<=2.
        if self.ctx is not None or self.reserved or not valid or not self._healthy(now) or not self._drained(now) or self.supervisor.owner or not self.acquire.service_is_ready() or not self.release.service_is_ready() or not self.backend.server_is_ready():
            return GoalResponse.REJECT
        self.reserved,self.reserved_at=True,now
        return GoalResponse.ACCEPT

    def _accepted(self,goal):
        self.ctx=Session(goal,'nav2_'+uuid.uuid4().hex[:12],time.monotonic())
        goal.execute()

    async def _execute(self,goal):
        c=self.ctx
        try:
            c.acquire_future=self.acquire.call_async(AcquireControl.Request(owner=c.owner,source='AUTONOMY',duration_ms=2000))
            c.acquire_future.add_done_callback(lambda f:self._acquired(c,f))
            outcome,code,reason=await c.done
            if c.cancelled and goal.is_cancel_requested and outcome!='ABORTED':
                goal.canceled()
            elif outcome=='SUCCEEDED':
                goal.succeed()
            else:
                goal.abort()
            c.public_finished=True
            return NavigateToPose.Result(error_code=code,error_msg=reason)
        except Exception:
            self._halt(c,'ADAPTER_ERROR',60006)
            c.public_finished=True
            if goal.is_active:
                goal.abort()
            return NavigateToPose.Result(error_code=60006,error_msg='ADAPTER_ERROR')

    def _acquired(self,c,future):
        try:
            result=future.result()
            if not result.granted:
                c.backend_terminal=True
                self._halt(c,'OWNERSHIP_DENIED',60001)
                return
            c.token=result.lease_id
            c.granted_at=time.monotonic()
            c.last_renew=c.granted_at
            if c.stop_at is not None:
                self._release(c)
                c.backend_terminal=True
                return
            c.phase='STARTING'
            c.send_future=self.backend.send_goal_async(c.public.request,feedback_callback=lambda m:self._feedback(c,m))
            c.send_future.add_done_callback(lambda f:self._backend_accepted(c,f))
        except Exception:
            # A failed response can leave the downstream outcome unknown. Keep
            # this session quarantined rather than auto-acquiring another lease.
            self._halt(c,'ACQUIRE_OR_DISPATCH_ERROR',60001)

    def _backend_accepted(self,c,future):
        try:
            result=future.result()
            if not result.accepted:
                c.backend_terminal=True
                self._halt(c,'NAV2_REJECTED',60004)
                return
            c.backend=result
            c.backend_at=time.monotonic()
            result.get_result_async().add_done_callback(lambda f:self._backend_result(c,f))
            if c.stop_at is not None:
                self._cancel_backend(c)
            else:
                c.phase='RUNNING'
                c.gate.begin(activation_stamp=self.get_clock().now().nanoseconds/1e9)
        except Exception:
            self._halt(c,'NAV2_ACK_ERROR',60004)

    def _backend_result(self,c,future):
        try:
            result=future.result()
            c.backend_terminal=True
            if c.stop_at is None:
                if result.status==GoalStatus.STATUS_SUCCEEDED and self._healthy(time.monotonic()):
                    self._halt(c,'NAV2_SUCCEEDED',result.result.error_code,outcome='SUCCEEDED')
                else:
                    self._halt(c,'NAV2_TERMINAL_'+str(result.status),result.result.error_code or 60004)
        except Exception:
            self._halt(c,'NAV2_RESULT_UNKNOWN',60004)

    def _feedback(self,c,message):
        if self.ctx is c and c.stop_at is None and c.public.is_active:
            c.public.publish_feedback(message.feedback)

    def _cancel(self,goal):
        c=self.ctx
        if c is None or bytes(goal.goal_id.uuid)!=bytes(c.public.goal_id.uuid) or c.public_finished:
            return CancelResponse.REJECT
        c.cancelled=True
        if c.stop_at is None:
            self._halt(c,'CLIENT_CANCELED',0,outcome='CANCELED')
        elif c.outcome=='SUCCEEDED':
            c.outcome,c.reason,c.code='CANCELED','CLIENT_CANCELED',0
        return CancelResponse.ACCEPT

    def _envelope(self,c,twist,age=0.):
        c.sequence+=1
        stamp=time.time_ns()-int(age*1e9)
        message=LeasedTwist(lease_id=c.token,sequence=c.sequence,valid_for_ms=200)
        message.issued_at.sec=stamp//10**9
        message.issued_at.nanosec=stamp%10**9
        message.twist.linear.x,message.twist.linear.y,message.twist.linear.z=twist[:3]
        message.twist.angular.x,message.twist.angular.y,message.twist.angular.z=twist[3:]
        self.commands.publish(message)

    def _velocity(self,message):
        c=self.ctx
        if c is None or c.phase!='RUNNING' or c.stop_at is not None or not c.confirmed or not self._healthy(time.monotonic()):
            return
        t=message.twist
        try:
            sample=c.gate.admit(message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                self.get_clock().now().nanoseconds/1e9,time.monotonic(),
                (t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z))
        except ValueError:
            self._halt(c,'INVALID_NAV2_VELOCITY',60003)
            return
        if sample:
            c.last_velocity=sample['received']
            self._envelope(c,sample['twist'],sample['age'])

    def _release(self,c):
        if c.token and c.release_future is None and self.release.service_is_ready():
            c.release_future=self.release.call_async(ReleaseControl.Request(lease_id=c.token))
            c.release_future.add_done_callback(lambda f:self._released(c,f))

    def _released(self,c,future):
        try:
            future.result()
            c.released=True  # False response also means this token owns no lease.
        except Exception:
            pass

    def _cancel_backend(self,c):
        if c.backend and not c.backend_terminal and not c.cancel_sent:
            c.cancel_sent=True
            c.backend.cancel_goal_async()

    def _halt(self,c,reason,code,outcome='ABORTED'):
        if c.stop_at is not None:
            return
        c.stop_at=time.monotonic()
        c.reason,c.code,c.outcome=reason,code,outcome
        c.phase='STOPPING'
        c.gate.end()  # Close before zero/release/cancel can wait for transport.
        self.motion_ack.clear()
        if c.token:
            self._envelope(c,(0.,)*6)
        elif c.send_future is None and c.acquire_future is not None and c.acquire_future.done():
            c.backend_terminal=True
        self._release(c)
        self._cancel_backend(c)

    def _renewed(self,c,future):
        try:
            if not future.result().success:
                self._halt(c,'LEASE_RENEWAL_REJECTED',60001)
        except Exception:
            self._halt(c,'LEASE_RENEWAL_UNAVAILABLE',60001)

    def _tick(self):
        now=time.monotonic()
        if getattr(self,'occlusion_trace',None) is not None:self._occlusion_trace_tick(now)
        self._poll_lifecycle(now)
        c=self.ctx
        if c is None:
            self._drain_startup(now)
            if self.reserved and now-self.reserved_at>1.:
                self.reserved=False
            drained=self._drained(now)
            self.diagnostics.publish(NavigationStatus(phase='IDLE' if drained else 'QUARANTINED',
                reason=('READY' if self._healthy(now) else (self._occlusion_reason(now) or 'HEALTH_UNAVAILABLE')) if drained else 'DRAINING_NAV2',quarantined=not drained))
            return
        if c.stop_at is None:
            if self._occlusion_reason(now):
                self._halt(c,self._occlusion_reason(now),60008)
            elif not self._healthy(now):
                self._halt(c,'LOCAL_INHIBIT_OR_STATUS_LOSS',60002)
            elif c.backend and any(identity!=bytes(c.backend.goal_id.uuid) for identity in self.nav_active):
                self._halt(c,'UNEXPECTED_NAV2_GOAL',60004)
            elif now-c.created>float(self.get_parameter('max_goal_seconds').value):
                self._halt(c,'GOAL_DEADLINE',60005)
            elif c.phase=='ACQUIRING' and now-c.created>.8:
                self._halt(c,'LEASE_ACK_TIMEOUT',60001)
            elif c.phase=='STARTING' and now-c.granted_at>1.5:
                self._halt(c,'NAV2_ACK_TIMEOUT',60004)
            elif c.token:
                s=self.supervisor
                if s.owner==c.owner and s.source=='AUTONOMY' and s.state in {'OWNED','WAITING_COMMAND'}:
                    c.confirmed=True
                elif c.confirmed or now-c.granted_at>.3:
                    self._halt(c,ownership_loss_reason(s),60001)
                if c.stop_at is None:
                    if c.last_velocity is None:
                        if c.phase=='RUNNING' and now-c.backend_at>3.:
                            self._halt(c,'INITIAL_NAV2_VELOCITY_TIMEOUT',60003)
                        else:
                            self._envelope(c,(0.,)*6)  # Bounded planning intent, never cached motion.
                    elif now-c.last_velocity>.35:
                        self._halt(c,'NAV2_VELOCITY_TIMEOUT',60003)
                    if c.stop_at is None and now-c.last_renew>=.5 and (c.renew_future is None or c.renew_future.done()):
                        if self.renew.service_is_ready():
                            c.last_renew=now
                            c.renew_future=self.renew.call_async(RenewControl.Request(lease_id=c.token,duration_ms=2000))
                            c.renew_future.add_done_callback(lambda f:self._renewed(c,f))
                        else:
                            self._halt(c,'LEASE_SERVICE_LOST',60001)
        if c.stop_at is not None:
            self._drain_motion(now)
            self._release(c)
            self._cancel_backend(c)
            local_closed=c.released or (now-self.supervisor_at<.3 and self.supervisor.owner!=c.owner) or now-c.stop_at>.5
            if c.outcome=='SUCCEEDED' and not self._drained(now) and now-c.stop_at>1.:
                c.outcome,c.code,c.reason='ABORTED',60007,'MOTION_CLEANUP_UNCONFIRMED'
            completion_known=c.outcome!='SUCCEEDED' or self._drained(now)
            if local_closed and completion_known and (c.backend_terminal or now-c.stop_at>1.) and not c.done.done():
                c.done.set_result((c.outcome,c.code,c.reason))
            settled=c.backend_terminal and self._drained(now) and c.acquire_future is not None and c.acquire_future.done() and (c.released or (now-self.supervisor_at<.3 and self.supervisor.owner!=c.owner))
            if c.public_finished and settled:
                self.ctx=None
                self.reserved=False
        self.diagnostics.publish(NavigationStatus(phase=c.phase,goal_id=bytes(c.public.goal_id.uuid).hex(),reason=c.reason,
            forwarding=c.phase=='RUNNING' and c.confirmed and c.stop_at is None,
            quarantined=c.public_finished and self.ctx is c))


def main(args=None):
    rclpy.init(args=args)
    node=NavigationAdapter()
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.ctx and node.ctx.stop_at is None and rclpy.ok():
            node._halt(node.ctx,'ADAPTER_SHUTDOWN',60006)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
