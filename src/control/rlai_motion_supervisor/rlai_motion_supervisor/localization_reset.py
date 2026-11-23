"""Explicit simulation localization reset; never clears a stop or acquires motion."""
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped

from .health import valid_covariance,valid_geometry


def initial_pose_message(*,x,y,yaw,xy_std,yaw_std,stamp):
    if not all(math.isfinite(v) for v in [x,y,yaw,xy_std,yaw_std]) or min(xy_std,yaw_std)<=0:
        raise ValueError('Pose must be finite and both standard deviations must be positive')
    xy_variance,yaw_variance=xy_std*xy_std,yaw_std*yaw_std
    if min(xy_variance,yaw_variance)<=0 or not math.isfinite(xy_variance) or not math.isfinite(yaw_variance):
        raise ValueError('Initial uncertainty cannot be represented as positive finite variance')
    message=PoseWithCovarianceStamped()
    message.header.frame_id='map'
    message.header.stamp=stamp
    message.pose.pose.position.x=float(x)
    message.pose.pose.position.y=float(y)
    angle=math.remainder(yaw,2*math.pi)/2
    message.pose.pose.orientation.z=math.sin(angle)
    message.pose.pose.orientation.w=math.cos(angle)
    message.pose.covariance[0]=xy_variance
    message.pose.covariance[7]=xy_variance
    message.pose.covariance[35]=yaw_variance
    return message


def reset_blocker(supervisor,navigation,samples,*,now):
    if not supervisor or not navigation or any(not 0<=now-item[1]<=.5 for item in [supervisor,navigation]):
        return 'STATUS_UNAVAILABLE'
    status=supervisor[0]
    nav=navigation[0]
    if status.owner:
        return 'OWNER_PRESENT'
    if status.state!='STOPPED':
        return 'STOP_NOT_LATCHED'
    if not status.health_required:
        return 'HEALTH_ADMISSION_DISABLED'
    if status.inhibited and status.health_reason not in {'AMCL_INVALID','AMCL_STALE','AMCL_MISSING','AMCL_TIMESTAMP',
        'MAP_TF_INVALID','MAP_TF_MISSING','MAP_TF_STALE','MAP_TF_TIMESTAMP'}:
        return 'OTHER_SOURCE_UNHEALTHY'
    if nav.phase!='IDLE' or nav.quarantined or nav.forwarding or nav.goal_id:
        return 'NAVIGATION_NOT_SETTLED'
    if len(samples)<4 or samples[-1][0]-samples[0][0]<.3 or not 0<=now-samples[-1][0]<=.3:
        return 'STATIONARITY_UNCONFIRMED'
    if any(not all(math.isfinite(value) for value in (at,v,w)) or not 0<=now-at<=.6 or abs(v)>.005 or abs(w)>.02 for at,v,w in samples):
        return 'STATIONARITY_UNCONFIRMED'
    return ''


def pose_consistent_with_prior(pose,prior):
    p,q=pose.pose.pose.position,pose.pose.pose.orientation
    expected,orientation=prior.pose.pose.position,prior.pose.pose.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    expected_yaw=math.atan2(2*(orientation.w*orientation.z+orientation.x*orientation.y),1-2*(orientation.y*orientation.y+orientation.z*orientation.z))
    return (abs(p.x-expected.x)<=3*math.sqrt(prior.pose.covariance[0])+.05 and
        abs(p.y-expected.y)<=3*math.sqrt(prior.pose.covariance[7])+.05 and
        abs(math.remainder(yaw-expected_yaw,2*math.pi))<=3*math.sqrt(prior.pose.covariance[35])+.05)


class ResetError(RuntimeError):
    def __init__(self,reason,request_sent=False):
        super().__init__(reason)
        self.request_sent=request_sent


def reset_localization(node,initial,*,timeout=6.):
    """Send one AMCL reset under a latched stop and confirm a real subsequent pose."""
    p,q=initial.pose.pose.position,initial.pose.pose.orientation
    covariance=initial.pose.covariance
    if initial.header.frame_id!='map' or not valid_geometry([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]) or not all(math.isfinite(v) for v in covariance) or any(covariance[i]<=0 for i in [0,7,35]) or any(v!=0 for i,v in enumerate(covariance) if i not in {0,7,35}):
        raise ValueError('Reset requires a finite map pose with explicit positive diagonal XY/yaw uncertainty')
    import rclpy
    from nav_msgs.msg import Odometry
    from nav2_msgs.srv import SetInitialPose
    from std_srvs.srv import Empty
    from rlai_motion_interfaces.msg import SupervisorStatus,NavigationStatus
    from rclpy.qos import qos_profile_sensor_data

    if not math.isfinite(timeout) or not 0<timeout<=10:
        raise ValueError('Reset deadline must be finite and in (0,10] seconds')
    current={'supervisor':None,'navigation':None,'pose':None}
    samples=[]
    last_odom_stamp=-1
    def receive(name,message):
        current[name]=(message,time.monotonic())
    def odometry(message):
        nonlocal last_odom_stamp
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        age=node.get_clock().now().nanoseconds/1e9-stamp
        if stamp<=last_odom_stamp or not -.1<=age<=.3:
            return
        last_odom_stamp=stamp
        t=message.twist.twist
        v=math.hypot(t.linear.x,t.linear.y,t.linear.z)
        w=math.hypot(t.angular.x,t.angular.y,t.angular.z)
        if message.header.frame_id!='odom' or message.child_frame_id!='base_footprint':
            v=math.inf
        now=time.monotonic()
        samples.append((now,v,w))
        while samples and now-samples[0][0]>.5:
            samples.pop(0)
    subscriptions=[
        node.create_subscription(SupervisorStatus,'motion_supervisor/status',lambda m:receive('supervisor',m),10),
        node.create_subscription(NavigationStatus,'nav2_adapter/status',lambda m:receive('navigation',m),10),
        node.create_subscription(Odometry,'odometry/filtered',odometry,qos_profile_sensor_data),
        node.create_subscription(PoseWithCovarianceStamped,'amcl_pose',lambda m:receive('pose',m),qos_profile_sensor_data)]
    client=node.create_client(SetInitialPose,'set_initial_pose')
    scan_client=node.create_client(Empty,'request_nomotion_update')
    scan_pending=None
    pending=None
    request_sent=False
    try:
        deadline=time.monotonic()+timeout
        blocker='STATUS_UNAVAILABLE'
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            blocker=reset_blocker(current['supervisor'],current['navigation'],samples,now=time.monotonic())
            if blocker in {'OWNER_PRESENT','STOP_NOT_LATCHED','HEALTH_ADMISSION_DISABLED','NAVIGATION_NOT_SETTLED','OTHER_SOURCE_UNHEALTHY'}:
                raise ResetError(blocker)
            if not blocker and client.service_is_ready() and node.get_clock().now().nanoseconds>0:
                break
        else:
            raise ResetError('PRECONDITION_TIMEOUT:'+blocker)
        initial.header.stamp=node.get_clock().now().to_msg()
        requested_stamp=initial.header.stamp.sec+initial.header.stamp.nanosec/1e9
        requested_at=time.monotonic()
        request=SetInitialPose.Request()
        request.pose=initial
        pending=client.call_async(request)
        request_sent=True
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            blocker=reset_blocker(current['supervisor'],current['navigation'],samples,now=time.monotonic())
            if blocker:
                raise ResetError('PRECONDITION_LOST:'+blocker,True)
            if not pending.done() or not current['pose']:
                continue
            if pending.exception():
                raise ResetError('RESET_SERVICE_FAILED',True)
            pose,receipt=current['pose']
            stamp=pose.header.stamp.sec+pose.header.stamp.nanosec/1e9
            age=node.get_clock().now().nanoseconds/1e9-stamp
            if receipt<=requested_at or not -.1<=age<=.5:
                continue
            p,q=pose.pose.pose.position,pose.pose.pose.orientation
            valid=pose.header.frame_id=='map' and valid_geometry([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]) and valid_covariance(pose.pose.covariance)
            if not valid:
                raise ResetError('RESET_ESTIMATE_REJECTED_STOP_HELD',True)
            if not pose_consistent_with_prior(pose,initial):
                raise ResetError('RESET_ESTIMATE_DISAGREES_WITH_PRIOR_STOP_HELD',True)
            if stamp<requested_stamp:
                # AMCL may consume one buffered pre-request scan after resetting.
                # Request one subsequent acquisition only after observing that pose.
                if scan_pending is None and scan_client.service_is_ready():
                    scan_pending=scan_client.call_async(Empty.Request())
                continue
            if scan_pending is not None:
                if not scan_pending.done():
                    continue
                if scan_pending.exception():
                    raise ResetError('POST_RESET_SCAN_FAILED_STOP_HELD',True)
            status,status_at=current['supervisor']
            if status_at<receipt or status.inhibited:
                continue
            return {'state':'ESTIMATE_CONFIRMED_STOP_LATCHED','pose_reset_requests':1,'scan_update_requests':int(scan_pending is not None),'motion_rearmed':False,
                'covariance_xy_yaw':[pose.pose.covariance[i] for i in [0,7,35]]}
        raise ResetError('RESET_OUTCOME_UNCONFIRMED_STOP_NOT_CLEARED',True)
    finally:
        if pending is not None and not pending.done():
            client.remove_pending_request(pending)
            pending.cancel()
        if scan_pending is not None and not scan_pending.done():
            scan_client.remove_pending_request(scan_pending)
            scan_pending.cancel()
        node.destroy_client(scan_client)
        node.destroy_client(client)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)


def main():
    import argparse
    import json
    import rclpy
    from builtin_interfaces.msg import Time
    from rclpy.parameter import Parameter
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['x','y','yaw','xy-std','yaw-std']:
        parser.add_argument('--'+name,type=float,required=True)
    args=parser.parse_args()
    try:
        initial=initial_pose_message(x=args.x,y=args.y,yaw=args.yaw,xy_std=args.xy_std,yaw_std=args.yaw_std,stamp=Time())
    except ValueError as exc:
        parser.error(str(exc))
    rclpy.init(args=[])
    node=rclpy.create_node('stopped_sim_localization_reset',parameter_overrides=[Parameter('use_sim_time',value=True)])
    try:
        print(json.dumps(reset_localization(node,initial)))
        return 0
    except ResetError as exc:
        print(json.dumps({'state':'RESET_NOT_CONFIRMED','reason':str(exc),'request_sent':exc.request_sent,
            'motion_rearmed':False}))
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
