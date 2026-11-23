#!/usr/bin/env python3
"""Real ROS actions/services with synthetic sensors/Nav2 in an isolated domain."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--domain',type=int,default=75)
    args=parser.parse_args()
    assert 1<=args.domain<=101
    args.out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    os.environ.pop('ROS_LOCALHOST_ONLY',None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.action import ActionServer,ActionClient,GoalResponse,CancelResponse
    from rclpy.clock import Clock as RClock,ClockType
    from rclpy.task import Future
    from action_msgs.msg import GoalStatus
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import TwistStamped,PoseWithCovarianceStamped,TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock
    from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
    from nav2_msgs.action import NavigateToPose,FollowPath
    from lifecycle_msgs.srv import GetState
    from rlai_motion_interfaces.msg import SupervisorStatus,NavigationStatus
    from rlai_motion_interfaces.srv import AcquireControl,ReleaseControl

    rclpy.init(args=[])
    node=Node('guarded_proxy_contract_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    group=ReentrantCallbackGroup()
    flags={name:True for name in ['clock','scan','odom','map_tf','covariance']}
    state={'mode':'run','fake':None,'child':None,'sim':100.,'origin':time.monotonic(),'emit_at':0.}
    health,status,drive=[],[],[]
    children=[]
    logs=[]
    report={'domain':args.domain,'checks':{},'metrics':{},'scope':'Synthetic scan/odometry/AMCL/TF and fake Nav2; real ROS proxy/supervisor; no simulator or physical actuator.'}
    clk=node.create_publisher(Clock,'/clock',10)
    scans=node.create_publisher(LaserScan,'/scan',10)
    odoms=node.create_publisher(Odometry,'/odometry/filtered',10)
    poses=node.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',10)
    velocities=node.create_publisher(TwistStamped,'/nav2_adapter/cmd_vel_input',1)
    tf=TransformBroadcaster(node)
    static=StaticTransformBroadcaster(node)
    extrinsic=TransformStamped()
    extrinsic.header.frame_id='base_footprint'
    extrinsic.child_frame_id='lidar_2d_link'
    extrinsic.transform.rotation.w=1.
    static.sendTransform(extrinsic)
    node.create_subscription(SupervisorStatus,'/motion_supervisor/status',lambda m:health.append(m),10)
    node.create_subscription(NavigationStatus,'/nav2_adapter/status',lambda m:status.append(m),10)
    node.create_subscription(TwistStamped,'/motion_supervisor/drive',lambda m:drive.append({'at':time.monotonic(),'x':m.twist.linear.x}),10)

    def fake_accepted(goal):
        assert state['fake'] is None
        state['fake']={'goal':goal,'mode':state['mode'],'start':time.monotonic(),'done':Future()}
        goal.execute()

    async def fake_execute(goal):
        context=state['fake']
        await context['done']
        return NavigateToPose.Result(error_code=0)

    def fake_cancel(goal):
        context=state['fake']
        return CancelResponse.REJECT if context and context['mode']=='ignore_cancel' else CancelResponse.ACCEPT

    fake=ActionServer(node,NavigateToPose,'/nav2_internal/navigate_to_pose',fake_execute,
        callback_group=group,goal_callback=lambda req:GoalResponse.ACCEPT if state['fake'] is None else GoalResponse.REJECT,
        handle_accepted_callback=fake_accepted,cancel_callback=fake_cancel)
    public=ActionClient(node,NavigateToPose,'/navigate_to_pose',callback_group=group)
    internal=ActionClient(node,NavigateToPose,'/nav2_internal/navigate_to_pose',callback_group=group)
    def child_accepted(goal):
        state['child']={'goal':goal,'done':Future()}
        goal.execute()
    async def child_execute(goal):
        context=state['child']
        await context['done']
        return FollowPath.Result()
    child_server=ActionServer(node,FollowPath,'follow_path',child_execute,callback_group=group,
        handle_accepted_callback=child_accepted,cancel_callback=lambda g:CancelResponse.REJECT)
    child_client=ActionClient(node,FollowPath,'follow_path',callback_group=group)
    def no_motion_cancel(request,response):
        response.return_code=CancelGoal.Response.ERROR_NONE
        return response
    recovery_services=[node.create_service(CancelGoal,name+'/_action/cancel_goal',no_motion_cancel) for name in ['spin','backup','drive_on_heading']]
    acquire=node.create_client(AcquireControl,'/motion_supervisor/acquire',callback_group=group)
    release=node.create_client(ReleaseControl,'/motion_supervisor/release',callback_group=group)
    def active_state(request,response):
        response.current_state.id=3
        response.current_state.label='active'
        return response
    lifecycle_services=[node.create_service(GetState,name+'/get_state',active_state) for name in
                        ['planner_server','controller_server','behavior_server','bt_navigator']]

    def finish_fake(success=False):
        context=state['fake']
        assert context
        goal=context['goal']
        if goal.is_cancel_requested:
            goal.canceled()
        elif success:
            goal.succeed()
        else:
            goal.abort()
        state['fake']=None
        context['done'].set_result(True)

    def put_stamp(message,stamp):
        ns=int(stamp*1e9)
        message.header.stamp.sec=ns//10**9
        message.header.stamp.nanosec=ns%10**9

    def emit():
        now=time.monotonic()
        if flags['clock']:
            state['sim']=100.+now-state['origin']
        stamp=state['sim']
        clock=Clock()
        ns=int(stamp*1e9)
        clock.clock.sec,clock.clock.nanosec=ns//10**9,ns%10**9
        clk.publish(clock)
        if now-state['emit_at']<.05:
            return
        state['emit_at']=now
        if flags['scan']:
            scan=LaserScan()
            scan.header.frame_id='lidar_2d_link'
            put_stamp(scan,stamp)
            scan.angle_min,scan.angle_max,scan.angle_increment=-1.,1.,.01
            scan.range_min,scan.range_max=.1,10.
            scan.ranges=[2.]*201
            scans.publish(scan)
        if flags['odom']:
            odom=Odometry()
            odom.header.frame_id='odom'
            odom.child_frame_id='base_footprint'
            put_stamp(odom,stamp)
            odom.pose.pose.orientation.w=1.
            odoms.publish(odom)
        pose=PoseWithCovarianceStamped()
        pose.header.frame_id='map'
        put_stamp(pose,stamp)
        pose.pose.pose.orientation.w=1.
        pose.pose.covariance[0]=pose.pose.covariance[7]=.01 if flags['covariance'] else .5
        pose.pose.covariance[35]=.01
        poses.publish(pose)
        transforms=[]
        for parent,child,future in [('odom','base_footprint',0.),('map','odom',1.)]:
            if parent=='map' and not flags['map_tf']:
                continue
            t=TransformStamped()
            t.header.frame_id,t.child_frame_id=parent,child
            put_stamp(t,stamp+future)
            t.transform.rotation.w=1.
            transforms.append(t)
        tf.sendTransform(transforms)
        context=state['fake']
        if context:
            if context['goal'].is_cancel_requested and context['mode']!='ignore_cancel':
                finish_fake()
            elif context['mode']=='success' and now-context['start']>.7:
                finish_fake(True)
            elif context['mode']!='no_velocity':
                velocity=TwistStamped()
                velocity.header.stamp=node.get_clock().now().to_msg()
                velocity.twist.linear.x=.12
                velocities.publish(velocity)

    timer=node.create_timer(.02,emit,clock=RClock(clock_type=ClockType.STEADY_TIME))

    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)

    def wait(predicate,seconds=5.):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():
                return
        raise AssertionError('Timed out waiting for guarded proxy condition')

    def launch(name,args_list):
        log=(args.out/(name+'_'+str(len(children))+'.log')).open('w')
        logs.append(log)
        extra=args_list[1:]
        if extra[:1]==['--ros-args']:
            extra=extra[1:]
        p=subprocess.Popen(['ros2','run','rlai_motion_supervisor',args_list[0],'--ros-args','-p','use_sim_time:=true',*extra],
            env=os.environ.copy(),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(p)
        return p

    def goal(mode='run',client=None):
        if client is None:
            client=public
        state['mode']=mode
        request=NavigateToPose.Goal()
        request.pose.header.frame_id='map'
        request.pose.header.stamp=node.get_clock().now().to_msg()
        request.pose.pose.position.x=1.
        request.pose.pose.orientation.w=1.
        f=client.send_goal_async(request)
        wait(f.done)
        return f.result()

    def result(handle):
        future=handle.get_result_async()
        wait(future.done,seconds=6.)
        return future.result()

    def ready():
        return bool(health and status) and not health[-1].inhibited and status[-1].phase=='IDLE' and status[-1].reason=='READY'

    def moving():
        return bool(drive) and abs(drive[-1]['x']-.12)<1e-6

    def zero():
        return bool(drive) and drive[-1]['x']==0.

    try:
        supervisor=launch('supervisor',['motion_supervisor','--ros-args','-p','health_required:=true'])
        wait(lambda:health and health[-1].health_required and not health[-1].inhibited,seconds=10.)
        wait(internal.server_is_ready)
        orphan=goal('ignore_cancel',internal)
        assert orphan.accepted
        wait(lambda:state['fake'] is not None)
        adapter=launch('adapter',['nav_adapter'])
        wait(public.server_is_ready,seconds=8.)
        wait(lambda:len(node.get_subscriptions_info_by_topic('/nav2_internal/navigate_to_pose/_action/status'))>=2)
        spin(1.)
        assert state['fake'] and not health[-1].inhibited
        wait(lambda:status and status[-1].quarantined)
        denied=goal('run')
        assert not denied.accepted,'Restarted proxy must drain unknown existing Nav2 goal before accepting ownership'
        assert not health[-1].owner and zero()
        report['checks']['startup_orphan_goal_blocks_ownership']=True
        finish_fake()
        wait(ready)
        report['checks']['known_backend_terminal_allows_new_explicit_goal']=True

        handle=goal('success')
        assert handle.accepted
        wait(moving)
        outcome=result(handle)
        assert outcome.status==GoalStatus.STATUS_SUCCEEDED
        wait(ready)
        assert zero() and not health[-1].owner
        report['checks']['success_acquires_forwards_and_releases']=True

        handle=goal('success')
        assert handle.accepted
        wait(moving)
        child_request=child_client.send_goal_async(FollowPath.Goal())
        wait(child_request.done)
        assert child_request.result().accepted
        child_outcome=result(handle)
        assert child_outcome.status==GoalStatus.STATUS_ABORTED and child_outcome.result.error_code==60007
        wait(lambda:status[-1].quarantined)
        assert zero() and not goal().accepted
        child=state['child']
        child['goal'].abort()
        state['child']=None
        child['done'].set_result(True)
        wait(ready)
        report['checks']['terminal_nav_result_waits_for_controller_child_cleanup']=True

        handle=goal()
        assert handle.accepted
        wait(moving)
        other=goal()
        assert not other.accepted
        started=time.monotonic()
        canceled=handle.cancel_goal_async()
        wait(canceled.done)
        outcome=result(handle)
        assert outcome.status==GoalStatus.STATUS_CANCELED
        wait(ready)
        assert zero()
        report['metrics']['client_cancel_to_zero_seconds']=next(d['at']-started for d in drive if d['at']>=started and d['x']==0.)
        late=TwistStamped()
        late.header.stamp=node.get_clock().now().to_msg()
        late.twist.linear.x=.4
        velocities.publish(late)
        spin(.1)
        assert zero() and not health[-1].owner
        report['checks']['cancel_competing_goal_and_late_velocity_do_not_rearm']=True

        for source in ['scan','odom','map_tf','covariance','clock']:
            handle=goal()
            assert handle.accepted
            wait(moving)
            at=time.monotonic()
            flags[source]=False
            outcome=result(handle)
            assert outcome.status==GoalStatus.STATUS_ABORTED
            assert zero()
            report['metrics'][source+'_loss_to_zero_seconds']=next(d['at']-at for d in drive if d['at']>=at and d['x']==0.)
            flags[source]=True
            wait(ready)
            spin(.1)
            assert zero() and not health[-1].owner
            report['checks'][source+'_loss_aborts_and_restoration_needs_new_goal']=True

        handle=goal('ignore_cancel')
        assert handle.accepted
        wait(moving)
        f=handle.cancel_goal_async()
        wait(f.done)
        outcome=result(handle)
        assert outcome.status==GoalStatus.STATUS_CANCELED
        assert zero()
        assert not goal().accepted
        wait(lambda:status[-1].quarantined)
        finish_fake()
        wait(ready)
        report['checks']['unconfirmed_downstream_cancel_quarantines_new_goals']=True

        handle=goal('no_velocity')
        assert handle.accepted
        assert result(handle).status==GoalStatus.STATUS_ABORTED
        wait(ready)
        assert zero()
        report['checks']['planning_without_velocity_is_bounded_and_zero_only']=True

        handle=goal('ignore_cancel')
        assert handle.accepted
        wait(moving)
        os.killpg(adapter.pid,signal.SIGKILL)
        adapter.wait(timeout=5)
        wait(lambda:zero() and not health[-1].owner)
        public.destroy()
        public=ActionClient(node,NavigateToPose,'/navigate_to_pose',callback_group=group)
        before_restart=len(status)
        adapter=launch('adapter_restart',['nav_adapter'])
        wait(public.server_is_ready,seconds=8.)
        wait(lambda:len(status)>before_restart+3 and status[-1].quarantined,seconds=8.)
        assert not goal().accepted
        assert zero() and not health[-1].owner
        finish_fake()
        wait(ready)
        report['checks']['adapter_restart_does_not_reuse_orphan_goal_or_lease']=True
        report['all_passed']=True
    except Exception as exc:
        report['all_passed']=False
        report['error_type']=type(exc).__name__
        raise
    finally:
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM)
                    p.wait(timeout=5)
        if state['fake']:
            finish_fake()
        if state['child']:
            state['child']['goal'].abort()
            state['child']['done'].set_result(True)
        node.destroy_node()
        rclpy.shutdown()
        for log in logs:
            log.close()
        (args.out/'report.json').write_text(json.dumps(report,indent=2))
        (args.out/'drive_samples.json').write_text(json.dumps(drive,indent=2))
        print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    main()
