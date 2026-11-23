#!/usr/bin/env python3
"""Full guarded Nav2 goals/cancellation/source-loss in one owned isolated simulator."""
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
    parser.add_argument('--domain',type=int,default=76)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--idle-preflight-only',action='store_true',help='Verify idle expiry and one explicit refresh without sending goals')
    args=parser.parse_args()
    assert args.map.is_file() and 1<=args.domain<=101
    args.out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    os.environ.pop('ROS_LOCALHOST_ONLY',None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    os.environ['GZ_PARTITION']='buildworks_guarded_nav_'+str(args.domain)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped,PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from nav2_msgs.action import NavigateToPose
    from rlai_motion_interfaces.msg import SupervisorStatus,NavigationStatus

    rclpy.init(args=[])
    node=Node('guarded_nav2_gazebo_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    health,adapter,wheel,truth,drive,amcl=[],[],[],[],[],[]
    children,logs=[],[]
    relay={'enabled':True,'count':0}
    forwarded=node.create_publisher(LaserScan,'/guard_probe/scan',qos_profile_sensor_data)
    def scan(message):
        if relay['enabled']:
            forwarded.publish(message)  # Preserve original frame, ranges and acquisition stamp.
            relay['count']+=1
    node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
    node.create_subscription(SupervisorStatus,'/motion_supervisor/status',lambda m:health.append({
        'at':time.monotonic(),'state':m.state,'owner':m.owner,'inhibited':m.inhibited,'reason':m.health_reason}),10)
    node.create_subscription(NavigationStatus,'/nav2_adapter/status',lambda m:adapter.append({
        'at':time.monotonic(),'phase':m.phase,'reason':m.reason,'quarantined':m.quarantined}),10)
    node.create_subscription(Odometry,'/diff_drive_controller/odom',lambda m:wheel.append({
        'at':time.monotonic(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,
        'speed':m.twist.twist.linear.x,'turn':m.twist.twist.angular.z}),qos_profile_sensor_data)
    node.create_subscription(Odometry,'/gazebo/ground_truth/odom',lambda m:truth.append({
        'at':time.monotonic(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,
        'x':m.pose.pose.position.x,'y':m.pose.pose.position.y}),qos_profile_sensor_data)
    node.create_subscription(TwistStamped,'/diff_drive_controller/cmd_vel',lambda m:drive.append({
        'at':time.monotonic(),'x':m.twist.linear.x,'z':m.twist.angular.z}),10)
    node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:amcl.append({
        'at':time.monotonic(),'frame':m.header.frame_id,'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,
        'covariance':list(m.pose.covariance),'position':[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z],
        'quaternion':[m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w]}),qos_profile_sensor_data)
    client=ActionClient(node,NavigateToPose,'/navigate_to_pose')
    report={'domain':args.domain,'partition':os.environ['GZ_PARTITION'],'map':str(args.map),
        'checks':{},'goals':[],'metrics':{},'scope':'Existing ROS/Nav2 and real Gazebo sensors/controller; simulated robot only.'}

    def wait(predicate,seconds=10.):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():
                return
            if any(child.poll() is not None for child in children):
                raise RuntimeError('Owned fixture process exited; inspect logs')
        raise AssertionError('Timed out waiting for full guarded navigation condition')

    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)

    def launch(label,command):
        log=(args.out/(label+'.log')).open('w')
        logs.append(log)
        p=subprocess.Popen(command,env=os.environ.copy(),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(p)

    def ready():
        return bool(health and adapter) and not health[-1]['inhibited'] and not health[-1]['owner'] and adapter[-1]['phase']=='IDLE' and adapter[-1]['reason']=='READY'

    def stationary():
        return len(wheel)>=4 and all(abs(s['speed'])<.005 and abs(s['turn'])<.02 for s in wheel[-4:])

    def goal(x,y=0.):
        from rlai_motion_supervisor.preflight import prepare_localization
        prepare_localization(node)
        request=NavigateToPose.Goal()
        request.pose.header.frame_id='map'
        request.pose.header.stamp=node.get_clock().now().to_msg()
        request.pose.pose.position.x,request.pose.pose.position.y=x,y
        request.pose.pose.orientation.w=1.
        future=client.send_goal_async(request)
        wait(future.done)
        handle=future.result()
        assert handle.accepted,'Guarded goal rejected'
        return handle

    def outcome(handle,seconds=45.):
        future=handle.get_result_async()
        wait(future.done,seconds)
        value=future.result()
        report['goals'].append({'status':value.status,'error_code':value.result.error_code,'reason':value.result.error_msg,
            'truth':truth[-1] if truth else None})
        return value

    def first_zero(at):
        return next(d['at'] for d in drive if d['at']>=at and d['x']==d['z']==0.)

    try:
        spin(.5)
        assert not node.get_publishers_info_by_topic('/clock'),'Choose an unused domain'
        launch('simulation',['ros2','launch','rlai_bringup','simulation.launch.py',
            'headless:=true','world:=small_warehouse','motion_supervision_enabled:=true',
            'motion_health_enabled:=true','motion_health_scan_topic:=/guard_probe/scan',
            'use_amcl:=true','map_yaml_file:='+str(args.map),'teleop_enabled:=false',
            'camera_processing_enabled:=false','depth_camera_enabled:=false'])
        launch('navigation',['ros2','launch','rlai_navigation','navigation.launch.py','use_motion_supervisor:=true'])
        wait(lambda:client.server_is_ready() and ready() and stationary(),seconds=55.)
        publishers=node.get_publishers_info_by_topic('/diff_drive_controller/cmd_vel')
        assert len(publishers)==1 and publishers[0].node_name=='motion_supervisor'
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        report['checks']['real_scan_odom_amcl_tf_health_and_exclusive_output']=True
        if args.idle_preflight_only:
            spin(5.4)
            assert health[-1]['inhibited'] and health[-1]['reason']=='AMCL_STALE'
            assert not health[-1]['owner'] and stationary()
            report['checks']['idle_localization_expires_without_automatic_refresh']=True
            from rlai_motion_supervisor.preflight import prepare_localization
            report['preflight']=prepare_localization(node)
            wait(ready)
            assert stationary() and all(d['x']==d['z']==0. for d in drive)
            assert all(not h['owner'] for h in health)
            report['checks']['one_explicit_refresh_restores_admission_without_motion']=True
            spin(5.4)
            assert health[-1]['inhibited'] and health[-1]['reason']=='AMCL_STALE'
            assert not health[-1]['owner'] and stationary()
            report['checks']['refresh_is_not_repeated_and_admission_expires_again']=True
            report['all_passed']=True
            return
        first=goal(1.)
        assert outcome(first).status==4
        wait(lambda:ready() and stationary())
        report['checks']['real_nav2_goal_succeeds_through_lease_proxy']=True

        moving=goal(3.)
        wait(lambda:wheel and abs(wheel[-1]['speed'])>.03)
        at=time.monotonic()
        cancelled=moving.cancel_goal_async()
        wait(cancelled.done)
        assert outcome(moving).status==5
        wait(lambda:ready() and stationary())
        report['metrics']['cancel_to_first_zero_seconds']=first_zero(at)-at
        report['metrics']['cancel_to_stationary_seconds']=wheel[-1]['at']-at
        report['checks']['real_nav2_cancel_releases_and_stops']=True

        interrupted=goal(0.)
        wait(lambda:wheel and abs(wheel[-1]['speed'])>.03)
        at=time.monotonic()
        relay['enabled']=False
        assert outcome(interrupted).status==6
        wait(lambda:health[-1]['inhibited'] and stationary())
        assert health[-1]['reason']=='SCAN_STALE'
        report['metrics']['scan_hop_loss_to_first_zero_seconds']=first_zero(at)-at
        report['metrics']['scan_hop_loss_to_stationary_seconds']=wheel[-1]['at']-at
        report['checks']['real_scan_consumer_loss_aborts_without_disabling_nav2_scan']=True
        relay['enabled']=True
        wait(ready)
        spin(.4)
        assert stationary() and not health[-1]['owner']
        report['checks']['real_source_recovery_does_not_resume_old_goal']=True
        final=goal(0.)
        assert outcome(final).status==4
        wait(lambda:ready() and stationary())
        report['checks']['explicit_new_goal_after_source_recovery_succeeds']=True
        report['metrics']['real_scan_messages_forwarded_unmodified']=relay['count']
        report['all_passed']=True
    except Exception as exc:
        report['all_passed']=False
        report['error_type']=type(exc).__name__
        report['error']=str(exc)
        report['last_health']=health[-1] if health else None
        report['last_adapter']=adapter[-1] if adapter else None
        report['last_amcl']=amcl[-1] if amcl else None
        raise
    finally:
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM)
                    p.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        for log in logs:
            log.close()
        for name,values in [('health',health),('adapter',adapter),('wheel',wheel),('truth',truth),('drive',drive),('amcl',amcl)]:
            (args.out/(name+'_samples.json')).write_text(json.dumps(values,indent=2))
        (args.out/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    main()
