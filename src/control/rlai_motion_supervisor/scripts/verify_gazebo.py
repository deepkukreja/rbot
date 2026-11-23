#!/usr/bin/env python3
"""Validate opt-in supervisor/controller routing in an owned, isolated Gazebo fixture."""
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
    parser.add_argument('--domain',type=int,default=74)
    args=parser.parse_args()
    if not 1<=args.domain<=101:
        parser.error('Use an isolated nonzero ROS domain in 1..101')
    args.out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    os.environ.pop('ROS_LOCALHOST_ONLY',None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    os.environ['GZ_PARTITION']='buildworks_supervisor_fixture_'+str(args.domain)
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from std_srvs.srv import SetBool
    from rlai_motion_interfaces.msg import LeasedTwist, SupervisorStatus
    from rlai_motion_interfaces.srv import AcquireControl

    rclpy.init(args=[])
    node=rclpy.create_node('supervised_gazebo_probe')
    odom,drive,status=[],[],[]
    node.create_subscription(Odometry,'/odometry/filtered',lambda m:odom.append({
        'received':time.monotonic(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,
        'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'speed':m.twist.twist.linear.x}),10)
    node.create_subscription(TwistStamped,'/diff_drive_controller/cmd_vel',lambda m:drive.append({
        'received':time.monotonic(),'x':m.twist.linear.x,'z':m.twist.angular.z}),10)
    node.create_subscription(SupervisorStatus,'/motion_supervisor/status',lambda m:status.append({
        'received':time.monotonic(),'state':m.state,'reason':m.reason,'owner':m.owner}),10)
    raw=node.create_publisher(TwistStamped,'/cmd_vel',1)
    commands=node.create_publisher(LeasedTwist,'/motion_supervisor/command',1)
    acquire=node.create_client(AcquireControl,'/motion_supervisor/acquire')
    stop=node.create_client(SetBool,'/motion_supervisor/set_stop')
    log=(args.out/'simulation.log').open('w')
    process=None
    report={'domain':args.domain,'partition':os.environ['GZ_PARTITION'],'checks':{},'metrics':{},
        'scope':'Small isolated differential-drive Gazebo fixture; no physical robot or live monitor control.'}
    deadline=time.monotonic()+1.
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
    assert not node.get_publishers_info_by_topic('/clock'), 'Choose an unused ROS domain'

    def wait(predicate,timeout=5.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():
                return
            if process.poll() is not None:
                raise RuntimeError('Owned fixture exited; inspect simulation.log')
        raise AssertionError('Timed out waiting for Gazebo contract condition')

    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)

    def call(client,request):
        future=client.call_async(request)
        wait(future.done)
        return future.result()

    def lease(owner='fixture-nav',source='AUTONOMY'):
        result=call(acquire,AcquireControl.Request(owner=owner,source=source,duration_ms=2000))
        assert result.granted,result.reason
        return result.lease_id

    def send(token,sequence,speed=.08):
        stamp=time.time_ns()
        message=LeasedTwist(lease_id=token,sequence=sequence,valid_for_ms=200)
        message.issued_at.sec=stamp//10**9
        message.issued_at.nanosec=stamp%10**9
        message.twist.linear.x=speed
        commands.publish(message)
        return time.monotonic()

    def drive_for(token,seconds=.6):
        end=time.monotonic()+seconds
        sequence=0
        last=None
        while time.monotonic()<end:
            sequence+=1
            last=send(token,sequence)
            spin(.07)
        return last,sequence

    def stationary():
        return len(odom)>=4 and all(abs(x['speed'])<.005 for x in odom[-4:])

    def descended_from(pid,ancestor):
        while pid>1:
            if pid==ancestor:
                return True
            try:
                text=Path(f'/proc/{pid}/status').read_text()
                pid=int(next(x.split(':')[1].strip() for x in text.splitlines() if x.startswith('PPid:')))
            except (OSError,StopIteration):
                return False
        return False

    try:
        process=subprocess.Popen(['ros2','launch','rlai_bringup','simulation.launch.py',
            'motion_supervision_enabled:=true','headless:=true','world:=empty',
            'teleop_enabled:=false','mapping_enabled:=false','use_amcl:=false',
            'camera_processing_enabled:=false','depth_camera_enabled:=false','lidar_2d_enabled:=false'],
            env=os.environ.copy(),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        wait(lambda:len(odom)>10 and drive and acquire.service_is_ready() and stop.service_is_ready(),timeout=40.)
        wait(stationary)
        publishers=node.get_publishers_info_by_topic('/diff_drive_controller/cmd_vel')
        assert len(publishers)==1 and publishers[0].node_name=='motion_supervisor'
        nodes=[name for name,_ in node.get_node_names_and_namespaces()]
        assert 'velocity_smoother' not in nodes and 'controller_server' not in nodes
        assert drive[-1]['x']==0.
        report['checks']['supervisor_is_only_configured_controller_publisher']=True
        report['nodes']=nodes

        start=len(odom)
        for _ in range(5):
            message=TwistStamped()
            message.twist.linear.x=.3
            raw.publish(message)
            spin(.06)
        assert stationary() and all(abs(x['speed'])<.005 for x in odom[start:])
        report['checks']['raw_cmd_vel_does_not_move_supervised_robot']=True

        token=lease()
        x0=odom[-1]['x']
        sent,seq=drive_for(token)
        assert max(x['speed'] for x in odom[-10:])>.03 and odom[-1]['x']>x0+.01
        report['checks']['valid_leased_command_reaches_real_simulated_controller']=True
        wait(lambda:drive[-1]['x']==0. and status[-1]['reason']=='COMMAND_TIMEOUT')
        zero=drive[-1]['received']
        wait(stationary)
        report['metrics']['watchdog_zero_after_last_input_wall_seconds']=zero-sent
        report['metrics']['watchdog_stationary_after_last_input_wall_seconds']=odom[-1]['received']-sent
        assert zero-sent<.6 and odom[-1]['received']-sent<1.5
        report['checks']['input_loss_zeros_and_stops_simulated_base']=True
        send(token,seq+1,.3)
        spin(.1)
        assert stationary() and drive[-1]['x']==0.
        report['checks']['expired_owner_cannot_restart_simulated_base']=True

        token=lease('fixture-operator','TELEOP')
        drive_for(token,seconds=.3)
        before_stop=time.monotonic()
        assert call(stop,SetBool.Request(data=True)).success
        wait(lambda:drive[-1]['x']==0. and status[-1]['state']=='STOPPED')
        wait(stationary)
        report['metrics']['latched_stop_stationary_wall_seconds']=odom[-1]['received']-before_stop
        denied=call(acquire,AcquireControl.Request(owner='competing-nav',source='AUTONOMY',duration_ms=1000))
        assert not denied.granted
        assert call(stop,SetBool.Request(data=False)).success
        spin(.08)
        assert stationary() and drive[-1]['x']==0.
        report['checks']['latched_stop_blocks_acquire_and_clear_does_not_resume']=True

        token=lease()
        drive_for(token,seconds=.4)
        candidates=[]
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                argv=(entry/'cmdline').read_bytes().decode().split('\0')
                if any(a.endswith('/lib/rlai_motion_supervisor/motion_supervisor') for a in argv[:2]) and descended_from(int(entry.name),process.pid):
                    candidates.append(int(entry.name))
            except (OSError,UnicodeError):
                pass
        assert len(candidates)==1, 'Expected one owned supervisor child'
        crash_at=time.monotonic()
        crash_x=odom[-1]['x']
        crash_stamp=odom[-1]['stamp']
        os.kill(candidates[0],signal.SIGKILL)
        wait(stationary,timeout=4.)
        report['metrics']['supervisor_crash_to_stationary_wall_seconds']=odom[-1]['received']-crash_at
        report['metrics']['supervisor_crash_to_stationary_sim_seconds']=odom[-1]['stamp']-crash_stamp
        report['metrics']['supervisor_crash_remaining_travel_m']=odom[-1]['x']-crash_x
        assert report['metrics']['supervisor_crash_to_stationary_sim_seconds']<1.2
        assert 0<=report['metrics']['supervisor_crash_remaining_travel_m']<.12
        report['checks']['controller_timeout_stops_after_owned_supervisor_crash']=True
        report['all_passed']=True
    except Exception as exc:
        report['all_passed']=False
        report['error_type']=type(exc).__name__
        raise
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        (args.out/'report.json').write_text(json.dumps(report,indent=2))
        (args.out/'odom_samples.json').write_text(json.dumps(odom,indent=2))
        (args.out/'drive_samples.json').write_text(json.dumps(drive,indent=2))
        (args.out/'status_samples.json').write_text(json.dumps(status,indent=2))
        print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    main()
