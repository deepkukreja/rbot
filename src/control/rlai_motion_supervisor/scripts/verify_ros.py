#!/usr/bin/env python3
"""Exercise real ROS leases/topics in an isolated domain with an unconnected drive sink."""
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
    parser.add_argument('--domain',type=int,default=73)
    args=parser.parse_args()
    if not 1<=args.domain<=101:
        parser.error('Choose an isolated nonzero domain in 1..101')
    args.out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    os.environ.pop('ROS_LOCALHOST_ONLY',None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import SetBool
    from rlai_motion_interfaces.msg import LeasedTwist, SupervisorStatus
    from rlai_motion_interfaces.srv import AcquireControl, RenewControl, ReleaseControl

    rclpy.init(args=[])
    node=rclpy.create_node('motion_supervisor_contract_probe')
    assert not node.get_publishers_info_by_topic('/motion_supervisor/drive'), 'Domain already has a drive source'
    outputs,statuses=[],[]
    node.create_subscription(TwistStamped,'/motion_supervisor/drive',lambda m:outputs.append({
        'received':time.monotonic(),'x':m.twist.linear.x,'z':m.twist.angular.z,
        'ros_stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9}),10)
    node.create_subscription(SupervisorStatus,'/motion_supervisor/status',lambda m:statuses.append({
        'received':time.monotonic(),'state':m.state,'owner':m.owner,'source':m.source,
        'reason':m.reason,'sequence':m.accepted_sequence,'has_command':m.has_command}),10)
    pub=node.create_publisher(LeasedTwist,'/motion_supervisor/command',1)
    raw=node.create_publisher(TwistStamped,'/cmd_vel',1)
    clock=node.create_publisher(Clock,'/clock',10)
    clients={name:node.create_client(kind,'/motion_supervisor/'+name) for name,kind in [
        ('acquire',AcquireControl),('renew',RenewControl),('release',ReleaseControl),('set_stop',SetBool)]}
    log=(args.out/'supervisor.log').open('w')
    process=None
    report={'domain':args.domain,'checks':{},'metrics':{},'scope':'Real ROS interfaces; no driver, simulator, hardware or cloud commands.'}
    tokens=[]

    def launch():
        return subprocess.Popen(['ros2','run','rlai_motion_supervisor','motion_supervisor','--ros-args','-p','use_sim_time:=true'],
            env=os.environ.copy(),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

    def stop_owned(child):
        if child and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                child.wait(timeout=5)

    def wait(predicate,timeout=3.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():
                return
            if process.poll() is not None:
                raise RuntimeError('Owned supervisor exited; inspect supervisor.log')
        raise AssertionError('Timed out waiting for ROS contract condition')

    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)

    def call(name,request):
        future=clients[name].call_async(request)
        wait(future.done)
        return future.result()

    def request(owner='nav',source='AUTONOMY',duration=1000):
        return AcquireControl.Request(owner=owner,source=source,duration_ms=duration)

    def acquire(owner='nav',source='AUTONOMY',duration=1000):
        response=call('acquire',request(owner,source,duration))
        assert response.granted, response.reason
        tokens.append(response.lease_id)
        return response.lease_id

    def reset():
        assert call('set_stop',SetBool.Request(data=True)).success
        assert call('set_stop',SetBool.Request(data=False)).success

    def send(token,seq=1,speed=.15,age=0.,ttl=250):
        message=LeasedTwist(lease_id=token,sequence=seq,valid_for_ms=ttl)
        stamp=time.time_ns()-int(age*1e9)
        message.issued_at.sec=stamp//10**9
        message.issued_at.nanosec=stamp%10**9
        message.twist.linear.x=speed
        sent=time.monotonic()
        pub.publish(message)
        return sent

    def moving():
        return bool(outputs) and abs(outputs[-1]['x']-.15)<1e-6

    def zero():
        return bool(outputs) and outputs[-1]['x']==0. and outputs[-1]['z']==0.

    try:
        process=launch()
        for client in clients.values():
            deadline=time.monotonic()+8
            while not client.wait_for_service(timeout_sec=.2):
                if process.poll() is not None:
                    raise RuntimeError('Supervisor startup failed; inspect supervisor.log')
                assert time.monotonic()<deadline, 'Supervisor service unavailable'
        wait(lambda:outputs and pub.get_subscription_count()==1)
        assert zero()
        report['checks']['startup_zero_without_owner']=True
        # A frozen ROS clock must not freeze the wall-time command watchdog.
        frozen=Clock()
        frozen.clock.sec=10
        for _ in range(3):
            clock.publish(frozen)
            spin(.03)
        wait(lambda:outputs[-1]['ros_stamp']==10.)
        raw_message=TwistStamped()
        raw_message.twist.linear.x=.4
        raw.publish(raw_message)
        spin(.08)
        assert zero() and node.count_subscribers('/cmd_vel')==0
        report['checks']['raw_cmd_vel_cannot_drive']=True

        first=clients['acquire'].call_async(request())
        second=clients['acquire'].call_async(request('operator','TELEOP'))
        wait(lambda:first.done() and second.done())
        winners=[f.result() for f in [first,second] if f.result().granted]
        losers=[f.result() for f in [first,second] if not f.result().granted]
        assert len(winners)==1 and losers[0].reason=='BUSY'
        token=winners[0].lease_id
        tokens.append(token)
        command_sent=send(token)
        wait(moving)
        assert len(node.get_publishers_info_by_topic('/motion_supervisor/drive'))==1
        report['checks']['concurrent_acquire_has_one_owner']=True
        send('invalid-token',seq=2,speed=.4)
        spin(.04)
        assert moving()
        report['checks']['wrong_session_cannot_replace_command']=True
        wait(lambda:zero() and statuses[-1]['reason']=='COMMAND_TIMEOUT')
        report['metrics']['watchdog_zero_after_command_seconds']=outputs[-1]['received']-command_sent
        assert report['metrics']['watchdog_zero_after_command_seconds']<.55
        assert outputs[-1]['ros_stamp']==10.
        report['checks']['watchdog_zero_and_revoke_with_frozen_sim_clock']=True
        send(token,seq=2)
        spin(.08)
        assert zero() and statuses[-1]['state']=='IDLE'
        report['checks']['expired_session_cannot_resume']=True

        token=acquire()
        send(token,age=1.)
        spin(.04)
        assert zero()
        send(token,seq=2)
        wait(moving)
        send(token,seq=1,speed=.4)
        spin(.03)
        assert moving()
        assert call('release',ReleaseControl.Request(lease_id=token)).success
        wait(zero)
        manual=acquire('operator','TELEOP')
        send(token,seq=3,speed=.4)
        spin(.03)
        assert zero()
        send(manual)
        wait(moving)
        report['checks']['delayed_and_reordered_packets_rejected']=True
        report['checks']['release_handover_needs_new_token_and_command']=True

        assert call('set_stop',SetBool.Request(data=True)).success
        wait(lambda:zero() and statuses[-1]['state']=='STOPPED')
        assert call('set_stop',SetBool.Request(data=True)).success
        denied=call('acquire',request())
        assert not denied.granted and denied.reason=='STOPPED'
        assert call('set_stop',SetBool.Request(data=False)).success
        send(manual,seq=2)
        spin(.04)
        assert zero()
        report['checks']['latched_stop_clear_does_not_resume']=True

        token=acquire(duration=120)
        send(token)
        wait(moving)
        wait(lambda:zero() and statuses[-1]['reason']=='LEASE_EXPIRED')
        report['checks']['lease_expiry_stops_even_with_valid_command']=True
        token=acquire()
        send(token,ttl=150)
        wait(moving)
        assert call('renew',RenewControl.Request(lease_id=token,duration_ms=1000)).success
        wait(lambda:zero() and statuses[-1]['reason']=='COMMAND_TIMEOUT')
        report['checks']['renewal_cannot_substitute_for_commands']=True

        token=acquire()
        send(token)
        wait(moving)
        send(token,seq=2,speed=float('nan'))
        wait(lambda:zero() and statuses[-1]['reason']=='INVALID_COMMAND')
        report['checks']['invalid_owned_velocity_revokes_and_zeros']=True

        token=acquire()
        send(token)
        wait(moving)
        stop_owned(process)
        before_restart=len(outputs)
        process=launch()
        wait(lambda:len(outputs)>before_restart and zero() and statuses[-1]['state']=='IDLE',timeout=8.)
        send(token,seq=2)
        spin(.08)
        assert zero()
        report['checks']['process_restart_rejects_previous_lease']=True
        assert not any(token in json.dumps(statuses) for token in tokens)
        report['checks']['status_omits_lease_capabilities']=True
        report['metrics']['recorded_drive_messages']=len(outputs)
        report['metrics']['frozen_ros_time_seconds']=10.
        report['all_passed']=True
    except Exception as exc:
        report['all_passed']=False
        # Do not dump request/response objects or lease capability strings.
        report['error_type']=type(exc).__name__
        raise
    finally:
        stop_owned(process)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        (args.out/'report.json').write_text(json.dumps(report,indent=2))
        (args.out/'drive_samples.json').write_text(json.dumps(outputs,indent=2))
        (args.out/'status_samples.json').write_text(json.dumps(statuses,indent=2))
        print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    main()
