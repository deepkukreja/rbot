#!/usr/bin/env python3
"""One bounded isolated Gazebo/Nav2 loop, with stopped-only initial localization."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

SCENARIOS={
    'nominal':{'spawn':[1.,1.,0.],'prior':[0.,0.,0.]},
    'translated':{'spawn':[1.2,.85,0.],'prior':[.2,-.15,0.]},
    'rotated':{'spawn':[1.,1.,math.radians(20)],'prior':[0.,0.,math.radians(20)]},
    'biased_prior':{'spawn':[1.15,1.1,math.radians(10)],'prior':[.1,.05,math.radians(7)]},
}
SCENARIOS.update({
    'corridor_a':{'spawn':[7.7,-2.,math.pi/2],'prior':[6.7,-3.,math.pi/2],
                  'route':[(6.7,1.,math.pi/2)],'corridor':{'axis':'x','lower':6.1,'upper':7.3}},
    'corridor_b':{'spawn':[-2.,-7.7,0.],'prior':[-3.,-8.7,0.],
                  'route':[(1.,-8.7,0.)],'corridor':{'axis':'y','lower':-9.3,'upper':-8.1}},
})
# Alternate goals configure the observer only. The actual initial path must
# satisfy the independent appearance intersection check; scoring is unchanged.
OBSTACLE_VIEWS={
    'straight':(4.,0.,0.),
    'northeast':(4.,.6,math.pi/4),
    'southeast':(4.,-.6,-math.pi/4),
    'turned':(4.,0.,math.pi/2),
}
ROUTE=[(2.,0.,0.),(2.,-2.,-math.pi/2),(0.,-2.,math.pi),(0.,0.,math.pi/2),(0.,0.,0.)]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--scenario',choices=list(SCENARIOS),required=True)
    parser.add_argument('--pause-controller',action='store_true',help='Pause only the owned controller for 0.30 s during motion to test command expiry')
    parser.add_argument('--record-control-bag',action='store_true',help='Start the token-free 14-topic control bag before the first goal')
    parser.add_argument('--trace-occlusion-receiver',action='store_true',help='Capture optional native adapter receive/timer diagnostics')
    parser.add_argument('--record-control-trace',action='store_true',help='Record token-free controller/envelope timing and full supervisor reasons')
    parser.add_argument('--loopback-udp',action='store_true',help='Use the explicit local UDP profile for this observer and all fixture children')
    parser.add_argument('--scan-fault',choices=['drop','frozen'],help='Inject an isolated public scan transport fault after motion begins')
    parser.add_argument('--dynamic-obstacle',action='store_true',help='Insert the known trial box during an isolated open-space goal')
    parser.add_argument('--staged-obstacle-view',action='store_true',help='Reach the original appearance goal before an explicit alternate-view goal with the box still present')
    parser.add_argument('--obstacle-view',choices=list(OBSTACLE_VIEWS),default='straight',help='Select the dynamic-box arrival position/heading; existing limits remain unchanged')
    parser.add_argument('--scan-occlusion',action='store_true',help='Enable the bounded global scan-occlusion profile')
    parser.add_argument('--remove-dynamic-obstacle',action='store_true',help='After arrival, remove the box and require observed costmap clearing')
    parser.add_argument('--pause-occlusion',action='store_true',help='Pause only the owned derived-source process while a nominal goal is moving')
    parser.add_argument('--restart-occlusion',action='store_true',help='Kill only the owned source, then explicitly restart it after abort/rejection checks')
    parser.add_argument('--reenter-cleared-space',action='store_true',help='After observed removal and stale-pose idle, explicitly request a goal inside the former box')
    parser.add_argument('--record-clearing',action='store_true',help='Record full raw scans, hypotheses, TF and costmaps after arrival and through removal')
    args=parser.parse_args()
    if args.trace_occlusion_receiver and not args.scan_occlusion:parser.error('Receiver diagnostics require --scan-occlusion')
    if args.record_control_bag and (not args.record_control_trace or not args.scan_occlusion or args.record_clearing):parser.error('Early control bag requires trace and occlusion, and replaces --record-clearing')
    if args.pause_controller and (not args.record_control_trace or args.scenario!='nominal' or args.dynamic_obstacle or args.scan_fault or args.pause_occlusion or args.restart_occlusion):parser.error('Controller pause requires a traced nominal run without other faults')
    if args.record_clearing and not (args.dynamic_obstacle and args.remove_dynamic_obstacle and args.scan_occlusion):parser.error('Clearing capture requires appearance/removal with scan occlusion')
    if args.reenter_cleared_space and not (args.remove_dynamic_obstacle and args.dynamic_obstacle and args.scan_occlusion):parser.error('Reentry requires dynamic appearance/removal with scan occlusion')
    if args.restart_occlusion:args.pause_occlusion=True
    if args.pause_occlusion and (not args.scan_occlusion or args.scenario!='nominal' or args.scan_fault or args.dynamic_obstacle):parser.error('Occlusion pause requires nominal with --scan-occlusion and no other fault')
    if args.remove_dynamic_obstacle and not args.dynamic_obstacle:parser.error('Removal requires the dynamic-obstacle fixture')
    if args.scan_fault and args.scenario!='nominal':parser.error('Scan faults use the nominal open-space route')
    if args.dynamic_obstacle and (args.scenario not in {'nominal','translated','rotated','biased_prior'} or args.scan_fault):parser.error('Dynamic obstacle mode requires an open-space scenario without a scan fault')
    if args.obstacle_view!='straight' and not args.dynamic_obstacle:parser.error('Alternate obstacle views require --dynamic-obstacle')
    if args.staged_obstacle_view and (not args.dynamic_obstacle or args.obstacle_view=='straight'):parser.error('Staged view requires a dynamic box and alternate arrival view')
    config=SCENARIOS[args.scenario]
    chosen_route=[OBSTACLE_VIEWS[args.obstacle_view]] if args.dynamic_obstacle else [(4.,0.,0.)] if args.pause_occlusion else [(2.,0.,0.)] if args.scan_fault or args.pause_controller else config.get('route',ROUTE)
    if args.staged_obstacle_view:chosen_route=[OBSTACLE_VIEWS['straight'],OBSTACLE_VIEWS[args.obstacle_view]]
    assert args.map.is_file() and 1<=args.domain<=101
    args.out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    if args.loopback_udp:
        from rlai_localization.loopback_udp import bundled_profile,loopback_environment
        os.environ.update(loopback_environment(bundled_profile(),os.environ))
    else:
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    os.environ.pop('ROS_LOCALHOST_ONLY',None)
    os.environ['GZ_PARTITION']='buildworks_route_'+str(args.domain)+'_'+args.scenario
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped,PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry,OccupancyGrid,Path as NavPath
    from sensor_msgs.msg import LaserScan,PointCloud2
    from nav2_msgs.action import NavigateToPose
    from std_srvs.srv import SetBool
    from rlai_motion_interfaces.msg import SupervisorStatus,NavigationStatus
    from rlai_motion_supervisor.localization_reset import initial_pose_message,reset_localization
    from rlai_motion_supervisor.preflight import prepare_localization
    from rlai_motion_supervisor.navigation_metrics import interpolate_truth,score_goal
    from rlai_motion_supervisor.owned_processes import snapshot_owned,finish_owned_cleanup
    from rlai_motion_supervisor.scenario_metrics import footprint_clearance,settled_latency,conservative_box_clearance

    rclpy.init(args=[])
    node=rclpy.create_node('guarded_route_benchmark',parameter_overrides=[Parameter('use_sim_time',value=True)])
    children,commands,logs=[],[],[]
    health,nav,truth,poses,drive,wheel=[],[],[],[],[],[]
    scans=[]
    raw_scans=[]
    forwarded_scans=[]
    relay={'mode':'forward','last':None}
    scan_output=node.create_publisher(LaserScan,'scan',qos_profile_sensor_data) if args.scan_fault else None
    stage='startup'
    occlusion_receive_trace=[]
    if args.trace_occlusion_receiver:
        import json as receiver_json
        from std_msgs.msg import String
        def receive_trace(message):
            value=receiver_json.loads(message.data)
            value['observer_received_steady']=time.monotonic()
            occlusion_receive_trace.append(value)
        node.create_subscription(String,'nav2_adapter/occlusion_trace',receive_trace,qos_profile_sensor_data)
    control_trace=[]
    if args.record_control_trace:
        from rlai_motion_interfaces.msg import LeasedTwist
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        trace_qos=QoSProfile(depth=50,reliability=ReliabilityPolicy.BEST_EFFORT)
        def trace_twist(message,kind):
            t=message.twist
            row={'kind':kind,'at':time.monotonic(),'wall':time.time(),'stage':stage,
                'twist':[v if math.isfinite(v) else repr(v) for v in
                    (t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)]}
            if kind=='raw':
                row['stamp']=message.header.stamp.sec+message.header.stamp.nanosec/1e9
                row['ros_now']=node.get_clock().now().nanoseconds/1e9
            else:
                # Never put the lease capability in JSON, bags, logs or status.
                row.update(sequence=message.sequence,issued_wall=message.issued_at.sec+message.issued_at.nanosec/1e9,
                    valid_for_s=message.valid_for_ms/1000.)
            control_trace.append(row)
        node.create_subscription(TwistStamped,'nav2_adapter/cmd_vel_input',lambda m:trace_twist(m,'raw'),trace_qos)
        node.create_subscription(LeasedTwist,'motion_supervisor/command',lambda m:trace_twist(m,'leased'),trace_qos)
    plans,static_roi,costmap_roi,obstacle_poses=[],[],[],[]
    occlusion_clouds,shadow_roi,deletions=[],[],[]
    gazebo_node=None
    paused={'reference':None}
    def stamp(m):
        return m.header.stamp.sec+m.header.stamp.nanosec/1e9
    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def truth_message(m):
        value={'at':time.monotonic(),'stamp':stamp(m),'x':m.pose.pose.position.x-1.,'y':m.pose.pose.position.y-1.,
            'yaw':yaw(m.pose.pose.orientation),'stage':stage}
        if not truth or value['stamp']>truth[-1]['stamp']:
            truth.append(value)
    def pose_message(m):
        poses.append({'at':time.monotonic(),'stamp':stamp(m),'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,
            'yaw':yaw(m.pose.pose.orientation),'covariance':list(m.pose.covariance),'stage':stage})
    node.create_subscription(SupervisorStatus,'motion_supervisor/status',lambda m:health.append({
        'at':time.monotonic(),'wall':time.time(),'state':m.state,'owner':m.owner,'inhibited':m.inhibited,'reason':m.health_reason,'stage':stage,
        'control_reason':m.reason,'source':m.source,'lease_remaining_ms':m.lease_remaining_ms,'accepted_sequence':m.accepted_sequence,'has_command':m.has_command}),10)
    node.create_subscription(NavigationStatus,'nav2_adapter/status',lambda m:nav.append({
        'at':time.monotonic(),'wall':time.time(),'phase':m.phase,'reason':m.reason,'quarantined':m.quarantined,'forwarding':m.forwarding,'goal_id':m.goal_id,'stage':stage}),10)
    node.create_subscription(Odometry,'gazebo/ground_truth/odom',truth_message,qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped,'amcl_pose',pose_message,qos_profile_sensor_data)
    node.create_subscription(Odometry,'diff_drive_controller/odom',lambda m:wheel.append({
        'at':time.monotonic(),'stamp':stamp(m),'speed':m.twist.twist.linear.x,'turn':m.twist.twist.angular.z}),qos_profile_sensor_data)
    node.create_subscription(TwistStamped,'diff_drive_controller/cmd_vel',lambda m:drive.append({
        'at':time.monotonic(),'wall':time.time(),'stage':stage,'x':m.twist.linear.x,'z':m.twist.angular.z}),10)
    def scan_message(m):
        finite=[v for v in m.ranges if math.isfinite(v)]
        scans.append({'at':time.monotonic(),'stamp':stamp(m),'stage':stage,'frame':m.header.frame_id,
            'count':len(m.ranges),'range_min':m.range_min,'range_max':m.range_max,
            'angle_increment':m.angle_increment,'finite_count':len(finite),
            'below_min':sum(v<m.range_min for v in finite),'above_max':sum(v>m.range_max for v in finite),
            'nan_count':sum(math.isnan(v) for v in m.ranges),'negative_infinity_count':sum(v==-math.inf for v in m.ranges),
            'min_finite':min(finite) if finite else None,'max_finite':max(finite) if finite else None})
    node.create_subscription(LaserScan,'scan',scan_message,qos_profile_sensor_data)
    if args.scan_fault:
        def raw_scan_message(message):
            raw_scans.append({'at':time.monotonic(),'stamp':stamp(message),'mode':relay['mode']})
            if relay['mode']=='forward':
                relay['last']=message
                outgoing=message
            else:
                outgoing=relay['last'] if relay['mode']=='frozen' else None
            if outgoing is not None:
                scan_output.publish(outgoing)
                forwarded_scans.append({'at':time.monotonic(),'stamp':stamp(outgoing),'mode':relay['mode']})
        node.create_subscription(LaserScan,'scan_raw',raw_scan_message,qos_profile_sensor_data)
    if args.scan_occlusion:
        def cloud_message(message):
            occlusion_clouds.append({'at':time.monotonic(),'stamp':stamp(message),'frame':message.header.frame_id,'points':message.width*message.height})
            if report.get('obstacle',{}).get('first_observed_pose') and stamp(message)>=report['obstacle']['first_observed_pose']['stamp'] and not (args.out/'first_occlusion_cloud.bin').exists():
                (args.out/'first_occlusion_cloud.bin').write_bytes(bytes(message.data))
                (args.out/'first_occlusion_cloud.json').write_text(json.dumps({'stamp':stamp(message),'frame':message.header.frame_id,'width':message.width,'height':message.height,'point_step':message.point_step,'is_bigendian':message.is_bigendian,'fields':[{'name':f.name,'offset':f.offset,'datatype':f.datatype,'count':f.count} for f in message.fields]},indent=2))
        node.create_subscription(PointCloud2,'scan_occlusion/points',cloud_message,qos_profile_sensor_data)
    if args.dynamic_obstacle:
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
        from gz.transport13 import Node as GazeboNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.uint32_v_pb2 import UInt32_V
        gazebo_node=GazeboNode()
        def obstacle_pose_message(message):
            for pose in message.pose:
                if pose.name=='guarded_trial_box':
                    obstacle_poses.append({'at':time.monotonic(),'stamp':message.header.stamp.sec+message.header.stamp.nsec/1e9,
                        'x':pose.position.x-1.,'y':pose.position.y-1.,'z':pose.position.z,'yaw':yaw(pose.orientation),'id':int(pose.id)})
        assert gazebo_node.subscribe(Pose_V,'/world/small_warehouse/pose/info',obstacle_pose_message)
        assert gazebo_node.subscribe(UInt32_V,'/world/small_warehouse/scene/deletion',lambda m:deletions.append({'at':time.monotonic(),'ids':list(m.data)}))
        def roi_message(message,destination,bounds=(1.6,2.4,-.4,.4)):
            if message.header.frame_id!='map' or abs(yaw(message.info.origin.orientation))>1e-8:
                raise ValueError('Trial ROI requires an axis-aligned map-frame grid')
            r=message.info.resolution;origin=message.info.origin.position
            ix0=math.floor((bounds[0]-origin.x)/r);ix1=math.floor((bounds[1]-origin.x)/r)
            iy0=math.floor((bounds[2]-origin.y)/r);iy1=math.floor((bounds[3]-origin.y)/r)
            assert 0<=ix0<=ix1<message.info.width and 0<=iy0<=iy1<message.info.height
            data=[int(message.data[iy*message.info.width+ix]) for iy in range(iy0,iy1+1) for ix in range(ix0,ix1+1)]
            destination.append({'at':time.monotonic(),'stamp':stamp(message),'frame':message.header.frame_id,
                'cells':len(data),'occupied':sum(v>=99 for v in data),'unknown':sum(v<0 for v in data),'data':data,
                'resolution':r,'origin_xy':[origin.x,origin.y],'indices_xyxy':[ix0,iy0,ix1,iy1]})
        qos=QoSProfile(depth=2,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid,'map',lambda m:roi_message(m,static_roi),qos)
        node.create_subscription(OccupancyGrid,'global_costmap/costmap',lambda m:(roi_message(m,costmap_roi),roi_message(m,shadow_roi,(.5,3.8,-1.6,1.1))),qos)
        node.create_subscription(NavPath,'plan',lambda m:plans.append({'at':time.monotonic(),'stamp':stamp(m),'frame':m.header.frame_id,
            'poses':[{'x':p.pose.position.x,'y':p.pose.position.y,'yaw':yaw(p.pose.orientation)} for p in m.poses]}),10)
    action=ActionClient(node,NavigateToPose,'navigate_to_pose')
    stop_client=node.create_client(SetBool,'motion_supervisor/set_stop')
    report={'scenario':args.scenario,'configuration':SCENARIOS[args.scenario],'domain':args.domain,
        'map':str(args.map),'goals':[],'checks':{},'route':chosen_route,'scan_fault':args.scan_fault,'pause_controller':args.pause_controller,'dynamic_obstacle':args.dynamic_obstacle,'scan_occlusion':args.scan_occlusion,'pause_occlusion':args.pause_occlusion,'restart_occlusion':args.restart_occlusion,'remove_dynamic_obstacle':args.remove_dynamic_obstacle,'obstacle_view':args.obstacle_view if args.dynamic_obstacle else None,'staged_obstacle_view':args.staged_obstacle_view,'reenter_cleared_space':args.reenter_cleared_space,'scope':'Isolated Gazebo physics and real Nav2/AMCL; no physical robot.',
        'fixture_source_sha256':__import__('hashlib').sha256(Path(__file__).read_bytes()).hexdigest(),
        'loopback_udp':args.loopback_udp,'discovery_range':os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'],'dds_profile':os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE'),
        'criteria':{'goal_truth_xy_m':.35,'goal_truth_yaw_rad':.2,'route_max_localization_error_m':.20,
            'max_truth_pair_gap_s':.1},'automatic_goal_retries':0,'mid_route_localization_resets':0}
    if args.scan_fault or args.pause_controller:
        report['criteria'].update(zero_output_deadline_s=.85,wheel_standstill_deadline_s=1.5,restored_hold_s=2.)
    if 'corridor' in config or args.dynamic_obstacle:
        import ast,yaml
        from ament_index_python.packages import get_package_share_directory
        params=Path(get_package_share_directory('rlai_navigation'))/'config/nav2_params.yaml'
        data=yaml.safe_load(params.read_text())
        dimensions=[]
        for name in ['global_costmap','local_costmap']:
            values=data[name][name]['ros__parameters']
            points=ast.literal_eval(values['footprint'])
            xs=sorted(set(p[0] for p in points));ys=sorted(set(p[1] for p in points))
            assert len(points)==4 and len(xs)==len(ys)==2 and xs[0]==-xs[1] and ys[0]==-ys[1]
            dimensions.append((xs[1]+values['footprint_padding'],ys[1]+values['footprint_padding']))
        assert dimensions[0]==dimensions[1]
        report['padded_footprint_half_dimensions_m']=dimensions[0]
        report['navigation_parameters_sha256']=__import__('hashlib').sha256(params.read_bytes()).hexdigest()
        if 'corridor' in config:report['criteria']['min_sampled_padded_sidewall_clearance_m']=.10
    if args.dynamic_obstacle:
        report['criteria'].update(min_conservative_padded_box_clearance_m=.10,max_costmap_mark_delay_sim_s=3.,max_safe_replan_delay_sim_s=5.)
    if args.pause_occlusion:report['criteria'].update(zero_output_deadline_s=.85,wheel_standstill_deadline_s=1.5,restored_hold_s=2.)
    handle=None
    started=time.monotonic()

    def launch(label,args):
        log=(out/(label+'.log')).open('w')
        logs.append(log)
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy(),start_new_session=True)
        children.append(p)
    def wait(predicate,seconds=10.):
        end=min(time.monotonic()+seconds,started+450.)
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if args.dynamic_obstacle and stage=='route' and report.get('obstacle') and truth:
                if obstacle_clearance(truth[-1])<.10:raise AssertionError('Dynamic obstacle clearance fell below the declared 0.10 m bound')
            if predicate():
                return
            if any(p.poll() is not None for p in children):
                raise RuntimeError('Owned fixture child exited')
        raise TimeoutError('Scenario condition did not complete within its bound')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
    def stationary():
        return len(wheel)>=5 and time.monotonic()-wheel[-1]['at']<.3 and all(abs(p['speed'])<.005 and abs(p['turn'])<.02 for p in wheel[-5:])
    def set_stop(value):
        request=SetBool.Request()
        request.data=value
        pending=stop_client.call_async(request)
        wait(pending.done)
        assert pending.result().success
    def idle():
        return health and nav and not health[-1]['owner'] and nav[-1]['phase']=='IDLE' and not nav[-1]['quarantined']
    def reset_inputs_ready():
        now=time.monotonic()
        recent=[h for h in health if now-h['at']<=.5]
        return (len(recent)>=5 and recent[-1]['at']-recent[0]['at']>=.35 and now-recent[-1]['at']<.3 and
            all(not h['owner'] and h['reason'] in {'','AMCL_STALE','AMCL_INVALID','AMCL_TIMESTAMP'} for h in recent))
    def exercise_scan_fault(pending,event):
        nonlocal stage
        wait(lambda:truth and truth[-1]['x']>.25 and wheel and wheel[-1]['speed']>.05 and drive and drive[-1]['x']>.05 and nav and nav[-1]['forwarding'] and health and not health[-1]['inhibited'] and relay['last'] is not None and not pending.done(),20.)
        stage='fault'
        onset=time.monotonic()
        last=forwarded_scans[-1].copy()
        start_truth=truth[-1].copy()
        start_clock=node.get_clock().now().nanoseconds/1e9
        raw_count=len(raw_scans);wheel_count=len(wheel)
        relay['mode']=args.scan_fault
        fault={'onset':onset,'last_forwarded':last,'truth_at_onset':start_truth,'mode':args.scan_fault}
        report['fault']=fault
        spin(1.6)
        fault_end=time.monotonic()
        fault['restored_at']=fault_end
        fault['clock_advanced_during_outage_s']=node.get_clock().now().nanoseconds/1e9-start_clock
        fault['raw_packets_during_outage']=len(raw_scans)-raw_count
        fault['wheel_packets_during_outage']=len(wheel)-wheel_count
        relay['mode']='forward'
        wait(pending.done,6.)
        value=pending.result()
        event.update(status=value.status,error_code=value.result.error_code,reason=value.result.error_msg)
        fault['action_terminal_status']=value.status
        wait(lambda:health and not health[-1]['inhibited'] and stationary() and idle(),6.)
        held=time.monotonic()
        spin(2.)
        until=time.monotonic()
        fault.update(hold_started=held,hold_ended=until)
        latency=settled_latency(drive,onset=onset,until=until)
        wheel_values=[{'at':p['at'],'x':p['speed'],'z':p['turn']} for p in wheel]
        wheel_latency=settled_latency(wheel_values,onset=onset,until=until,linear_tolerance=.005,angular_tolerance=.02)
        fault.update(zero_output_latency_s=latency,wheel_standstill_latency_s=wheel_latency)
        stopped=[start_truth]+[p for p in truth if onset<=p['at']<=onset+(wheel_latency or 0.)]
        fault['sampled_stopping_path_m']=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(stopped,stopped[1:]))
        rejected=[h for h in health if onset<=h['at']<fault_end and h['inhibited']]
        fault['first_inhibit']=rejected[0] if rejected else None
        fault['forwarded_packets_during_outage']=[p for p in forwarded_scans if onset<=p['at']<fault_end]
        held_health=[h for h in health if held<=h['at']<=until]
        held_nav=[n for n in nav if held<=n['at']<=until]
        report['checks'].update({
            'raw_scan_clock_and_odometry_continued':fault['raw_packets_during_outage']>=3 and fault['wheel_packets_during_outage']>=5 and fault['clock_advanced_during_outage_s']>.1,
            'stale_scan_caused_health_inhibition':bool(rejected) and rejected[0]['reason']=='SCAN_STALE' and rejected[0]['state']=='INHIBITED',
            'zero_output_within_declared_deadline':latency is not None and latency<=.85,
            'wheel_standstill_within_declared_deadline':wheel_latency is not None and wheel_latency<=1.5,
            'goal_terminated_without_success':value.status in (5,6),
            'restored_input_did_not_reacquire_or_resume':len(held_health)>=10 and all(not h['owner'] for h in held_health) and len(held_nav)>=10 and all(not n['forwarding'] and not n['goal_id'] and not n['quarantined'] and n['phase']=='IDLE' for n in held_nav),
            'drop_or_frozen_transport_observed':not fault['forwarded_packets_during_outage'] if args.scan_fault=='drop' else len(fault['forwarded_packets_during_outage'])>=3 and all(p['stamp']==last['stamp'] for p in fault['forwarded_packets_during_outage']),
        })
        if not all(report['checks'].values()):raise AssertionError('Sensor-fault scenario failed its declared criteria')

    def signal_source(reference,value):
        current=next((p for p in snapshot_owned([reference['pid']]) if p['pid']==reference['pid']),None)
        assert current and current['start_ticks']==reference['start_ticks'] and current['uid']==reference['uid']==os.getuid(),'Owned source identity changed'
        environment=Path('/proc',str(reference['pid']),'environ').read_bytes().split(b'\0')
        assert ('ROS_DOMAIN_ID='+str(args.domain)).encode() in environment and ('GZ_PARTITION='+os.environ['GZ_PARTITION']).encode() in environment
        os.kill(reference['pid'],value)
    def resume_source():
        if paused['reference'] is not None:
            signal_source(paused['reference'],signal.SIGCONT)
            paused['reference']=None
    def exercise_controller_pause(pending,event):
        nonlocal stage
        wait(lambda:truth and truth[-1]['x']>.25 and wheel and wheel[-1]['speed']>.05 and drive and drive[-1]['x']>.05
            and nav and nav[-1]['forwarding'] and health and not health[-1]['inhibited'] and health[-1]['lease_remaining_ms']>1000
            and any(p['kind']=='leased' for p in control_trace) and not pending.done(),20.)
        matches=[]
        for reference in snapshot_owned([p.pid for p in children]):
            try:
                arguments=Path('/proc',str(reference['pid']),'cmdline').read_bytes().split(b'\0')
                if any(a.endswith(b'/nav2_controller/controller_server') for a in arguments):matches.append(reference)
            except OSError:pass
        assert len(matches)==1,'Expected one owned controller process'
        stage='controller_fault';onset=time.monotonic()
        fault={'onset':onset,'wall_at_onset':time.time(),'process_identity':matches[0],
            'clock_at_onset':node.get_clock().now().nanoseconds/1e9,'status_before_pause':health[-1],
            'requested_pause_wall_s':.30,'last_leased_before_pause':next(p for p in reversed(control_trace) if p['kind']=='leased')}
        report['controller_fault']=fault
        paused['reference']=matches[0];signal_source(matches[0],signal.SIGSTOP)
        try:spin(.30)
        finally:resume_source()
        fault['resumed_at']=time.monotonic();fault['actual_pause_wall_s']=fault['resumed_at']-onset
        wait(pending.done,6.)
        value=pending.result();event.update(status=value.status,error_code=value.result.error_code,reason=value.result.error_msg)
        wait(lambda:idle() and stationary(),6.)
        fault['hold_started']=time.monotonic();spin(2.);fault['hold_ended']=time.monotonic()
        rows=[p for p in health if onset<=p['at']<=fault['hold_ended']]
        expired=[p for p in rows if p['control_reason']=='COMMAND_TIMEOUT' and not p['owner']]
        fault['first_expiry_status']=expired[0] if expired else None
        fault['zero_output_latency_s']=settled_latency(drive,onset=onset,until=fault['hold_ended'])
        fault['wheel_standstill_latency_s']=settled_latency([{'at':p['at'],'x':p['speed'],'z':p['turn']} for p in wheel],
            onset=onset,until=fault['hold_ended'],linear_tolerance=.005,angular_tolerance=.02)
        held_nav=[p for p in nav if fault['hold_started']<=p['at']<=fault['hold_ended']]
        held_health=[p for p in health if fault['hold_started']<=p['at']<=fault['hold_ended']]
        initial=[p for p in rows if p['at']<=fault['resumed_at']+.2]
        raw=[p for p in scans if onset<=p['at']<=fault['resumed_at']+.2]
        report['checks'].update(controller_pause_command_expiry_observed=bool(expired),
            controller_pause_health_and_scan_continued=len(initial)>=5 and all(not p['inhibited'] for p in initial) and len(raw)>=2,
            controller_pause_aborted_goal=value.status==6 and value.result.error_code==60001,
            controller_pause_zero_output_within_deadline=fault['zero_output_latency_s'] is not None and fault['zero_output_latency_s']<=.85,
            controller_pause_wheels_stopped_within_deadline=fault['wheel_standstill_latency_s'] is not None and fault['wheel_standstill_latency_s']<=1.5,
            controller_resume_did_not_resume_motion=len(held_nav)>=10 and all(p['phase']=='IDLE' and not p['forwarding'] and not p['goal_id'] for p in held_nav)
                and len(held_health)>=10 and all(not p['owner'] for p in held_health))
        if not all(report['checks'].values()):raise AssertionError('Controller pause failed command-expiry contract')

    def exercise_occlusion_pause(pending,event):
        nonlocal stage
        wait(lambda:truth and truth[-1]['x']>.25 and wheel and wheel[-1]['speed']>.05 and drive and drive[-1]['x']>.05 and nav and nav[-1]['forwarding'] and health and not health[-1]['inhibited'] and not pending.done(),20.)
        matches=[]
        for reference in snapshot_owned([p.pid for p in children]):
            try:
                arguments=Path('/proc',str(reference['pid']),'cmdline').read_bytes().split(b'\0')
                if any(a.endswith(b'/rlai_navigation/scan_occlusion') for a in arguments):matches.append(reference)
            except OSError:pass
        assert len(matches)==1,'Expected one owned scan-occlusion process'
        stage='fault';onset=time.monotonic()
        fault={'onset':onset,'process_identity':matches[0],'mode':'SIGKILL/EXPLICIT_RESTART' if args.restart_occlusion else 'SIGSTOP/SIGCONT','truth_at_onset':truth[-1].copy(),
            'clock_at_onset':node.get_clock().now().nanoseconds/1e9}
        report['occlusion_fault']=fault
        paused['reference']=None if args.restart_occlusion else matches[0]
        signal_source(matches[0],signal.SIGKILL if args.restart_occlusion else signal.SIGSTOP)
        try:
            spin(2.)
            observed=time.monotonic();fault['initial_stop_window_ended']=observed
            latency=settled_latency(drive,onset=onset,until=observed)
            velocities=[{'at':p['at'],'x':p['speed'],'z':p['turn']} for p in wheel]
            wheel_latency=settled_latency(velocities,onset=onset,until=observed,linear_tolerance=.005,angular_tolerance=.02)
            fault.update(zero_output_latency_s=latency,wheel_standstill_latency_s=wheel_latency,
                clock_advanced_while_paused=node.get_clock().now().nanoseconds/1e9-fault['clock_at_onset'])
            raw=[p for p in scans if onset<=p['at']<=observed]
            state=[p for p in health if onset<=p['at']<=observed]
            late=[p for p in occlusion_clouds if onset+.25<=p['at']<=observed]
            report['checks'].update({
                'raw_scan_clock_and_health_continued':len(raw)>=3 and fault['clock_advanced_while_paused']>.1 and len(state)>=10 and all(not p['inhibited'] for p in state),
                'only_derived_observations_paused':not late,
                'derived_loss_zero_output_within_deadline':latency is not None and latency<=.85,
                'derived_loss_wheels_settled_within_deadline':wheel_latency is not None and wheel_latency<=1.5})
            if not all(report['checks'].values()):raise AssertionError('Derived source loss failed stopping criteria while raw inputs remained healthy')
            wait(pending.done,6.)
            value=pending.result();event.update(status=value.status,error_code=value.result.error_code,reason=value.result.error_msg)
            fault['action_terminal_status']=value.status
            wait(lambda:idle() and stationary(),6.)
            fault['idle_missing_source_reason']=nav[-1]['reason']
            request=NavigateToPose.Goal();request.pose.header.frame_id='map';request.pose.header.stamp=node.get_clock().now().to_msg()
            request.pose.pose.position.x=4.;request.pose.pose.orientation.w=1.
            probe=action.send_goal_async(request);wait(probe.done,3.)
            rejected=probe.result();fault['probe_goal_accepted']=rejected.accepted
            if rejected.accepted:
                rejected.cancel_goal_async()
                raise AssertionError('A new goal was admitted while the required source was paused')
            report['checks']['derived_source_fault_aborted_and_rejected_new_goal']=value.status==6 and value.result.error_msg.startswith('SCAN_OCCLUSION_') and nav[-1]['reason'].startswith('SCAN_OCCLUSION_')
        finally:
            if args.restart_occlusion:
                launch('occlusion_restart',['ros2','run','rlai_navigation','scan_occlusion','--ros-args','-p','use_sim_time:=true'])
            else:resume_source()
            fault['restored_at']=time.monotonic()
        wait(lambda:len([p for p in occlusion_clouds if p['at']>=fault['restored_at']])>=3 and idle() and stationary()
            and health[-1]['reason'] in {'','AMCL_STALE'} and nav[-1]['reason'] in {'READY','HEALTH_UNAVAILABLE'},8.)
        fault['restored_navigation_reason']=nav[-1]['reason']
        fault['restored_health_reason']=health[-1]['reason']
        fault['fresh_clouds_after_restoration']=len([p for p in occlusion_clouds if p['at']>=fault['restored_at']])
        held=time.monotonic();spin(2.);until=time.monotonic()
        fault.update(hold_started=held,hold_ended=until)
        kept=[p for p in nav if held<=p['at']<=until];healthy=[p for p in health if held<=p['at']<=until]
        report['checks']['resuming_derived_source_did_not_resume_goal']=len(kept)>=10 and all(p['phase']=='IDLE' and not p['goal_id'] and not p['forwarding'] and not p['quarantined'] for p in kept) and len(healthy)>=10 and all(not p['owner'] and p['reason'] in {'','AMCL_STALE'} for p in healthy)
        fault['held_health_reasons']=sorted({p['reason'] for p in healthy})
        fault['zero_output_latency_s']=settled_latency(drive,onset=onset,until=until)
        fault['wheel_standstill_latency_s']=settled_latency([{'at':p['at'],'x':p['speed'],'z':p['turn']} for p in wheel],onset=onset,until=until,linear_tolerance=.005,angular_tolerance=.02)
        if not all(report['checks'].values()):raise AssertionError('Required-source restoration failed its no-resume contract')

    def obstacle_clearance(pose):
        return conservative_box_clearance(pose,center=(2.,0.),size=(.8,.8),half_length=report['padded_footprint_half_dimensions_m'][0],half_width=report['padded_footprint_half_dimensions_m'][1])
    def insert_dynamic_obstacle(pending):
        wait(lambda:truth and truth[-1]['x']>.25 and wheel and wheel[-1]['speed']>.05 and drive and drive[-1]['x']>.05
            and nav and nav[-1]['forwarding'] and health and not health[-1]['inhibited'] and plans and static_roi and costmap_roi and not pending.done(),20.)
        assert not obstacle_poses,'Trial obstacle unexpectedly present before insertion'
        assert all(v==0 for v in static_roi[-1]['data']) and all(v==0 for v in costmap_roi[-1]['data']),'Obstacle ROI must start known-free'
        initial_plan=plans[-1]
        radius=math.hypot(*report['padded_footprint_half_dimensions_m'])
        assert initial_plan['frame']=='map' and initial_plan['poses'] and min(obstacle_clearance(p)+radius for p in initial_plan['poses'])<0.,'Initial plan did not intersect the future box'
        obstacle={'requested_at':time.monotonic(),'truth_at_request':truth[-1].copy(),'initial_plan_index':len(plans)-1,
            'world_center_xyz':[3.,1.,.5],'map_center_xy':[2.,0.],'size_xyz':[.8,.8,1.],'static':True,
            'static_roi_before':static_roi[-1],'global_roi_before':costmap_roi[-1]}
        report['obstacle']=obstacle
        sdf=out/'obstacle.sdf'
        sdf.write_text('''<sdf version="1.11"><model name="guarded_trial_box"><static>true</static><pose>3 1 0.5 0 0 0</pose><link name="box"><collision name="collision"><geometry><box><size>0.8 0.8 1</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.8 0.8 1</size></box></geometry><material><ambient>1 0.25 0.05 1</ambient><diffuse>1 0.25 0.05 1</diffuse></material></visual></link></model></sdf>''')
        log=(out/'obstacle_create.log').open('w');logs.append(log)
        command=['gz','service','-s','/world/small_warehouse/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','4000','--req','sdf_filename: '+json.dumps(str(sdf))+' allow_renaming: false']
        creator=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy(),start_new_session=True)
        commands.append(creator)
        wait(lambda:creator.poll() is not None,6.)
        response=(out/'obstacle_create.log').read_text()
        obstacle['create_response']={'exit_code':creator.returncode,'output':response}
        assert creator.returncode==0 and 'data: true' in response,'Obstacle create service did not acknowledge'
        wait(lambda:bool(obstacle_poses),5.)
        pose=obstacle_poses[0]
        assert abs(pose['x']-2.)<1e-6 and abs(pose['y'])<1e-6 and abs(pose['z']-.5)<1e-6 and abs(pose['yaw'])<1e-6,'Observed box pose differs from requested geometry'
        obstacle['first_observed_pose']=pose
        report['checks']['dynamic_obstacle_inserted_while_moving']=True
        report['checks']['initial_plan_crossed_known_free_obstacle_roi']=True
        print(json.dumps({'obstacle_created':True,'pose':pose}),flush=True)

    def score_dynamic_obstacle():
        obstacle=report['obstacle'];onset=obstacle['requested_at'];appearance=obstacle['first_observed_pose']['stamp']
        radius=math.hypot(*report['padded_footprint_half_dimensions_m'])
        detected=[p for p in costmap_roi if p['at']>=onset and p['stamp']>=appearance and p['occupied']>0]
        safe_plans=[(i,p) for i,p in enumerate(plans) if p['at']>=onset and p['stamp']>=appearance and p['frame']=='map' and p['poses'] and min(obstacle_clearance(q) for q in p['poses'])>=.10]
        trajectory=[p for p in truth if p['stage']=='route' and p['at']>=onset]
        clearance=[obstacle_clearance(p) for p in trajectory]
        gaps=[b['stamp']-a['stamp'] for a,b in zip(trajectory,trajectory[1:])]
        obstacle.update(first_marked_roi=detected[0] if detected else None,first_safe_plan_index=safe_plans[0][0] if safe_plans else None,
            costmap_mark_delay_sim_s=detected[0]['stamp']-appearance if detected else None,
            safe_replan_delay_sim_s=safe_plans[0][1]['stamp']-appearance if safe_plans else None,
            min_sampled_conservative_padded_clearance_m=min(clearance) if clearance else None,
            max_truth_gap_s=max(gaps,default=None),max_lateral_excursion_m=max((abs(p['y']) for p in trajectory),default=None))
        report['checks'].update({
            'saved_map_obstacle_roi_remained_free':bool(static_roi) and all(all(v==0 for v in p['data']) for p in static_roi),
            'new_obstacle_marked_in_live_global_costmap':bool(detected) and obstacle['costmap_mark_delay_sim_s']<=3.,
            'post_appearance_plan_clears_obstacle':bool(safe_plans) and obstacle['safe_replan_delay_sim_s']<=5.,
            'executed_padded_footprint_kept_declared_clearance':bool(clearance) and min(clearance)>=.10 and bool(gaps) and max(gaps)<=.1,
            'obstacle_pose_matches_declared_stationary_geometry':bool(obstacle_poses) and all(abs(p['x']-2.)<1e-6 and abs(p['y'])<1e-6 and abs(p['z']-.5)<1e-6 and abs(p['yaw'])<1e-6 for p in obstacle_poses),
        })
        if not all(report['checks'].values()):raise AssertionError('Dynamic obstacle scenario failed its declared criteria')

    def record_clearing(before_goal=False):
        nonlocal stage
        stage='control_capture' if before_goal else 'post_goal_capture'
        qos=out/'bag_qos.yaml'
        qos.write_text("/tf_static:\n  reliability: reliable\n  durability: transient_local\n  depth: 1\n/global_costmap/costmap:\n  reliability: reliable\n  durability: transient_local\n  depth: 2\n/global_costmap/costmap_raw:\n  reliability: reliable\n  durability: transient_local\n  depth: 2\n")
        topics=['/scan','/scan_occlusion/points','/tf','/tf_static','/clock','/global_costmap/costmap','/global_costmap/costmap_raw','/gazebo/ground_truth/odom']
        if args.record_control_trace:
            extras=['/nav2_adapter/cmd_vel_input','/motion_supervisor/status','/nav2_adapter/status','/diff_drive_controller/cmd_vel','/diff_drive_controller/odom','/amcl_pose']
            topics+=extras
            if args.trace_occlusion_receiver:topics.append('/nav2_adapter/occlusion_trace')
            import yaml
            overrides=yaml.safe_load(qos.read_text())
            for topic in extras:overrides[topic]={'reliability':'best_effort','durability':'volatile','depth':50}
            qos.write_text(yaml.safe_dump(overrides))
        launch('clearing_bag',['ros2','bag','record','--disable-keyboard-controls','--use-sim-time','--node-name','clearing_recorder',
            '-s','mcap','--storage-preset-profile','zstd_fast','--qos-profile-overrides-path',str(qos),'-o',str(out/'clearing_bag'),'--topics',*topics])
        wait(lambda:all(any(p.node_name=='clearing_recorder' for p in node.get_subscriptions_info_by_topic(topic)) for topic in topics),12.)
        start=node.get_clock().now().nanoseconds/1e9
        wait(lambda:node.get_clock().now().nanoseconds/1e9-start>=1.5,12.)
        report['clearing_capture']={'topics':topics,'began_before_removal':args.remove_dynamic_obstacle,'capture_phase':'before_first_goal' if before_goal else 'after_arrival','pre_removal_hold_sim_s':node.get_clock().now().nanoseconds/1e9-start,'path':str(out/'clearing_bag')}

    def remove_dynamic_obstacle():
        nonlocal stage
        stage='post_obstacle'
        assert stationary() and idle()
        removed={'requested_at':time.monotonic(),'requested_sim':node.get_clock().now().nanoseconds/1e9,'shadow_roi_before':shadow_roi[-1]}
        report['removal']=removed
        log=(out/'obstacle_remove.log').open('w');logs.append(log)
        command=['gz','service','-s','/world/small_warehouse/remove','--reqtype','gz.msgs.Entity','--reptype','gz.msgs.Boolean','--timeout','4000','--req','name: "guarded_trial_box" type: MODEL']
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy(),start_new_session=True)
        commands.append(process)
        wait(lambda:process.poll() is not None,6.)
        response=(out/'obstacle_remove.log').read_text()
        removed['response']={'exit_code':process.returncode,'output':response}
        assert process.returncode==0 and 'data: true' in response
        identity=report['obstacle']['first_observed_pose']['id']
        wait(lambda:any(identity in p['ids'] and p['at']>=removed['requested_at'] for p in deletions),6.)
        def cleared_twice():
            rows=[p for p in shadow_roi if p['stamp']>removed['requested_sim']]
            return len(rows)>=2 and rows[-1]['stamp']-rows[-2]['stamp']>=.5 and all(p['occupied']==0 for p in rows[-2:])
        wait(cleared_twice,14.)
        rows=[p for p in shadow_roi if p['stamp']>removed['requested_sim'] and p['occupied']==0]
        removed.update(first_clear_delay_sim_s=rows[0]['stamp']-removed['requested_sim'],first_cleared_shadow_roi=rows[0])
        held=time.monotonic();anchor=shadow_roi[-1]['stamp']
        removed.update(hold_started_wall=held,hold_start_stamp=anchor,required_clear_span_sim_s=2.,clear_deadline_sim_s=5.)
        wait(lambda:len([p for p in shadow_roi if p['stamp']>=anchor])>=3 and shadow_roi[-1]['stamp']-anchor>=2.,16.)
        held_rows=[p for p in shadow_roi if p['stamp']>=anchor]
        removed.update(hold_ended_wall=time.monotonic(),observed_clear_span_sim_s=held_rows[-1]['stamp']-anchor,held_costmap_samples=len(held_rows))
        report['checks']['removed_obstacle_and_shadow_cleared_from_observed_space']=removed['first_clear_delay_sim_s']<=5. and len(held_rows)>=3 and removed['observed_clear_span_sim_s']>=2. and all(p['occupied']==0 for p in held_rows)
        report['checks']['removal_did_not_start_another_goal']=idle() and stationary()
        if not all(report['checks'].values()):raise AssertionError('Obstacle removal/clearing failed')

    def reenter_cleared_space():
        nonlocal stage,handle
        # Deliberately let idle AMCL expire. The explicit request below is the
        # only reason to invoke the existing one-goal localization preflight.
        wait(lambda:health and health[-1]['reason']=='AMCL_STALE' and idle() and stationary(),8.)
        begin=report['removal']['hold_started_wall'];until=time.monotonic()
        held_nav=[p for p in nav if begin<=p['at']<=until]
        held_health=[p for p in health if begin<=p['at']<=until]
        held_drive=[p for p in drive if begin<=p['at']<=until]
        held_wheels=[p for p in wheel if begin<=p['at']<=until]
        quiet=(len(held_nav)>=10 and len(held_health)>=10 and len(held_drive)>=10 and len(held_wheels)>=10
            and all(p['phase']=='IDLE' and not p['goal_id'] and not p['forwarding'] and not p['quarantined'] for p in held_nav)
            and all(not p['owner'] for p in held_health)
            and all(abs(p['x'])<1e-6 and abs(p['z'])<1e-6 for p in held_drive)
            and all(abs(p['speed'])<.005 and abs(p['turn'])<.02 for p in held_wheels))
        report['checks']['idle_pose_expired_without_automatic_resume']=quiet and health[-1]['reason']=='AMCL_STALE'
        assert report['checks']['idle_pose_expired_without_automatic_resume'],'Motion/ownership changed without a requested goal'
        goal=(2.,0.,math.pi)
        explicit=time.monotonic()
        reentry={'explicit_requested_at':explicit,'goal':goal,'prior_health_reason':health[-1]['reason'],
            'idle_hold_wall_s':until-begin,'idle_nav_samples':len(held_nav),'idle_drive_samples':len(held_drive),
            'idle_wheel_samples':len(held_wheels),'no_automatic_resume':quiet}
        report['reentry']=reentry
        event={'index':len(report['goals']),'role':'post_removal_reentry','goal':goal,'started_at':explicit}
        report['goals'].append(event)
        stage='reentry'
        event['preflight']=prepare_localization(node)
        reentry['preflight']=event['preflight']
        request=NavigateToPose.Goal();request.pose.header.frame_id='map'
        request.pose.header.stamp=node.get_clock().now().to_msg()
        request.pose.pose.position.x,request.pose.pose.position.y=goal[:2]
        request.pose.pose.orientation.z=math.sin(goal[2]/2);request.pose.pose.orientation.w=math.cos(goal[2]/2)
        reentry.update(action_requested_at=time.monotonic(),action_requested_sim=node.get_clock().now().nanoseconds/1e9)
        future=action.send_goal_async(request);wait(future.done)
        handle=future.result();event['accepted']=handle.accepted
        assert handle.accepted,'Explicit reentry was rejected; no retry'
        pending=handle.get_result_async();wait(pending.done,90.)
        value=pending.result()
        event.update(status=value.status,error_code=value.result.error_code,reason=value.result.error_msg,
            completed_at=time.monotonic(),completion_ros_time=node.get_clock().now().nanoseconds/1e9)
        handle=None
        wait(lambda:stationary() and idle(),12.);spin(.2)
        observed=truth[-1];now_ros=node.get_clock().now().nanoseconds/1e9
        actual=observed if time.monotonic()-observed['at']<=.5 and -.1<=now_ros-observed['stamp']<=.3 else None
        event['truth']=actual;event['score']=score_goal(value.status,goal,actual)
        event['terminal_health']=health[-1];event['terminal_navigation']=nav[-1]
        received=[p for p in plans if p['at']>=reentry['action_requested_at'] and p['stamp']>=reentry['action_requested_sim']
            and p['frame']=='map' and p['poses'] and math.hypot(p['poses'][-1]['x']-2.,p['poses'][-1]['y'])<=.35]
        planned_inside=any(any(1.6<q['x']<2.4 and -.4<q['y']<.4 for q in p['poses']) for p in received)
        inside=actual is not None and 1.6<actual['x']<2.4 and -.4<actual['y']<.4
        trajectory=[p for p in truth if p['stage']=='reentry' and p['at']>=reentry['action_requested_at']]
        gaps=[b['stamp']-a['stamp'] for a,b in zip(trajectory,trajectory[1:])]
        matched=[];missing=0
        for p in poses:
            if p['stage']!='reentry' or p['at']<reentry['action_requested_at']:continue
            actual_pose=interpolate_truth(truth,p['stamp'])
            if actual_pose is None:missing+=1;continue
            matched.append(math.hypot(p['x']-actual_pose['x'],p['y']-actual_pose['y']))
        reentry.update(goal_score=event['score'],actual_arrival=actual,plan_samples=len(received),
            planned_inside_former_box=planned_inside,truth_inside_former_box=inside,
            paired_localization_poses=len(matched),unpaired_localization_poses=missing,
            max_localization_error_m=max(matched,default=None),max_truth_gap_s=max(gaps,default=None))
        report['checks'].update(explicit_reentry_passed_action_and_truth_score=event['score']['passed'],
            reentry_plan_and_truth_use_former_box=planned_inside and inside,
            reentry_localization_truth_error_within_declared_limit=bool(matched) and missing==0 and max(matched)<=.20,
            reentry_truth_observation_gap_within_declared_limit=bool(gaps) and max(gaps)<=.1,
            explicit_reentry_ended_idle_and_stationary=idle() and stationary())
        print(json.dumps({'reentry':reentry}),flush=True)
        if not all(report['checks'].values()):raise AssertionError('Explicit reentry failed independent criteria; no retry')

    out=args.out
    try:
        spin(.4)
        assert not node.get_publishers_info_by_topic('/clock'),'Domain is occupied'
        launch('simulation',['ros2','launch','rlai_bringup','simulation.launch.py','headless:=true','world:=small_warehouse',
            'motion_supervision_enabled:=true','motion_health_enabled:=true','use_amcl:=true','map_yaml_file:='+str(args.map),
            'x:='+str(config['spawn'][0]),'y:='+str(config['spawn'][1]),'yaw:='+str(config['spawn'][2]),
            'teleop_enabled:=false','camera_processing_enabled:=false','depth_camera_enabled:=false']+(['bridge_scan_topic:=scan_raw'] if args.scan_fault else []))
        launch('navigation',['ros2','launch','rlai_navigation','supervised_navigation.launch.py']+(['use_scan_occlusion:=true'] if args.scan_occlusion else [])+(['trace_occlusion_receiver:=true'] if args.trace_occlusion_receiver else []))
        if args.scan_fault:
            wait(lambda:len(raw_scans)>=5,25.)
            assert {p.node_name for p in node.get_publishers_info_by_topic('scan')}=={node.get_name()},'Public scan has an unexpected publisher'
            assert {p.node_name for p in node.get_publishers_info_by_topic('scan_raw')}=={'ros_gz_bridge'}
            report['checks']['isolated_scan_relay_topology']=True
        wait(lambda:action.server_is_ready() and stop_client.service_is_ready() and reset_inputs_ready() and nav and truth and stationary() and idle(),75.)
        publishers=node.get_publishers_info_by_topic('diff_drive_controller/cmd_vel')
        assert len(publishers)==1 and publishers[0].node_name=='motion_supervisor'
        report['checks']['exclusive_supervisor_drive_output']=True
        stage='reset'
        set_stop(True)
        wait(lambda:health[-1]['state']=='STOPPED' and idle() and stationary())
        prior=config['prior']
        initial=initial_pose_message(x=prior[0],y=prior[1],yaw=prior[2],xy_std=.1,yaw_std=math.radians(5),stamp=node.get_clock().now().to_msg())
        report['reset']=reset_localization(node,initial)
        assert health[-1]['state']=='STOPPED' and not health[-1]['inhibited'] and stationary()
        report['checks']['explicit_reset_preserves_stop']=True
        set_stop(False)
        wait(lambda:health[-1]['state']=='IDLE' and not health[-1]['inhibited'] and idle() and nav[-1]['reason']=='READY')
        if args.scan_fault:
            subscribers=node.get_subscriptions_info_by_topic('scan')
            report['scan_subscribers']=[{'name':s.node_name,'namespace':s.node_namespace} for s in subscribers]
            assert {'amcl','motion_supervisor','global_costmap','local_costmap'}<={s.node_name for s in subscribers}
            report['checks']['all_required_scan_consumers_use_faulted_topic']=True
        if args.scan_occlusion:
            wait(lambda:len(occlusion_clouds)>=5,10.)
            assert {p.node_name for p in node.get_publishers_info_by_topic('scan_occlusion/points')}=={'scan_occlusion'}
            assert 'global_costmap' in {p.node_name for p in node.get_subscriptions_info_by_topic('scan_occlusion/points')}
            report['checks']['scan_occlusion_connected_to_global_costmap']=True
        if args.record_control_bag:record_clearing(before_goal=True)
        stage='route'
        for index,goal in enumerate(chosen_route):
            event={'index':index,'goal':goal,'started_at':time.monotonic()}
            report['goals'].append(event)
            event['preflight']=prepare_localization(node)
            request=NavigateToPose.Goal()
            request.pose.header.frame_id='map'
            request.pose.header.stamp=node.get_clock().now().to_msg()
            request.pose.pose.position.x,request.pose.pose.position.y=goal[:2]
            request.pose.pose.orientation.z=math.sin(goal[2]/2)
            request.pose.pose.orientation.w=math.cos(goal[2]/2)
            future=action.send_goal_async(request)
            wait(future.done)
            handle=future.result()
            event['accepted']=handle.accepted
            if not handle.accepted:
                raise RuntimeError('Goal was rejected; no retry')
            result=handle.get_result_async()
            if args.pause_controller:
                exercise_controller_pause(result,event)
                handle=None
                break
            if args.pause_occlusion:
                exercise_occlusion_pause(result,event)
                handle=None
                break
            if args.scan_fault:
                exercise_scan_fault(result,event)
                handle=None
                break
            if args.dynamic_obstacle and not report.get('obstacle'):insert_dynamic_obstacle(result)
            wait(result.done,90.)
            value=result.result()
            event.update(status=value.status,error_code=value.result.error_code,reason=value.result.error_msg,
                completed_at=time.monotonic(),completion_ros_time=node.get_clock().now().nanoseconds/1e9)
            # Preserve the actual terminal result before waiting for cleanup/standstill.
            handle=None
            wait(lambda:stationary() and idle(),12.)
            spin(.2)
            observed=truth[-1]
            now_ros=node.get_clock().now().nanoseconds/1e9
            actual=observed if time.monotonic()-observed['at']<=.5 and -.1<=now_ros-observed['stamp']<=.3 else None
            event['truth']=actual
            event['score']=score_goal(value.status,goal,actual)
            event['terminal_health']=health[-1]
            event['terminal_navigation']=nav[-1]
            print(json.dumps({'scenario':args.scenario,'goal_index':index,'status':value.status,'score':event['score']}),flush=True)
            if not event['score']['passed']:
                raise AssertionError('Goal failed independent completion criteria; no retry')
        if args.dynamic_obstacle:score_dynamic_obstacle()
        if args.record_clearing:record_clearing()
        if args.remove_dynamic_obstacle:remove_dynamic_obstacle()
        if args.reenter_cleared_space:reenter_cleared_space()
        if not (args.scan_fault or args.pause_occlusion or args.pause_controller):report['checks']['all_explicit_goals_pass_action_and_truth_criteria']=True
        report['complete']=True
    except Exception as exc:
        report.update(all_passed=False,error_type=type(exc).__name__,error=str(exc))
    finally:
        resume_source()
        stage='cleanup'
        tracked=snapshot_owned([p.pid for p in children+commands])
        if rclpy.ok() and handle is not None and handle.accepted:
            try:
                cancelled=handle.cancel_goal_async()
                wait(cancelled.done,3.)
            except Exception:
                report['cancellation_outcome_unconfirmed']=True
        if children and rclpy.ok() and stop_client.service_is_ready():
            try:
                set_stop(True)
                wait(lambda:health and health[-1]['state']=='STOPPED' and stationary(),5.)
                report['cleanup_stop_confirmed']=True
            except Exception:
                report['cleanup_stop_confirmed']=False
        report['latest_health']=health[-1] if health else None
        report['latest_navigation']=nav[-1] if nav else None
        tracked.extend(snapshot_owned([p.pid for p in children+commands]))
        for p in reversed(children+commands):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM)
                    p.wait(timeout=5)
        report['owned_process_cleanup']=finish_owned_cleanup(tracked,domain=str(args.domain),partition=os.environ['GZ_PARTITION'])
        matched=[]
        missing=0
        for p in poses:
            if p['stage']!='route':
                continue
            actual=interpolate_truth(truth,p['stamp'])
            if actual is None:
                missing+=1
                continue
            matched.append({'stamp':p['stamp'],'xy_error_m':math.hypot(p['x']-actual['x'],p['y']-actual['y']),
                'yaw_error_rad':abs(math.remainder(p['yaw']-actual['yaw'],2*math.pi)),
                'max_xy_variance':max(p['covariance'][0],p['covariance'][7]),'yaw_variance':p['covariance'][35]})
        trajectory=[p for p in truth if p['stage']=='route']
        report['metrics']={'truth_distance_m':sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(trajectory,trajectory[1:])),
            'paired_route_poses':len(matched),'unpaired_route_poses':missing,
            'max_localization_error_m':max((p['xy_error_m'] for p in matched),default=None),
            'max_localization_yaw_error_rad':max((p['yaw_error_rad'] for p in matched),default=None),
            'max_xy_variance':max((p['max_xy_variance'] for p in matched),default=None),
            'elapsed_wall_seconds':time.monotonic()-started}
        if not (args.scan_fault or args.pause_occlusion or args.pause_controller):
            report['checks']['route_localization_truth_error_within_declared_limit']=bool(matched) and missing==0 and max(p['xy_error_m'] for p in matched)<=.20
        if 'corridor' in config:
            half_length,half_width=report['padded_footprint_half_dimensions_m']
            clearance=[footprint_clearance(p,**config['corridor'],half_length=half_length,half_width=half_width) for p in trajectory]
            gaps=[b['stamp']-a['stamp'] for a,b in zip(trajectory,trajectory[1:])]
            report['metrics'].update(min_sampled_padded_sidewall_clearance_m=min(clearance) if clearance and all(c is not None for c in clearance) else None,max_truth_gap_s=max(gaps,default=None))
            longitudinal='y' if config['corridor']['axis']=='x' else 'x'
            report['checks']['path_stays_in_central_passage_section']=bool(trajectory) and all(-6.<=p[longitudinal]<=4. for p in trajectory)
            report['checks']['sampled_padded_footprint_clearance']=bool(clearance) and all(c is not None and c>=.10 for c in clearance) and bool(gaps) and max(gaps)<=.1
        report['all_passed']=bool(report.get('complete')) and all(report['checks'].values()) and report.get('cleanup_stop_confirmed',False) and report['owned_process_cleanup']['clean']
        for name,values in [('health',health),('navigation',nav),('truth',truth),('amcl',poses),('drive',drive),('wheel',wheel),('localization_errors',matched),('scan_statistics',scans),('raw_scan',raw_scans),('forwarded_scan',forwarded_scans),('plans',plans),('static_roi',static_roi),('global_roi',costmap_roi),('obstacle_pose',obstacle_poses),('occlusion_cloud',occlusion_clouds),('shadow_roi',shadow_roi),('deletion',deletions)]:
            (out/(name+'_samples.json')).write_text(json.dumps(values,indent=2,allow_nan=False))
        if args.record_control_trace:
            (out/'control_trace.json').write_text(json.dumps(control_trace,indent=2,allow_nan=False))
            report['control_trace']={'raw_events':sum(p['kind']=='raw' for p in control_trace),'leased_events':sum(p['kind']=='leased' for p in control_trace),'lease_tokens_recorded':False,'observer_qos':'BEST_EFFORT_KEEP_LAST_50','complete_delivery_guaranteed':False}
        if args.trace_occlusion_receiver:
            (out/'occlusion_receive_trace.json').write_text(json.dumps(occlusion_receive_trace,indent=2,allow_nan=False))
            report['occlusion_receive_trace_events']=len(occlusion_receive_trace)
        (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
        print(json.dumps({k:v for k,v in report.items() if k not in {'goals','route'}},indent=2),flush=True)
        if gazebo_node is not None:
            gazebo_node.unsubscribe('/world/small_warehouse/pose/info')
            gazebo_node.unsubscribe('/world/small_warehouse/scene/deletion')
        action.destroy()
        node.destroy_node()
        rclpy.shutdown()
        for log in logs:
            log.close()
    return 0 if report['all_passed'] else 2


if __name__=='__main__':
    raise SystemExit(main())
