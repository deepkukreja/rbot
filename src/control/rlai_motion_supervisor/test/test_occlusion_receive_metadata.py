"""A live publisher must be distinguishable from missing derived samples."""
import json
import time
import uuid
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header,String
from rlai_motion_supervisor.nav_adapter import NavigationAdapter


def test_receiver_trace_distinguishes_matched_silent_publisher_from_publisher_loss():
    context=Context();rclpy.init(context=context)
    namespace='/receive_metadata_'+uuid.uuid4().hex
    adapter=NavigationAdapter(context=context,namespace=namespace,parameter_overrides=[
        Parameter('require_scan_occlusion',value=True),Parameter('trace_occlusion_receiver',value=True)])
    observer=rclpy.create_node('observer',context=context,namespace=namespace)
    executor=SingleThreadedExecutor(context=context);executor.add_node(adapter);executor.add_node(observer)
    events=[]
    observer.create_subscription(String,'nav2_adapter/occlusion_trace',lambda m:events.append(json.loads(m.data)),qos_profile_sensor_data)
    publisher=observer.create_publisher(PointCloud2,'scan_occlusion/points',qos_profile_sensor_data)
    def until(predicate,seconds=3.):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.01)
            if predicate():return
        assert predicate(),'Native trace did not describe the receiver boundary'
    try:
        until(lambda:publisher.get_subscription_count()==1)
        until(lambda:any(p['kind']=='gate' and p.get('matched_publishers')==1 for p in events))
        header=Header();header.frame_id='test_laser';header.stamp=observer.get_clock().now().to_msg()
        before=time.time_ns();publisher.publish(create_cloud_xyz32(header,[(1.,0.,0.)]))
        until(lambda:any(p['kind']=='cloud' for p in events))
        cloud=next(p for p in events if p['kind']=='cloud')
        assert before<=cloud['middleware_source_wall_ns']<=cloud['callback_wall_ns']<=time.time_ns()
        assert cloud['middleware_received_wall_ns']>=cloud['middleware_source_wall_ns']
        assert cloud['callback_wall_ns']>=cloud['middleware_received_wall_ns']
        assert 'publication_sequence' in cloud and 'reception_sequence' in cloud
        # An alive but silent producer must retain a match while freshness expires.
        until(lambda:any(p['kind']=='gate' and p.get('matched_publishers')==1 and p['reason']=='SCAN_OCCLUSION_STALE' for p in events))
        receipt=adapter.occlusion.received;stamp=adapter.occlusion.stamp
        observer.destroy_publisher(publisher);publisher=None
        until(lambda:any(p['kind']=='gate' and p.get('matched_publishers')==0 and p['accepted_stamp']==stamp for p in events))
        assert adapter.occlusion.received==receipt and adapter.occlusion.stamp==stamp
    finally:
        executor.shutdown();adapter.destroy_node();observer.destroy_node();rclpy.shutdown(context=context)
