"""Actual ROS callback tracing must preserve the existing acquisition gate."""
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


def test_receive_trace_records_callback_and_duplicate_without_refreshing_gate():
    context=Context();rclpy.init(context=context)
    namespace='/receive_trace_'+uuid.uuid4().hex
    adapter=NavigationAdapter(context=context,namespace=namespace,parameter_overrides=[
        Parameter('require_scan_occlusion',value=True),Parameter('trace_occlusion_receiver',value=True)])
    observer=rclpy.create_node('observer',context=context,namespace=namespace)
    executor=SingleThreadedExecutor(context=context);executor.add_node(adapter);executor.add_node(observer)
    events=[]
    observer.create_subscription(String,'nav2_adapter/occlusion_trace',lambda m:events.append(json.loads(m.data)),qos_profile_sensor_data)
    publisher=observer.create_publisher(PointCloud2,'scan_occlusion/points',qos_profile_sensor_data)
    def until(predicate,seconds=2.):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.01)
            if predicate():return
        assert predicate(),'Receive diagnostics did not arrive'
    try:
        until(lambda:publisher.get_subscription_count()==1)
        header=Header();header.frame_id='test_laser';header.stamp=observer.get_clock().now().to_msg()
        message=create_cloud_xyz32(header,[(1.,0.,0.)]);publisher.publish(message)
        until(lambda:any(p['kind']=='cloud' for p in events))
        first=next(p for p in events if p['kind']=='cloud')
        assert first['acquisition_ns']==header.stamp.sec*10**9+header.stamp.nanosec
        assert first['valid'] and first['advanced'] and first['payload_bytes']==12
        receipt=adapter.occlusion.received
        publisher.publish(message)
        until(lambda:len([p for p in events if p['kind']=='cloud'])>=2)
        second=[p for p in events if p['kind']=='cloud'][1]
        assert second['valid'] and not second['advanced'] and adapter.occlusion.received==receipt
        # The steady timer and cloud subscription are independent callbacks.
        until(lambda:any(p['kind']=='gate' and 'callback_gap_s' in p for p in events))
    finally:
        executor.shutdown();adapter.destroy_node();observer.destroy_node();rclpy.shutdown(context=context)
