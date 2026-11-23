"""Required derived observations must gate admission and ongoing forwarding."""
import math
import struct
import time
import uuid

import pytest
from rlai_motion_supervisor import nav_gate


def test_required_source_starts_missing():
    assert nav_gate.OcclusionGate().reason(1., 10.) == 'SCAN_OCCLUSION_MISSING'


def test_fresh_advancing_acquisition_is_available():
    gate = nav_gate.OcclusionGate()
    gate.observe(10., 1., valid=True)
    assert gate.reason(1.1, 10.1) == ''


def test_repeated_acquisition_does_not_refresh_steady_liveness_with_a_paused_ros_clock():
    gate = nav_gate.OcclusionGate()
    gate.observe(10., 1., valid=True)
    gate.observe(10., 1.7, valid=True)
    assert gate.reason(1.7, 10.) == 'SCAN_OCCLUSION_STALE'


@pytest.mark.parametrize('stamp,ros_now',[(9.,10.),(11.,10.)])
def test_fresh_receipt_cannot_hide_old_or_future_acquisition(stamp,ros_now):
    gate = nav_gate.OcclusionGate()
    gate.observe(stamp, 1., valid=True)
    assert gate.reason(1.01, ros_now) == 'SCAN_OCCLUSION_TIMESTAMP'


def test_rewind_invalidates_and_a_duplicate_cannot_restore_admission():
    gate = nav_gate.OcclusionGate()
    gate.observe(10., 1., valid=True)
    gate.observe(9., 1.1, valid=True)
    gate.observe(10., 1.2, valid=True)
    assert gate.reason(1.2, 10.2) == 'SCAN_OCCLUSION_INVALID'
    gate.observe(10.3, 1.3, valid=True)
    assert gate.reason(1.3, 10.3) == ''


@pytest.mark.parametrize('stamp',[0.,-1.,math.nan,math.inf])
def test_invalid_acquisition_is_not_live(stamp):
    gate = nav_gate.OcclusionGate()
    gate.observe(stamp,1.,valid=True)
    assert gate.reason(1.,10.) == 'SCAN_OCCLUSION_INVALID'


def test_malformed_observation_invalidates_even_if_previous_valid_data_was_fresh():
    gate = nav_gate.OcclusionGate()
    gate.observe(10.,1.,valid=True)
    gate.observe(10.1,1.1,valid=False)
    assert gate.reason(1.1,10.1) == 'SCAN_OCCLUSION_INVALID'


def cloud(points=()):
    from sensor_msgs.msg import PointCloud2,PointField
    from std_msgs.msg import Header
    return PointCloud2(header=Header(frame_id='laser'),height=1,width=len(points),
        fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z'])],
        is_bigendian=False,point_step=12,row_step=12*len(points),
        data=b''.join(struct.pack('<fff',*p) for p in points))


@pytest.mark.parametrize('points',[[],[(1.,0.,0.),(2.,.5,0.)]])
def test_actual_xyz32_wire_contract_accepts_empty_and_finite_hypotheses(points):
    assert nav_gate.valid_occlusion_cloud(cloud(points))


@pytest.mark.parametrize('kind',['frame','fields','point_step','row_step','data','endian','nan','height','off_plane'])
def test_malformed_or_unusable_cloud_is_rejected(kind):
    message = cloud([(1.,0.,0.)])
    if kind=='frame':message.header.frame_id=''
    elif kind=='fields':message.fields[0].datatype=8
    elif kind=='point_step':message.point_step=16
    elif kind=='row_step':message.row_step=0
    elif kind=='data':message.data=b'\x00'
    elif kind=='endian':message.is_bigendian=True
    elif kind=='height':message.height=2
    elif kind=='nan':message.data=struct.pack('<fff',math.nan,0.,0.)
    elif kind=='off_plane':message.data=struct.pack('<fff',1.,0.,2.)
    assert not nav_gate.valid_occlusion_cloud(message)


@pytest.mark.parametrize('required',[False,True])
def test_real_adapter_subscription_is_conditional_and_gates_on_valid_output(required):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from rlai_motion_interfaces.msg import SupervisorStatus
    from rlai_motion_supervisor.nav_adapter import NavigationAdapter

    context=Context();rclpy.init(context=context)
    namespace='/occlusion_gate_test_'+uuid.uuid4().hex
    adapter=NavigationAdapter(context=context,namespace=namespace,
        parameter_overrides=[Parameter('require_scan_occlusion',value=required)])
    adapter.timer.cancel()
    observer=rclpy.create_node('observer',context=context,namespace=namespace)
    executor=SingleThreadedExecutor(context=context);executor.add_node(adapter);executor.add_node(observer)
    publisher=observer.create_publisher(PointCloud2,'scan_occlusion/points',qos_profile_sensor_data)

    def raw_inputs_healthy():
        now=time.monotonic()
        adapter._status(SupervisorStatus(state='IDLE',health_required=True,inhibited=False))
        for name in adapter.lifecycle.required:adapter.lifecycle.observe(name,3,now)

    def wait(predicate):
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01)
            raw_inputs_healthy()
            if predicate():return
        pytest.fail('Adapter did not observe the required-source state')

    try:
        raw_inputs_healthy()
        assert adapter._healthy(time.monotonic()) is (not required)
        if required:
            assert adapter._occlusion_reason(time.monotonic())=='SCAN_OCCLUSION_MISSING'
            wait(lambda:publisher.get_subscription_count()==1)
            message=cloud();message.header.stamp=adapter.get_clock().now().to_msg()
            publisher.publish(message)
            wait(lambda:adapter._healthy(time.monotonic()))
            malformed=cloud([(1.,0.,0.)]);malformed.header.stamp=adapter.get_clock().now().to_msg();malformed.data=b'\x00'
            publisher.publish(malformed)
            wait(lambda:adapter._occlusion_reason(time.monotonic())=='SCAN_OCCLUSION_INVALID')
            assert not adapter._healthy(time.monotonic())
            restored=cloud();restored.header.stamp=adapter.get_clock().now().to_msg();publisher.publish(restored)
            wait(lambda:adapter._healthy(time.monotonic()))
    finally:
        executor.shutdown();observer.destroy_node();adapter.destroy_node();rclpy.shutdown(context=context)
