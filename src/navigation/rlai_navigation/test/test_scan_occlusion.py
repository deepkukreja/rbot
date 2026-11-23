"""Guard the bounded hidden-space policy and its acquisition-time contract."""
import importlib
import math
import time
import uuid

import pytest


def module():
    try:
        return importlib.import_module('rlai_navigation.scan_occlusion')
    except ModuleNotFoundError as error:
        pytest.fail(f'Missing scan-occlusion implementation: {error}')


def points(ranges, **kwargs):
    settings = dict(angle_min=0., angle_increment=1., range_min=.12,
                    range_max=12., depth=.5, spacing=.25, max_range=8.)
    settings.update(kwargs)
    return module().occlusion_points(ranges, **settings)


def test_shadow_starts_at_the_return_and_has_bounded_depth():
    result = points([1.])
    assert [p[0] for p in result] == pytest.approx([1., 1.25, 1.5])
    assert [p[1] for p in result] == pytest.approx([0., 0., 0.])
    assert all(p[2] == 0. for p in result)


def test_projection_cannot_exceed_the_configured_marking_horizon():
    assert [p[0] for p in points([7.75], depth=1.)] == pytest.approx([7.75, 8.])


def test_projection_cannot_exceed_the_sensor_range():
    assert [p[0] for p in points([1.75], range_max=2.)] == pytest.approx([1.75, 2.])


def test_missing_beams_do_not_create_a_connected_solid_surface():
    result = points([1., math.inf, 1.], angle_min=-math.pi/2, angle_increment=math.pi/2)
    assert [p[0] for p in result] == pytest.approx([0.] * 6, abs=1e-10)
    assert [p[1] for p in result] == pytest.approx([-1., -1.25, -1.5, 1., 1.25, 1.5])


def test_invalid_or_out_of_horizon_ranges_are_not_marked():
    assert points([math.inf, -math.inf, math.nan, -1., 0., .1, 9.]) == []


def test_clockwise_scan_uses_the_actual_beam_directions():
    result = points([1., 1.], angle_min=math.pi/2, angle_increment=-math.pi/2)
    assert result[0] == pytest.approx((0., 1., 0.), abs=1e-10)
    assert result[3] == pytest.approx((1., 0., 0.), abs=1e-10)


@pytest.mark.parametrize('changes',[
    {'angle_min':math.nan}, {'angle_increment':0.}, {'depth':-1.},
    {'depth':3.}, {'spacing':0.}, {'spacing':1e-10},
    {'range_min':2., 'range_max':1.}, {'range_max':math.nan},
])
def test_invalid_geometry_or_unbounded_work_is_rejected(changes):
    with pytest.raises(ValueError):
        points([1.], **changes)


def test_ros_output_keeps_acquisition_stamps_rejects_duplicates_and_replaces_with_empty():
    implementation = module()
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2

    context = Context()
    rclpy.init(context=context)
    namespace = '/occlusion_test_' + uuid.uuid4().hex
    source = implementation.ScanOcclusion(context=context, namespace=namespace)
    observer = rclpy.create_node('observer', context=context, namespace=namespace)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(source)
    executor.add_node(observer)
    output = []
    observer.create_subscription(PointCloud2, 'scan_occlusion/points', output.append, qos_profile_sensor_data)
    publisher = observer.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)

    def until(predicate, timeout=3.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail('ROS observation did not arrive within its bound')

    try:
        until(lambda:publisher.get_subscription_count() == 1 and source.output.get_subscription_count() == 1)
        message = LaserScan()
        message.header.frame_id = 'test_laser'
        message.header.stamp.sec = 10
        message.header.stamp.nanosec = 123
        message.angle_increment = .1
        message.range_min = .12
        message.range_max = 12.
        message.ranges = [1.]
        publisher.publish(message)
        until(lambda:len(output) == 1)
        assert output[0].header == message.header
        assert output[0].width > 1
        message.ranges = [2.]
        publisher.publish(message)
        end = time.monotonic() + .15
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        assert len(output) == 1
        message.header.stamp.sec = 11
        message.ranges = [math.inf]
        publisher.publish(message)
        until(lambda:len(output) == 2)
        assert output[1].width == 0 and output[1].header == message.header
        message.header.stamp.sec = 9
        message.ranges = [1.]
        publisher.publish(message)
        end = time.monotonic() + .15
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        assert len(output) == 2
    finally:
        executor.shutdown()
        source.destroy_node()
        observer.destroy_node()
        rclpy.shutdown(context=context)
