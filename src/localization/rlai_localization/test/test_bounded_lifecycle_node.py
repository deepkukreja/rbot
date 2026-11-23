"""Construct the real ROS relay in an isolated domain, without a simulator."""
import rclpy
from rlai_localization.bounded_lifecycle import BoundedLifecycle


def test_public_interfaces_wait_for_actual_server_readiness():
    rclpy.init(args=[],domain_id=69)
    node=None
    try:
        node=BoundedLifecycle()
        services={node.resolve_service_name(service.srv_name) for service in node.services}
        assert not services.intersection({'/map_server/change_state','/amcl/change_state','/map_server/get_state','/amcl/get_state'})
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()
