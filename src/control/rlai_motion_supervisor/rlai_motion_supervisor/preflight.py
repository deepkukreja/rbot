"""One explicit fresh localization check for a requested simulated goal, never a timer."""
import time


def prepare_localization(node,timeout=3.):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_srvs.srv import Empty
    from rlai_motion_interfaces.msg import SupervisorStatus,NavigationStatus
    from rclpy.qos import qos_profile_sensor_data

    current={'supervisor':None,'navigation':None,'pose':None}
    def receive(name,message):
        current[name]=(message,time.monotonic())
    subscriptions=[
        node.create_subscription(SupervisorStatus,'motion_supervisor/status',lambda m:receive('supervisor',m),10),
        node.create_subscription(NavigationStatus,'nav2_adapter/status',lambda m:receive('navigation',m),10),
        node.create_subscription(PoseWithCovarianceStamped,'amcl_pose',lambda m:receive('pose',m),qos_profile_sensor_data)]
    client=node.create_client(Empty,'request_nomotion_update')
    pending=None

    def wait(predicate,deadline):
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():
                return
        reason=current['supervisor'][0].health_reason if current['supervisor'] else 'STATUS_UNAVAILABLE'
        raise RuntimeError('Localization preflight incomplete: '+reason)

    try:
        deadline=time.monotonic()+timeout
        wait(lambda:current['supervisor'] and current['navigation'] and client.service_is_ready(),deadline)
        s,received=current['supervisor']
        n,_=current['navigation']
        if time.monotonic()-received>.5 or s.owner or not s.health_required or s.state=='STOPPED' or n.quarantined or (s.inhibited and s.health_reason not in {'AMCL_STALE','AMCL_MISSING'}):
            raise RuntimeError('Local control does not permit localization preflight: '+s.health_reason)
        requested=time.monotonic()
        pending=client.call_async(Empty.Request())

        def refreshed():
            if not pending.done() or pending.exception() is not None or not current['pose']:
                return False
            pose,receipt=current['pose']
            stamp=pose.header.stamp.sec+pose.header.stamp.nanosec/1e9
            age=node.get_clock().now().nanoseconds/1e9-stamp
            s,sr=current['supervisor']
            n,nr=current['navigation']
            return receipt>=requested and -.1<=age<=.5 and time.monotonic()-sr<.5 and time.monotonic()-nr<.5 and not s.inhibited and not s.owner and n.phase=='IDLE' and n.reason=='READY'
        wait(refreshed,deadline)
        return {'refreshed':True,'requests':1}
    finally:
        if pending is not None and not pending.done():
            client.remove_pending_request(pending)
            pending.cancel()
        node.destroy_client(client)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
