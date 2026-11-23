#!/usr/bin/env python3
"""Submit one fresh map goal to the local guarded simulation proxy; never publish velocity."""
import argparse
import json
import math
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x',type=float,required=True)
    parser.add_argument('--y',type=float,required=True)
    parser.add_argument('--yaw',type=float,default=0.)
    parser.add_argument('--cancel-after',type=float)
    args=parser.parse_args()
    if not all(math.isfinite(x) for x in (args.x,args.y,args.yaw)) or (args.cancel_after is not None and (not math.isfinite(args.cancel_after) or args.cancel_after<=0)):
        parser.error('Coordinates/heading must be finite; cancellation delay must be positive')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from nav2_msgs.action import NavigateToPose
    rclpy.init(args=[])
    node=rclpy.create_node('guarded_goal_client',parameter_overrides=[Parameter('use_sim_time',value=True)])
    client=ActionClient(node,NavigateToPose,'navigate_to_pose')
    try:
        end=time.monotonic()+10
        while time.monotonic()<end and (not client.server_is_ready() or node.get_clock().now().nanoseconds==0):
            rclpy.spin_once(node,timeout_sec=.05)
        if not client.server_is_ready() or node.get_clock().now().nanoseconds==0:
            raise RuntimeError('Guarded action server or simulation clock unavailable')
        from rlai_motion_supervisor.preflight import prepare_localization
        prepare_localization(node)
        request=NavigateToPose.Goal()
        request.pose.header.frame_id='map'
        request.pose.header.stamp=node.get_clock().now().to_msg()
        request.pose.pose.position.x,request.pose.pose.position.y=args.x,args.y
        request.pose.pose.orientation.z=math.sin(args.yaw/2)
        request.pose.pose.orientation.w=math.cos(args.yaw/2)
        sent=client.send_goal_async(request)
        end=time.monotonic()+5
        while not sent.done() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
        if not sent.done():
            raise RuntimeError('Goal acknowledgement unavailable; outcome unknown, inspect local status')
        handle=sent.result()
        if not handle.accepted:
            print(json.dumps({'accepted':False,'reason':'Inspect motion_supervisor/status and nav2_adapter/status'}))
            return 2
        future=handle.get_result_async()
        start=time.monotonic()
        canceled=False
        while not future.done() and time.monotonic()-start<130:
            rclpy.spin_once(node,timeout_sec=.05)
            if args.cancel_after is not None and not canceled and time.monotonic()-start>=args.cancel_after:
                handle.cancel_goal_async()
                canceled=True
        if not future.done():
            handle.cancel_goal_async()
            raise RuntimeError('Result unavailable; cancellation requested, inspect outcome locally')
        result=future.result()
        print(json.dumps({'accepted':True,'status':result.status,'error_code':result.result.error_code,'reason':result.result.error_msg}))
        return 0 if result.status in {4,5} else 2
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
