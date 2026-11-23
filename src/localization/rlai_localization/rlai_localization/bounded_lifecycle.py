"""Bounded lifecycle service relay; stock Nav2 retains orchestration and bonds."""
from dataclasses import asdict
import json
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.task import Future
from lifecycle_msgs.srv import ChangeState, GetState
from std_srvs.srv import Trigger

from rlai_localization.lifecycle_transition import Dispatch, transition_once

READY_LINE = 'RLAI_LOCALIZATION_STARTUP_READY'


class BoundedLifecycle(Node):
    def __init__(self):
        super().__init__('localization_startup_guard')
        # One executor thread; coroutines yield while clients and steady timers
        # run. A request holds its per-node busy flag across those yields.
        self.group = ReentrantCallbackGroup()
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.busy = set()
        self.ready = False
        self.managed_clients = {}
        self.servers = []
        for name in ('map_server', 'amcl'):
            self.managed_clients[name] = (
                self.create_client(GetState, '_localization_control/'+name+'/get_state', callback_group=self.group),
                self.create_client(ChangeState, '_localization_control/'+name+'/change_state',
                                   callback_group=self.group))
        self.manager = self.create_client(
            Trigger, 'lifecycle_manager_localization/is_active', callback_group=self.group)
        # Default mutually exclusive group prevents overlapping readiness polls;
        # response clients and timeout timers use the separate reentrant group.
        self.readiness_timer = self.create_timer(.2, self.check_ready, clock=self.steady)

    def advertise_lifecycle(self):
        for name in ('map_server', 'amcl'):
            async def handle(request, response, name=name):
                return await self.change(name, request, response)
            self.servers.append(self.create_service(
                ChangeState, name+'/change_state', handle, callback_group=self.group))
            async def observe(request, response, name=name):
                _, actual = await self.call(self.managed_clients[name][0], request, 1.5)
                if actual is not None:
                    response.current_state = actual.current_state
                else:
                    # Explicit UNKNOWN, never a cached or fabricated primary
                    # state. This lets the manager continue a hard-reset sweep
                    # past an absent node instead of throwing out of the sweep.
                    response.current_state.id = 0
                    response.current_state.label = 'unavailable'
                return response
            self.servers.append(self.create_service(
                GetState, name+'/get_state', observe, callback_group=self.group))

    async def wait(self, predicate, seconds):
        if predicate():
            return True
        completed = Future(executor=self.executor)
        deadline = time.monotonic()+seconds
        def tick():
            if completed.done():
                return
            if predicate():
                completed.set_result(True)
            elif time.monotonic() >= deadline:
                completed.set_result(False)
        timer = self.create_timer(.02, tick, clock=self.steady, callback_group=self.group)
        try:
            return await completed
        finally:
            self.destroy_timer(timer)

    async def call(self, client, request, seconds):
        """Availability plus response share one steady-clock budget."""
        started = time.monotonic()
        if not await self.wait(client.service_is_ready, seconds):
            self.get_logger().warning('LIFECYCLE_SERVICE_UNAVAILABLE '+client.srv_name)
            return False, None
        try:
            future = client.call_async(request)
        except Exception as error:
            self.get_logger().error('Lifecycle dispatch failed: '+str(error))
            return False, None
        try:
            remaining = max(0., seconds-(time.monotonic()-started))
            if not await self.wait(future.done, remaining):
                self.get_logger().warning('LIFECYCLE_REPLY_TIMEOUT '+client.srv_name)
                return True, None
            return True, future.result()
        except Exception as error:
            self.get_logger().error('Lifecycle response unavailable: '+str(error))
            return True, None
        finally:
            # A late response cannot become the response to a later request.
            client.remove_pending_request(future)

    async def state(self, name):
        _, response = await self.call(self.managed_clients[name][0], GetState.Request(), 1.5)
        return response.current_state.id if response is not None else None

    async def change(self, name, request, response):
        if name in self.busy:
            response.success = False
            self.get_logger().error('LIFECYCLE_BUSY '+name)
            return response
        self.busy.add(name)
        try:
            async def get_state():
                return await self.state(name)
            async def dispatch(transition):
                forwarded = ChangeState.Request()
                forwarded.transition.id = transition
                sent, reply = await self.call(self.managed_clients[name][1], forwarded, 3.)
                return Dispatch(sent, reply.success if reply is not None else None)
            result = await transition_once(request.transition.id, get_state, dispatch)
            response.success = result.success
            self.get_logger().info('LIFECYCLE_RESULT '+json.dumps({
                'node': name, 'transition': request.transition.id, **asdict(result)}))
            return response
        finally:
            self.busy.remove(name)

    async def check_ready(self):
        if self.ready or self.busy:
            return
        if not self.servers:
            # Preserve the stock manager's service-presence admission barrier.
            # Read-only probes may repeat until the independent launch deadline;
            # no transition is dispatched or retried during this discovery gate.
            if not all(client.service_is_ready() for pair in self.managed_clients.values() for client in pair):
                return
            states = [await self.state(name) for name in ('map_server', 'amcl')]
            if states == [1, 1]:
                self.advertise_lifecycle()
                self.get_logger().info('LIFECYCLE_ADMISSION_CONFIRMED: both actual servers UNCONFIGURED')
            return
        _, active = await self.call(self.manager, Trigger.Request(), .4)
        if active is None or not active.success or self.busy:
            return
        states = [await self.state(name) for name in ('map_server', 'amcl')]
        if states == [3, 3] and not self.busy:
            self.ready = True
            self.readiness_timer.cancel()
            # Dedicated launch protocol line, emitted only after real state and
            # manager responses. The launch process owns the independent timer.
            print(READY_LINE, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = BoundedLifecycle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
