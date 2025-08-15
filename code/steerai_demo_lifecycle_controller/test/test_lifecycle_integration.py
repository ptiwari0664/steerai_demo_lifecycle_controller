# -*- coding: utf-8 -*-
"""
End-to-end lifecycle tests for TurtleLifecycleController.

Brings up:
- turtlesim_node
- steerai_demo_lifecycle_controller (LifecycleNode)

Validates:
- Lifecycle transitions (configure -> activate -> deactivate -> cleanup)
- Service gating: commands rejected while INACTIVE, accepted while ACTIVE
- Parameter validation & dynamic update of timer period
- Action goal acceptance while ACTIVE
"""

import asyncio
import time
import unittest
from typing import Tuple

import launch
import launch_ros
import pytest

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from lifecycle_msgs.msg import State as LcState
from lifecycle_msgs.srv import ChangeState, GetState
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci


PKG = 'steerai_demo_lifecycle_controller'
NODE_NAME = 'turtle_lifecycle_controller'

# ---------- Launch description ----------

@pytest.mark.launch_test
def generate_test_description():
    turtle = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        output='screen',
    )

    ctrl = launch_ros.actions.LifecycleNode(
        package=PKG,
        executable='node',
        name=NODE_NAME,
        output='screen',
        parameters=[{
            'linear_speed': 0.5,
            'angular_speed': 0.0,
            'publish_period_ms': 50,
        }],
    )

    return (
        launch.LaunchDescription([
            turtle,
            ctrl,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'ctrl': ctrl,
            'turtle': turtle,
        }
    )


# ---------- Test case ----------

class TestLifecycleIntegration(unittest.IsolatedAsyncioTestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._node = rclpy.create_node('it_lifecycle_tester')
        # QoS for service discovery can be default.
        cls._spin_thread = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        cls._spin_thread.add_node(cls._node)

    @classmethod
    def tearDownClass(cls):
        cls._spin_thread.shutdown()
        cls._node.destroy_node()
        rclpy.shutdown()

    # -------- helpers --------

    async def _wait_for_service(self, name: str, timeout: float = 10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._node.service_is_available(name):
                return True
            await asyncio.sleep(0.05)
        self.fail(f'service {name} not available in time')

    async def _get_state(self) -> int:
        cli = self._node.create_client(GetState, f'/{NODE_NAME}/get_state')
        await cli.wait_for_service(timeout_sec=5.0)
        future = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self._node, future)
        self._node.destroy_client(cli)
        self.assertTrue(future.result() is not None)
        return int(future.result().current_state.id)

    async def _change_state(self, transition_id: int) -> bool:
        cli = self._node.create_client(ChangeState, f'/{NODE_NAME}/change_state')
        await cli.wait_for_service(timeout_sec=5.0)
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        self._node.destroy_client(cli)
        self.assertIsNotNone(future.result())
        return bool(future.result().success)

    async def _call_setbool(self, srv_name: str, value: bool) -> Tuple[bool, str]:
        cli = self._node.create_client(SetBool, srv_name)
        await cli.wait_for_service(timeout_sec=5.0)
        req = SetBool.Request()
        req.data = value
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        self._node.destroy_client(cli)
        res = future.result()
        self.assertIsNotNone(res)
        return bool(res.success), str(res.message)

    async def _call_trigger(self, srv_name: str) -> Tuple[bool, str]:
        cli = self._node.create_client(Trigger, srv_name)
        await cli.wait_for_service(timeout_sec=5.0)
        req = Trigger.Request()
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut)
        self._node.destroy_client(cli)
        res = fut.result()
        self.assertIsNotNone(res)
        return bool(res.success), str(res.message)

    async def _set_param(self, name: str, value) -> bool:
        cli = self._node.create_client(SetParameters, f'/{NODE_NAME}/set_parameters')
        await cli.wait_for_service(timeout_sec=5.0)
        p = Parameter()
        p.name = name
        if isinstance(value, float):
            p.value.type = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = float(value)
        elif isinstance(value, int):
            p.value.type = ParameterType.PARAMETER_INTEGER
            p.value.integer_value = int(value)
        else:
            self.fail(f'unsupported param type for {name}')
        req = SetParameters.Request(parameters=[p])
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut)
        self._node.destroy_client(cli)
        res = fut.result()
        self.assertIsNotNone(res)
        return bool(res.results[0].successful)

    # -------- tests --------

    async def test_01_services_visible(self):
        # lifecycle services
        await self._wait_for_service(f'/{NODE_NAME}/get_state')
        await self._wait_for_service(f'/{NODE_NAME}/change_state')
        # app services
        await self._wait_for_service('/set_mode')
        await self._wait_for_service('/toggle_circle')
        await self._wait_for_service('/go_home')

    async def test_02_configure_and_activate(self):
        # Initially UNCONFIGURED
        st = await self._get_state()
        self.assertEqual(st, LcState.PRIMARY_STATE_UNCONFIGURED)

        ok = await self._change_state(TransitionId.CONFIGURE)
        self.assertTrue(ok)
        st = await self._get_state()
        self.assertEqual(st, LcState.PRIMARY_STATE_INACTIVE)

        ok = await self._change_state(TransitionId.ACTIVATE)
        self.assertTrue(ok)
        st = await self._get_state()
        self.assertEqual(st, LcState.PRIMARY_STATE_ACTIVE)

    async def test_03_service_gating(self):
        # Deactivate to check gating
        await self._change_state(TransitionId.DEACTIVATE)
        st = await self._get_state()
        self.assertEqual(st, LcState.PRIMARY_STATE_INACTIVE)

        ok, msg = await self._call_setbool('/toggle_circle', True)
        self.assertFalse(ok)
        self.assertIn('Node not active', msg)

        # Activate and expect success
        await self._change_state(TransitionId.ACTIVATE)
        ok, msg = await self._call_setbool('/toggle_circle', True)
        self.assertTrue(ok)
        self.assertIn('circle ON', msg)

    async def test_04_parameter_validation_and_update(self):
        # Activate
        await self._change_state(TransitionId.ACTIVATE)
        # Invalid publish period -> must fail
        ok = await self._set_param('publish_period_ms', 5)   # < 10 ms
        self.assertFalse(ok)
        # Valid update -> should succeed and timer should keep running
        ok = await self._set_param('publish_period_ms', 50)
        self.assertTrue(ok)

    async def test_05_action_goal_acceptance(self):
        # Ensure ACTIVE
        await self._change_state(TransitionId.ACTIVATE)
        act = ActionClient(self._node, Fibonacci, '/demo_action')
        self.assertTrue(act.wait_for_server(timeout_sec=5.0))

        goal = Fibonacci.Goal()
        goal.order = 10
        send_future: Future = act.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)
        handle = send_future.result()
        self.assertIsNotNone(handle)
        self.assertTrue(handle.accepted)

        # Get result (don’t block forever)
        res_future: Future = handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, res_future, timeout_sec=10.0)
        result = res_future.result()
        # Either succeeded or canceled; in happy path we should succeed with >= 2 items
        self.assertIsNotNone(result)
        self.assertGreaterEqual(len(result.result.sequence), 2)

    async def test_06_deactivate_and_cleanup(self):
        ok = await self._change_state(TransitionId.DEACTIVATE)
        self.assertTrue(ok)
        st = await self._get_state()
        self.assertEqual(st, LcState.PRIMARY_STATE_INACTIVE)

        ok = await self._change_state(TransitionId.CLEANUP)
        self.assertTrue(ok)
        st = await self._get_state()
        self.assertEqual(st, LcState.PRIMARY_STATE_UNCONFIGURED)


# ---------- Transition IDs helper ----------

class TransitionId:
    """Convenience constants for lifecycle transitions."""
    CONFIGURE = 1
    CLEANUP = 4
    ACTIVATE = 3
    DEACTIVATE = 5
    SHUTDOWN = 6
