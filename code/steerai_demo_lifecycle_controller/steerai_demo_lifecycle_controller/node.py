#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional
from math import isfinite
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    SetParametersResult,
    ParameterDescriptor,
    FloatingPointRange,
    IntegerRange,
)

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from example_interfaces.action import Fibonacci
from example_interfaces.srv import Trigger as EITrigger

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

from .core.state_machine import StateMachine, Event
from .core.behavior import compute_command


class TurtleLifecycleController(LifecycleNode):
    """Lifecycle-aware, modular controller for turtlesim."""

    def __init__(self) -> None:
        super().__init__('turtle_lifecycle_controller')
        self._cbg = ReentrantCallbackGroup()

        # ----- Core, ROS-agnostic -----
        self._sm = StateMachine()
        self._circle_enabled = False
        self._is_active = False  # robust ACTIVE flag

        # ----- ROS interfaces (created in on_configure) -----
        self._pub: Optional['rclpy.lifecycle.Publisher'] = None
        self._pose_sub = None
        self._timer = None
        self._srv_toggle_circle = None
        self._srv_set_mode = None
        self._srv_go_home = None
        self._srv_health = None
        self._teleport_cli = None
        self._action: Optional[ActionServer] = None

        # Telemetry
        self._last_pose: Optional[Pose] = None

        # ---------------- Parameters (declare with descriptors in __init__) ----------------
        self.declare_parameter(
            'linear_speed',
            1.0,
            descriptor=ParameterDescriptor(
                description='Linear speed of the turtle in m/s.',
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=5.0, step=0.0)],
            ),
        )
        self.declare_parameter(
            'angular_speed',
            1.0,
            descriptor=ParameterDescriptor(
                description='Angular speed of the turtle in rad/s.',
                floating_point_range=[FloatingPointRange(from_value=-5.0, to_value=5.0, step=0.0)],
            ),
        )
        self.declare_parameter(
            'publish_period_ms',
            100,
            descriptor=ParameterDescriptor(
                description='Publishing period for velocity commands in milliseconds.',
                integer_range=[IntegerRange(from_value=10, to_value=1000, step=1)],
            ),
        )
        # Dynamic updates
        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)

        self.get_logger().info('Constructed (UNCONFIGURED)')

    # ---------------- Lifecycle hooks ----------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure()')
        try:
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )

            # Pub/Sub
            self._pub = self.create_lifecycle_publisher(Twist, '/turtle1/cmd_vel', qos)
            self._pose_sub = self.create_subscription(
                Pose, '/turtle1/pose', self._on_pose, qos, callback_group=self._cbg
            )

            # Services
            self._srv_toggle_circle = self.create_service(
                SetBool, 'toggle_circle', self._on_toggle_circle, callback_group=self._cbg
            )
            self._srv_set_mode = self.create_service(
                SetBool, 'set_mode', self._on_set_mode, callback_group=self._cbg
            )
            self._srv_go_home = self.create_service(
                Trigger, 'go_home', self._on_go_home, callback_group=self._cbg
            )
            self._srv_health = self.create_service(
                EITrigger, 'health', self._on_health, callback_group=self._cbg
            )

            # Client
            self._teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

            # Action server (demo)
            self._action = ActionServer(
                self,
                Fibonacci,
                'demo_action',
                execute_callback=self._on_action_execute,
                goal_callback=self._on_action_goal,
                cancel_callback=self._on_action_cancel,
                callback_group=self._cbg,
            )

            # Timer (created but stopped until ACTIVE)
            period_ms = int(self.get_parameter('publish_period_ms').value)
            period = max(0.01, period_ms / 1000.0)
            self._timer = self.create_timer(period, self._on_timer, callback_group=self._cbg)
            self._timer.cancel()

            self.get_logger().info('Configured resources (INACTIVE)')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Configure failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_activate()')
        try:
            if self._pub is None or self._timer is None:
                self.get_logger().error('Missing resources in activate')
                return TransitionCallbackReturn.FAILURE
            self._pub.on_activate(state) 
            self._is_active = True
            self._timer.reset()
            self.get_logger().info(f'Activated, mode={self._sm.status.mode.name}')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Activate failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_deactivate()')
        try:
            self._is_active = False
            if self._timer:
                self._timer.cancel()
            if self._pub:
                self._pub.on_deactivate(state)
            self.get_logger().info('Deactivated (timer stopped, publisher inactive)')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Deactivate failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup()')
        try:
            self._is_active = False

            # Destroy timer first
            if self._timer:
                self._timer.cancel()
                self.destroy_timer(self._timer)
                self._timer = None

            # Destroy subs/services/action
            if self._pose_sub:
                self.destroy_subscription(self._pose_sub); self._pose_sub = None
            if self._srv_toggle_circle:
                self.destroy_service(self._srv_toggle_circle); self._srv_toggle_circle = None
            if self._srv_set_mode:
                self.destroy_service(self._srv_set_mode); self._srv_set_mode = None
            if self._srv_go_home:
                self.destroy_service(self._srv_go_home); self._srv_go_home = None
            if self._srv_health:
                self.destroy_service(self._srv_health); self._srv_health = None
            if self._action:
                self._action.destroy(); self._action = None

            # Destroy publisher
            if self._pub:
                try:
                    self._pub.on_deactivate(state)
                except Exception:
                    pass
                self.destroy_publisher(self._pub)
                self._pub = None

            # Client can be dropped
            self._teleport_cli = None

            self.get_logger().info('Cleaned up (UNCONFIGURED)')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Cleanup failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown()')
        try:
            self._is_active = False
            if self._timer:
                self._timer.cancel()
            return TransitionCallbackReturn.SUCCESS
        except Exception:
            return TransitionCallbackReturn.SUCCESS

    # ---------------- Parameter handling (pure validate + react) ----------------
    def _on_param_update(self, params: list[Parameter]) -> SetParametersResult:
        lin = float(self.get_parameter('linear_speed').value)
        ang = float(self.get_parameter('angular_speed').value)
        per = int(self.get_parameter('publish_period_ms').value)

        for p in params:
            if p.name == 'linear_speed':
                if p.type_ != Parameter.Type.DOUBLE or p.value < 0 or not isfinite(p.value):
                    return SetParametersResult(successful=False, reason='linear_speed must be finite and >= 0')
                lin = float(p.value)
            elif p.name == 'angular_speed':
                if p.type_ != Parameter.Type.DOUBLE or not isfinite(p.value):
                    return SetParametersResult(successful=False, reason='angular_speed must be finite')
                ang = float(p.value)
            elif p.name == 'publish_period_ms':
                v = float(p.value)
                if v < 10.0:
                    return SetParametersResult(successful=False, reason='publish_period_ms must be >= 10 ms')
                per = int(v)

        # react to new period safely (ROS applies params after we return SUCCESS)
        if self._timer is not None:
            was_running = self._is_active
            self._timer.cancel()
            new_period = max(0.01, per / 1000.0)
            self._timer.timer_period_ns = int(new_period * 1e9)
            if was_running:
                self._timer.reset()

        return SetParametersResult(successful=True)

    # ---------------- Helpers & ROS Callbacks ----------------
    def _node_is_active(self) -> bool:
        return self._is_active

    def _on_pose(self, pose: Pose) -> None:
        self._last_pose = pose  # hook for future analytics

    def _on_timer(self) -> None:
        if not self._node_is_active() or self._pub is None:
            return
        lin = float(self.get_parameter('linear_speed').value)
        ang = float(self.get_parameter('angular_speed').value)
        cmd = compute_command(self._sm.status, self._circle_enabled, lin, ang)
        twist = Twist()
        twist.linear.x = cmd.lin_x
        twist.angular.z = cmd.ang_z
        self._pub.publish(twist)

    # ---------------- Services ----------------
    def _on_toggle_circle(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if not self._node_is_active():
            res.success = False; res.message = 'Node not active'; return res
        self._circle_enabled = bool(req.data)
        res.success = True; res.message = 'circle ON' if self._circle_enabled else 'circle OFF'
        return res

    def _on_set_mode(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if not self._node_is_active():
            res.success = False; res.message = 'Node not active'; return res
        st = self._sm.dispatch(Event.SET_CIRCLE if req.data else Event.SET_IDLE)
        res.success = True; res.message = f'Mode={st.mode.name}'
        self.get_logger().info(f'Operational mode -> {st.mode.name}')
        return res

    def _on_health(self, req: EITrigger.Request, res: EITrigger.Response) -> EITrigger.Response:
        lc = 'ACTIVE' if self._is_active else 'INACTIVE/OTHER'
        st = self._sm.status
        res.success = True
        res.message = f'lifecycle={lc}, mode={st.mode.name}, error={st.has_error}({st.reason})'
        return res

    def _on_go_home(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if self._teleport_cli is None:
            res.success = False; res.message = 'Teleport client not available'; return res
        if not self._teleport_cli.service_is_ready():
            if not self._teleport_cli.wait_for_service(timeout_sec=1.0):
                res.success = False; res.message = 'Teleport service not available'; return res

        request = TeleportAbsolute.Request()
        request.x = 5.544; request.y = 5.544; request.theta = 0.0
        future = self._teleport_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        res.success = True; res.message = 'Teleported to center'
        return res

    # ---------------- Action (demo): Fibonacci while ACTIVE ----------------
    def _on_action_goal(self, goal: Fibonacci.Goal) -> GoalResponse:
        if not self._node_is_active():
            return GoalResponse.REJECT
        if goal.order < 0 or goal.order > 25:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_action_cancel(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _on_action_execute(self, goal_handle):
        if not self._node_is_active():
            goal_handle.abort()
            return Fibonacci.Result(sequence=[])

        order = goal_handle.request.order
        seq = [0, 1][:max(0, min(order, 2))]
        feedback = Fibonacci.Feedback()

        while len(seq) < order:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result(sequence=seq)

            nxt = (seq[-1] if seq else 1) + (seq[-2] if len(seq) > 1 else 0)
            seq.append(nxt)

            feedback.sequence = list(seq)
            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)  # sync sleep is fine with MultiThreadedExecutor

        goal_handle.succeed()
        return Fibonacci.Result(sequence=seq)


def main() -> None:
    rclpy.init()
    node = TurtleLifecycleController()
    exe = MultiThreadedExecutor(num_threads=2)
    exe.add_node(node)
    try:
        exe.spin()
    finally:
        exe.shutdown()
        node.destroy_node()
        rclpy.shutdown()
