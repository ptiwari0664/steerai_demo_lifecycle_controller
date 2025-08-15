#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy, pytest
from rclpy.node import Node
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node as RosNode
from launch_testing.actions import ReadyToTest
import launch_testing
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from std_srvs.srv import SetBool

def generate_test_description():
    turtle = RosNode(package='turtlesim', executable='turtlesim_node', name='sim')
    ctrl = LifecycleNode(package='steerai_demo_lifecycle_controller',
                         executable='node', name='turtle_lifecycle_controller',
                         parameters=[{'linear_speed': 0.2,'angular_speed': 0.2,'publish_period_ms': 50}],
                         output='screen')
    return LaunchDescription([turtle, ctrl, ReadyToTest()]), {'ctrl': ctrl}

@pytest.mark.launch_test
def test_configure_activate_and_service(launch_service, ctrl, proc_info):
    rclpy.init()
    try:
        n = Node('itest')
        change = n.create_client(ChangeState, '/turtle_lifecycle_controller/change_state')
        set_mode = n.create_client(SetBool, '/set_mode')
        assert change.wait_for_service(timeout_sec=10.0)
        req = ChangeState.Request(); req.transition.id = Transition.TRANSITION_CONFIGURE
        assert change.call(req).success
        req.transition.id = Transition.TRANSITION_ACTIVATE
        assert change.call(req).success
        assert set_mode.wait_for_service(timeout_sec=5.0)
        assert set_mode.call(SetBool.Request(data=True)).success
    finally:
        rclpy.shutdown()
