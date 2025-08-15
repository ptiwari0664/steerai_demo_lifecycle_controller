#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    turtle = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        namespace='',
        output='screen'
    )

    ctrl = LifecycleNode(
        package='steerai_demo_lifecycle_controller',
        executable='node',
        name='turtle_lifecycle_controller',
        namespace='',
        output='screen',
        parameters=[{
            'linear_speed': 0.2,
            'angular_speed': 0.2,
            'publish_period_ms': 20
        }]
    )

    return LaunchDescription([turtle, ctrl])
