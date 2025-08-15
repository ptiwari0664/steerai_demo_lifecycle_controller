#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# steerai_demo_lifecycle_controller/core/__init__.py
"""
Core, ROS-agnostic logic for steerai_demo_lifecycle_controller.
Exports the operational state machine and behavior functions.
"""
from .state_machine import StateMachine, Event, Mode, Status, TransitionRec
from .behavior import compute_command

__all__ = [
    "StateMachine",
    "Event",
    "Mode",
    "Status",
    "TransitionRec",
    "compute_command",
]
