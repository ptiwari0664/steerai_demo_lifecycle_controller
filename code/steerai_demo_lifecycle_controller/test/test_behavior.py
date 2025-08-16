#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: Puneet Tiwari
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from steerai_demo_lifecycle_controller.core.state_machine import StateMachine, Event, Mode
from steerai_demo_lifecycle_controller.core.behavior import compute_command

def test_state_machine_transitions():
    sm = StateMachine()
    assert sm.status.mode is Mode.IDLE
    sm.dispatch(Event.SET_CIRCLE)
    assert sm.status.mode is Mode.CIRCLE
    sm.dispatch(Event.ERROR, "fail")
    assert sm.status.has_error
    sm.dispatch(Event.SET_IDLE)  # ignored due to error
    assert sm.status.mode is Mode.CIRCLE
    sm.dispatch(Event.CLEAR_ERROR)
    assert not sm.status.has_error
    sm.dispatch(Event.SET_IDLE)
    assert sm.status.mode is Mode.IDLE

def test_compute_command():
    sm = StateMachine()
    # IDLE -> zero cmd
    cmd = compute_command(sm.status, circle_enabled=True, lin=1.0, ang=2.0)
    assert cmd.lin_x == 0.0 and cmd.ang_z == 0.0
    # CIRCLE + enabled -> nonzero
    sm.dispatch(Event.SET_CIRCLE)
    cmd = compute_command(sm.status, circle_enabled=True, lin=1.0, ang=2.0)
    assert cmd.lin_x == 1.0 and cmd.ang_z == 2.0
    # error -> zero
    sm.dispatch(Event.ERROR, "x")
    cmd = compute_command(sm.status, circle_enabled=True, lin=1.0, ang=2.0)
    assert cmd.lin_x == 0.0 and cmd.ang_z == 0.0

def test_circle_disabled_gate_produces_zero():
    sm = StateMachine(); sm.dispatch(Event.SET_CIRCLE)
    cmd = compute_command(sm.status, circle_enabled=False, lin=1.0, ang=2.0)
    assert cmd.lin_x == 0.0 and cmd.ang_z == 0.0

def test_error_freezes_mode_changes():
    sm = StateMachine(); sm.dispatch(Event.SET_CIRCLE)
    sm.dispatch(Event.ERROR, "oops")
    sm.dispatch(Event.SET_IDLE)  # should be ignored
    assert sm.status.has_error is True
    assert sm.status.mode is Mode.CIRCLE
    sm.dispatch(Event.CLEAR_ERROR)
    sm.dispatch(Event.SET_IDLE)
    assert sm.status.has_error is False
    assert sm.status.mode is Mode.IDLE

def test_compute_command_passes_numbers_through():
    sm = StateMachine(); sm.dispatch(Event.SET_CIRCLE)
    lin, ang = 1.234, 2.345
    cmd = compute_command(sm.status, True, lin, ang)
    assert cmd.lin_x == lin and cmd.ang_z == ang
