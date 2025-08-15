#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from steerai_demo_lifecycle_controller.core.state_machine import StateMachine, Event, Mode

def test_freeze_on_error():
    sm = StateMachine()
    sm.dispatch(Event.ERROR, "boom")
    before = sm.status
    sm.dispatch(Event.SET_CIRCLE)
    after = sm.status
    assert before == after  # frozen
    sm.dispatch(Event.CLEAR_ERROR)
    sm.dispatch(Event.SET_CIRCLE)
    assert sm.status.mode is Mode.CIRCLE
