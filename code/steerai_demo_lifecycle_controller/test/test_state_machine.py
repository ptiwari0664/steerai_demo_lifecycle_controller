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
