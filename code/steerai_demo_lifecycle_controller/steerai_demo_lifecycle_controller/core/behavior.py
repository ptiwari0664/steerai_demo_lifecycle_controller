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


from __future__ import annotations
from dataclasses import dataclass
from .state_machine import Mode, Status

@dataclass
class Command:
    lin_x: float = 0.0
    ang_z: float = 0.0

def compute_command(status: Status, circle_enabled: bool, lin: float, ang: float) -> Command:
    if status.has_error:
        return Command()  # fail-safe
    if status.mode is Mode.CIRCLE and circle_enabled:
        return Command(lin_x=float(lin), ang_z=float(ang))
    return Command()