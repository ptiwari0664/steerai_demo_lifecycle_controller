#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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