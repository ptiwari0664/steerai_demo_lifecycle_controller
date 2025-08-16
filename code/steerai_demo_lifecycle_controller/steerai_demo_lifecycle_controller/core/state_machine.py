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

"""
Operational state machine for the turtlesim controller.

- Pure Python (no ROS imports) for easy unit testing.
- Modes: IDLE, CIRCLE
- Events: SET_IDLE, SET_CIRCLE, ERROR(reason), CLEAR_ERROR
- Freeze-on-error semantics: while in error, mode-change events are ignored
  until CLEAR_ERROR; current mode is preserved (behavior layer decides to
  publish zero commands whenever has_error is True).

Also records a ring-buffer of recent transitions for traceability.
"""

from __future__ import annotations
from dataclasses import dataclass, replace
from enum import Enum, auto
from collections import deque
from typing import Deque, List, Optional
import time


# -------------------------- Public enums & dataclasses -------------------------- #

class Mode(Enum):
    """Operational mode."""
    IDLE = auto()
    CIRCLE = auto()


class Event(Enum):
    """Inputs to the state machine."""
    SET_IDLE = auto()
    SET_CIRCLE = auto()
    ERROR = auto()         # set error (freeze)
    CLEAR_ERROR = auto()   # clear error (unfreeze)


@dataclass(frozen=True)
class Status:
    """
    Snapshot of the state machine state.

    Attributes:
        mode: Current operational mode.
        has_error: True if the SM is in a frozen error condition.
        reason: Optional human-readable reason for the error.
        updated_at: Unix timestamp (float seconds) when last updated.
    """
    mode: Mode
    has_error: bool = False
    reason: Optional[str] = None
    updated_at: float = 0.0


@dataclass(frozen=True)
class TransitionRec:
    """
    A single transition record captured in the ring buffer.

    Attributes:
        t: Unix timestamp when the transition was recorded.
        frm: Mode before handling the event.
        evt: Event applied.
        to: Mode after handling the event.
        reason: Optional reason string (e.g., when evt is ERROR).
        has_error: Whether the resulting status has_error is True.
    """
    t: float
    frm: Mode
    evt: Event
    to: Mode
    reason: Optional[str]
    has_error: bool


# ------------------------------- State machine --------------------------------- #

class StateMachine:
    """
    Simple operational state machine with error freeze & transition history.

    Semantics:
        - Default mode: IDLE
        - SET_CIRCLE => CIRCLE, unless frozen by error
        - SET_IDLE   => IDLE,   unless frozen by error
        - ERROR(reason) => sets has_error=True and stores reason, preserves mode
        - CLEAR_ERROR    => clears has_error and reason, preserves mode

    Notes:
        - The behavior layer (e.g., compute_command) should *also* check
          Status.has_error to publish safe commands (often zeros).
        - Transitions are recorded in an in-memory ring buffer (maxlen=64 by default).
    """

    def __init__(self, history_size: int = 64) -> None:
        now = time.time()
        self._status: Status = Status(mode=Mode.IDLE, has_error=False, reason=None, updated_at=now)
        self._hist: Deque[TransitionRec] = deque(maxlen=max(1, history_size))

    # ------- Public API ------- #

    @property
    def status(self) -> Status:
        """Return an immutable snapshot of the current status."""
        return self._status

    def dispatch(self, evt: Event, reason: Optional[str] = None) -> Status:
        """
        Apply an event to the state machine and return the updated status.

        Args:
            evt: Event to apply.
            reason: Optional human-readable reason (used when evt == ERROR).

        Returns:
            Updated Status snapshot.
        """
        before = self._status
        after = self._apply(before, evt, reason)
        self._status = after
        # record transition
        self._hist.append(
            TransitionRec(
                t=after.updated_at,
                frm=before.mode,
                evt=evt,
                to=after.mode,
                reason=after.reason if evt is Event.ERROR else reason,
                has_error=after.has_error,
            )
        )
        return self._status

    def history(self) -> List[TransitionRec]:
        """Return a copy of the transition history (most-recent last)."""
        return list(self._hist)

    def last_transition(self) -> Optional[TransitionRec]:
        """Return the most recent transition record, or None if empty."""
        try:
            return self._hist[-1]
        except IndexError:
            return None

    def clear_history(self) -> None:
        """Erase the transition ring buffer."""
        self._hist.clear()

    # ------- Internal logic ------- #

    def _apply(self, status: Status, evt: Event, reason: Optional[str]) -> Status:
        now = time.time()

        # Error events mutate only the error flags; mode is preserved.
        if evt is Event.ERROR:
            # Enter frozen state; keep current mode.
            return replace(status, has_error=True, reason=reason or "unspecified", updated_at=now)

        if evt is Event.CLEAR_ERROR:
            # Clear frozen state; keep current mode.
            return replace(status, has_error=False, reason=None, updated_at=now)

        # If frozen, ignore mode-change events.
        if status.has_error:
            # No change to mode; only timestamp updates to reflect the ignored attempt.
            return replace(status, updated_at=now)

        # Handle mode changes when not frozen.
        if evt is Event.SET_CIRCLE:
            return replace(status, mode=Mode.CIRCLE, updated_at=now)

        if evt is Event.SET_IDLE:
            return replace(status, mode=Mode.IDLE, updated_at=now)

        # Unknown event (shouldn't happen with Enum) -> no change.
        return replace(status, updated_at=now)
