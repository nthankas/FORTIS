"""
FORTIS mission-level state machine.

Tracks the high-level operational phase of the robot during an inspection
mission. This is pure logic with no ROS or hardware dependencies so it can be
unit tested in isolation. A ROS 2 node wraps this class, subscribing to event
topics and publishing the current state.

States and transitions are defined as data (TRANSITIONS table) rather than
nested if/elif chains. To add or change behavior, edit the table.

Stow/deploy is intentionally NOT modeled here. Stow is a pose flag handled by
the arm controller; the mission can be in any state with the arm stowed or
deployed. Forcing stow into this state machine would create a combinatorial
explosion of states.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Optional


# --- States and events ------------------------------------------------------


class State(Enum):
    IDLE = auto()
    ORBIT = auto()
    TARGETING = auto()
    ARM_AT_VIEW = auto()
    INSPECT = auto()
    PICK = auto()
    HOLDING = auto()
    RETURN_HOME = auto()
    FAULT = auto()


class Event(Enum):
    START_ORBIT = auto()
    CHASSIS_CAM_CLICK = auto()
    STOP = auto()
    ARM_AT_VIEW_POSE = auto()
    IK_FAILED = auto()
    CANCEL = auto()
    SELECT_OBSERVE = auto()
    SELECT_PICK = auto()
    GRASP_SUCCESS = auto()
    GRASP_FAIL = auto()
    RELEASE = auto()
    DONE = auto()
    HOME_REACHED = auto()
    FAULT = auto()
    RESET = auto()


# --- Guards -----------------------------------------------------------------
#
# A guard is a function that takes the runtime context dict and returns True
# if the transition is allowed. Guards let the same (state, event) pair lead
# to different transitions based on runtime conditions.
#
# The context dict is provided by the caller of step(). Typical keys:
#   ik_ok               bool  IK solver found a valid arm trajectory
#   target_pose_valid   bool  clicked target produced a reachable pose
#   grasp_candidate_ok  bool  grasp planner returned at least one valid grasp
#   gripper_closed      bool  gripper closure verified by encoder/force
#   gripper_open        bool  gripper open verified
#   arm_at_stow         bool  arm joint angles within stow tolerance
#   arm_at_home         bool  arm joint angles within home tolerance
#   chassis_at_home     bool  chassis position within home radius
#   pick_in_contact     bool  gripper has begun closing on target
#   operator_ack        bool  operator has acknowledged a fault and reset

Guard = Callable[[dict], bool]


def always(ctx: dict) -> bool:
    return True


def target_pose_valid(ctx: dict) -> bool:
    return ctx.get("target_pose_valid", False)


def ik_ok(ctx: dict) -> bool:
    return ctx.get("ik_ok", False)


def grasp_candidate_ok(ctx: dict) -> bool:
    return ctx.get("grasp_candidate_ok", False)


def gripper_closed(ctx: dict) -> bool:
    return ctx.get("gripper_closed", False)


def gripper_open(ctx: dict) -> bool:
    return ctx.get("gripper_open", False)


def home_reached(ctx: dict) -> bool:
    return ctx.get("arm_at_home", False) and ctx.get("chassis_at_home", False)


def pick_pre_contact(ctx: dict) -> bool:
    """Cancelling PICK is safe only before the gripper has touched the target."""
    return not ctx.get("pick_in_contact", False)


def operator_ack(ctx: dict) -> bool:
    return ctx.get("operator_ack", False)


# --- Transition table -------------------------------------------------------


@dataclass(frozen=True)
class Transition:
    from_state: Optional[State]  # None = wildcard, applies from any state
    event: Event
    to_state: State
    guard: Guard = always


# Order matters when two transitions match the same (state, event). The first
# one whose guard returns True wins. Put the more specific guards first.
TRANSITIONS: list[Transition] = [
    # IDLE
    Transition(State.IDLE,        Event.START_ORBIT,       State.ORBIT),

    # ORBIT
    Transition(State.ORBIT,       Event.CHASSIS_CAM_CLICK, State.TARGETING, target_pose_valid),
    Transition(State.ORBIT,       Event.STOP,              State.IDLE),

    # TARGETING
    Transition(State.TARGETING,   Event.ARM_AT_VIEW_POSE,  State.ARM_AT_VIEW, ik_ok),
    Transition(State.TARGETING,   Event.IK_FAILED,         State.ORBIT),
    Transition(State.TARGETING,   Event.CANCEL,            State.ORBIT),

    # ARM_AT_VIEW
    Transition(State.ARM_AT_VIEW, Event.SELECT_OBSERVE,    State.INSPECT),
    Transition(State.ARM_AT_VIEW, Event.SELECT_PICK,       State.PICK, grasp_candidate_ok),
    Transition(State.ARM_AT_VIEW, Event.CANCEL,            State.RETURN_HOME),

    # INSPECT (no contact, always safe to back out)
    Transition(State.INSPECT,     Event.CANCEL,            State.ARM_AT_VIEW),
    Transition(State.INSPECT,     Event.DONE,              State.RETURN_HOME),

    # PICK (cancel logic depends on contact state)
    Transition(State.PICK,        Event.GRASP_SUCCESS,     State.HOLDING, gripper_closed),
    Transition(State.PICK,        Event.GRASP_FAIL,        State.ARM_AT_VIEW),
    Transition(State.PICK,        Event.CANCEL,            State.ARM_AT_VIEW, pick_pre_contact),
    Transition(State.PICK,        Event.CANCEL,            State.FAULT),  # mid-grasp cancel

    # HOLDING
    Transition(State.HOLDING,     Event.RELEASE,           State.ARM_AT_VIEW, gripper_open),
    Transition(State.HOLDING,     Event.DONE,              State.RETURN_HOME),

    # RETURN_HOME
    Transition(State.RETURN_HOME, Event.HOME_REACHED,      State.IDLE, home_reached),

    # Fault handling (wildcard from any state)
    Transition(None,              Event.FAULT,             State.FAULT),
    Transition(State.FAULT,       Event.RESET,             State.IDLE, operator_ack),
]


# --- Errors -----------------------------------------------------------------


class IllegalTransition(Exception):
    """Raised when an event has no matching transition from the current state."""

    def __init__(self, state: State, event: Event):
        self.state = state
        self.event = event
        super().__init__(f"No transition from {state.name} on {event.name}")


# --- The state machine ------------------------------------------------------


@dataclass
class StepResult:
    """Returned by step() so callers can react to what happened."""
    from_state: State
    to_state: State
    event: Event
    transition_taken: bool


class MissionStateMachine:
    """
    Table-driven mission state machine.

    Usage:
        sm = MissionStateMachine()
        sm.step(Event.START_ORBIT, ctx={})
        sm.current  # State.ORBIT

    The caller supplies a context dict on each step() call. The machine
    inspects ctx to evaluate guards but does not store it; the caller owns
    runtime state.
    """

    def __init__(
        self,
        initial: State = State.IDLE,
        transitions: Optional[list[Transition]] = None,
    ):
        self._current = initial
        self._transitions = transitions if transitions is not None else TRANSITIONS
        self._history: list[StepResult] = []

    @property
    def current(self) -> State:
        return self._current

    @property
    def history(self) -> list[StepResult]:
        return list(self._history)  # defensive copy

    def step(self, event: Event, ctx: Optional[dict] = None) -> StepResult:
        """
        Process an event. If a matching transition exists and its guard
        passes, update current state. Otherwise raise IllegalTransition.

        Returns a StepResult describing what happened.
        """
        if ctx is None:
            ctx = {}

        for t in self._transitions:
            if t.event != event:
                continue
            if t.from_state is not None and t.from_state != self._current:
                continue
            if not t.guard(ctx):
                continue

            # Match found. Apply the transition.
            result = StepResult(
                from_state=self._current,
                to_state=t.to_state,
                event=event,
                transition_taken=True,
            )
            self._current = t.to_state
            self._history.append(result)
            return result

        # No matching transition.
        raise IllegalTransition(self._current, event)

    def try_step(self, event: Event, ctx: Optional[dict] = None) -> StepResult:
        """
        Same as step() but returns a no-op StepResult instead of raising
        when the transition is illegal. Useful for events that are noise
        in the current state and should be silently ignored (e.g. STOP
        when already IDLE).
        """
        try:
            return self.step(event, ctx)
        except IllegalTransition:
            return StepResult(
                from_state=self._current,
                to_state=self._current,
                event=event,
                transition_taken=False,
            )

    def reset(self, to: State = State.IDLE) -> None:
        """Force the machine back to a known state. For testing or recovery."""
        self._current = to
        self._history.clear()


# --- Diagram export ---------------------------------------------------------


def to_mermaid(transitions: Optional[list[Transition]] = None) -> str:
    """
    Produce a Mermaid stateDiagram-v2 representation of the transition table.
    Useful for sanity-checking the machine visually and for documentation.
    """
    if transitions is None:
        transitions = TRANSITIONS

    lines = ["stateDiagram-v2", "    [*] --> IDLE"]
    for t in transitions:
        src = t.from_state.name if t.from_state is not None else "[*]"
        label = t.event.name
        if t.guard is not always:
            label += f" [{t.guard.__name__}]"
        lines.append(f"    {src} --> {t.to_state.name}: {label}")
    return "\n".join(lines)


if __name__ == "__main__":
    print(to_mermaid())