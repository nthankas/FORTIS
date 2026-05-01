"""
Tests for MissionStateMachine.

Run with:
    cd /workspace
    source install/setup.bash
    python -m pytest src/fortis_safety/test/test_mission_state_machine.py -v
"""

import pytest

from fortis_safety.mission_state_machine import (
    Event,
    IllegalTransition,
    MissionStateMachine,
    State,
    to_mermaid,
)

# --- Happy path: a full mission --------------------------------------------


def test_full_pick_mission():
    """IDLE -> ORBIT -> TARGETING -> ARM_AT_VIEW -> PICK -> HOLDING -> RETURN_HOME -> IDLE."""
    sm = MissionStateMachine()
    assert sm.current == State.IDLE

    sm.step(Event.START_ORBIT)
    assert sm.current == State.ORBIT

    sm.step(Event.CHASSIS_CAM_CLICK, {"target_pose_valid": True})
    assert sm.current == State.TARGETING

    sm.step(Event.ARM_AT_VIEW_POSE, {"ik_ok": True})
    assert sm.current == State.ARM_AT_VIEW

    sm.step(Event.SELECT_PICK, {"grasp_candidate_ok": True})
    assert sm.current == State.PICK

    sm.step(Event.GRASP_SUCCESS, {"gripper_closed": True})
    assert sm.current == State.HOLDING

    sm.step(Event.DONE)
    assert sm.current == State.RETURN_HOME

    sm.step(Event.HOME_REACHED, {"arm_at_home": True, "chassis_at_home": True})
    assert sm.current == State.IDLE


def test_full_observe_mission():
    """Observe path returns to ARM_AT_VIEW first, then home."""
    sm = MissionStateMachine()
    sm.step(Event.START_ORBIT)
    sm.step(Event.CHASSIS_CAM_CLICK, {"target_pose_valid": True})
    sm.step(Event.ARM_AT_VIEW_POSE, {"ik_ok": True})
    sm.step(Event.SELECT_OBSERVE)
    assert sm.current == State.INSPECT

    sm.step(Event.DONE)
    assert sm.current == State.RETURN_HOME


# --- Guard rejections ------------------------------------------------------


def test_chassis_click_rejected_when_target_invalid():
    sm = MissionStateMachine(initial=State.ORBIT)
    with pytest.raises(IllegalTransition):
        sm.step(Event.CHASSIS_CAM_CLICK, {"target_pose_valid": False})
    assert sm.current == State.ORBIT


def test_select_pick_rejected_when_no_grasp_candidate():
    sm = MissionStateMachine(initial=State.ARM_AT_VIEW)
    with pytest.raises(IllegalTransition):
        sm.step(Event.SELECT_PICK, {"grasp_candidate_ok": False})
    assert sm.current == State.ARM_AT_VIEW


def test_grasp_success_rejected_when_gripper_not_closed():
    sm = MissionStateMachine(initial=State.PICK)
    with pytest.raises(IllegalTransition):
        sm.step(Event.GRASP_SUCCESS, {"gripper_closed": False})
    assert sm.current == State.PICK


def test_home_reached_rejected_when_chassis_not_home():
    sm = MissionStateMachine(initial=State.RETURN_HOME)
    with pytest.raises(IllegalTransition):
        sm.step(Event.HOME_REACHED, {"arm_at_home": True, "chassis_at_home": False})
    assert sm.current == State.RETURN_HOME


# --- PICK cancel: pre-contact vs mid-grasp ---------------------------------


def test_pick_cancel_pre_contact_returns_to_arm_at_view():
    sm = MissionStateMachine(initial=State.PICK)
    sm.step(Event.CANCEL, {"pick_in_contact": False})
    assert sm.current == State.ARM_AT_VIEW


def test_pick_cancel_mid_grasp_goes_to_fault():
    sm = MissionStateMachine(initial=State.PICK)
    sm.step(Event.CANCEL, {"pick_in_contact": True})
    assert sm.current == State.FAULT


# --- Fault handling --------------------------------------------------------


@pytest.mark.parametrize("state", [
    State.IDLE, State.ORBIT, State.TARGETING, State.ARM_AT_VIEW,
    State.INSPECT, State.PICK, State.HOLDING, State.RETURN_HOME,
])
def test_fault_can_be_raised_from_any_state(state):
    sm = MissionStateMachine(initial=state)
    sm.step(Event.FAULT)
    assert sm.current == State.FAULT


def test_fault_reset_requires_operator_ack():
    sm = MissionStateMachine(initial=State.FAULT)
    with pytest.raises(IllegalTransition):
        sm.step(Event.RESET, {"operator_ack": False})
    assert sm.current == State.FAULT


def test_fault_reset_succeeds_with_operator_ack():
    sm = MissionStateMachine(initial=State.FAULT)
    sm.step(Event.RESET, {"operator_ack": True})
    assert sm.current == State.IDLE


# --- INSPECT cancel returns to ARM_AT_VIEW (no contact made) --------------


def test_inspect_cancel_returns_to_arm_at_view():
    sm = MissionStateMachine(initial=State.INSPECT)
    sm.step(Event.CANCEL)
    assert sm.current == State.ARM_AT_VIEW


# --- HOLDING release ------------------------------------------------------


def test_holding_release_requires_gripper_open():
    sm = MissionStateMachine(initial=State.HOLDING)
    with pytest.raises(IllegalTransition):
        sm.step(Event.RELEASE, {"gripper_open": False})
    assert sm.current == State.HOLDING


def test_holding_release_succeeds_with_gripper_open():
    sm = MissionStateMachine(initial=State.HOLDING)
    sm.step(Event.RELEASE, {"gripper_open": True})
    assert sm.current == State.ARM_AT_VIEW


# --- Illegal events --------------------------------------------------------


def test_illegal_event_from_idle():
    sm = MissionStateMachine()
    with pytest.raises(IllegalTransition):
        sm.step(Event.GRASP_SUCCESS, {"gripper_closed": True})
    assert sm.current == State.IDLE


def test_try_step_silently_ignores_illegal_events():
    sm = MissionStateMachine()
    result = sm.try_step(Event.STOP)  # STOP is not legal from IDLE
    assert sm.current == State.IDLE
    assert result.transition_taken is False


# --- History tracking -----------------------------------------------------


def test_history_records_transitions():
    sm = MissionStateMachine()
    sm.step(Event.START_ORBIT)
    sm.step(Event.STOP)
    assert len(sm.history) == 2
    assert sm.history[0].from_state == State.IDLE
    assert sm.history[0].to_state == State.ORBIT
    assert sm.history[1].from_state == State.ORBIT
    assert sm.history[1].to_state == State.IDLE


def test_history_does_not_record_failed_transitions():
    sm = MissionStateMachine(initial=State.ORBIT)
    with pytest.raises(IllegalTransition):
        sm.step(Event.CHASSIS_CAM_CLICK, {"target_pose_valid": False})
    assert len(sm.history) == 0


# --- Reset -----------------------------------------------------------------


def test_reset_returns_to_idle_and_clears_history():
    sm = MissionStateMachine()
    sm.step(Event.START_ORBIT)
    sm.reset()
    assert sm.current == State.IDLE
    assert sm.history == []


# --- Diagram export -------------------------------------------------------


def test_mermaid_export_runs():
    text = to_mermaid()
    assert "stateDiagram-v2" in text
    assert "IDLE" in text
    assert "FAULT" in text


if __name__ == "__main__":
    pytest.main([__file__, "-v"])