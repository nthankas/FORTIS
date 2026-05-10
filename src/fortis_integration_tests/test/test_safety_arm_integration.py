"""
Scaffold for the fortis_safety + fortis_arm integration test.

Mirrors the fortis_safety + fortis_drive contract test on the arm side:
verifies that the mission state machine actually gates the arm action
server, that the gate latches motion on / off across state transitions,
and that the FAULT path forces an arm shutdown regardless of prior
state.

Every test method is decorated with @pytest.mark.skip until the test
bodies are written. The class structure is intentional -- it documents
the four scenarios the integration suite needs to cover, so reviewers
can see the contract surface even before the implementation lands.
"""

import pytest


@pytest.mark.launch_test
class TestSafetyArmIntegration:
    """End-to-end tests exercising fortis_safety + fortis_arm together."""

    @pytest.mark.skip(reason="scaffolding only")
    def test_arm_blocked_in_idle(self):
        """
        IDLE rejects MoveToPose goals.

        Arm action server must reject (or refuse to accept) MoveToPose
        goals while the mission state is IDLE, since IDLE is not in the
        allowed-states list for the arm. Rejection must be explicit
        (action result with error code, not silence) so the operator
        UI can surface it.
        """
        ...

    @pytest.mark.skip(reason="scaffolding only")
    def test_arm_allowed_in_arm_at_view(self):
        """
        ARM_AT_VIEW accepts MoveToPose goals.

        After the FSM is driven to ARM_AT_VIEW, a MoveToPose goal must
        be accepted and produce a result indicating motion was at least
        attempted (terminal state succeeded / aborted, not rejected).
        """
        ...

    @pytest.mark.skip(reason="scaffolding only")
    def test_arm_blocked_during_orbit(self):
        """
        ORBIT does not allow arm motion.

        ORBIT is in the drive allow-list, not the arm allow-list. While
        the FSM is in ORBIT, arm goals must be rejected the same way as
        in IDLE. This test exists separately from the IDLE case to
        catch the failure mode of "we accidentally added every
        non-FAULT state to the arm allow-list".
        """
        ...

    @pytest.mark.skip(reason="scaffolding only")
    def test_fault_disables_arm(self):
        """
        FAULT forces arm rejection from any prior state.

        Drive the FSM to ARM_AT_VIEW (arm allowed), then publish FAULT.
        The next MoveToPose goal must be rejected within the same
        TRANSITION_RESPONSE_DEADLINE_S budget the drive integration
        test uses, regardless of where the arm was in its previous
        motion.
        """
        ...
