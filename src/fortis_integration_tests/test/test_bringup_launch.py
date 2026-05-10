"""
Scaffold for the fortis_bringup launch validation test.

When the bringup launch files stop being stubs, this suite verifies
that each one produces the launch graph it claims to (right set of
nodes, right namespaces, no orphan ros2_control controller spawners),
and that the composed system reaches a steady running state inside the
launch_testing harness.

Every test method is decorated with @pytest.mark.skip until the
bringup launch composition lands. The class structure documents the
two minimum-viable assertions the suite needs.
"""

import pytest


@pytest.mark.launch_test
class TestBringupLaunch:
    """Launch-graph validation for fortis_bringup composition."""

    @pytest.mark.skip(reason="scaffolding only")
    def test_bringup_launches_required_nodes(self):
        """
        bringup.launch.py brings up the required node set.

        Once bringup.launch.py composes real includes, this test runs
        the launch under launch_testing and asserts that the expected
        node names (mission_state_node, drive_node, arm controller,
        perception, localization) are alive within a timeout. The
        intent is to catch silent regressions where a refactor drops a
        node include from the composition.
        """
        ...

    @pytest.mark.skip(reason="scaffolding only")
    def test_bringup_reaches_idle_state(self):
        """
        bringup.launch.py reaches IDLE within startup deadline.

        End-to-end smoke check that the composed system actually
        latches /fortis/mission_state to IDLE within a small startup
        budget. This catches the failure mode where every node comes
        up but mission_state_node never publishes (e.g. wrong QoS,
        wrong topic name, or a circular subscription). Same deadline
        approach as the safety + drive integration test.
        """
        ...
