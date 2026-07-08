"""Unit tests for fortis_arm.arm_ik: pure analytic FK/IK, no ROS."""

from __future__ import annotations

import itertools

import numpy as np
import pytest

from fortis_arm.arm_ik import (
    A2,
    A3,
    forward_kinematics,
    inverse_kinematics,
    reachable_workspace,
)

#: Round-trip position budget: analytic solutions land at numeric noise,
#: so 1 mm is generous.
TOL_M = 1e-3

#: Joint grid kept inside the URDF limits AND inside the u > 0 half-space
#: (tool radially in front of the J1 axis), so every FK pose is recoverable
#: by IK's principal atan2 yaw branch.
GRID_Q1 = (-1.2, 0.0, 1.2)
GRID_Q2 = (-0.8, -0.3, 0.4)
GRID_Q3 = (-1.2, -0.5, 0.8)
GRID_Q4 = (-0.5, 0.0, 0.5)


def test_fk_ik_round_trip_over_joint_grid():
    """Verify IK recovers every FK pose of an in-limit joint grid to <1 mm."""
    checked = 0
    for q in itertools.product(GRID_Q1, GRID_Q2, GRID_Q3, GRID_Q4):
        xyz, pitch = forward_kinematics(*q)
        solutions = inverse_kinematics(xyz, approach_pitch=pitch)
        assert solutions, f"IK found no solution for FK pose of {q}"
        best = min(
            float(np.linalg.norm(forward_kinematics(*s)[0] - xyz))
            for s in solutions)
        assert best < TOL_M, f"round-trip error {best:.4f} m for {q}"
        checked += 1
    assert checked == 81


def test_elbow_up_and_down_both_returned():
    """Verify a mid-workspace target yields two distinct elbow branches."""
    # base_link click (-0.55, 0, 0.30) mapped into the arm frame; wrist
    # lands ~0.53 m from the shoulder, well inside both elbow branches.
    target = (0.6398, 0.0, 0.097)
    solutions = inverse_kinematics(target, approach_pitch=0.0)
    assert len(solutions) == 2
    q3s = sorted(s[2] for s in solutions)
    assert q3s[0] < 0.0 < q3s[1], f"expected opposite elbows, got {q3s}"
    for s in solutions:
        xyz, _ = forward_kinematics(*s)
        assert float(np.linalg.norm(xyz - np.array(target))) < TOL_M


def test_unreachable_targets_return_empty():
    """Verify out-of-workspace targets produce no solutions at any pitch."""
    # Far beyond max radial reach (~0.69 m wrist + ~0.11 m tool).
    assert inverse_kinematics((2.0, 0.0, 0.2)) == []
    # 1 m below the arm base: beyond the fully-down vertical reach.
    assert inverse_kinematics((0.0, 0.0, -1.0)) == []


def test_joint_limit_filtering_rejects_folded_elbow():
    """Verify targets needing |q3| beyond the URDF limit are filtered."""
    # A level-approach wrist 0.10 m from the shoulder needs |q3| ~3.05 rad
    # on BOTH elbow branches -- geometrically solvable (|cos q3| < 1) but
    # beyond the 2.618 rad joint_j3 limit, so IK must return nothing.
    target = (0.10 + 0.1085, 0.0, 0.0559 + 0.02286)
    assert inverse_kinematics(target, approach_pitch=0.0) == []


def test_reachable_workspace_bounds():
    """Verify the workspace helper matches the URDF link lengths."""
    r_min, r_max = reachable_workspace()
    assert r_min == pytest.approx(abs(A2 - A3))
    assert r_max == pytest.approx(A2 + A3)
    assert 0.09 < r_min < 0.10
    assert 0.69 < r_max < 0.70
