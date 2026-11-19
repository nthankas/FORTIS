"""Unit tests for fortis_arm.arm_planner: pure math, no ROS."""

from __future__ import annotations

from fortis_arm.arm_ik import inverse_kinematics
from fortis_arm.arm_planner import in_collision, plan_path, segment_free

HOME = (0.0, 0.0, 0.0, 0.0)


def test_home_and_raised_poses_are_free():
    assert not in_collision(HOME)
    assert not in_collision((2.0, -1.2, 1.5, -0.3))


def test_floor_chassis_self_and_limits_are_rejected():
    assert in_collision((0.0, 0.6, 0.5, 0.0)), "tool below the floor"
    assert in_collision((1.571, 0.3, 0.3, 0.0)), "link through the chassis"
    assert in_collision((0.0, -0.5, 2.6, 1.0)), "tool folded onto link2"
    assert in_collision((0.0, 2.0, 0.0, 0.0)), "J2 beyond its limit"


def test_direct_move_when_free():
    goal = (0.5, -0.6, 1.2, -0.6)
    assert plan_path(HOME, goal) == [goal]


def test_colliding_goal_has_no_path():
    assert plan_path(HOME, (0.0, 0.6, 0.5, 0.0)) is None


def test_blocked_direct_move_goes_over_the_top():
    start = (-1.7, -0.47, 2.6, -0.52)
    goal = (0.6, -0.41, -1.77, 0.64)
    assert not in_collision(start) and not in_collision(goal)
    assert not segment_free(start, goal), "the elbow flip should collide"
    path = plan_path(start, goal)
    assert path is not None and len(path) == 3 and path[-1] == goal
    legs = [start] + path
    assert all(segment_free(a, b) for a, b in zip(legs, legs[1:]))


def test_only_the_elbow_up_ik_branch_is_free_for_a_low_target():
    solutions = inverse_kinematics((0.6398, 0.0, 0.097))
    assert len(solutions) == 2
    assert [in_collision(s) for s in solutions] == [False, True]
