"""
Collision-checked joint-space planning for the FORTIS 4-DOF arm.

Pure math, no ROS. The arm is three line segments (link2, link3, tool) in
the arm base (arm_mount) frame from fortis_arm.arm_ik. A configuration is
rejected when a link point comes within clearance of the floor or the
chassis box, or when the tool folds back onto link2. Same checks as
sim/isaac/xdrive/lib/arm_ik_v3.check_collisions, with J1 included.

plan_path() tries the straight joint-space move first and falls back to
lifting the arm to a raised pose, yawing, then descending.
"""

from __future__ import annotations

import math

from fortis_arm.arm_ik import A2, A3, D1, JOINT_LIMITS, L4X, L4Z

#: Half the 28.7 mm CF tube plus 5 mm, as in the sim filter (m).
LINK_RADIUS = 0.0287 / 2.0 + 0.005
#: Extra clearance kept around the chassis and floor (m).
MARGIN = 0.025

#: Chassis box in the arm frame, from fortis_constants.xacro: the arm mount
#: sits j1_mount_x ahead of chassis center on the top face, yawed pi.
_CHASSIS_X = (0.0898 - 0.332 / 2.0, 0.0898 + 0.332 / 2.0)
_CHASSIS_HALF_Y = 0.217 / 2.0
_CHASSIS_Z = (-0.152, 0.0)
#: Floor height in the arm frame: belly_clearance + chassis_height below.
_FLOOR_Z = -(0.051 + 0.152)

#: Raised J2..J4 pose used as the via point (arm up, elbow folded forward).
_RAISED = (-1.2, 1.5, -0.3)
#: Joint-space step between collision samples along a move (rad).
_STEP_RAD = 0.05
#: Samples per link segment.
_LINK_SAMPLES = 8


def _chain(q):
    """Return the J2, J3, J4 and TCP points as (u, z) in the arm plane."""
    _, q2, q3, q4 = q
    pitch = q2 + q3 + q4
    j2 = (0.0, D1)
    j3 = (A2 * math.cos(q2), D1 - A2 * math.sin(q2))
    j4 = (j3[0] + A3 * math.cos(q2 + q3), j3[1] - A3 * math.sin(q2 + q3))
    tcp = (j4[0] + L4X * math.cos(pitch) + L4Z * math.sin(pitch),
           j4[1] - L4X * math.sin(pitch) + L4Z * math.cos(pitch))
    return j2, j3, j4, tcp


def _samples(a, b, start=0.0):
    """Yield points along the segment a -> b from fraction start to 1."""
    for i in range(_LINK_SAMPLES + 1):
        t = start + (1.0 - start) * i / _LINK_SAMPLES
        yield a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])


def in_collision(q) -> bool:
    """Return True when configuration q hits the floor, chassis or itself."""
    if not all(lo <= v <= hi for v, (lo, hi) in zip(q, JOINT_LIMITS)):
        return True
    j2, j3, j4, tcp = _chain(q)
    cos1, sin1 = math.cos(q[0]), math.sin(q[0])
    pad = LINK_RADIUS + MARGIN
    for a, b in ((j2, j3), (j3, j4), (j4, tcp)):
        for u, z in _samples(a, b):
            if z < _FLOOR_Z + pad:
                return True
            x, y = u * cos1, u * sin1
            if (_CHASSIS_X[0] - pad < x < _CHASSIS_X[1] + pad
                    and abs(y) < _CHASSIS_HALF_Y + pad
                    and _CHASSIS_Z[0] - pad < z < _CHASSIS_Z[1] + pad):
                return True
    # Tool against link2. The first part of link2 is skipped: the tool
    # cannot reach the shoulder without link3 crossing it first.
    for p in _samples(j4, tcp):
        for s in _samples(j2, j3, start=0.1):
            if math.hypot(p[0] - s[0], p[1] - s[1]) < 2.0 * LINK_RADIUS:
                return True
    return False


def segment_free(q_from, q_to) -> bool:
    """Return True when the straight joint-space move stays collision free."""
    span = max(abs(b - a) for a, b in zip(q_from, q_to))
    steps = max(1, math.ceil(span / _STEP_RAD))
    for i in range(1, steps + 1):
        t = i / steps
        if in_collision([a + t * (b - a) for a, b in zip(q_from, q_to)]):
            return False
    return True


def plan_path(start, goal):
    """Return the joint waypoints from start to goal, or None.

    The list ends at goal and excludes start. The direct move is used when
    it is free; otherwise the arm lifts to the raised pose at the start
    yaw, yaws to the goal, then descends.
    """
    start, goal = tuple(start), tuple(goal)
    if in_collision(goal):
        return None
    if segment_free(start, goal):
        return [goal]
    up_start = (start[0],) + _RAISED
    up_goal = (goal[0],) + _RAISED
    legs = [start, up_start, up_goal, goal]
    if all(segment_free(a, b) for a, b in zip(legs, legs[1:])):
        return legs[1:]
    return None
