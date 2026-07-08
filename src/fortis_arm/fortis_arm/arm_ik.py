"""
Analytic kinematics for the FORTIS 4-DOF arm (math + numpy only).

Geometry is hardcoded from fortis_description/urdf (fortis_constants.xacro
and fortis_arm.urdf.xacro are the source of truth; every value below cites
its xacro property):

* joint_j1  yaw about +Z at the arm base origin (the arm_mount / J1 axis).
* joint_j2  pitch about +Y, D1 = j1_to_j2_axis above the J1 origin.
* joint_j3  pitch about +Y, A2 = arm_link2_axis along link2 +X.
* joint_j4  pitch about +Y, A3 = arm_link3_axis along link3 +X.
* gripper_tcp: link4_to_gripper_base puts the gripper at
  (arm_link4_axis, 0, j4_to_gripper_z) in link4 with rpy (pi/2, 0, pi/2),
  which maps gripper +Z (the approach axis) onto link4 +X; gripper_tcp sits
  gripper_tcp_z further along gripper +Z. Net TCP offset in the link4 frame
  is therefore (arm_link4_axis + gripper_tcp_z, 0, j4_to_gripper_z).

With the URDF's +Y pitch axes a positive pitch tips the tool DOWN, so the
J2/J3 sub-chain is solved in (u, v) coordinates where u is radial reach and
v = down-positive height; that turns it into a textbook two-link planar arm
with exactly two elbow branches.

All poses are expressed in the ARM BASE frame (the arm_mount frame). The
fixed base_link -> arm_mount transform is owned by arm_motion_node, not by
this module, so the math here stays frame-agnostic and hardware-free.
"""

from __future__ import annotations

import math

import numpy as np

#: j1_to_j2_axis: J2 pitch-axis height above the J1 yaw origin (m).
D1 = 0.0559
#: arm_link2_axis: J2 -> J3 axis distance (m).
A2 = 0.393903
#: arm_link3_axis: J3 -> J4 axis distance (m).
A3 = 0.299237
#: arm_link4_axis + gripper_tcp_z: J4 axis -> TCP along link4 +X (m).
L4X = 0.0635 + 0.045
#: j4_to_gripper_z: TCP lift above the link4 +X line (m).
L4Z = 0.02286

#: URDF joint limits (rad), in (q1, q2, q3, q4) order: j1 +/-170 deg,
#: j2 +/-90 deg, j3 +/-150 deg, j4 +/-90 deg (fortis_constants.xacro).
JOINT_LIMITS = (
    (-2.967, 2.967),
    (-1.571, 1.571),
    (-2.618, 2.618),
    (-1.571, 1.571),
)

#: Tool pitches tried, nearest-level first, when the caller leaves
#: approach_pitch unset. Spans the J4-reachable +/-90 deg band.
_PITCH_CANDIDATES = (0.0, -0.3, 0.3, -0.6, 0.6, -0.9, 0.9, -1.2, 1.2, -1.5, 1.5)

_EPS = 1e-9


def _wrap(angle: float) -> float:
    """Wrap an angle into [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _within(value: float, limits: "tuple[float, float]") -> bool:
    """Return True when value lies inside the closed [lower, upper] interval."""
    return limits[0] <= value <= limits[1]


def forward_kinematics(q1: float, q2: float, q3: float, q4: float):
    """Return (tcp_xyz, tool_pitch) for a joint configuration.

    tcp_xyz is a numpy (3,) array locating the gripper_tcp origin in the arm
    base frame; tool_pitch is q2 + q3 + q4 (positive pitches the approach
    axis down). The tool yaw equals q1.
    """
    pitch = q2 + q3 + q4
    u = (A2 * math.cos(q2) + A3 * math.cos(q2 + q3)
         + L4X * math.cos(pitch) + L4Z * math.sin(pitch))
    z = (D1 - A2 * math.sin(q2) - A3 * math.sin(q2 + q3)
         - L4X * math.sin(pitch) + L4Z * math.cos(pitch))
    xyz = np.array([u * math.cos(q1), u * math.sin(q1), z])
    return xyz, pitch


def inverse_kinematics(target_xyz, approach_pitch: "float | None" = None):
    """Solve joint angles placing gripper_tcp at target_xyz (arm base frame).

    Return a list of (q1, q2, q3, q4) tuples -- elbow-up and elbow-down when
    both respect the URDF joint limits -- or an empty list when the target
    is unreachable. approach_pitch fixes the tool pitch (rad, positive =
    down); when None, candidate pitches are scanned nearest-level first and
    the first pitch yielding any limit-respecting solution wins, so results
    stay deterministic.
    """
    x, y, z = (float(c) for c in target_xyz)
    r = math.hypot(x, y)
    q1 = math.atan2(y, x) if r > _EPS else 0.0
    if not _within(q1, JOINT_LIMITS[0]):
        return []
    if approach_pitch is None:
        pitches = _PITCH_CANDIDATES
    else:
        pitches = (float(approach_pitch),)
    for pitch in pitches:
        solutions = _solve_at_pitch(r, z, q1, pitch)
        if solutions:
            return solutions
    return []


def _solve_at_pitch(r: float, z: float, q1: float, pitch: float):
    """Solve the J2/J3/J4 planar sub-chain for one fixed tool pitch."""
    # Subtract the pitched tool offset to get the wrist (J4 axis) point,
    # then flip to down-positive v so J2/J3 form a standard 2R chain.
    u = r - (L4X * math.cos(pitch) + L4Z * math.sin(pitch))
    v = D1 - z - L4X * math.sin(pitch) + L4Z * math.cos(pitch)
    c3 = (u * u + v * v - A2 * A2 - A3 * A3) / (2.0 * A2 * A3)
    if abs(c3) > 1.0:
        return []
    q3_mag = math.acos(max(-1.0, min(1.0, c3)))
    elbows = (q3_mag, -q3_mag) if q3_mag > _EPS else (0.0,)
    solutions = []
    for q3 in elbows:
        q2 = _wrap(math.atan2(v, u)
                   - math.atan2(A3 * math.sin(q3), A2 + A3 * math.cos(q3)))
        q4 = _wrap(pitch - q2 - q3)
        candidate = (q1, q2, q3, q4)
        if all(_within(q, lim) for q, lim in zip(candidate, JOINT_LIMITS)):
            solutions.append(candidate)
    return solutions


def reachable_workspace() -> "tuple[float, float]":
    """Return (min_radius, max_radius) of the wrist about the J2 axis (m).

    The wrist is the J4 axis: min is the folded-elbow radius |A2 - A3|
    (~0.095 m), max the outstretched radius A2 + A3 (~0.693 m). The TCP
    extends up to hypot(L4X, L4Z) (~0.111 m) beyond the wrist along the
    tool, so gross TCP reach depends on the chosen approach pitch.
    """
    return abs(A2 - A3), A2 + A3
