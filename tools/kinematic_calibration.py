#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Compute corrected X-drive kinematic constants from empirical drive tests.

WHAT THIS IS
------------
A standalone, off-robot calculator that turns UMBmark-style drive measurements
into corrected values for the three geometry constants the X-drive kinematics
depend on:

    WHEEL_RADIUS         effective rolling radius at the loaded contact patch
    LEN_X + LEN_Y        the combined lever arm the H-matrix rotation column uses

It does NOT touch the robot, ROS, or any source file. It only prints corrected
numbers alongside the nominals and tells you where to apply them.

WHY EMPIRICAL CALIBRATION IS NEEDED
-----------------------------------
The nominal constants come from CAD, but two things the CAD cannot know move the
real behaviour:

  * The 8 in omni's *effective* rolling radius under a ~40 lb load is smaller
    than the catalog 0.1016 m (tire squash, contact-patch geometry). A radius
    that is off by k% scales every commanded distance by k%.
  * The H-matrix collapses the four 45-deg wheels into a single (LEN_X + LEN_Y)
    lever arm for rotation, and that 45-deg scaling was carried over verbatim
    from a senior-design module, never re-derived (see src/fortis_comms/
    README.md "Known open question"). So the rotational term can carry a scale
    error independent of the radius error.

UMBmark (Borenstein & Feng, 1996) is the standard way to separate these: drive
a known shape, measure the end-pose error, attribute the error to a constant.

THE THREE MEASUREMENTS AND THEIR MATH
-------------------------------------
1. Effective wheel radius (--radius-*)
     Roll the LOADED robot dead-straight for N wheel revolutions and tape-measure
     the base_link ground travel D. N is read as a delta of a drive ODrive's
     pos_estimate, which is already in turns (revolutions), so no unit math is
     needed. One revolution advances 2*pi*r at the contact patch, so:
         r_eff = D / (2 * pi * N)
     base_link is chassis center at ground level, so the floor mark sits directly
     under chassis center and D is a true body-frame translation.

2. Pure-forward UMBmark (--fwd-*)
     Command a forward move of D_cmd (computed by the CURRENT model) and measure
     the actual end pose (dx, dy, dyaw) of base_link.
         forward_scale = dx / D_cmd
     forward_scale > 1 means the robot overshoots -> the model under-estimates how
     far each wheel turn carries it -> the effective radius is LARGER than modeled
     (and vice-versa). dy and dyaw are the lateral / heading cross-coupling: ideal
     is zero; non-zero means asymmetric wheels, misalignment, or an H-matrix sign
     issue, and is reported as a diagnostic (not auto-corrected, since the fix is
     mechanical or structural, not a single scalar).

3. Pure-rotation UMBmark (--rot-*)
     Command a yaw theta_cmd and measure the actual yaw theta_act. The H-matrix
     rotation column is linear in (LEN_X + LEN_Y), so the commanded-vs-actual yaw
     ratio corrects exactly that lever arm:
         (LEN_X + LEN_Y)_eff = (LEN_X + LEN_Y)_nominal * theta_cmd / theta_act
     theta_act < theta_cmd (under-rotation) means the effective lever arm is
     larger than nominal, so the corrected sum goes UP. This isolates the
     rotational scaling from the radius error in (1)/(2).

WHERE THE OUTPUT GOES (IMPORTANT)
---------------------------------
The corrected constants must be applied to BOTH:
    * src/fortis_description/urdf/fortis_constants.xacro  (wheel_radius,
      wheel_x_offset / wheel_y_offset -- the URDF is the source of truth)
    * src/fortis_comms/fortis_comms/xdrive_kinematics.py  (WHEEL_RADIUS,
      LEN_X / LEN_Y)
test/test_kinematics_urdf_sync.py asserts the two sides match to 1e-4 m, so
editing only one side fails CI. This script never edits either; it prints.

Note on splitting the corrected (LEN_X + LEN_Y) back into LEN_X and LEN_Y: the
rotation test only constrains the SUM. Keep the nominal LEN_X : LEN_Y ratio
(0.176 : 0.125) unless a separate measurement says otherwise; this script prints
the suggested split on that assumption.

USAGE EXAMPLES
--------------
    # all three calibrations at once
    python3 tools/kinematic_calibration.py \
        --radius-distance 4.985 --radius-revs 7.81 \
        --fwd-cmd 2.0 --fwd-dx 2.07 --fwd-dy 0.015 --fwd-dyaw-deg 0.9 \
        --rot-cmd-deg 360 --rot-act-deg 351.4

    # just the wheel-radius sub-calibration
    python3 tools/kinematic_calibration.py --radius-distance 4.985 --radius-revs 7.81

    # override the nominals (e.g. after a prior calibration round)
    python3 tools/kinematic_calibration.py --rot-cmd-deg 360 --rot-act-deg 351.4 \
        --nominal-len-x 0.176 --nominal-len-y 0.125
"""

from __future__ import annotations

import argparse
import math

# Nominal geometry, mirrored from fortis_constants.xacro / xdrive_kinematics.py.
# Used as the baseline the corrections are reported against. CLI flags can
# override these so a second calibration round can build on the first.
NOMINAL_WHEEL_RADIUS = 0.1016
NOMINAL_LEN_X = 0.176
NOMINAL_LEN_Y = 0.125


def effective_wheel_radius(distance_m: float, revolutions: float) -> float:
    """Return effective rolling radius from a dead-straight roll.

    distance_m is the measured base_link ground travel; revolutions is the wheel
    turn count (ODrive pos_estimate delta, already in turns). One turn advances
    2*pi*r at the contact patch.
    """
    if revolutions == 0:
        raise ValueError("--radius-revs must be non-zero")
    return distance_m / (2.0 * math.pi * revolutions)


def forward_scale(commanded_m: float, measured_dx_m: float) -> float:
    """Return forward distance scale (actual / commanded along the drive axis).

    >1 means the robot overshoots, i.e. the model under-estimates travel per
    wheel turn and the true effective radius is larger than modeled.
    """
    if commanded_m == 0:
        raise ValueError("--fwd-cmd must be non-zero")
    return measured_dx_m / commanded_m


def corrected_lever_arm(
    nominal_sum_m: float, commanded_rad: float, actual_rad: float
) -> float:
    """Return corrected (LEN_X + LEN_Y) from a pure-rotation UMBmark.

    The H-matrix rotation column is linear in (LEN_X + LEN_Y), so the corrected
    lever arm scales by commanded/actual yaw. Under-rotation (actual < commanded)
    raises the effective lever arm.
    """
    if actual_rad == 0:
        raise ValueError("--rot-act-deg must be non-zero")
    return nominal_sum_m * commanded_rad / actual_rad


def _fmt(value: float, nominal: float, unit: str = "m") -> str:
    """Format a corrected value next to its nominal and percent delta."""
    pct = (value / nominal - 1.0) * 100.0 if nominal else float("nan")
    return f"{value:.6f} {unit}  (nominal {nominal:.6f} {unit}, {pct:+.2f}%)"


def _report_radius(args) -> None:
    """Print the effective-wheel-radius sub-calibration."""
    r_eff = effective_wheel_radius(args.radius_distance, args.radius_revs)
    print("== 1. Effective wheel radius ==")
    print(f"  measured: D = {args.radius_distance} m over N = {args.radius_revs} rev")
    print(f"  r_eff = D / (2*pi*N) = {_fmt(r_eff, NOMINAL_WHEEL_RADIUS)}")
    print("  -> corrected WHEEL_RADIUS / wheel_radius")
    print()


def _report_forward(args) -> None:
    """Print the pure-forward UMBmark scale + cross-coupling summary."""
    scale = forward_scale(args.fwd_cmd, args.fwd_dx)
    # Implied radius correction: a forward over/undershoot is a pure radius scale,
    # so the modeled radius times the scale is what the robot actually rolled on.
    implied_r = NOMINAL_WHEEL_RADIUS * scale
    print("== 2. Pure-forward UMBmark ==")
    print(f"  commanded D_cmd = {args.fwd_cmd} m")
    print(
        f"  measured end pose: dx = {args.fwd_dx} m, dy = {args.fwd_dy} m, "
        f"dyaw = {math.degrees(args.fwd_dyaw):.3f} deg"
    )
    print(f"  forward_scale = dx / D_cmd = {scale:.5f}  ({(scale - 1) * 100:+.2f}%)")
    print(f"  implied WHEEL_RADIUS from forward scale = {_fmt(implied_r, NOMINAL_WHEEL_RADIUS)}")
    # Cross-coupling: diagnostic only. Express lateral drift and heading drift
    # relative to the commanded distance so they are comparable across runs.
    lateral_frac = args.fwd_dy / args.fwd_cmd if args.fwd_cmd else float("nan")
    print(
        f"  cross-coupling (should be ~0): lateral dy/D_cmd = {lateral_frac:+.4f} "
        f"({lateral_frac * 100:+.2f}%), heading dyaw = {math.degrees(args.fwd_dyaw):+.3f} deg"
    )
    print("  note: cross-coupling is a mechanical/sign diagnostic, not a single-scalar fix.")
    print()


def _report_rotation(args) -> None:
    """Print the pure-rotation UMBmark lever-arm correction."""
    nominal_sum = args.nominal_len_x + args.nominal_len_y
    cmd_rad = math.radians(args.rot_cmd_deg)
    act_rad = math.radians(args.rot_act_deg)
    eff_sum = corrected_lever_arm(nominal_sum, cmd_rad, act_rad)
    # Split back to LEN_X / LEN_Y on the nominal ratio (rotation only constrains
    # the sum; the ratio is unobservable from a pure-yaw test).
    ratio_x = args.nominal_len_x / nominal_sum
    eff_len_x = eff_sum * ratio_x
    eff_len_y = eff_sum * (1.0 - ratio_x)
    print("== 3. Pure-rotation UMBmark ==")
    print(f"  commanded {args.rot_cmd_deg} deg, actual {args.rot_act_deg} deg")
    print(f"  (LEN_X+LEN_Y)_eff = sum * cmd/act = {_fmt(eff_sum, nominal_sum)}")
    print(
        f"  suggested split on nominal ratio: "
        f"LEN_X = {_fmt(eff_len_x, args.nominal_len_x)}"
    )
    print(f"                                    LEN_Y = {_fmt(eff_len_y, args.nominal_len_y)}")
    print("  -> corrected LEN_X/LEN_Y (wheel_x_offset/wheel_y_offset)")
    print()


def _apply_note() -> None:
    """Print the dual-location apply reminder (sync test depends on it)."""
    print("== APPLY TO BOTH SOURCES (or test_kinematics_urdf_sync.py fails) ==")
    print("  1. src/fortis_description/urdf/fortis_constants.xacro")
    print("       wheel_radius, wheel_x_offset, wheel_y_offset  (URDF = source of truth)")
    print("  2. src/fortis_comms/fortis_comms/xdrive_kinematics.py")
    print("       WHEEL_RADIUS, LEN_X, LEN_Y")
    print("  The sync test asserts the two sides match to 1e-4 m; edit both together.")


def build_parser() -> argparse.ArgumentParser:
    """Build the argparse parser; each sub-calibration's args are optional."""
    parser = argparse.ArgumentParser(
        description=(
            "Compute corrected X-drive kinematic constants from empirical "
            "drive measurements. Run any subset of the three sub-calibrations."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    radius = parser.add_argument_group("1. effective wheel radius (dead-straight roll)")
    radius.add_argument(
        "--radius-distance", type=float, metavar="D_M",
        help="measured base_link ground travel, meters",
    )
    radius.add_argument(
        "--radius-revs", type=float, metavar="N",
        help="wheel revolutions over that travel (ODrive pos_estimate delta, turns)",
    )

    fwd = parser.add_argument_group("2. pure-forward UMBmark")
    fwd.add_argument("--fwd-cmd", type=float, metavar="M", help="commanded forward distance, m")
    fwd.add_argument("--fwd-dx", type=float, metavar="M", help="measured forward travel dx, m")
    fwd.add_argument(
        "--fwd-dy", type=float, default=0.0, metavar="M",
        help="measured lateral drift dy, m (cross-coupling diagnostic, default 0)",
    )
    fwd.add_argument(
        "--fwd-dyaw-deg", type=float, default=0.0, dest="fwd_dyaw_deg", metavar="DEG",
        help="measured heading drift, degrees (cross-coupling diagnostic, default 0)",
    )

    rot = parser.add_argument_group("3. pure-rotation UMBmark")
    rot.add_argument("--rot-cmd-deg", type=float, metavar="DEG", help="commanded yaw, degrees")
    rot.add_argument(
        "--rot-act-deg", type=float, metavar="DEG", help="measured actual yaw, degrees"
    )

    nominal = parser.add_argument_group("nominal overrides (default to CAD constants)")
    nominal.add_argument("--nominal-wheel-radius", type=float, default=NOMINAL_WHEEL_RADIUS)
    nominal.add_argument("--nominal-len-x", type=float, default=NOMINAL_LEN_X)
    nominal.add_argument("--nominal-len-y", type=float, default=NOMINAL_LEN_Y)

    return parser


def main(argv=None) -> int:
    """Parse args, run the requested sub-calibrations, print results."""
    args = build_parser().parse_args(argv)
    # Let nominal overrides flow into the module-level baselines used by helpers.
    global NOMINAL_WHEEL_RADIUS, NOMINAL_LEN_X, NOMINAL_LEN_Y
    NOMINAL_WHEEL_RADIUS = args.nominal_wheel_radius
    NOMINAL_LEN_X = args.nominal_len_x
    NOMINAL_LEN_Y = args.nominal_len_y
    args.fwd_dyaw = math.radians(args.fwd_dyaw_deg)

    ran_any = False

    if args.radius_distance is not None and args.radius_revs is not None:
        _report_radius(args)
        ran_any = True
    elif (args.radius_distance is None) != (args.radius_revs is None):
        raise SystemExit("radius calibration needs BOTH --radius-distance and --radius-revs")

    if args.fwd_cmd is not None and args.fwd_dx is not None:
        _report_forward(args)
        ran_any = True
    elif (args.fwd_cmd is None) != (args.fwd_dx is None):
        raise SystemExit("forward calibration needs BOTH --fwd-cmd and --fwd-dx")

    if args.rot_cmd_deg is not None and args.rot_act_deg is not None:
        _report_rotation(args)
        ran_any = True
    elif (args.rot_cmd_deg is None) != (args.rot_act_deg is None):
        raise SystemExit("rotation calibration needs BOTH --rot-cmd-deg and --rot-act-deg")

    if not ran_any:
        raise SystemExit(
            "No complete measurement set given. Provide at least one of:\n"
            "  --radius-distance + --radius-revs\n"
            "  --fwd-cmd + --fwd-dx\n"
            "  --rot-cmd-deg + --rot-act-deg\n"
            "Run with -h for full help."
        )

    _apply_note()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
