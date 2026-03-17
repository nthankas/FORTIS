#!/usr/bin/env python3
"""
turn_calculator.py -- Analytical calculator for FORTIS straddle-turn feasibility.

Input your robot parameters, get:
  - Whether the 90-degree straddle turn is geometrically possible
  - Maximum rotation before first wheel hits the step
  - Minimum torque needed to climb the step
  - Floor fit checks
  - Tunnel fit checks

Run: python turn_calculator.py
  or: python turn_calculator.py --interactive
  or: python turn_calculator.py --wr 4.0 --ww 3.0 --wb 12 --tw 10 --mass 40

No Isaac Sim required.
"""

import math
import argparse
import sys

# ---------------------------------------------------------------------------
# Reactor geometry (fixed, from STL measurements)
# ---------------------------------------------------------------------------
STEP_R = 55.5          # step radial position (inches)
STEP_HEIGHT = 4.5      # step height (inches)
R_INNER_MIN = 45.8     # inner floor inner edge
R_INNER_MAX = 54.8     # inner floor outer edge
R_OUTER_MIN = 56.2     # outer floor inner edge
R_OUTER_MAX = 70.3     # outer floor outer edge
INNER_FLOOR_WIDTH = R_INNER_MAX - R_INNER_MIN  # 9.0"
OUTER_FLOOR_WIDTH = R_OUTER_MAX - R_OUTER_MIN  # 14.1"

TUNNEL_SIZE = 14.75    # usable tunnel dimension (inches)
CHASSIS_HEIGHT = 5.0   # chassis height (inches, fixed)
ARM_BASE_HEIGHT = 2.0  # arm base stowed on top (inches)

GRAPHITE_MU_STATIC = 0.4   # rubber on graphite static friction
GRAPHITE_MU_DYNAMIC = 0.3  # rubber on graphite dynamic friction
GRAVITY = 9.81              # m/s^2
IN_TO_M = 0.0254            # inches to meters


def analyze_config(wheel_radius_in, wheel_width_in, wheelbase_in,
                   track_width_in, chassis_mass_lbs, chamfer_in=1.0,
                   verbose=True):
    """
    Analyze whether a given robot configuration can complete a 90-degree
    straddle turn on the reactor step.

    Returns dict with all analysis results.
    """
    results = {
        "inputs": {
            "wheel_radius_in": wheel_radius_in,
            "wheel_width_in": wheel_width_in,
            "wheelbase_in": wheelbase_in,
            "track_width_in": track_width_in,
            "chassis_mass_lbs": chassis_mass_lbs,
            "chamfer_in": chamfer_in,
        },
        "checks": {},
        "feasible": True,
        "issues": [],
    }

    wr = wheel_radius_in
    ww = wheel_width_in
    wb = wheelbase_in
    tw = track_width_in
    mass_lbs = chassis_mass_lbs
    mass_kg = mass_lbs * 0.4536
    wheel_mass_kg = 1.13  # per wheel

    total_mass_kg = mass_kg + 4 * wheel_mass_kg + 1.0  # chassis + 4 wheels + arm

    # -----------------------------------------------------------------------
    # 1. Tunnel constraints
    # -----------------------------------------------------------------------
    stowed_height = CHASSIS_HEIGHT + wr + ARM_BASE_HEIGHT
    chassis_width = tw - ww - 0.5  # space between inner wheel faces
    stowed_width = max(chassis_width, tw)  # widest dimension (wheels fold for transit)

    height_ok = stowed_height <= TUNNEL_SIZE
    results["checks"]["stowed_height"] = {
        "value": round(stowed_height, 1),
        "limit": TUNNEL_SIZE,
        "pass": height_ok,
    }
    if not height_ok:
        results["feasible"] = False
        results["issues"].append(f"Stowed height {stowed_height:.1f}\" > tunnel {TUNNEL_SIZE}\"")

    # -----------------------------------------------------------------------
    # 2. Floor fit constraints
    # -----------------------------------------------------------------------
    wx = wb / 2  # half wheelbase (radial offset of axles from robot center)
    wy = tw / 2  # half track width

    rear_r = STEP_R - wx   # rear axle radial position (inner floor)
    front_r = STEP_R + wx  # front axle radial position (outer floor)

    # Rear wheel must fit on inner floor
    rear_inner_edge = rear_r - wr  # innermost extent of rear wheel
    rear_outer_edge = rear_r + wr  # outermost extent of rear wheel

    rear_inner_ok = rear_inner_edge >= R_INNER_MIN
    rear_outer_ok = rear_outer_edge <= R_INNER_MAX + 0.5  # 0.5" overhang allowed
    rear_clearance = min(rear_r - R_INNER_MIN, R_INNER_MAX - rear_r) - wr

    results["checks"]["rear_wheel_inner_edge"] = {
        "value": round(rear_inner_edge, 1),
        "limit": R_INNER_MIN,
        "clearance": round(rear_inner_edge - R_INNER_MIN, 1),
        "pass": rear_inner_ok,
    }
    results["checks"]["rear_wheel_outer_edge"] = {
        "value": round(rear_outer_edge, 1),
        "limit": R_INNER_MAX,
        "clearance": round(R_INNER_MAX - rear_outer_edge, 1),
        "pass": rear_outer_ok,
    }

    if not rear_inner_ok:
        results["issues"].append(
            f"Rear wheel inner edge at r={rear_inner_edge:.1f}\" extends past "
            f"inner floor boundary ({R_INNER_MIN}\")")
    if not rear_outer_ok:
        results["issues"].append(
            f"Rear wheel outer edge at r={rear_outer_edge:.1f}\" extends past "
            f"step ({R_INNER_MAX}\")")

    # Front wheel must fit on outer floor
    front_inner_edge = front_r - wr
    front_outer_edge = front_r + wr

    front_inner_ok = front_inner_edge >= R_OUTER_MIN - 0.5
    front_outer_ok = front_outer_edge <= R_OUTER_MAX

    results["checks"]["front_wheel_outer_floor"] = {
        "front_r": round(front_r, 1),
        "inner_edge": round(front_inner_edge, 1),
        "outer_edge": round(front_outer_edge, 1),
        "pass": front_inner_ok and front_outer_ok,
    }

    if not (rear_inner_ok and rear_outer_ok and front_inner_ok and front_outer_ok):
        results["feasible"] = False

    # -----------------------------------------------------------------------
    # 3. Crossing angle -- rotation before first wheel hits step
    # -----------------------------------------------------------------------
    d2 = wx**2 + wy**2
    d = math.sqrt(d2)
    rhs = -d2 / (2 * STEP_R)

    # RR wheel crossing (inner floor → needs to climb UP)
    # -wx*cos(θ) + wy*sin(θ) = rhs
    R_solve = d
    phi = math.atan2(wx, wy)
    sin_val = rhs / R_solve
    if abs(sin_val) <= 1:
        theta_rr = math.degrees(phi + math.asin(sin_val))
        if theta_rr < 0:
            theta_rr += 360
    else:
        theta_rr = 0

    # FL wheel crossing (outer floor → steps DOWN, easier)
    # wx*cos(θ) - wy*sin(θ) = rhs
    # Equivalent: R*sin(-(θ - phi)) = rhs → different solution
    phi_fl = math.atan2(wx, wy)
    # wx*cos(θ) - wy*sin(θ) = rhs → -(wy*sin(θ) - wx*cos(θ)) = rhs
    # → wy*sin(θ) - wx*cos(θ) = -rhs
    sin_val_fl = rhs / R_solve  # same magnitude
    # Solving: R*sin(θ - phi_fl) = -rhs
    if abs(rhs / R_solve) <= 1:
        theta_fl = math.degrees(phi_fl - math.asin(rhs / R_solve))
        if theta_fl < 0:
            theta_fl += 360
        if theta_fl > 180:
            theta_fl -= 360
    else:
        theta_fl = 0

    first_crossing = min(theta_rr, theta_fl) if theta_rr > 0 and theta_fl > 0 else max(theta_rr, theta_fl)

    results["crossing_analysis"] = {
        "RR_crossing_deg": round(theta_rr, 1),
        "FL_crossing_deg": round(theta_fl, 1),
        "first_crossing_deg": round(first_crossing, 1),
        "remaining_after_crossing": round(90 - first_crossing, 1),
    }

    # -----------------------------------------------------------------------
    # 4. Step climbing feasibility
    # -----------------------------------------------------------------------
    effective_step = STEP_HEIGHT - chamfer_in
    can_mechanically_climb = wr >= effective_step
    climb_margin = wr - effective_step

    results["climb_analysis"] = {
        "step_height_in": STEP_HEIGHT,
        "chamfer_in": chamfer_in,
        "effective_step_in": round(effective_step, 1),
        "wheel_radius_in": wr,
        "can_climb": can_mechanically_climb,
        "margin_in": round(climb_margin, 1),
    }

    if not can_mechanically_climb and first_crossing < 90:
        results["feasible"] = False
        results["issues"].append(
            f"Wheel radius {wr}\" < effective step {effective_step}\" "
            f"(cannot climb step at {first_crossing:.0f}° rotation)")

    # -----------------------------------------------------------------------
    # 5. Torque analysis
    # -----------------------------------------------------------------------
    # Step climbing torque (per wheel)
    if wr >= effective_step and effective_step > 0:
        # Classic step climbing: τ = F_normal * sqrt(2*R*H - H²)
        # where F_normal = wheel load = total_weight / 4
        R_m = wr * IN_TO_M
        H_m = effective_step * IN_TO_M
        wheel_load_N = (total_mass_kg * GRAVITY) / 4
        climb_moment_arm = math.sqrt(max(0, 2 * R_m * H_m - H_m**2))
        torque_climb = wheel_load_N * climb_moment_arm
    else:
        torque_climb = float('inf')
        climb_moment_arm = 0

    # Flat ground spinning torque (all 4 wheels scrubbing)
    # Scrub torque = μ * N * r_scrub, where r_scrub ≈ wheel_width/2
    r_scrub = (ww / 2) * IN_TO_M
    scrub_torque_per_wheel = GRAPHITE_MU_DYNAMIC * (total_mass_kg * GRAVITY / 4) * r_scrub

    # Total torque needed = max(climb, scrub) with safety margin
    min_torque = max(torque_climb, scrub_torque_per_wheel) * 1.5  # 50% safety margin

    results["torque_analysis"] = {
        "wheel_load_N": round(total_mass_kg * GRAVITY / 4, 1),
        "climb_torque_per_wheel_Nm": round(torque_climb, 1) if torque_climb < 1000 else "impossible",
        "scrub_torque_per_wheel_Nm": round(scrub_torque_per_wheel, 1),
        "recommended_min_torque_Nm": round(min_torque, 1) if min_torque < 1000 else "impossible",
    }

    # -----------------------------------------------------------------------
    # 6. Stability analysis
    # -----------------------------------------------------------------------
    # Tilt angle from step height difference
    # The step creates a height difference between left and right (or front and rear) wheels
    # When straddling front-to-back: tilt around Y axis
    tilt_rad = math.atan2(STEP_HEIGHT * IN_TO_M,
                          wb * IN_TO_M)  # simplification
    tilt_deg = math.degrees(tilt_rad)

    # Tip-over risk: if tilt exceeds critical angle
    # Critical angle ≈ arctan(track_width / (2 * CG_height))
    cg_height = (CHASSIS_HEIGHT / 2 + wr) * IN_TO_M  # CG above ground
    critical_angle = math.degrees(math.atan2(tw / 2 * IN_TO_M, cg_height))

    results["stability"] = {
        "settled_tilt_deg": round(tilt_deg, 1),
        "critical_tip_angle_deg": round(critical_angle, 1),
        "stability_margin_deg": round(critical_angle - tilt_deg, 1),
        "stable": critical_angle > tilt_deg * 1.5,
    }

    # -----------------------------------------------------------------------
    # 7. Overall verdict
    # -----------------------------------------------------------------------
    if first_crossing >= 90:
        results["verdict"] = "EASY - All wheels stay on their floors for full 90° turn"
    elif can_mechanically_climb and first_crossing >= 30:
        results["verdict"] = "POSSIBLE - Wheels can climb step after crossing"
    elif first_crossing >= 60:
        results["verdict"] = "MARGINAL - High crossing angle but wheel too small to climb"
    else:
        results["verdict"] = "UNLIKELY - Low crossing angle and/or can't climb step"

    # -----------------------------------------------------------------------
    # Print summary
    # -----------------------------------------------------------------------
    if verbose:
        print(f"\n{'='*65}")
        print(f"  FORTIS Straddle Turn Analysis")
        print(f"{'='*65}")
        print(f"  Wheel: R={wr}\" W={ww}\"")
        print(f"  Wheelbase: {wb}\"  Track: {tw}\"")
        print(f"  Mass: {mass_lbs} lbs ({total_mass_kg:.1f} kg total)")
        print(f"  Step chamfer: {chamfer_in}\"")
        print(f"{'='*65}")

        print(f"\n  TUNNEL FIT:")
        print(f"    Stowed height: {stowed_height:.1f}\" / {TUNNEL_SIZE}\" "
              f"{'PASS' if height_ok else 'FAIL'}")

        print(f"\n  FLOOR FIT:")
        print(f"    Rear axle at r={rear_r:.1f}\", wheel {wr}\" radius")
        print(f"    Inner edge: r={rear_inner_edge:.1f}\" (floor starts at {R_INNER_MIN}\") "
              f"{'PASS' if rear_inner_ok else 'FAIL'}")
        print(f"    Outer edge: r={rear_outer_edge:.1f}\" (step at {R_INNER_MAX}\") "
              f"{'PASS' if rear_outer_ok else 'FAIL'}")
        print(f"    Front axle at r={front_r:.1f}\" (outer floor)")

        print(f"\n  CROSSING ANGLE:")
        print(f"    First wheel crosses step at: {first_crossing:.1f}°")
        print(f"    (RR at {theta_rr:.1f}°, FL at {theta_fl:.1f}°)")
        print(f"    Remaining after crossing: {90 - first_crossing:.1f}°")

        print(f"\n  STEP CLIMBING:")
        print(f"    Step: {STEP_HEIGHT}\" - {chamfer_in}\" chamfer = {effective_step}\" effective")
        print(f"    Wheel radius: {wr}\" {'>=' if can_mechanically_climb else '<'} "
              f"{effective_step}\" effective step")
        print(f"    Can climb: {'YES' if can_mechanically_climb else 'NO'} "
              f"(margin: {climb_margin:+.1f}\")")

        print(f"\n  TORQUE:")
        if torque_climb < 1000:
            print(f"    Step climb: {torque_climb:.1f} Nm/wheel")
        else:
            print(f"    Step climb: IMPOSSIBLE (R < H)")
        print(f"    Scrub resistance: {scrub_torque_per_wheel:.1f} Nm/wheel")
        if min_torque < 1000:
            print(f"    Recommended (1.5x safety): {min_torque:.1f} Nm/wheel")

        print(f"\n  STABILITY:")
        print(f"    Tilt from step: {tilt_deg:.1f}°")
        print(f"    Tip-over angle: {critical_angle:.1f}°")
        print(f"    Margin: {critical_angle - tilt_deg:.1f}°")

        print(f"\n  VERDICT: {results['verdict']}")

        if results["issues"]:
            print(f"\n  ISSUES:")
            for issue in results["issues"]:
                print(f"    - {issue}")

        print(f"{'='*65}\n")

    return results


def find_best_configs(chamfer_in=1.0, verbose=True):
    """Sweep parameter space and find the best configurations."""

    if verbose:
        print(f"\n{'='*65}")
        print(f"  Searching for feasible configurations...")
        print(f"  Step: {STEP_HEIGHT}\", Chamfer: {chamfer_in}\", "
              f"Effective: {STEP_HEIGHT - chamfer_in}\"")
        print(f"{'='*65}\n")

    feasible = []
    total = 0

    for wr in [x * 0.25 for x in range(12, 19)]:  # 3.0 to 4.5 in 0.25 steps
        for ww in [x * 0.5 for x in range(4, 9)]:  # 2.0 to 4.0 in 0.5 steps
            for wb in range(8, 16):                  # 8 to 15 inches
                for tw in range(7, 14):              # 7 to 13 inches
                    total += 1
                    res = analyze_config(wr, ww, wb, tw, 40,
                                        chamfer_in=chamfer_in, verbose=False)
                    if (res["feasible"] and
                        res["crossing_analysis"]["first_crossing_deg"] >= 25):
                        score = (res["crossing_analysis"]["first_crossing_deg"] +
                                (10 if res["climb_analysis"]["can_climb"] else 0) +
                                res["stability"]["stability_margin_deg"] * 0.5)
                        feasible.append((score, res))

    feasible.sort(key=lambda x: x[0], reverse=True)

    if verbose:
        print(f"Searched {total} configs, found {len(feasible)} feasible\n")

        if feasible:
            print(f"Top 20 configurations:")
            print(f"{'R':>5} {'W':>5} {'WB':>4} {'TW':>4} {'Cross':>6} "
                  f"{'Climb':>6} {'Torque':>7} {'Tilt':>5} {'Verdict'}")
            print("-" * 70)
            for score, r in feasible[:20]:
                inp = r["inputs"]
                cross = r["crossing_analysis"]["first_crossing_deg"]
                climb = "YES" if r["climb_analysis"]["can_climb"] else "no"
                trq = r["torque_analysis"]["recommended_min_torque_Nm"]
                tilt = r["stability"]["settled_tilt_deg"]
                print(f"{inp['wheel_radius_in']:5.2f} {inp['wheel_width_in']:5.1f} "
                      f"{inp['wheelbase_in']:4d} {inp['track_width_in']:4d} "
                      f"{cross:5.1f}° {climb:>5} {trq:>6} "
                      f"{tilt:4.1f}° {r['verdict'][:30]}")
        else:
            print("No feasible configurations found!")
            print("Try increasing chamfer size or relaxing constraints.")

    return feasible


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FORTIS straddle turn calculator")
    parser.add_argument("--wr", type=float, help="Wheel radius (inches)")
    parser.add_argument("--ww", type=float, help="Wheel width (inches)")
    parser.add_argument("--wb", type=int, help="Wheelbase - front/rear axle distance (inches)")
    parser.add_argument("--tw", type=int, help="Track width - left/right wheel center distance (inches)")
    parser.add_argument("--mass", type=float, default=40, help="Chassis mass (lbs)")
    parser.add_argument("--chamfer", type=float, default=1.0, help="Step chamfer size (inches)")
    parser.add_argument("--search", action="store_true", help="Search for best configs")
    parser.add_argument("--interactive", action="store_true", help="Interactive mode")
    args = parser.parse_args()

    if args.search:
        find_best_configs(chamfer_in=args.chamfer)
    elif args.wr and args.ww and args.wb and args.tw:
        analyze_config(args.wr, args.ww, args.wb, args.tw, args.mass,
                       chamfer_in=args.chamfer)
    elif args.interactive:
        print("\nFORTIS Straddle Turn Calculator")
        print("Enter 'q' to quit, 'search' to find best configs\n")
        while True:
            try:
                line = input(">> Enter: wheel_R wheel_W wheelbase track [mass] [chamfer]: ").strip()
                if line.lower() in ('q', 'quit', 'exit'):
                    break
                if line.lower() == 'search':
                    chamfer = float(input("   Chamfer size (inches, default 1.0): ") or "1.0")
                    find_best_configs(chamfer_in=chamfer)
                    continue
                parts = line.split()
                if len(parts) < 4:
                    print("   Need at least 4 values: wheel_R wheel_W wheelbase track")
                    continue
                wr = float(parts[0])
                ww = float(parts[1])
                wb = int(parts[2])
                tw = int(parts[3])
                mass = float(parts[4]) if len(parts) > 4 else 40
                chamfer = float(parts[5]) if len(parts) > 5 else 1.0
                analyze_config(wr, ww, wb, tw, mass, chamfer_in=chamfer)
            except (ValueError, KeyboardInterrupt):
                print()
                break
    else:
        # Default: show a few example configs and search
        print("\n--- Example: Default Phase 1 config ---")
        analyze_config(3.0, 2.0, 12, 13, 50, chamfer_in=1.0)

        print("\n--- Example: Optimized for turn ---")
        analyze_config(4.0, 3.0, 10, 8, 40, chamfer_in=1.0)

        print("\n--- Full search ---")
        find_best_configs(chamfer_in=1.0)

        print("\nUsage:")
        print(f"  python {sys.argv[0]} --wr 4.0 --ww 3.0 --wb 10 --tw 8 --mass 40")
        print(f"  python {sys.argv[0]} --search --chamfer 1.5")
        print(f"  python {sys.argv[0]} --interactive")
