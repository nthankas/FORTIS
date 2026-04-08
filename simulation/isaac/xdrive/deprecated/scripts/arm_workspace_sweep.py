"""
Phase 1: Analytical arm workspace sweep.

Pure Python + numpy (no Isaac Sim).  Sweeps a cylindrical (r, z) grid of EE
targets, solves analytical IK, filters collisions / joint limits, computes
gravity torques and tipping margins.  Outputs CSV + JSON summary + matplotlib
workspace maps.

Usage:
  python arm_workspace_sweep.py --36arm --step
  python arm_workspace_sweep.py --30arm --armloaded --step --coarse
  python arm_workspace_sweep.py --24arm --reactor
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

# Add lib/ to path
_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_XDRIVE_ROOT = os.path.abspath(os.path.join(_TOOLS_DIR, ".."))
sys.path.insert(0, os.path.join(_XDRIVE_ROOT, "lib"))

from arm_ik import (
    IN, GRAV, ARM_CONFIGS,
    CHASSIS_H, CHASSIS_L, J1_STACK_H, BELLY_HEIGHT,
    get_arm_params, fk_planar, ik_planar, gravity_torques,
    check_collisions, tipping_sweep_j1,
)

# ---- Grid defaults ----
FINE_R_STEPS = 30
FINE_Z_STEPS = 30
COARSE_R_STEPS = 8
COARSE_Z_STEPS = 8
EE_PITCHES_DEG = [-90.0, -60.0, -45.0, -30.0, 0.0, 30.0, 45.0, 60.0, 90.0]
COARSE_PITCHES_DEG = [-90.0, -45.0, 0.0, 45.0, 90.0]
J1_TIPPING_STEP = 15  # degrees


def compute_z_j2(env):
    """J2 pivot height above the reference floor (m)."""
    # Chassis bottom sits at BELLY_HEIGHT above floor.
    # J2 = chassis top + J1 stack.
    base = BELLY_HEIGHT + CHASSIS_H + J1_STACK_H
    if env == "step":
        # Robot straddles the step.  Back wheels on outer floor (z=0),
        # front wheels on inner floor (z = -4.5").  Use the outer-floor
        # height (conservative for floor collision).
        return base
    else:  # reactor
        return base


def build_grid(arm_params, env, coarse=False):
    """Return (r_range, z_range, ee_pitches_deg) arrays."""
    max_reach = arm_params["max_reach"]
    n_r = COARSE_R_STEPS if coarse else FINE_R_STEPS
    n_z = COARSE_Z_STEPS if coarse else FINE_Z_STEPS
    pitches = COARSE_PITCHES_DEG if coarse else EE_PITCHES_DEG

    z_j2 = compute_z_j2(env)

    # Radial: from small positive offset to max reach
    r_min = 0.05
    r_max = max_reach * 0.98  # stay slightly inside kinematic limit
    r_range = np.linspace(r_min, r_max, n_r)

    # Vertical: from below J2 (arm reaching down near floor) to above
    # z is relative to J2.  Lowest: -(z_j2 - margin) i.e. near floor.
    # Highest: max_reach above J2.
    z_min = -(z_j2 - 0.05)  # 5cm above floor
    z_max = max_reach * 0.98
    z_range = np.linspace(z_min, z_max, n_z)

    return r_range, z_range, pitches


def run_sweep(arm_params, env, coarse=False):
    """Run the full analytical workspace sweep. Returns list of result dicts."""
    r_range, z_range, pitches = build_grid(arm_params, env, coarse)
    z_j2 = compute_z_j2(env)
    L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]

    j1_angles = list(range(0, 360, J1_TIPPING_STEP))
    n_targets = len(r_range) * len(z_range) * len(pitches)

    results = []
    n_ik_fail = 0
    n_joint_limit = 0
    n_collision = 0
    n_valid = 0

    t0 = time.time()

    for r in r_range:
        for z in z_range:
            for phi_deg in pitches:
                phi_rad = math.radians(phi_deg)
                solutions = ik_planar(r, z, phi_rad, L2, L3, L4)

                if not solutions:
                    n_ik_fail += 1
                    continue

                for sol_idx, sol in enumerate(solutions):
                    j2, j3, j4 = sol["j2"], sol["j3"], sol["j4"]

                    # Joint limits: +/-180 deg
                    if any(abs(a) > math.pi for a in [j2, j3, j4]):
                        n_joint_limit += 1
                        continue

                    # Collision check
                    col = check_collisions(j2, j3, j4, arm_params, z_j2)
                    if not col["valid"]:
                        n_collision += 1
                        continue

                    # FK for actual EE position
                    fk = fk_planar(j2, j3, j4, L2, L3, L4)

                    # Gravity torques
                    torques = gravity_torques(j2, j3, j4, arm_params)

                    # Tipping sweep across J1
                    tip = tipping_sweep_j1(j2, j3, j4, arm_params, j1_angles)

                    n_valid += 1
                    results.append({
                        "target_r": round(r, 5),
                        "target_z": round(z, 5),
                        "ee_pitch_deg": phi_deg,
                        "solution_idx": sol_idx,
                        "j2_deg": round(math.degrees(j2), 3),
                        "j3_deg": round(math.degrees(j3), 3),
                        "j4_deg": round(math.degrees(j4), 3),
                        "ee_r_m": round(fk["ee"][0], 5),
                        "ee_z_m": round(fk["ee"][1], 5),
                        "tau_j2": round(torques["tau_j2"], 4),
                        "tau_j3": round(torques["tau_j3"], 4),
                        "tau_j4": round(torques["tau_j4"], 4),
                        "tau_max": round(max(abs(torques["tau_j2"]),
                                             abs(torques["tau_j3"]),
                                             abs(torques["tau_j4"])), 4),
                        "tip_worst_margin": round(tip["worst_margin"], 5),
                        "tip_worst_j1_deg": tip["worst_j1_deg"],
                        "tip_stable_all": tip["stable_all"],
                        "tip_n_stable": tip["n_stable"],
                        "tip_n_total": tip["n_total"],
                    })

    elapsed = time.time() - t0

    stats = {
        "n_targets": n_targets,
        "n_ik_fail": n_ik_fail,
        "n_joint_limit": n_joint_limit,
        "n_collision": n_collision,
        "n_valid": n_valid,
        "elapsed_s": round(elapsed, 2),
    }
    return results, stats


def config_label(reach_in, loaded, env):
    load_tag = "loaded" if loaded else "bare"
    return f"{reach_in}in_{load_tag}_{env}"


def write_csv(results, path):
    if not results:
        return
    keys = list(results[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(results)


def write_summary(results, stats, arm_params, env, path):
    """Write JSON summary with stats, peak torques, workspace bounds."""
    if not results:
        summary = {"stats": stats, "arm": arm_params, "env": env}
        with open(path, "w") as f:
            json.dump(summary, f, indent=2, default=str)
        return

    tau_j2 = [abs(r["tau_j2"]) for r in results]
    tau_j3 = [abs(r["tau_j3"]) for r in results]
    tau_j4 = [abs(r["tau_j4"]) for r in results]
    tau_max = [r["tau_max"] for r in results]
    ee_r = [r["ee_r_m"] for r in results]
    ee_z = [r["ee_z_m"] for r in results]
    margins = [r["tip_worst_margin"] for r in results]

    summary = {
        "stats": stats,
        "arm_config": {
            "reach_in": arm_params["reach_in"],
            "loaded": arm_params["loaded"],
            "L2_m": round(arm_params["L2"], 4),
            "L3_m": round(arm_params["L3"], 4),
            "L4_m": round(arm_params["L4"], 4),
            "max_reach_m": round(arm_params["max_reach"], 4),
        },
        "env": env,
        "torques": {
            "j2_peak_Nm": round(max(tau_j2), 3),
            "j2_mean_Nm": round(np.mean(tau_j2), 3),
            "j2_p95_Nm": round(np.percentile(tau_j2, 95), 3),
            "j3_peak_Nm": round(max(tau_j3), 3),
            "j3_mean_Nm": round(np.mean(tau_j3), 3),
            "j3_p95_Nm": round(np.percentile(tau_j3, 95), 3),
            "j4_peak_Nm": round(max(tau_j4), 3),
            "j4_mean_Nm": round(np.mean(tau_j4), 3),
            "j4_p95_Nm": round(np.percentile(tau_j4, 95), 3),
            "any_peak_Nm": round(max(tau_max), 3),
        },
        "workspace": {
            "ee_r_min_m": round(min(ee_r), 4),
            "ee_r_max_m": round(max(ee_r), 4),
            "ee_z_min_m": round(min(ee_z), 4),
            "ee_z_max_m": round(max(ee_z), 4),
        },
        "tipping": {
            "worst_margin_m": round(min(margins), 5),
            "n_always_stable": sum(1 for r in results if r["tip_stable_all"]),
            "n_sometimes_unstable": sum(1 for r in results if not r["tip_stable_all"]),
            "pct_always_stable": round(
                100.0 * sum(1 for r in results if r["tip_stable_all"]) / len(results), 1),
        },
    }
    with open(path, "w") as f:
        json.dump(summary, f, indent=2)


def plot_workspace(results, label, out_dir):
    """Generate workspace visualisation plots."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import Normalize
        from matplotlib import cm
    except ImportError:
        print("  matplotlib not available, skipping plots")
        return

    if not results:
        return

    r = np.array([x["ee_r_m"] for x in results]) / IN  # convert to inches
    z = np.array([x["ee_z_m"] for x in results]) / IN
    tau2 = np.array([abs(x["tau_j2"]) for x in results])
    tau3 = np.array([abs(x["tau_j3"]) for x in results])
    tau_max = np.array([x["tau_max"] for x in results])
    stable = np.array([x["tip_stable_all"] for x in results])

    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle(f"Arm Workspace: {label}", fontsize=14, fontweight="bold")

    # 1. Reachable workspace (all valid poses)
    ax = axes[0, 0]
    ax.scatter(r, z, c="steelblue", s=2, alpha=0.3)
    ax.set_xlabel("Radial distance from J2 (in)")
    ax.set_ylabel("Height relative to J2 (in)")
    ax.set_title(f"Reachable workspace ({len(results)} valid poses)")
    ax.set_aspect("equal")
    ax.axhline(0, color="gray", ls="--", lw=0.5)
    ax.grid(True, alpha=0.3)

    # 2. Workspace coloured by max torque
    ax = axes[0, 1]
    sc = ax.scatter(r, z, c=tau_max, s=3, alpha=0.4, cmap="hot_r",
                    vmin=0, vmax=min(25, np.percentile(tau_max, 99)))
    ax.set_xlabel("Radial distance from J2 (in)")
    ax.set_ylabel("Height relative to J2 (in)")
    ax.set_title("Max holding torque (Nm)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    plt.colorbar(sc, ax=ax, label="Nm")

    # 3. J2 torque
    ax = axes[1, 0]
    sc = ax.scatter(r, z, c=tau2, s=3, alpha=0.4, cmap="hot_r",
                    vmin=0, vmax=min(25, np.percentile(tau2, 99)))
    ax.set_xlabel("Radial distance from J2 (in)")
    ax.set_ylabel("Height relative to J2 (in)")
    ax.set_title("J2 holding torque (Nm)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    plt.colorbar(sc, ax=ax, label="Nm")

    # 4. Tipping stability
    ax = axes[1, 1]
    colors = np.where(stable, "green", "red")
    ax.scatter(r[stable], z[stable], c="green", s=3, alpha=0.3, label="Stable all J1")
    ax.scatter(r[~stable], z[~stable], c="red", s=3, alpha=0.3, label="Tipping risk")
    ax.set_xlabel("Radial distance from J2 (in)")
    ax.set_ylabel("Height relative to J2 (in)")
    ax.set_title("Tipping stability across J1 sweep")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"workspace_{label}.png"), dpi=150)
    plt.close(fig)
    print(f"  Saved workspace_{label}.png")


def main():
    parser = argparse.ArgumentParser(description="Analytical arm workspace sweep")
    arm_grp = parser.add_mutually_exclusive_group(required=True)
    arm_grp.add_argument("--36arm", dest="reach", action="store_const", const=36)
    arm_grp.add_argument("--30arm", dest="reach", action="store_const", const=30)
    arm_grp.add_argument("--24arm", dest="reach", action="store_const", const=24)
    parser.add_argument("--armloaded", action="store_true",
                        help="Add 3-lb payload to gripper")
    env_grp = parser.add_mutually_exclusive_group(required=True)
    env_grp.add_argument("--step", dest="env", action="store_const", const="step")
    env_grp.add_argument("--reactor", dest="env", action="store_const", const="reactor")
    parser.add_argument("--coarse", action="store_true",
                        help="Coarse grid for quick validation")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip matplotlib plots")
    args = parser.parse_args()

    arm_params = get_arm_params(args.reach, loaded=args.armloaded)
    label = config_label(args.reach, args.armloaded, args.env)

    out_dir = os.path.join(_XDRIVE_ROOT, "results", "arm_workspace_sweep")
    os.makedirs(out_dir, exist_ok=True)

    print(f"=== Arm Workspace Sweep: {label} ===")
    print(f"  Reach: {args.reach}\" ({arm_params['L2']/IN:.0f}+{arm_params['L3']/IN:.0f}"
          f"+{arm_params['L4']/IN:.0f}\")  Loaded: {args.armloaded}")
    print(f"  Env: {args.env}  Coarse: {args.coarse}")
    print(f"  Max reach: {arm_params['max_reach']/IN:.1f}\" ({arm_params['max_reach']:.3f} m)")
    z_j2 = compute_z_j2(args.env)
    print(f"  J2 height above floor: {z_j2/IN:.1f}\" ({z_j2:.3f} m)")
    print()

    results, stats = run_sweep(arm_params, args.env, coarse=args.coarse)

    print(f"Sweep complete in {stats['elapsed_s']:.1f}s")
    print(f"  Targets:     {stats['n_targets']}")
    print(f"  IK fails:    {stats['n_ik_fail']}")
    print(f"  Joint limit: {stats['n_joint_limit']}")
    print(f"  Collision:   {stats['n_collision']}")
    print(f"  Valid:        {stats['n_valid']}")

    if results:
        tau_j2 = [abs(r["tau_j2"]) for r in results]
        tau_j3 = [abs(r["tau_j3"]) for r in results]
        tau_j4 = [abs(r["tau_j4"]) for r in results]
        print(f"\n  J2 torque: peak={max(tau_j2):.2f} Nm, mean={np.mean(tau_j2):.2f} Nm")
        print(f"  J3 torque: peak={max(tau_j3):.2f} Nm, mean={np.mean(tau_j3):.2f} Nm")
        print(f"  J4 torque: peak={max(tau_j4):.2f} Nm, mean={np.mean(tau_j4):.2f} Nm")
        n_stable = sum(1 for r in results if r["tip_stable_all"])
        print(f"  Tipping: {n_stable}/{len(results)} always stable "
              f"({100*n_stable/len(results):.1f}%)")

    # Write outputs
    csv_path = os.path.join(out_dir, f"{label}.csv")
    json_path = os.path.join(out_dir, f"{label}_summary.json")
    write_csv(results, csv_path)
    write_summary(results, stats, arm_params, args.env, json_path)
    print(f"\n  CSV: {csv_path}")
    print(f"  JSON: {json_path}")

    if not args.no_plots:
        plot_workspace(results, label, out_dir)

    print("\nDone.")


if __name__ == "__main__":
    main()
