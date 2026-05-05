"""
Post-processing filter for FORTIS arm MC sweep v4.

Differences from v3:
  - Reads per-row sampled J1 from CSV and uses it for analytical tipping
    (no 24-angle re-sweep -- the MC sample already covers J1).
  - Reads physics tipping flag from CSV and combines it with the analytical
    one (tip_either, tip_disagreement).
  - Does NOT drop unstable poses. The output includes every pose along with
    pass/fail flags so plot scripts can choose what to display.
  - Defaults chassis mass to 3.629 kg (v4 hardware target).

Usage:
  python sim/isaac/xdrive/tools/arm_mc_filter_v4.py \
      sim/isaac/xdrive/results/arm_mc_sweep/30in_loaded_step_v4_mc5000.csv
"""
import os, sys, math, argparse, csv, json
import numpy as np

XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(XDRIVE_ROOT, "lib"))
import arm_ik_v3 as arm_ik
import sim_config as cfg

parser = argparse.ArgumentParser(description="Filter MC sweep v4 results")
parser.add_argument("csv_path", help="Path to raw MC sweep CSV")
parser.add_argument("--margin", type=float, default=0.010,
                    help="Collision margin in meters (default: 10mm)")
parser.add_argument("--max-pos-error", type=float, default=5.0,
                    help="Max position error to accept (deg, default 5)")
parser.add_argument("--orbit-radius-in", type=float, default=62.6,
                    help="Robot orbit radius from column (in)")
parser.add_argument("--no-reactor-filter", action="store_true",
                    help="Skip the reactor poloidal cross-section filter")
parser.add_argument("--z-j2-floor", type=float, default=None,
                    help="J2 height above lowest floor (m). Auto if omitted.")
parser.add_argument("--chassis-mass-kg", type=float, default=3.629,
                    help="Chassis mass override for tipping calc (kg). "
                         "Default 3.629 = 14.144 kg total - arm - wheels.")
args = parser.parse_args()

# Apply chassis mass override BEFORE arm_params computation
arm_ik.CHASSIS_MASS = float(args.chassis_mass_kg)
print(f"[chassis mass] CHASSIS_MASS = {arm_ik.CHASSIS_MASS:.3f} kg", flush=True)

if not os.path.exists(args.csv_path):
    print(f"ERROR: CSV not found: {args.csv_path}")
    sys.exit(1)

arm_params = arm_ik.get_arm_params()
L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]

IN = 0.0254
CHASSIS_H = 6.0 * IN
J1_STACK_H = 1.5 * IN
BELLY_HEIGHT = 2.0 * IN
STEP_H = cfg.STEP_HEIGHT_IN * IN
ARM_MOUNT_X = arm_ik.ARM_MOUNT_X

basename = os.path.basename(args.csv_path)
env = "step" if "step" in basename else ("reactor" if "reactor" in basename else "flat")

if args.z_j2_floor is not None:
    z_j2_floor = args.z_j2_floor
elif env in ("step", "reactor"):
    z_j2_floor = STEP_H + BELLY_HEIGHT + CHASSIS_H + J1_STACK_H
else:
    z_j2_floor = BELLY_HEIGHT + CHASSIS_H + J1_STACK_H


# ==========================================================================
# Reactor poloidal envelope (same as v3 filter)
# ==========================================================================
R_CENTER_POST = cfg.R_CENTER_POST_IN * IN
R_OUTER_WALL = cfg.R_OUTER_FLOOR_MAX_IN * IN
Z_OUTER_FLOOR = cfg.Z_OUTER_IN * IN
Z_INNER_FLOOR = cfg.Z_INNER_IN * IN
Z_CEILING = Z_OUTER_FLOOR + 60.0 * IN

ORBIT_R = args.orbit_radius_in * IN
ARM_MOUNT_R = ORBIT_R + ARM_MOUNT_X
Z_J2_OUTER = BELLY_HEIGHT + CHASSIS_H + J1_STACK_H


def reactor_envelope_check(j2_rad, j3_rad, j4_rad, j1_rad=0.0):
    """Same as v3 filter. Uses J1 for horizontal projection."""
    fk = arm_ik.fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4)
    c1 = math.cos(j1_rad)

    points = []
    for label in ("j3", "j4", "ee", "L2_mid", "L3_mid", "L4_mid", "cam"):
        fx, fz = fk[label]
        world_r = ARM_MOUNT_R - fx * c1
        world_z = Z_OUTER_FLOOR + Z_J2_OUTER + fz
        points.append((label, world_r, world_z))

    for label, r, z in points:
        if r < R_CENTER_POST + 0.020:
            return False, f"reactor_column ({label} r={r/IN:+.1f}\")"
        if r > R_OUTER_WALL - 0.020:
            return False, f"reactor_outer_wall ({label} r={r/IN:+.1f}\")"
        if z < Z_OUTER_FLOOR - STEP_H - 0.020:
            return False, f"reactor_floor ({label} z={z/IN:+.1f}\")"
        if z > Z_CEILING:
            return False, f"reactor_ceiling ({label} z={z/IN:+.1f}\")"
    return True, None


print(f"Config: 30\" CF, LOADED, env={env}")
print(f"  L2={L2/IN:.3f}\", L3={L3/IN:.3f}\", L4={L4/IN:.3f}\"")
print(f"  Collision margin: {args.margin*1000:.1f} mm")
print(f"  J2 height above lowest floor: {z_j2_floor/IN:.1f}\" ({z_j2_floor:.4f} m)")
print(f"  Reactor filter: {'OFF' if args.no_reactor_filter else 'ON'}")


# ==========================================================================
# Read CSV
# ==========================================================================
print(f"\nReading: {args.csv_path}")
rows = []
with open(args.csv_path, "r", encoding="utf-8", errors="replace") as f:
    reader = csv.DictReader(f)
    raw_fields = reader.fieldnames
    for row in reader:
        rows.append(row)
print(f"  {len(rows)} rows")


# ==========================================================================
# Filter
# ==========================================================================
FILTER_COLS = [
    "filter_status",
    "filter_chassis",
    "filter_self_L4_L2",
    "filter_self_L4_J1",
    "filter_floor",
    "filter_reactor",
    "filter_reactor_reason",
    "filter_converged",
    # Tipping (analytical at sampled J1; physics from CSV)
    "tip_analytical",       # bool: analytical CG outside support polygon at sampled J1
    "tip_margin_m",         # signed margin (m), negative = unstable
    "tip_cg_x_m",
    "tip_cg_y_m",
    "tip_physics",          # bool: from CSV physics_tipped
    "tip_either",           # bool: tip_analytical or tip_physics
    "tip_disagreement",     # bool: analytical != physics
]

n_geom_pass = 0
n_geom_reactor_pass = 0
n_rej_chassis = n_rej_self_l4l2 = n_rej_self_l4j1 = n_rej_floor = 0
n_rej_reactor = n_rej_converge = 0
n_tip_analytical = n_tip_physics = n_tip_either = n_tip_disagree = 0

filtered_rows = []

for r in rows:
    try:
        j1_deg = float(r.get("j1_deg", "0"))
        j2_fk = float(r["j2_fk_deg"])
        j3_fk = float(r["j3_fk_deg"])
        j4_fk = float(r["j4_fk_deg"])
    except (ValueError, KeyError):
        r.update({c: "" for c in FILTER_COLS})
        r["filter_status"] = "missing_data"
        filtered_rows.append(r)
        continue

    j1_rad = math.radians(j1_deg)
    j2_rad = math.radians(j2_fk)
    j3_rad = math.radians(j3_fk)
    j4_rad = math.radians(j4_fk)

    # Geometric collision
    coll = arm_ik.check_collisions(j2_rad, j3_rad, j4_rad, arm_params,
                                   z_j2_above_floor=z_j2_floor,
                                   margin=args.margin)
    f_chassis = (not coll["valid"] and coll["reason"] == "chassis")
    f_self_l4l2 = (not coll["valid"] and coll["reason"] == "self_L4_L2")
    f_self_l4j1 = (not coll["valid"] and coll["reason"] == "self_L4_J1")
    f_floor = (not coll["valid"] and coll["reason"] == "floor")

    # Reactor envelope (uses sampled J1)
    f_reactor = False
    f_reactor_reason = ""
    if not args.no_reactor_filter:
        ok, why = reactor_envelope_check(j2_rad, j3_rad, j4_rad, j1_rad=j1_rad)
        if not ok:
            f_reactor = True
            f_reactor_reason = why or ""

    # Convergence
    try:
        pos_err = float(r.get("position_error_deg", "0"))
    except ValueError:
        pos_err = 999.0
    f_converge = pos_err > args.max_pos_error

    # Analytical tipping at sampled J1 (per-pose, not a sweep)
    tip = arm_ik.tipping_at_j1(j1_rad, j2_rad, j3_rad, j4_rad, arm_params)
    tip_analytical = (not tip["stable"])
    tip_margin = tip["margin_worst"]

    # Physics tipping (from CSV, set by sweep)
    tip_physics_str = r.get("physics_tipped", "False")
    tip_physics = tip_physics_str == "True"
    tip_either = tip_analytical or tip_physics
    tip_disagree = tip_analytical != tip_physics

    # Tally
    if f_chassis: n_rej_chassis += 1
    if f_self_l4l2: n_rej_self_l4l2 += 1
    if f_self_l4j1: n_rej_self_l4j1 += 1
    if f_floor: n_rej_floor += 1
    if f_reactor: n_rej_reactor += 1
    if f_converge: n_rej_converge += 1
    if tip_analytical: n_tip_analytical += 1
    if tip_physics: n_tip_physics += 1
    if tip_either: n_tip_either += 1
    if tip_disagree: n_tip_disagree += 1

    geom_pass = not (f_chassis or f_self_l4l2 or f_self_l4j1 or f_floor or f_converge)
    if geom_pass: n_geom_pass += 1
    if geom_pass and not f_reactor: n_geom_reactor_pass += 1

    reasons = []
    if f_chassis: reasons.append("chassis_collision")
    if f_self_l4l2: reasons.append("self_L4_L2")
    if f_self_l4j1: reasons.append("self_L4_J1")
    if f_floor: reasons.append("floor_collision")
    if f_reactor: reasons.append("reactor_envelope")
    if f_converge: reasons.append("convergence_error")
    status = ";".join(reasons) if reasons else "valid"

    r["filter_status"] = status
    r["filter_chassis"] = str(f_chassis)
    r["filter_self_L4_L2"] = str(f_self_l4l2)
    r["filter_self_L4_J1"] = str(f_self_l4j1)
    r["filter_floor"] = str(f_floor)
    r["filter_reactor"] = str(f_reactor)
    r["filter_reactor_reason"] = f_reactor_reason
    r["filter_converged"] = str(not f_converge)
    r["tip_analytical"] = str(tip_analytical)
    r["tip_margin_m"] = f"{tip_margin:.6f}"
    r["tip_cg_x_m"] = f"{tip['cg_x']:.6f}"
    r["tip_cg_y_m"] = f"{tip['cg_y']:.6f}"
    r["tip_physics"] = str(tip_physics)
    r["tip_either"] = str(tip_either)
    r["tip_disagreement"] = str(tip_disagree)

    filtered_rows.append(r)


# ==========================================================================
# Write output
# ==========================================================================
out_path = args.csv_path.replace(".csv", "_filtered.csv")
out_fields = list(raw_fields) + [c for c in FILTER_COLS if c not in raw_fields]

with open(out_path, "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=out_fields, extrasaction="ignore")
    writer.writeheader()
    for r in filtered_rows:
        writer.writerow(r)

total = len(filtered_rows)
print(f"\n{'='*60}")
print("FILTER RESULTS (no stability gating; every pose retained)")
print(f"{'='*60}")
print(f"  Total poses:          {total}")
print(f"  Geom pass:            {n_geom_pass} ({100*n_geom_pass/max(total,1):.1f}%)")
print(f"  Geom + Reactor pass:  {n_geom_reactor_pass} ({100*n_geom_reactor_pass/max(total,1):.1f}%)")
print(f"  Rejection breakdown:")
print(f"    chassis collision:  {n_rej_chassis}")
print(f"    self L4-L2:         {n_rej_self_l4l2}")
print(f"    self L4-J1:         {n_rej_self_l4j1}")
print(f"    floor collision:    {n_rej_floor}")
print(f"    reactor envelope:   {n_rej_reactor}")
print(f"    convergence error:  {n_rej_converge}")
print(f"  Tipping:")
print(f"    analytical unstable: {n_tip_analytical}")
print(f"    physics tipped:      {n_tip_physics}")
print(f"    either flag set:     {n_tip_either}")
print(f"    disagreement:        {n_tip_disagree}")

# Margins summary on geom + reactor pass set
margins = []
phys_tip_in_pass = ana_tip_in_pass = 0
for r in filtered_rows:
    if r["filter_status"] != "valid":
        continue
    try:
        margins.append(float(r["tip_margin_m"]))
    except (ValueError, KeyError):
        pass
    if r.get("tip_analytical") == "True": ana_tip_in_pass += 1
    if r.get("tip_physics") == "True": phys_tip_in_pass += 1

if margins:
    arr = np.array(margins)
    print(f"\nSTABILITY MARGINS on (geom+reactor)-pass set (n={len(margins)}):")
    print(f"    worst (mm):  {arr.min()*1000:.1f}")
    print(f"    mean (mm):   {arr.mean()*1000:.1f}")
    print(f"    p5 (mm):     {np.percentile(arr,5)*1000:.1f}")
    print(f"    margin<10mm: {int((arr<0.010).sum())}")
    print(f"    margin<0:    {int((arr<0).sum())} (would tip per analytical)")
    print(f"    physics tipped in pass set: {phys_tip_in_pass}")
    print(f"    analytical unstable in pass set: {ana_tip_in_pass}")

print(f"\nOutput: {out_path}")

json_path = out_path.replace("_filtered.csv", "_filter_summary.json")
summary = {
    "source_csv": args.csv_path,
    "build": "v4_30in_CF_loaded_mc",
    "env": env,
    "chassis_mass_kg": arm_ik.CHASSIS_MASS,
    "margin_m": args.margin,
    "max_pos_error_deg": args.max_pos_error,
    "orbit_R_in": args.orbit_radius_in,
    "reactor_filter_enabled": not args.no_reactor_filter,
    "z_j2_floor_m": z_j2_floor,
    "total": total,
    "n_geom_pass": n_geom_pass,
    "n_geom_reactor_pass": n_geom_reactor_pass,
    "n_rej_chassis": n_rej_chassis,
    "n_rej_self_l4l2": n_rej_self_l4l2,
    "n_rej_self_l4j1": n_rej_self_l4j1,
    "n_rej_floor": n_rej_floor,
    "n_rej_reactor": n_rej_reactor,
    "n_rej_converge": n_rej_converge,
    "n_tip_analytical": n_tip_analytical,
    "n_tip_physics": n_tip_physics,
    "n_tip_either": n_tip_either,
    "n_tip_disagreement": n_tip_disagree,
    "tip_worst_margin_m_pass": float(min(margins)) if margins else None,
    "tip_mean_margin_m_pass": float(np.mean(margins)) if margins else None,
}
with open(json_path, "w") as jf:
    json.dump(summary, jf, indent=2)
print(f"Summary: {json_path}")
