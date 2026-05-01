"""
Post-processing filter for FORTIS arm collision-free sweep results.

Reads raw CSV from arm_continuous_sweep.py, applies analytical filters,
outputs filtered CSV with rejection reasons and summary statistics.

Filters applied:
  1. Self-collision (L4 vs L2, L4 vs J1 base)
  2. Chassis collision (link segments vs chassis bounding box)
  3. Floor collision (any joint/EE below ground)
  4. Convergence (position error threshold)

Usage:
  python tools/arm_sweep_filter.py results/arm_continuous_sweep/36in_bare_step.csv
  python tools/arm_sweep_filter.py results/arm_continuous_sweep/36in_bare_step.csv --margin 0.015
"""
import os, sys, math, argparse, csv, json
import numpy as np

XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(XDRIVE_ROOT, "lib"))
import arm_ik

parser = argparse.ArgumentParser(description="Filter arm sweep results")
parser.add_argument("csv_path", help="Path to raw sweep CSV")
parser.add_argument("--margin", type=float, default=0.010,
                    help="Collision margin in meters (default: 0.010 = 10mm)")
parser.add_argument("--max-pos-error", type=float, default=5.0,
                    help="Max position error to accept (degrees, default: 5.0)")
parser.add_argument("--tipping-thresh", type=float, default=8.0,
                    help="Chassis tilt change threshold for tipping (degrees, default: 8.0)")
parser.add_argument("--z-j2-floor", type=float, default=None,
                    help="J2 height above lowest floor (m). Auto-detect from env if omitted.")
args = parser.parse_args()

if not os.path.exists(args.csv_path):
    print(f"ERROR: CSV not found: {args.csv_path}")
    sys.exit(1)


# ==========================================================================
# Detect arm config from filename
# ==========================================================================
basename = os.path.basename(args.csv_path)
arm_reach = 36  # default
if "36in" in basename: arm_reach = 36
elif "30in" in basename: arm_reach = 30
elif "24in" in basename: arm_reach = 24

loaded = "loaded" in basename

arm_params = arm_ik.get_arm_params(arm_reach, loaded=loaded)
L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]

# J2 height above floor depends on environment
# Step env: chassis straddles 4.5" step, so J2 is higher on one side
# For conservative check, use the minimum height
IN = 0.0254
CHASSIS_H = 7.1 * IN
J1_STACK_H = 1.5 * IN
BELLY_HEIGHT = 2.5 * IN
STEP_H = 4.5 * IN

if args.z_j2_floor is not None:
    z_j2_floor = args.z_j2_floor
elif "step" in basename:
    # On step: wheels on both levels. Lowest floor is the inner (lower) surface.
    # Chassis center is at BELLY_HEIGHT + CHASSIS_H/2 above the higher floor.
    # The lower floor is STEP_H below the higher floor.
    # J2 is at chassis top + J1_STACK_H.
    z_j2_floor = BELLY_HEIGHT + CHASSIS_H + J1_STACK_H  # conservative (from lower floor)
elif "reactor" in basename:
    z_j2_floor = BELLY_HEIGHT + CHASSIS_H + J1_STACK_H
else:
    # Flat ground
    z_j2_floor = BELLY_HEIGHT + CHASSIS_H / 2.0 + CHASSIS_H / 2.0 + J1_STACK_H

print(f"Config: {arm_reach}\" arm, {'loaded' if loaded else 'bare'}")
print(f"  L2={L2/IN:.1f}\", L3={L3/IN:.1f}\", L4={L4/IN:.1f}\"")
print(f"  Collision margin: {args.margin*1000:.1f}mm")
print(f"  Max position error: {args.max_pos_error:.1f}°")
print(f"  Tipping threshold: {args.tipping_thresh:.1f}°")
print(f"  J2 height above floor: {z_j2_floor/IN:.1f}\" ({z_j2_floor:.4f}m)")


# ==========================================================================
# Read raw CSV
# ==========================================================================
print(f"\nReading: {args.csv_path}")
rows = []
with open(args.csv_path, "r", encoding="utf-8", errors="replace") as f:
    reader = csv.DictReader(f)
    raw_fields = reader.fieldnames
    for row in reader:
        rows.append(row)
print(f"  {len(rows)} rows loaded")


# ==========================================================================
# Apply filters
# ==========================================================================
# New columns to add
FILTER_COLS = [
    "filter_status",      # "valid" or rejection reason(s)
    "filter_chassis",     # True/False
    "filter_self_L4_L2",  # True/False
    "filter_self_L4_J1",  # True/False
    "filter_floor",       # True/False
    "filter_converged",   # True/False
    "filter_tipping",     # True/False — analytical CG projection, worst-case J1
    "tip_worst_margin_m", # distance from CG to nearest support polygon edge (m), >0 = stable
    "tip_worst_j1_deg",   # J1 angle that gives worst stability margin
    "tip_j1_stable_count",# how many J1 angles are stable out of 24 tested
    "tip_cg_x_m",         # CG X at worst J1 (chassis frame)
    "tip_cg_y_m",         # CG Y at worst J1 (chassis frame)
]

# Compute baseline chassis orientation from the first few converged poses
baseline_rolls = []
baseline_pitches = []
for r in rows[:50]:
    if r.get("converged", "").strip() == "True":
        try:
            baseline_rolls.append(float(r["chassis_roll_deg"]))
            baseline_pitches.append(float(r["chassis_pitch_deg"]))
        except (ValueError, KeyError):
            pass
if baseline_rolls:
    base_roll = np.median(baseline_rolls)
    base_pitch = np.median(baseline_pitches)
    print(f"  Baseline chassis orientation: roll={base_roll:.2f}°, pitch={base_pitch:.2f}°")
else:
    base_roll = 0.0
    base_pitch = 0.0
    print(f"  WARNING: No converged poses found for baseline, using (0,0)")

# Apply filters to each row
n_valid = 0
n_rej_chassis = 0
n_rej_self_l4l2 = 0
n_rej_self_l4j1 = 0
n_rej_floor = 0
n_rej_converge = 0
n_rej_tipping = 0

filtered_rows = []

for r in rows:
    # Parse angles
    try:
        j2_fk = float(r["j2_fk_deg"])
        j3_fk = float(r["j3_fk_deg"])
        j4_fk = float(r["j4_fk_deg"])
    except (ValueError, KeyError):
        # Missing FK angles — skip
        r.update({c: "" for c in FILTER_COLS})
        r["filter_status"] = "missing_data"
        filtered_rows.append(r)
        continue

    j2_rad = math.radians(j2_fk)
    j3_rad = math.radians(j3_fk)
    j4_rad = math.radians(j4_fk)

    # Run collision check from arm_ik
    coll = arm_ik.check_collisions(j2_rad, j3_rad, j4_rad, arm_params,
                                    z_j2_above_floor=z_j2_floor,
                                    margin=args.margin)

    # Decompose collision reasons
    f_chassis = (not coll["valid"] and coll["reason"] == "chassis")
    f_self_l4l2 = (not coll["valid"] and coll["reason"] == "self_L4_L2")
    f_self_l4j1 = (not coll["valid"] and coll["reason"] == "self_L4_J1")
    f_floor = (not coll["valid"] and coll["reason"] == "floor")

    # Convergence check
    try:
        pos_err = float(r.get("position_error_deg", "0"))
    except ValueError:
        pos_err = 999.0
    f_converge = pos_err > args.max_pos_error

    # Analytical tipping: sweep J1 0-345 deg, check CG projection vs wheel polygon
    # Only compute for poses that pass geometry filters (skip known-bad poses)
    f_tipping = False
    tip_worst_margin = ""
    tip_worst_j1 = ""
    tip_n_stable = ""
    tip_cg_x = ""
    tip_cg_y = ""
    geom_rejected = f_chassis or f_self_l4l2 or f_self_l4j1 or f_floor or f_converge

    if not geom_rejected:
        tip = arm_ik.tipping_sweep_j1(j2_rad, j3_rad, j4_rad, arm_params)
        f_tipping = not tip["stable_all"]
        tip_worst_margin = f"{tip['worst_margin']:.6f}"
        tip_worst_j1 = f"{tip['worst_j1_deg']:.0f}"
        tip_n_stable = f"{tip['n_stable']}/{tip['n_total']}"
        # Get CG at worst J1 for diagnostics
        tip_at_worst = arm_ik.tipping_at_j1(
            math.radians(tip["worst_j1_deg"]), j2_rad, j3_rad, j4_rad, arm_params)
        tip_cg_x = f"{tip_at_worst['cg_x']:.6f}"
        tip_cg_y = f"{tip_at_worst['cg_y']:.6f}"

    # Determine overall status
    reasons = []
    if f_chassis: reasons.append("chassis_collision")
    if f_self_l4l2: reasons.append("self_L4_L2")
    if f_self_l4j1: reasons.append("self_L4_J1")
    if f_floor: reasons.append("floor_collision")
    if f_converge: reasons.append("convergence_error")
    if f_tipping: reasons.append("tipping")

    if reasons:
        status = ";".join(reasons)
    else:
        status = "valid"
        n_valid += 1

    if f_chassis: n_rej_chassis += 1
    if f_self_l4l2: n_rej_self_l4l2 += 1
    if f_self_l4j1: n_rej_self_l4j1 += 1
    if f_floor: n_rej_floor += 1
    if f_converge: n_rej_converge += 1
    if f_tipping: n_rej_tipping += 1

    r["filter_status"] = status
    r["filter_chassis"] = str(f_chassis)
    r["filter_self_L4_L2"] = str(f_self_l4l2)
    r["filter_self_L4_J1"] = str(f_self_l4j1)
    r["filter_floor"] = str(f_floor)
    r["filter_converged"] = str(not f_converge)
    r["filter_tipping"] = str(f_tipping)
    r["tip_worst_margin_m"] = tip_worst_margin
    r["tip_worst_j1_deg"] = tip_worst_j1
    r["tip_j1_stable_count"] = tip_n_stable
    r["tip_cg_x_m"] = tip_cg_x
    r["tip_cg_y_m"] = tip_cg_y

    filtered_rows.append(r)


# ==========================================================================
# Write filtered CSV
# ==========================================================================
out_path = args.csv_path.replace(".csv", "_filtered.csv")
out_fields = list(raw_fields) + FILTER_COLS

with open(out_path, "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=out_fields, extrasaction="ignore")
    writer.writeheader()
    for r in filtered_rows:
        writer.writerow(r)

total = len(filtered_rows)
print(f"\n{'='*60}")
print(f"FILTER RESULTS")
print(f"{'='*60}")
print(f"  Total poses:   {total}")
print(f"  Valid:          {n_valid} ({100*n_valid/max(total,1):.1f}%)")
print(f"  Rejected:")
print(f"    Chassis coll: {n_rej_chassis} ({100*n_rej_chassis/max(total,1):.1f}%)")
print(f"    Self L4-L2:   {n_rej_self_l4l2} ({100*n_rej_self_l4l2/max(total,1):.1f}%)")
print(f"    Self L4-J1:   {n_rej_self_l4j1} ({100*n_rej_self_l4j1/max(total,1):.1f}%)")
print(f"    Floor:        {n_rej_floor} ({100*n_rej_floor/max(total,1):.1f}%)")
print(f"    Convergence:  {n_rej_converge} ({100*n_rej_converge/max(total,1):.1f}%)")
print(f"    Tipping:      {n_rej_tipping} ({100*n_rej_tipping/max(total,1):.1f}%)")

# Torque summary for valid poses
print(f"\nTORQUE SUMMARY (valid poses, n={n_valid}):")
if n_valid > 0:
    valid_rows = [r for r in filtered_rows if r["filter_status"] == "valid"]
    for jn in ["j2", "j3", "j4"]:
        phys_col = f"tau_{jn}_Nm"
        ana_col = f"tau_{jn}_analytical"
        phys_vals = []
        ana_vals = []
        for r in valid_rows:
            try:
                v = float(r.get(phys_col, ""))
                if math.isfinite(v): phys_vals.append(v)
            except (ValueError, TypeError):
                pass
            try:
                v = float(r.get(ana_col, ""))
                if math.isfinite(v): ana_vals.append(v)
            except (ValueError, TypeError):
                pass
        if phys_vals:
            arr = np.array(phys_vals)
            print(f"  {jn.upper()} physics:     mean={np.abs(arr).mean():.3f} Nm, "
                  f"max={np.abs(arr).max():.3f} Nm")
        if ana_vals:
            arr = np.array(ana_vals)
            print(f"  {jn.upper()} analytical:  mean={np.abs(arr).mean():.3f} Nm, "
                  f"max={np.abs(arr).max():.3f} Nm")

# Stability summary for valid poses
valid_rows = [r for r in filtered_rows if r["filter_status"] == "valid"]
margins = []
for r in valid_rows:
    try:
        m = float(r["tip_worst_margin_m"])
        if math.isfinite(m): margins.append(m)
    except (ValueError, TypeError):
        pass

if margins:
    margins_arr = np.array(margins)
    print(f"\nSTABILITY SUMMARY (valid poses, n={len(margins)}):")
    print(f"  Worst-case margin (CG to support edge): {margins_arr.min():.4f} m ({margins_arr.min()/0.0254:.2f} in)")
    print(f"  Mean margin:   {margins_arr.mean():.4f} m ({margins_arr.mean()/0.0254:.2f} in)")
    print(f"  p5 margin:     {np.percentile(margins_arr, 5):.4f} m ({np.percentile(margins_arr, 5)/0.0254:.2f} in)")
    n_tight = np.sum(margins_arr < 0.010)
    print(f"  Poses with margin < 10mm: {n_tight} ({100*n_tight/len(margins):.1f}%)")
    n_negative = np.sum(margins_arr < 0)
    print(f"  Poses with margin < 0 (would tip at some J1): {n_negative} ({100*n_negative/len(margins):.1f}%)")

# Tipping-rejected poses details
tip_rejected = [r for r in filtered_rows if "tipping" in r.get("filter_status", "")]
if tip_rejected:
    print(f"\n  TIPPING-REJECTED POSES ({len(tip_rejected)}):")
    # Show worst 10
    tip_rejected.sort(key=lambda r: float(r.get("tip_worst_margin_m", "0")))
    for r in tip_rejected[:10]:
        print(f"    J2={r['j2_deg']:>6s} J3={r['j3_deg']:>6s} J4={r['j4_deg']:>6s}  "
              f"margin={float(r['tip_worst_margin_m']):.4f}m  "
              f"worst_J1={r['tip_worst_j1_deg']}deg  "
              f"stable={r['tip_j1_stable_count']}")

print(f"\nOutput: {out_path}")

# Write JSON summary
json_path = out_path.replace("_filtered.csv", "_filter_summary.json")
summary = {
    "source_csv": args.csv_path,
    "arm_reach_in": arm_reach,
    "loaded": loaded,
    "margin_m": args.margin,
    "max_pos_error_deg": args.max_pos_error,
    "tipping_thresh_deg": args.tipping_thresh,
    "z_j2_floor_m": z_j2_floor,
    "total": total,
    "n_valid": n_valid,
    "n_rej_chassis": n_rej_chassis,
    "n_rej_self_l4l2": n_rej_self_l4l2,
    "n_rej_self_l4j1": n_rej_self_l4j1,
    "n_rej_floor": n_rej_floor,
    "n_rej_converge": n_rej_converge,
    "n_rej_tipping": n_rej_tipping,
    "tip_worst_margin_m": float(min(margins)) if margins else None,
    "tip_mean_margin_m": float(np.mean(margins)) if margins else None,
    "tip_n_tight_10mm": int(np.sum(np.array(margins) < 0.010)) if margins else 0,
}
with open(json_path, "w") as jf:
    json.dump(summary, jf, indent=2)
print(f"Summary: {json_path}")
