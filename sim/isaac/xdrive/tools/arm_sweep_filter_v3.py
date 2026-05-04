"""
Post-processing filter for FORTIS arm collision-free sweep results (v3).

Imports arm_ik_v3 so the collision envelope (1.128" square CF tube) and
per-joint tipping masses match the v3 hardware spec.

V3 changes vs v2:
  - Hardcoded for the 30" CF loaded build (no arm-config detection)
  - New reactor poloidal cross-section filter: rejects poses where any
    arm point would intersect the central column or pass outside the
    outer reactor wall in (r, z), assuming the chassis sits at the
    nominal orbit radius (1.59 m from the column).
  - Tipping is NOT used as a rejection reason; every geometrically valid
    pose is kept and gets a tip_worst_margin_m column for visualization
    coloring downstream.

Usage:
  python tools/arm_sweep_filter_v3.py results/arm_continuous_sweep/30in_loaded_step_v3.csv
"""
import os, sys, math, argparse, csv, json
import numpy as np

XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(XDRIVE_ROOT, "lib"))
import arm_ik_v3 as arm_ik
import sim_config as cfg

parser = argparse.ArgumentParser(description="Filter v3 arm sweep results")
parser.add_argument("csv_path", help="Path to raw sweep CSV")
parser.add_argument("--margin", type=float, default=0.010,
                    help="Collision margin in meters (default: 0.010 = 10mm)")
parser.add_argument("--max-pos-error", type=float, default=5.0,
                    help="Max position error to accept (degrees, default: 5.0)")
parser.add_argument("--orbit-radius-in", type=float, default=62.6,
                    help="Robot orbit radius from column (in). Default 62.6\" "
                         "matches outer-floor spawn at R=1.59m.")
parser.add_argument("--no-reactor-filter", action="store_true",
                    help="Skip the reactor poloidal cross-section filter")
parser.add_argument("--z-j2-floor", type=float, default=None,
                    help="J2 height above lowest floor (m). Auto-detect if omitted.")
parser.add_argument("--chassis-mass-kg", type=float, default=None,
                    help="Override CHASSIS_MASS for tipping calc (kg). "
                         "If omitted, uses arm_ik_v3 default (20.4 kg).")
parser.add_argument("--out-suffix", type=str, default="",
                    help="Suffix appended to output filenames (e.g. _30lb).")
args = parser.parse_args()

if args.chassis_mass_kg is not None:
    arm_ik.CHASSIS_MASS = args.chassis_mass_kg
    print(f"[override] CHASSIS_MASS = {args.chassis_mass_kg:.3f} kg "
          f"(chassis-side total = "
          f"{(args.chassis_mass_kg + arm_ik.WHEELS_TOTAL_MASS + arm_ik.M_J1 + arm_ik.M_J2):.3f} kg "
          f"= {(args.chassis_mass_kg + arm_ik.WHEELS_TOTAL_MASS + arm_ik.M_J1 + arm_ik.M_J2) / 0.45359237:.2f} lb)")

if not os.path.exists(args.csv_path):
    print(f"ERROR: CSV not found: {args.csv_path}")
    sys.exit(1)


# ==========================================================================
# V3 arm params (single config)
# ==========================================================================
arm_params = arm_ik.get_arm_params()
L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]

IN = 0.0254
CHASSIS_H = 6.0 * IN
J1_STACK_H = 1.5 * IN
BELLY_HEIGHT = 2.0 * IN
STEP_H = (cfg.STEP_HEIGHT_IN) * IN
ARM_MOUNT_X = arm_ik.ARM_MOUNT_X

basename = os.path.basename(args.csv_path)
env = "step" if "step" in basename else ("reactor" if "reactor" in basename else "flat")

# J2 height above the lowest accessible floor surface.
# Step env / reactor: lowest floor is the inner (lower) reactor floor.
if args.z_j2_floor is not None:
    z_j2_floor = args.z_j2_floor
elif env in ("step", "reactor"):
    # Robot rides on outer floor; inner floor is STEP_H below.
    # Conservative: J2 height = belly + chassis_H + J1_stack above the lower floor.
    z_j2_floor = STEP_H + BELLY_HEIGHT + CHASSIS_H + J1_STACK_H
else:
    z_j2_floor = BELLY_HEIGHT + CHASSIS_H + J1_STACK_H

# ==========================================================================
# Reactor poloidal cross-section bounds (from sim_config.py)
# ==========================================================================
# All in meters
R_CENTER_POST = cfg.R_CENTER_POST_IN * IN          # ~0.965 m
R_OUTER_WALL = cfg.R_OUTER_FLOOR_MAX_IN * IN       # ~1.785 m (outermost floor radius)
Z_OUTER_FLOOR = cfg.Z_OUTER_IN * IN                # ~-1.252 m (outer floor)
Z_INNER_FLOOR = cfg.Z_INNER_IN * IN                # ~-1.367 m (inner floor)
# Reactor "ceiling" is hard to pin without the STL; the inspection target is
# the lower hemisphere walls. Use a generous upper bound that won't reject
# realistic reach poses but does reject "arm pointing straight up to space".
Z_CEILING = Z_OUTER_FLOOR + 60.0 * IN              # 60" above outer floor (loose bound)

# Robot is at orbit radius along -Y by convention; arm hangs in vertical plane
# rotated by J1. The arm-mount sits on the chassis at ARM_MOUNT_X; in world:
# arm_mount_world_R = orbit_R + ARM_MOUNT_X (negative ARM_MOUNT_X moves mount
# toward column).
ORBIT_R = args.orbit_radius_in * IN  # default 62.6" = 1.59 m
# Arm mount world radial position (measured from column axis):
ARM_MOUNT_R = ORBIT_R + ARM_MOUNT_X  # ARM_MOUNT_X is negative -> mount is closer to column
# J2 pivot height above OUTER floor:
Z_J2_OUTER = ARM_MOUNT_X * 0  # placeholder; J2 height above outer floor:
Z_J2_OUTER = BELLY_HEIGHT + CHASSIS_H + J1_STACK_H  # m above outer floor


def reactor_envelope_check(j2_rad, j3_rad, j4_rad, j1_rad=0.0):
    """Return True if the arm at this pose stays inside the reactor's
    poloidal cross-section, assuming the chassis sits at the nominal orbit
    pose (radial distance ORBIT_R from column, on the OUTER floor).

    Checks key arm points (j3, j4, ee, link midpoints, camera) for:
      - radial position r in [R_CENTER_POST, R_OUTER_WALL]
      - vertical position z in [Z_OUTER_FLOOR, Z_CEILING]
    """
    fk = arm_ik.fk_planar(j2_rad, j3_rad, j4_rad, L2, L3, L4)
    # Each FK position is in J2-local (x,z). Translate to world:
    #   world_R = ARM_MOUNT_R + fx * cos(j1)   (with J1 reference frame so
    #   arm "x" points from column outward at j1 = pi rad i.e. tangent dir
    #   when chassis faces column).
    # Simpler: assume J1=0 means arm extends in +X chassis frame, which
    # at orbit pose points in the radial-inward (toward column) direction.
    # So arm "x" measured from mount = +x in chassis frame = -R direction
    # in world. Hence world_R = ARM_MOUNT_R - fx (fx>0 -> closer to column).
    # We apply J1 in the horizontal plane: rotation about Z.
    c1 = math.cos(j1_rad)
    s1 = math.sin(j1_rad)

    points = []
    for label in ("j3", "j4", "ee", "L2_mid", "L3_mid", "L4_mid", "cam"):
        fx, fz = fk[label]
        # Horizontal radial component when J1=0 (arm pointing toward column).
        # General J1: project (fx*c1, fx*s1) onto radial direction.
        # Robot is at orbit angle -90 deg (south of column on -Y axis).
        # Robot's "forward" (chassis +X) at spawn faces column = +Y world.
        # Chassis +Y at spawn = -X world. With J1=0, arm extends along chassis +X = +Y world.
        # World position of arm point: (mount_x_world + fx*sin(j1), mount_y_world + fx*cos(j1)*?).
        # For the heuristic filter, simplify: world radial offset from mount =
        # |projection of (fx*c1, fx*s1) onto the radial direction|.
        # But mount is at radius ARM_MOUNT_R; arm "x" at J1=0 points radially
        # inward (toward column). So world radius = ARM_MOUNT_R - fx*c1.
        world_r = ARM_MOUNT_R - fx * c1
        # Vertical: z is in arm-local from J2; world z = Z_OUTER_FLOOR + Z_J2_OUTER + fz
        world_z = Z_OUTER_FLOOR + Z_J2_OUTER + fz
        points.append((label, world_r, world_z))

    for label, r, z in points:
        if r < R_CENTER_POST + 0.020:        # 20 mm clearance from column
            return False, f"reactor_column ({label} r={r/IN:+.1f}\")"
        if r > R_OUTER_WALL - 0.020:         # 20 mm clearance from outer wall
            return False, f"reactor_outer_wall ({label} r={r/IN:+.1f}\")"
        if z < Z_OUTER_FLOOR - STEP_H - 0.020:
            return False, f"reactor_floor ({label} z={z/IN:+.1f}\")"
        if z > Z_CEILING:
            return False, f"reactor_ceiling ({label} z={z/IN:+.1f}\")"
    return True, None


# ==========================================================================
print(f"Config: 30\" CF, LOADED, env={env}")
print(f"  L2={L2/IN:.3f}\", L3={L3/IN:.3f}\", L4={L4/IN:.3f}\"")
print(f"  Collision margin: {args.margin*1000:.1f} mm")
print(f"  Max position error: {args.max_pos_error:.1f} deg")
print(f"  J2 height above lowest floor: {z_j2_floor/IN:.1f}\" ({z_j2_floor:.4f} m)")
print(f"  Reactor filter: {'OFF' if args.no_reactor_filter else 'ON'}")
print(f"    Orbit R = {ORBIT_R/IN:.1f}\"  Arm mount R = {ARM_MOUNT_R/IN:.1f}\"")
print(f"    Column R = {R_CENTER_POST/IN:.1f}\"  Outer wall R = {R_OUTER_WALL/IN:.1f}\"")


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
FILTER_COLS = [
    "filter_status",
    "filter_chassis",
    "filter_self_L4_L2",
    "filter_self_L4_J1",
    "filter_floor",
    "filter_reactor",       # NEW: reactor poloidal cross-section
    "filter_reactor_reason",
    "filter_converged",
    "tip_worst_margin_m",   # tipping is informational, not a rejection
    "tip_worst_j1_deg",
    "tip_j1_stable_count",
    "tip_cg_x_m",
    "tip_cg_y_m",
]

n_valid = 0
n_rej_chassis = 0
n_rej_self_l4l2 = 0
n_rej_self_l4j1 = 0
n_rej_floor = 0
n_rej_reactor = 0
n_rej_converge = 0
n_unstable = 0   # informational
filtered_rows = []

for r in rows:
    try:
        j2_fk = float(r["j2_fk_deg"])
        j3_fk = float(r["j3_fk_deg"])
        j4_fk = float(r["j4_fk_deg"])
    except (ValueError, KeyError):
        r.update({c: "" for c in FILTER_COLS})
        r["filter_status"] = "missing_data"
        filtered_rows.append(r)
        continue

    j2_rad = math.radians(j2_fk)
    j3_rad = math.radians(j3_fk)
    j4_rad = math.radians(j4_fk)

    # Geometric collision check (chassis / self / floor)
    coll = arm_ik.check_collisions(j2_rad, j3_rad, j4_rad, arm_params,
                                   z_j2_above_floor=z_j2_floor,
                                   margin=args.margin)
    f_chassis = (not coll["valid"] and coll["reason"] == "chassis")
    f_self_l4l2 = (not coll["valid"] and coll["reason"] == "self_L4_L2")
    f_self_l4j1 = (not coll["valid"] and coll["reason"] == "self_L4_J1")
    f_floor = (not coll["valid"] and coll["reason"] == "floor")

    # Reactor poloidal cross-section
    f_reactor = False
    f_reactor_reason = ""
    if not args.no_reactor_filter:
        ok, why = reactor_envelope_check(j2_rad, j3_rad, j4_rad)
        if not ok:
            f_reactor = True
            f_reactor_reason = why or ""

    # Convergence
    try:
        pos_err = float(r.get("position_error_deg", "0"))
    except ValueError:
        pos_err = 999.0
    f_converge = pos_err > args.max_pos_error

    # Tipping (informational only -- compute for ALL non-rejected poses)
    geom_rejected = (f_chassis or f_self_l4l2 or f_self_l4j1 or f_floor
                     or f_reactor or f_converge)

    tip_worst_margin = ""
    tip_worst_j1 = ""
    tip_n_stable = ""
    tip_cg_x = ""
    tip_cg_y = ""
    if not geom_rejected:
        tip = arm_ik.tipping_sweep_j1(j2_rad, j3_rad, j4_rad, arm_params)
        tip_worst_margin = f"{tip['worst_margin']:.6f}"
        tip_worst_j1 = f"{tip['worst_j1_deg']:.0f}"
        tip_n_stable = f"{tip['n_stable']}/{tip['n_total']}"
        tip_at_worst = arm_ik.tipping_at_j1(
            math.radians(tip["worst_j1_deg"]), j2_rad, j3_rad, j4_rad, arm_params)
        tip_cg_x = f"{tip_at_worst['cg_x']:.6f}"
        tip_cg_y = f"{tip_at_worst['cg_y']:.6f}"
        if not tip["stable_all"]:
            n_unstable += 1

    # Status
    reasons = []
    if f_chassis: reasons.append("chassis_collision")
    if f_self_l4l2: reasons.append("self_L4_L2")
    if f_self_l4j1: reasons.append("self_L4_J1")
    if f_floor: reasons.append("floor_collision")
    if f_reactor: reasons.append("reactor_envelope")
    if f_converge: reasons.append("convergence_error")

    if reasons:
        status = ";".join(reasons)
    else:
        status = "valid"
        n_valid += 1

    if f_chassis: n_rej_chassis += 1
    if f_self_l4l2: n_rej_self_l4l2 += 1
    if f_self_l4j1: n_rej_self_l4j1 += 1
    if f_floor: n_rej_floor += 1
    if f_reactor: n_rej_reactor += 1
    if f_converge: n_rej_converge += 1

    r["filter_status"] = status
    r["filter_chassis"] = str(f_chassis)
    r["filter_self_L4_L2"] = str(f_self_l4l2)
    r["filter_self_L4_J1"] = str(f_self_l4j1)
    r["filter_floor"] = str(f_floor)
    r["filter_reactor"] = str(f_reactor)
    r["filter_reactor_reason"] = f_reactor_reason
    r["filter_converged"] = str(not f_converge)
    r["tip_worst_margin_m"] = tip_worst_margin
    r["tip_worst_j1_deg"] = tip_worst_j1
    r["tip_j1_stable_count"] = tip_n_stable
    r["tip_cg_x_m"] = tip_cg_x
    r["tip_cg_y_m"] = tip_cg_y
    filtered_rows.append(r)


# ==========================================================================
# Write filtered CSV
# ==========================================================================
out_path = args.csv_path.replace(".csv", f"_filtered{args.out_suffix}.csv")
out_fields = list(raw_fields) + FILTER_COLS
with open(out_path, "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=out_fields, extrasaction="ignore")
    writer.writeheader()
    for r in filtered_rows:
        writer.writerow(r)

total = len(filtered_rows)
print(f"\n{'='*60}")
print(f"FILTER RESULTS (V3)")
print(f"{'='*60}")
print(f"  Total poses:         {total}")
print(f"  Valid:                {n_valid} ({100*n_valid/max(total,1):.1f}%)")
print(f"  Rejected:")
print(f"    Chassis collision:  {n_rej_chassis} ({100*n_rej_chassis/max(total,1):.1f}%)")
print(f"    Self L4-L2:         {n_rej_self_l4l2} ({100*n_rej_self_l4l2/max(total,1):.1f}%)")
print(f"    Self L4-J1:         {n_rej_self_l4j1} ({100*n_rej_self_l4j1/max(total,1):.1f}%)")
print(f"    Floor:              {n_rej_floor} ({100*n_rej_floor/max(total,1):.1f}%)")
print(f"    Reactor envelope:   {n_rej_reactor} ({100*n_rej_reactor/max(total,1):.1f}%)")
print(f"    Convergence error:  {n_rej_converge} ({100*n_rej_converge/max(total,1):.1f}%)")
print(f"  (Tipping informational; {n_unstable} valid poses tip at some J1.)")

# Torque summary for valid poses
print(f"\nTORQUE SUMMARY (valid poses, n={n_valid}):")
if n_valid > 0:
    valid_rows = [r for r in filtered_rows if r["filter_status"] == "valid"]
    for jn in ("j2", "j3", "j4"):
        phys_col = f"tau_{jn}_Nm"
        ana_col = f"tau_{jn}_analytical"
        phys_vals, ana_vals = [], []
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
            arr = np.abs(np.array(phys_vals))
            print(f"  {jn.upper()} physics:    "
                  f"mean={arr.mean():.3f}  p95={np.percentile(arr,95):.3f}  peak={arr.max():.3f} Nm")
        if ana_vals:
            arr = np.abs(np.array(ana_vals))
            print(f"  {jn.upper()} analytical: "
                  f"mean={arr.mean():.3f}  p95={np.percentile(arr,95):.3f}  peak={arr.max():.3f} Nm")

# Stability summary (informational)
margins = []
for r in filtered_rows:
    if r["filter_status"] != "valid":
        continue
    try:
        m = float(r["tip_worst_margin_m"])
        if math.isfinite(m): margins.append(m)
    except (ValueError, TypeError):
        pass
if margins:
    margins_arr = np.array(margins)
    print(f"\nSTABILITY (valid poses, n={len(margins)}):")
    print(f"  Worst-case margin: {margins_arr.min()*1000:+.1f} mm  (CG to support edge, worst J1)")
    print(f"  Mean margin:        {margins_arr.mean()*1000:+.1f} mm")
    print(f"  p5 margin:          {np.percentile(margins_arr, 5)*1000:+.1f} mm")
    print(f"  Poses < 10 mm margin: {int(np.sum(margins_arr < 0.010))}")
    print(f"  Poses < 0 mm  (tip):  {int(np.sum(margins_arr < 0))}")

print(f"\nOutput: {out_path}")

# JSON summary
json_path = out_path.replace(f"_filtered{args.out_suffix}.csv",
                              f"_filter_summary{args.out_suffix}.json")
summary = {
    "source_csv": args.csv_path,
    "build": "v3_30in_CF_loaded",
    "env": env,
    "margin_m": args.margin,
    "max_pos_error_deg": args.max_pos_error,
    "z_j2_floor_m": z_j2_floor,
    "reactor_filter_enabled": not args.no_reactor_filter,
    "orbit_R_in": args.orbit_radius_in,
    "total": total,
    "n_valid": n_valid,
    "n_rej_chassis": n_rej_chassis,
    "n_rej_self_l4l2": n_rej_self_l4l2,
    "n_rej_self_l4j1": n_rej_self_l4j1,
    "n_rej_floor": n_rej_floor,
    "n_rej_reactor": n_rej_reactor,
    "n_rej_converge": n_rej_converge,
    "n_unstable_at_some_j1": n_unstable,
    "tip_worst_margin_m": float(min(margins)) if margins else None,
    "tip_mean_margin_m": float(np.mean(margins)) if margins else None,
    "tip_n_tight_10mm": int(np.sum(np.array(margins) < 0.010)) if margins else 0,
    "tip_n_tip_at_some_j1": int(np.sum(np.array(margins) < 0)) if margins else 0,
}
with open(json_path, "w") as jf:
    json.dump(summary, jf, indent=2)
print(f"Summary: {json_path}")
