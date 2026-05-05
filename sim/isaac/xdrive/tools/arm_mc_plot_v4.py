"""Plots for the FORTIS arm MC sweep v4.

Reads the *_filtered.csv from arm_mc_filter_v4.py and produces:
  - sampling_hist_v4.png   4-panel histogram of (J1, J2, J3, J4) MC draws
                           (sanity check that the sampler covered each axis
                            uniformly).
  - torque_table_v4.png    Torque + utilization table, computed over the
                           (geom + reactor)-pass set.  Includes unstable
                           poses (no stability gating per v4 spec).
  - poloidal_v4.png        Poloidal cross-section with EE points colored by
                           analytical tipping margin AND markered by physics-
                           tipping flag.

Run:
  python sim/isaac/xdrive/tools/arm_mc_plot_v4.py \
      sim/isaac/xdrive/results/arm_mc_sweep/30in_loaded_step_v4_mc5000_filtered.csv
"""
import os, sys, csv, math, argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.path import Path as MplPath

XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(XDRIVE_ROOT, "lib"))
import arm_ik_v3 as arm_ik
import sim_config as cfg

parser = argparse.ArgumentParser()
parser.add_argument("csv_path", help="Filtered CSV from arm_mc_filter_v4.py")
parser.add_argument("--out-dir", default=None,
                    help="Output directory (default: same as input CSV)")
parser.add_argument("--orbit-radius-in", type=float, default=66.1,
                    help="Arm-mount radial distance from column (in)")
parser.add_argument("--chassis-mass-kg", type=float, default=3.629,
                    help="Chassis mass override for I_zz / margins (kg)")
parser.add_argument("--out-suffix", type=str, default="",
                    help="Suffix appended to output PNG filenames")
args = parser.parse_args()

if not os.path.exists(args.csv_path):
    print(f"ERROR: not found: {args.csv_path}")
    sys.exit(1)

arm_ik.CHASSIS_MASS = float(args.chassis_mass_kg)
print(f"[chassis mass] CHASSIS_MASS = {arm_ik.CHASSIS_MASS:.3f} kg")

out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.csv_path))
SUFFIX = args.out_suffix

IN = 0.0254
arm_params = arm_ik.get_arm_params()
L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]


# ============================================================================
# Load
# ============================================================================
all_rows = []
with open(args.csv_path) as fh:
    for r in csv.DictReader(fh):
        all_rows.append(r)


def passed_geom(r):
    return (r.get("filter_chassis") == "False"
            and r.get("filter_self_L4_L2") == "False"
            and r.get("filter_self_L4_J1") == "False"
            and r.get("filter_floor") == "False"
            and r.get("filter_converged") == "True")


valid_rows = [r for r in all_rows if r.get("filter_status") == "valid"]
geom_rows = [r for r in all_rows if passed_geom(r)]

n_total = len(all_rows)
n_geom = len(geom_rows)
n_valid = len(valid_rows)

phys_tip_total = sum(1 for r in all_rows if r.get("tip_physics") == "True")
ana_tip_total = sum(1 for r in all_rows if r.get("tip_analytical") == "True")
phys_tip_valid = sum(1 for r in valid_rows if r.get("tip_physics") == "True")
ana_tip_valid = sum(1 for r in valid_rows if r.get("tip_analytical") == "True")
disagree_valid = sum(1 for r in valid_rows if r.get("tip_disagreement") == "True")

print(f"Total rows:           {n_total}")
print(f"Geom-pass:            {n_geom}")
print(f"Valid (geom+reactor): {n_valid}")
print(f"  physics tipped in valid: {phys_tip_valid}")
print(f"  analytical unstable in valid: {ana_tip_valid}")
print(f"  disagreements in valid: {disagree_valid}")


# ============================================================================
# (1) Sampling histogram
# ============================================================================
def col_floats(rows, key):
    out = []
    for r in rows:
        try:
            v = float(r.get(key, ""))
            if math.isfinite(v):
                out.append(v)
        except (ValueError, TypeError):
            pass
    return np.array(out)


fig_h, axes_h = plt.subplots(1, 4, figsize=(16, 4))
hist_specs = [
    ("j1_deg", (-180, 180), "J1 (yaw)"),
    ("j2_deg", (-180, 180), "J2 (shoulder)"),
    ("j3_deg", (-180, 180), "J3 (elbow)"),
    ("j4_deg", (-100, 100), "J4 (wrist)"),
]
for ax, (key, rng, label) in zip(axes_h, hist_specs):
    vals = col_floats(all_rows, key)
    ax.hist(vals, bins=36, range=rng, color="#1f4e79",
            edgecolor="white", linewidth=0.5)
    expected = len(vals) / 36.0
    ax.axhline(expected, color="#c00", linestyle="--", linewidth=1.2,
               label=f"uniform: {expected:.0f}/bin")
    ax.set_xlim(rng)
    ax.set_xlabel(f"{label} (deg)")
    ax.set_ylabel("Count")
    ax.set_title(f"{label}: n={len(vals)}", fontsize=10)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="lower right", fontsize=8)

fig_h.suptitle(f"FORTIS v4 Monte Carlo sampling -- {n_total} draws (uniform on each joint)",
               fontsize=13, fontweight="bold", y=1.02)
plt.tight_layout()
out_hist = os.path.join(out_dir, f"sampling_hist_v4{SUFFIX}.png")
plt.savefig(out_hist, dpi=170, bbox_inches="tight")
plt.close(fig_h)
print(f"wrote {out_hist}")


# ============================================================================
# (2) Torque table -- over (geom + reactor) pass set, including unstable
# ============================================================================
TORQUE_LIMIT = {"j2": 30.0, "j3": 12.0, "j4": 4.9}
MOTOR_LABEL = {
    "j2": "NEMA 23 + EG23 20:1",
    "j3": "NEMA 17 + Cricket 25:1",
    "j4": "Hitec D845WP",
}
LABEL = {"j2": "J2 (shoulder)", "j3": "J3 (elbow)", "j4": "J4 (wrist)"}

tau = {jn: np.abs(col_floats(valid_rows, f"tau_{jn}_Nm")) for jn in ("j2", "j3", "j4")}
stats = {}
for jn, arr in tau.items():
    if len(arr) == 0:
        stats[jn] = None
        continue
    stats[jn] = {"mean": arr.mean(),
                 "p95":  np.percentile(arr, 95),
                 "peak": arr.max()}

# J1 inertial worst-case (for the J1 inertial subtable)
I_max = 0.0
worst_pose = None
for r in valid_rows:
    try:
        j2 = math.radians(float(r["j2_fk_deg"]))
        j3 = math.radians(float(r["j3_fk_deg"]))
        j4 = math.radians(float(r["j4_fk_deg"]))
    except (ValueError, KeyError):
        continue
    I_zz = arm_ik.j1_inertia_about_axis(j2, j3, j4, arm_params)
    if I_zz > I_max:
        I_max = I_zz
        worst_pose = (r.get("j2_fk_deg"), r.get("j3_fk_deg"), r.get("j4_fk_deg"))
J1_ACCELS_DEGS2 = [10, 30, 60, 90, 180]


fig = plt.figure(figsize=(13, 7.0))
gs = fig.add_gridspec(3, 1, height_ratios=[3.2, 2.4, 1.2], hspace=0.45)
ax_top = fig.add_subplot(gs[0]); ax_top.axis("off")
ax_mid = fig.add_subplot(gs[1]); ax_mid.axis("off")
ax_bot = fig.add_subplot(gs[2]); ax_bot.axis("off")

# Per-joint torque table
header_t = ["Joint", "Motor", "Mean (Nm)", "P95 (Nm)", "Peak (Nm)",
            "Cont. rating (Nm)", "P95 util", "Peak util"]
rows_t = []
for jn in ("j2", "j3", "j4"):
    s = stats[jn]
    if s is None:
        rows_t.append([LABEL[jn], MOTOR_LABEL[jn], "-", "-", "-", "-", "-", "-"])
        continue
    cont = TORQUE_LIMIT[jn]
    rows_t.append([
        LABEL[jn], MOTOR_LABEL[jn],
        f"{s['mean']:.2f}", f"{s['p95']:.2f}", f"{s['peak']:.2f}",
        f"{cont:.1f}",
        f"{100*s['p95']/cont:.1f}%",
        f"{100*s['peak']/cont:.1f}%",
    ])

col_widths_t = [0.13, 0.22, 0.10, 0.10, 0.10, 0.13, 0.11, 0.11]
tbl_t = ax_top.table(cellText=rows_t, colLabels=header_t,
                     loc="center", cellLoc="center", colWidths=col_widths_t)
tbl_t.auto_set_font_size(False); tbl_t.set_fontsize(11); tbl_t.scale(1.0, 1.7)
for col_i in range(len(header_t)):
    cell = tbl_t[(0, col_i)]
    cell.set_facecolor("#1f4e79")
    cell.set_text_props(color="white", weight="bold")
for row_i, jn in enumerate(("j2", "j3", "j4"), start=1):
    s = stats[jn]
    if s is None: continue
    cont = TORQUE_LIMIT[jn]
    for util_col in (6, 7):
        u = (s["p95"] if util_col == 6 else s["peak"]) / cont
        if u < 0.5:    color = "#c6efce"
        elif u < 0.8:  color = "#ffeb9c"
        else:          color = "#ffc7ce"
        tbl_t[(row_i, util_col)].set_facecolor(color)
ax_top.set_title(
    f'FORTIS v4 arm MC -- 30" CF loaded, {n_valid} of {n_total} samples '
    f'pass geom + reactor (unstable kept)',
    fontsize=13, fontweight="bold", pad=10)

# J1 inertial subtable
header_b = ["alpha (deg/s^2)", "T_J1 (Nm)", "% of 12 Nm (NEMA17 + Cricket 25:1)"]
rows_b = []
for a in J1_ACCELS_DEGS2:
    T = I_max * math.radians(a)
    rows_b.append([f"{a}", f"{T:.3f}", f"{100*T/12.0:.1f}%"])
col_widths_b = [0.20, 0.20, 0.35]
tbl_b = ax_mid.table(cellText=rows_b, colLabels=header_b,
                     loc="center", cellLoc="center", colWidths=col_widths_b)
tbl_b.auto_set_font_size(False); tbl_b.set_fontsize(11); tbl_b.scale(1.0, 1.7)
for col_i in range(len(header_b)):
    cell = tbl_b[(0, col_i)]
    cell.set_facecolor("#1f4e79")
    cell.set_text_props(color="white", weight="bold")
worst_str = ""
if worst_pose is not None:
    try:
        worst_str = (f"  (worst pose J2={float(worst_pose[0]):.0f}deg, "
                     f"J3={float(worst_pose[1]):.0f}deg, J4={float(worst_pose[2]):.0f}deg)")
    except (TypeError, ValueError):
        pass
ax_mid.set_title(
    f"J1 inertial torque  --  gravity is 0, I_zz_max = {I_max:.4f} kg*m^2{worst_str}",
    fontsize=12, fontweight="bold", pad=10)

# Stability summary subtable
header_s = ["Stability check", "In valid set", "Pct of valid"]
rows_s = [
    ["Analytical unstable", f"{ana_tip_valid}", f"{100*ana_tip_valid/max(n_valid,1):.1f}%"],
    ["Physics tipped",       f"{phys_tip_valid}", f"{100*phys_tip_valid/max(n_valid,1):.1f}%"],
    ["Methods disagree",     f"{disagree_valid}", f"{100*disagree_valid/max(n_valid,1):.1f}%"],
]
col_widths_s = [0.28, 0.16, 0.16]
tbl_s = ax_bot.table(cellText=rows_s, colLabels=header_s,
                     loc="center", cellLoc="center", colWidths=col_widths_s)
tbl_s.auto_set_font_size(False); tbl_s.set_fontsize(11); tbl_s.scale(1.0, 1.6)
for col_i in range(len(header_s)):
    cell = tbl_s[(0, col_i)]
    cell.set_facecolor("#1f4e79")
    cell.set_text_props(color="white", weight="bold")
ax_bot.set_title(
    f"Stability (informational; chassis = {arm_ik.CHASSIS_MASS:.3f} kg)",
    fontsize=11, fontweight="bold", pad=8)

out_torque = os.path.join(out_dir, f"torque_table_v4{SUFFIX}.png")
plt.savefig(out_torque, dpi=200, bbox_inches="tight")
plt.close(fig)
print(f"wrote {out_torque}")


# ============================================================================
# (2b) Per-joint torque bar chart (J1, J2, J3, J4)
# ============================================================================
J1_TORQUE_LIMIT = 12.0   # NEMA 17 + Cricket 25:1
ALL_LIMITS = {"j1": J1_TORQUE_LIMIT, **TORQUE_LIMIT}
ALL_LABEL  = {"j1": "J1 (yaw)", **LABEL}
ALL_MOTOR  = {"j1": "NEMA 17 + Cricket 25:1", **MOTOR_LABEL}

joints4 = ("j1", "j2", "j3", "j4")
tau4 = {jn: np.abs(col_floats(valid_rows, f"tau_{jn}_Nm")) for jn in joints4}
stats4 = {}
for jn, arr in tau4.items():
    if len(arr) == 0:
        stats4[jn] = None; continue
    stats4[jn] = {"mean": arr.mean(),
                  "p95":  np.percentile(arr, 95),
                  "peak": arr.max()}

fig, ax = plt.subplots(figsize=(11, 6.0))
x = np.arange(len(joints4))
w = 0.25
means = [stats4[j]["mean"] if stats4[j] else 0 for j in joints4]
p95s  = [stats4[j]["p95"]  if stats4[j] else 0 for j in joints4]
peaks = [stats4[j]["peak"] if stats4[j] else 0 for j in joints4]

b1 = ax.bar(x - w, means, w, label="Mean",
            color="#9fbbe0", edgecolor="#1f3d6b")
b2 = ax.bar(x,     p95s,  w, label="P95",
            color="#5a7fb8", edgecolor="#1f3d6b")
b3 = ax.bar(x + w, peaks, w, label="Peak",
            color="#1f4e79", edgecolor="#1f3d6b")

# Continuous-rating reference lines per joint
for i, jn in enumerate(joints4):
    cont = ALL_LIMITS[jn]
    ax.hlines(cont, i - 1.5*w, i + 1.5*w, colors="#c00000",
              linestyles="--", linewidth=1.6,
              label="Cont. rating" if i == 0 else None)
    ax.text(i + 1.55*w, cont, f" {cont:.1f} Nm",
            va="center", ha="left", fontsize=9, color="#c00000")

# Bar value labels
for bars in (b1, b2, b3):
    for b in bars:
        h = b.get_height()
        if h > 0.02:
            ax.text(b.get_x() + b.get_width()/2, h, f"{h:.2f}",
                    ha="center", va="bottom", fontsize=8)

ax.set_xticks(x)
ax.set_xticklabels([ALL_LABEL[j] + f"\n({ALL_MOTOR[j]})" for j in joints4],
                   fontsize=9)
ax.set_ylabel("Torque (Nm, |physics|)")
ax.set_title(
    f'FORTIS v4 arm torques per joint  --  30" CF loaded, '
    f'{n_valid} of {n_total} MC samples (geom + reactor pass)',
    fontsize=11, fontweight="bold", pad=10)
ax.set_ylim(bottom=0)
ax.grid(True, axis="y", alpha=0.25)
ax.legend(loc="upper right", fontsize=9, framealpha=0.95)

plt.tight_layout()
out_bar = os.path.join(out_dir, f"torques_v4{SUFFIX}.png")
plt.savefig(out_bar, dpi=180, bbox_inches="tight")
plt.close(fig)
print(f"wrote {out_bar}")


# ============================================================================
# (3) Poloidal cross-section
# ============================================================================
R_INBOARD      = cfg.R_CENTER_POST_IN
Z_OUT_FLOOR    = cfg.Z_OUTER_IN
Z_IN_FLOOR     = cfg.Z_INNER_IN
R_STEP         = cfg.STEP_R_IN
R_INNER_FL_MAX = cfg.R_INNER_FLOOR_MAX_IN
R_OUTER_FL_MIN = cfg.R_OUTER_FLOOR_MIN_IN
R_OUTER_FL_MAX = cfg.R_OUTER_FLOOR_MAX_IN
WHEEL_R_IN     = (cfg.WHEEL_DIAMETER_MM / 2.0) / 25.4
BELLY_IN       = arm_ik.BELLY_HEIGHT / IN
CHASSIS_H      = 6.0
J1_STACK       = arm_ik.J1_STACK_H / IN
CHASSIS_FOOTPRINT_RADIAL = 14.5

# Vessel D-shape
R0_PLASMA = 65.7
A_PLASMA  = 27.5
KAPPA_T   = 2.20
KAPPA_B   = 1.85
DELTA     = 0.30

n_arc = 120
th_top = np.linspace(0.0, np.pi/2, n_arc)
th_bot = np.linspace(0.0, np.pi/2, n_arc)
top_R = R0_PLASMA + A_PLASMA * np.cos(th_top + DELTA * np.sin(th_top))
top_Z = KAPPA_T * A_PLASMA * np.sin(th_top)
bot_R = R0_PLASMA + A_PLASMA * np.cos(-th_bot + DELTA * np.sin(-th_bot))
bot_Z = KAPPA_B * A_PLASMA * np.sin(-th_bot)
keep = bot_Z > Z_OUT_FLOOR
bot_R = np.concatenate([bot_R[keep], [R_OUTER_FL_MAX]])
bot_Z = np.concatenate([bot_Z[keep], [Z_OUT_FLOOR]])
keep_t = top_R > R_INBOARD + 1.0
top_R = np.concatenate([top_R[keep_t], [R_INBOARD]])
top_Z = np.concatenate([top_Z[keep_t], [top_Z[keep_t][-1]]])

vessel_R = np.concatenate([
    top_R[::-1], bot_R,
    [R_OUTER_FL_MAX, R_STEP, R_STEP, R_INNER_FL_MAX, R_INBOARD, R_INBOARD],
])
vessel_Z = np.concatenate([
    top_Z[::-1], bot_Z,
    [Z_OUT_FLOOR, Z_OUT_FLOOR, Z_IN_FLOOR, Z_IN_FLOOR, Z_IN_FLOOR, top_Z[0]],
])
vessel_path = MplPath(np.column_stack([vessel_R, vessel_Z]))

# Tilted chassis at orbit pose
chassis_outer_face_R = 66.0
chassis_inner_face_R = chassis_outer_face_R - CHASSIS_FOOTPRINT_RADIAL
inner_wheel = np.array([chassis_inner_face_R, Z_IN_FLOOR + WHEEL_R_IN])
outer_wheel = np.array([chassis_outer_face_R, Z_OUT_FLOOR + WHEEL_R_IN])
dx_w = outer_wheel - inner_wheel
chassis_x_hat = dx_w / np.linalg.norm(dx_w)
chassis_z_hat = np.array([-chassis_x_hat[1], chassis_x_hat[0]])
tilt_rad = math.atan2(chassis_x_hat[1], chassis_x_hat[0])
tilt_deg = math.degrees(tilt_rad)

belly_to_pivot = WHEEL_R_IN - BELLY_IN
inner_bot = inner_wheel - chassis_z_hat * belly_to_pivot
outer_bot = outer_wheel - chassis_z_hat * belly_to_pivot
inner_top = inner_bot + chassis_z_hat * CHASSIS_H
outer_top = outer_bot + chassis_z_hat * CHASSIS_H
chassis_corners = np.array([inner_bot, outer_bot, outer_top, inner_top])

chassis_top_mid = 0.5 * (inner_top + outer_top)
arm_mount_x_offset = CHASSIS_FOOTPRINT_RADIAL * 0.30
J2_pos = chassis_top_mid + chassis_x_hat * arm_mount_x_offset \
         + chassis_z_hat * J1_STACK
J2_R = float(J2_pos[0])
J2_Z = float(J2_pos[1])

fig, ax = plt.subplots(figsize=(11, 9))
ax.fill(vessel_R, vessel_Z, color="#fafafa", edgecolor="black",
        linewidth=2.0, zorder=1)
ax.axhline(0.0, color="#888888", linestyle=":", linewidth=0.8, zorder=2)
ax.text(R0_PLASMA + A_PLASMA + 1.5, 0.0, "midplane", fontsize=8,
        color="#666666", va="center")
ax.plot([R_STEP, R_STEP], [Z_OUT_FLOOR, Z_IN_FLOOR],
        color="black", linewidth=2.0, zorder=3)

# Plot poses from geom_rows (skipping reactor failures shown via vessel containment)
ee_R, ee_Z, margins_mm = [], [], []
phys_tip_flags = []
ana_tip_flags = []
j3_R_all, j3_Z_all = [], []
j4_R_all, j4_Z_all = [], []
for r in valid_rows:
    try:
        j2 = math.radians(float(r["j2_fk_deg"]))
        j3 = math.radians(float(r["j3_fk_deg"]))
        j4 = math.radians(float(r["j4_fk_deg"]))
    except (ValueError, KeyError):
        continue
    fk = arm_ik.fk_planar(j2, j3, j4, L2, L3, L4)

    def to_world(pt):
        x = pt[0] / IN
        z = pt[1] / IN
        wp = J2_pos + (-x) * chassis_x_hat + z * chassis_z_hat
        return float(wp[0]), float(wp[1])

    R_ee, Z_ee = to_world(fk["ee"])
    R_j3, Z_j3 = to_world(fk["j3"])
    R_j4, Z_j4 = to_world(fk["j4"])

    margin = None
    s_m = r.get("tip_margin_m", "")
    if s_m:
        try: margin = float(s_m)
        except ValueError: margin = None
    if margin is None:
        margin = float("nan")

    ee_R.append(R_ee); ee_Z.append(Z_ee)
    j3_R_all.append(R_j3); j3_Z_all.append(Z_j3)
    j4_R_all.append(R_j4); j4_Z_all.append(Z_j4)
    margins_mm.append(margin * 1000.0)
    phys_tip_flags.append(r.get("tip_physics") == "True")
    ana_tip_flags.append(r.get("tip_analytical") == "True")

ee_R = np.array(ee_R); ee_Z = np.array(ee_Z)
j3_R_all = np.array(j3_R_all); j3_Z_all = np.array(j3_Z_all)
j4_R_all = np.array(j4_R_all); j4_Z_all = np.array(j4_Z_all)
margins_mm = np.array(margins_mm)
phys_tip_flags = np.array(phys_tip_flags)
ana_tip_flags = np.array(ana_tip_flags)


def all_inside(idx):
    pts = np.array([
        [J2_R, J2_Z],
        [j3_R_all[idx], j3_Z_all[idx]],
        [j4_R_all[idx], j4_Z_all[idx]],
        [ee_R[idx],     ee_Z[idx]],
    ])
    return bool(np.all(vessel_path.contains_points(pts)))


inside_mask = np.array([all_inside(i) for i in range(len(ee_R))]) if len(ee_R) else np.array([], dtype=bool)
n_total_raw = len(ee_R)
n_inside = int(inside_mask.sum())

ee_R_v = ee_R[inside_mask]; ee_Z_v = ee_Z[inside_mask]
j3_R_v = j3_R_all[inside_mask]; j3_Z_v = j3_Z_all[inside_mask]
j4_R_v = j4_R_all[inside_mask]; j4_Z_v = j4_Z_all[inside_mask]
margins_v = margins_mm[inside_mask]
phys_v = phys_tip_flags[inside_mask]
ana_v = ana_tip_flags[inside_mask]

# Linkage lines (faint background + few foreground samples)
seg_R = np.column_stack([np.full(n_inside, J2_R), j3_R_v, j4_R_v, ee_R_v])
seg_Z = np.column_stack([np.full(n_inside, J2_Z), j3_Z_v, j4_Z_v, ee_Z_v])
for i in range(n_inside):
    ax.plot(seg_R[i], seg_Z[i], color="#5a7fb8",
            linewidth=0.30, alpha=0.06, zorder=4)
n_show = min(40, n_inside)
if n_inside > 0:
    sample_idx = np.linspace(0, n_inside - 1, n_show).astype(int)
    for i in sample_idx:
        ax.plot(seg_R[i], seg_Z[i], color="#1f3d6b",
                linewidth=0.9, alpha=0.5, zorder=4.5)

# EE scatter, colored by analytical margin
if n_inside > 0:
    vmax = max(40.0, np.percentile(np.abs(margins_v), 99))
    sc = ax.scatter(ee_R_v, ee_Z_v, c=margins_v, cmap="RdYlGn",
                    vmin=-vmax, vmax=vmax, s=14, alpha=0.85,
                    edgecolors="none", zorder=5)
    plt.colorbar(sc, ax=ax, label="Tipping margin (mm, analytical)",
                 shrink=0.6, pad=0.02)

# Tilted chassis polygon
chassis_poly = MplPolygon(chassis_corners, closed=True,
                          facecolor="#9fbbe0", edgecolor="#1f3d6b",
                          linewidth=1.6, alpha=0.95, zorder=7,
                          joinstyle="round")
ax.add_patch(chassis_poly)
top_anchor = chassis_top_mid + chassis_x_hat * arm_mount_x_offset
ax.plot([top_anchor[0], J2_R], [top_anchor[1], J2_Z],
        color="#1f3d6b", linewidth=2.2, zorder=8)
for wp in (inner_wheel, outer_wheel):
    ax.add_patch(plt.Circle((wp[0], wp[1]), WHEEL_R_IN,
                            facecolor="#444", edgecolor="black",
                            linewidth=0.8, zorder=8))
ax.scatter([J2_R], [J2_Z], marker="s", color="black", s=70, zorder=9)
ax.annotate("J2", (J2_R, J2_Z), xytext=(7, 4), textcoords="offset points",
            fontsize=10, fontweight="bold", zorder=10)

ax.text(R_INBOARD - 1.0, 0.0, "centerstack",
        rotation=90, ha="right", va="center", fontsize=9, color="#444444")
ax.text(R0_PLASMA + A_PLASMA + 0.5, -10.0, "outer wall",
        rotation=-90, ha="left", va="center", fontsize=9, color="#444444")
ax.text(R_INBOARD + 12, Z_IN_FLOOR - 2.5, "lower divertor",
        ha="left", va="top", fontsize=9, color="#444444")

ax.set_xlabel("R (inches from reactor center)")
ax.set_ylabel("Z (inches)")
ax.set_xlim(R_INBOARD - 4, R0_PLASMA + A_PLASMA + 8)
ax.set_ylim(Z_IN_FLOOR - 4, 22.0)
ax.set_aspect("equal")
ax.grid(True, alpha=0.25)

n_ana_inside = int(ana_v.sum()) if n_inside else 0
ax.set_title(
    f'FORTIS v4 MC at orbit pose (chassis tilt {tilt_deg:.1f} deg over step) '
    f'-- 30" CF loaded\n{n_inside} of {n_total_raw} arm poses inside vessel '
    f'(of {n_total} MC draws); analytical-unstable={n_ana_inside}',
    fontsize=11)
plt.suptitle("DIII-D Poloidal Cross-Section -- Monte Carlo (v4)",
             fontsize=14, fontweight="bold", y=0.97)
plt.tight_layout(rect=(0, 0, 1, 0.96))
out_pol = os.path.join(out_dir, f"poloidal_v4{SUFFIX}.png")
plt.savefig(out_pol, dpi=170)
plt.close(fig)
print(f"wrote {out_pol}  ({n_inside}/{n_total_raw} inside, "
      f"tilt={tilt_deg:.1f} deg)")
