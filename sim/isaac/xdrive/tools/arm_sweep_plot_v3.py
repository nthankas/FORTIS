"""Build presentation-ready outputs for the v3 arm sweep.

Reads the *_filtered.csv and produces:
  - poloidal_v3.png    DIII-D poloidal cross-section with EE points colored
                       by tipping margin (chassis at orbit pose). Mirrors
                       the v1 stability-sweep visualization.
  - torque_table_v3.png  Rendered torque + utilization table for reports.
  - torques_v3.png       Bar chart of mean / P95 / peak per joint.
  - torque_hist_v3.png   Distribution histograms.
  - stability_v3.png     Joint-space scatter (kept for diagnostics).

Run:
  python tools/arm_sweep_plot_v3.py results/arm_continuous_sweep/30in_loaded_step_v3_filtered.csv
"""
import os, sys, csv, math, argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Polygon as MplPolygon
from matplotlib.path import Path as MplPath

XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(XDRIVE_ROOT, "lib"))
import arm_ik_v3 as arm_ik
import sim_config as cfg

# args parsed below; chassis-mass override applied after parse

parser = argparse.ArgumentParser()
parser.add_argument("csv_path", help="Filtered CSV from arm_sweep_filter_v3.py")
parser.add_argument("--out-dir", default=None,
                    help="Output directory (default: same as input CSV)")
parser.add_argument("--orbit-radius-in", type=float, default=66.1,
                    help="Arm-mount radial distance from column at orbit pose, "
                         "inches (default 66.1\" = 1.68 m, matches outer-floor spawn)")
parser.add_argument("--chassis-mass-kg", type=float, default=None,
                    help="Override CHASSIS_MASS for tipping calc (kg).")
parser.add_argument("--out-suffix", type=str, default="",
                    help="Suffix appended to output PNG filenames (e.g. _30lb).")
args = parser.parse_args()

if not os.path.exists(args.csv_path):
    print(f"ERROR: not found: {args.csv_path}")
    sys.exit(1)

if args.chassis_mass_kg is not None:
    arm_ik.CHASSIS_MASS = args.chassis_mass_kg
    cs_total_kg = (args.chassis_mass_kg + arm_ik.WHEELS_TOTAL_MASS
                   + arm_ik.M_J1 + arm_ik.M_J2)
    print(f"[override] CHASSIS_MASS = {args.chassis_mass_kg:.3f} kg "
          f"(chassis-side total = {cs_total_kg:.3f} kg "
          f"= {cs_total_kg / 0.45359237:.2f} lb)")

out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.csv_path))
SUFFIX = args.out_suffix

IN = 0.0254
arm_params = arm_ik.get_arm_params()
L2, L3, L4 = arm_params["L2"], arm_params["L3"], arm_params["L4"]

# ============================================================================
# Load rows. Three buckets we care about:
#   valid_rows  -- pass everything (geom + reactor envelope) -> torque tables
#   geom_rows   -- pass geometry (chassis/self/floor/conv) regardless of reactor
#                  -> poloidal plot ("can we physically move there", with
#                     reactor wall drawn for context)
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
geom_rows  = [r for r in all_rows if passed_geom(r)]

print(f"Total rows:  {len(all_rows)}")
print(f"Geom-pass:   {len(geom_rows)} (chassis + self + floor + conv all OK)")
print(f"Valid:       {len(valid_rows)} (geom + reactor envelope + tipping informational)")


# ============================================================================
# Torque stats (over the reactor-valid set)
# ============================================================================
def col(rows, key):
    out = []
    for r in rows:
        try:
            v = float(r.get(key, ""))
            if math.isfinite(v):
                out.append(v)
        except (ValueError, TypeError):
            pass
    return np.array(out)

tau = {jn: np.abs(col(valid_rows, f"tau_{jn}_Nm")) for jn in ("j2", "j3", "j4")}

TORQUE_LIMIT = {"j2": 30.0, "j3": 12.0, "j4": 4.9}
MOTOR_LABEL = {
    "j2": "NEMA 23 + EG23 20:1",
    "j3": "NEMA 17 + Cricket 25:1",
    "j4": "Hitec D845WP",
}
LABEL = {"j2": "J2 (shoulder)", "j3": "J3 (elbow)", "j4": "J4 (wrist)"}

stats = {}
for jn, arr in tau.items():
    if len(arr) == 0:
        stats[jn] = None
        continue
    stats[jn] = {
        "mean": arr.mean(),
        "p95":  np.percentile(arr, 95),
        "peak": arr.max(),
    }

# J1 inertial worst case
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
        worst_pose = (r["j2_fk_deg"], r["j3_fk_deg"], r["j4_fk_deg"])
J1_ACCELS_DEGS2 = [10, 30, 60, 90, 180]


# ============================================================================
# RENDERED TABLE PNG (presentation-quality)
# Two sub-tables stacked: (1) per-joint torque + utilization, (2) J1 inertial.
# ============================================================================
fig = plt.figure(figsize=(13, 6.2))
gs = fig.add_gridspec(2, 1, height_ratios=[3.2, 2.4], hspace=0.4)
ax_top = fig.add_subplot(gs[0])
ax_bot = fig.add_subplot(gs[1])
ax_top.axis("off")
ax_bot.axis("off")

# ---- Top: per-joint torque table ----
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
        LABEL[jn],
        MOTOR_LABEL[jn],
        f"{s['mean']:.2f}",
        f"{s['p95']:.2f}",
        f"{s['peak']:.2f}",
        f"{cont:.1f}",
        f"{100*s['p95']/cont:.1f}%",
        f"{100*s['peak']/cont:.1f}%",
    ])

col_widths_t = [0.13, 0.22, 0.10, 0.10, 0.10, 0.13, 0.11, 0.11]
tbl_t = ax_top.table(cellText=rows_t, colLabels=header_t,
                     loc="center", cellLoc="center",
                     colWidths=col_widths_t)
tbl_t.auto_set_font_size(False)
tbl_t.set_fontsize(11)
tbl_t.scale(1.0, 1.7)

# Header styling
for col_i in range(len(header_t)):
    cell = tbl_t[(0, col_i)]
    cell.set_facecolor("#1f4e79")
    cell.set_text_props(color="white", weight="bold")

# Utilization cell colors
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

ax_top.set_title(f'FORTIS v3 arm  --  30" CF loaded, {len(valid_rows)} valid poses',
                 fontsize=13, fontweight="bold", pad=10)

# ---- Bottom: J1 inertial table ----
header_b = ["alpha (deg/s^2)", "T_J1 (Nm)", "% of 12 Nm (NEMA17 + Cricket 25:1)"]
rows_b = []
for a in J1_ACCELS_DEGS2:
    T = I_max * math.radians(a)
    rows_b.append([f"{a}", f"{T:.3f}", f"{100*T/12.0:.1f}%"])

col_widths_b = [0.20, 0.20, 0.35]
tbl_b = ax_bot.table(cellText=rows_b, colLabels=header_b,
                     loc="center", cellLoc="center",
                     colWidths=col_widths_b)
tbl_b.auto_set_font_size(False)
tbl_b.set_fontsize(11)
tbl_b.scale(1.0, 1.7)
for col_i in range(len(header_b)):
    cell = tbl_b[(0, col_i)]
    cell.set_facecolor("#1f4e79")
    cell.set_text_props(color="white", weight="bold")

worst_str = ""
if worst_pose is not None:
    worst_str = (f"  (worst pose J2={float(worst_pose[0]):.0f}deg, "
                 f"J3={float(worst_pose[1]):.0f}deg, J4={float(worst_pose[2]):.0f}deg)")
ax_bot.set_title(
    f"J1 inertial torque  --  gravity is 0, I_zz_max = {I_max:.4f} kg*m^2{worst_str}",
    fontsize=12, fontweight="bold", pad=10)

out = os.path.join(out_dir, f"torque_table_v3{SUFFIX}.png")
plt.savefig(out, dpi=200, bbox_inches="tight")
plt.close(fig)
print(f"wrote {out}")


# ============================================================================
# POLOIDAL CROSS-SECTION
# Realistic DIII-D vessel outline + tilted chassis straddling the step at
# the orbit pose, with arm poses overlaid. Poses landing outside the vessel
# polygon (in this 2D view) are dropped from the plot.
# ============================================================================

# ---- Reactor / robot constants (inches) ----
R_INBOARD      = cfg.R_CENTER_POST_IN       # 38.0   centerstack inner wall
Z_OUT_FLOOR    = cfg.Z_OUTER_IN             # -49.3  outer divertor floor
Z_IN_FLOOR     = cfg.Z_INNER_IN             # -53.8  inner divertor floor
R_STEP         = cfg.STEP_R_IN              # 55.5
R_INNER_FL_MAX = cfg.R_INNER_FLOOR_MAX_IN   # 54.8
R_OUTER_FL_MIN = cfg.R_OUTER_FLOOR_MIN_IN   # 56.2
R_OUTER_FL_MAX = cfg.R_OUTER_FLOOR_MAX_IN   # 70.3
WHEEL_R_IN     = (cfg.WHEEL_DIAMETER_MM / 2.0) / 25.4   # 3.54"
BELLY_IN       = arm_ik.BELLY_HEIGHT / IN   # 2.0
CHASSIS_H      = 6.0
J1_STACK       = arm_ik.J1_STACK_H / IN     # 1.5
CHASSIS_FOOTPRINT_RADIAL = 14.5             # along R at orbit pose
CHASSIS_FOOTPRINT_TANGENTIAL = 19.022       # along chassis +X (out of poloidal page)

# ---- Vessel D-shape outline (more realistic DIII-D inner wall) ----
# Parametric Miller-style D for the outer wall; vertical inboard centerstack
# joined by slanted upper/lower baffles; lower divertor floor + step + inner
# floor matches sim_config measurements.
R0_PLASMA = 65.7        # major radius proxy
A_PLASMA  = 27.5        # outer minor radius
KAPPA_T   = 2.20        # vertical elongation (top)
KAPPA_B   = 1.85        # vertical elongation (bottom, slightly less to fit divertor)
DELTA     = 0.30        # triangularity

# Outboard half (outer wall): theta from -π/2 (bottom outboard) to +π/2 (top outboard)
n_arc = 120
th_top = np.linspace(0.0, np.pi/2, n_arc)
th_bot = np.linspace(0.0, np.pi/2, n_arc)
# Top quadrant: from outboard midplane (R0+a, 0) sweeping up to (R_in_top, Z_top)
top_R = R0_PLASMA + A_PLASMA * np.cos(th_top + DELTA * np.sin(th_top))
top_Z = KAPPA_T * A_PLASMA * np.sin(th_top)
# Bottom quadrant: from outboard midplane (R0+a, 0) sweeping down
bot_R = R0_PLASMA + A_PLASMA * np.cos(-th_bot + DELTA * np.sin(-th_bot))
bot_Z = KAPPA_B * A_PLASMA * np.sin(-th_bot)

# Where each quadrant meets the centerstack / divertor:
# - Top transitions from outer-D arc to upper baffle to centerstack
# - Bottom transitions from outer-D arc to outer divertor floor at (R_OUTER_FL_MAX, Z_OUT_FLOOR)
# Trim the bottom arc at Z = Z_OUT_FLOOR (where outboard wall meets divertor)
keep = bot_Z > Z_OUT_FLOOR
bot_R = np.concatenate([bot_R[keep], [R_OUTER_FL_MAX]])
bot_Z = np.concatenate([bot_Z[keep], [Z_OUT_FLOOR]])
# Trim the top arc where R approaches centerstack (R = R_INBOARD)
keep_t = top_R > R_INBOARD + 1.0
top_R = np.concatenate([top_R[keep_t], [R_INBOARD]])
top_Z = np.concatenate([top_Z[keep_t], [top_Z[keep_t][-1]]])

# Build vessel polygon (CCW from top-inboard, down inboard wall, divertor,
# outer wall up, top dome down to inboard).
# We have:
#   top arc: from outboard midplane (R0+a, 0) up to inboard top
#   bot arc: from outboard midplane (R0+a, 0) down to outer floor
# Stitch into one closed polygon:
vessel_R = np.concatenate([
    top_R[::-1],                    # inboard-top -> outboard-midplane
    bot_R,                          # outboard-midplane -> outer floor edge
    [R_OUTER_FL_MAX, R_STEP, R_STEP, R_INNER_FL_MAX, R_INBOARD, R_INBOARD],
])
vessel_Z = np.concatenate([
    top_Z[::-1],
    bot_Z,
    [Z_OUT_FLOOR, Z_OUT_FLOOR, Z_IN_FLOOR, Z_IN_FLOOR, Z_IN_FLOOR, top_Z[0]],
])

vessel_path = MplPath(np.column_stack([vessel_R, vessel_Z]))

# ---- Tilted chassis straddling the step at orbit pose ----
# Outboard wheel pair sits on outer floor; inboard wheel pair on inner
# floor. Place the wheels so neither pair rolls onto the step face:
#   inner wheel center R: must keep wheel rim < R_STEP (R + WHEEL_R < 55.5)
#   outer wheel center R: must keep wheel rim < R_OUTER_FL_MAX (R+WHEEL_R<70.3)
# Choose outer_wheel_R = 66" (rim at 62.5..69.5, fits on outer floor),
#        inner_wheel_R = 66 - 14.5 = 51.5" (rim at 47.96..55.04, fits on
#        inner floor with ~0.5" clearance from step face).
chassis_outer_face_R = 66.0
chassis_inner_face_R = chassis_outer_face_R - CHASSIS_FOOTPRINT_RADIAL  # 51.5
# Wheel centers: assume wheel pivots are at the radial faces of the footprint
inner_wheel = np.array([chassis_inner_face_R, Z_IN_FLOOR + WHEEL_R_IN])
outer_wheel = np.array([chassis_outer_face_R, Z_OUT_FLOOR + WHEEL_R_IN])
dx_w = outer_wheel - inner_wheel
chassis_x_hat = dx_w / np.linalg.norm(dx_w)
chassis_z_hat = np.array([-chassis_x_hat[1], chassis_x_hat[0]])
tilt_rad = math.atan2(chassis_x_hat[1], chassis_x_hat[0])
tilt_deg = math.degrees(tilt_rad)

# Chassis bottom corners offset (BELLY_IN - WHEEL_R_IN) along chassis -z from
# wheel pivot line (belly is below wheel pivots since wheel sticks below).
belly_to_pivot = WHEEL_R_IN - BELLY_IN          # +1.54"
inner_bot = inner_wheel - chassis_z_hat * belly_to_pivot
outer_bot = outer_wheel - chassis_z_hat * belly_to_pivot
inner_top = inner_bot + chassis_z_hat * CHASSIS_H
outer_top = outer_bot + chassis_z_hat * CHASSIS_H
chassis_corners = np.array([inner_bot, outer_bot, outer_top, inner_top])

# J2 axis: top of J1 stack on chassis center top, in chassis +z direction
chassis_top_mid = 0.5 * (inner_top + outer_top)
# Arm mount placed nearer the outboard side of chassis (consistent with
# previous convention that J2 sits at R=66.1 area). Move J2 along chassis +x
# halfway between center and outboard top.
arm_mount_x_offset = CHASSIS_FOOTPRINT_RADIAL * 0.30   # 30% of width past center
J2_pos = chassis_top_mid + chassis_x_hat * arm_mount_x_offset \
         + chassis_z_hat * J1_STACK
J2_R = float(J2_pos[0])
J2_Z = float(J2_pos[1])

# ---- Figure ----
fig, ax = plt.subplots(figsize=(11, 9))

# Vessel fill + outline
ax.fill(vessel_R, vessel_Z, color="#fafafa", edgecolor="black",
        linewidth=2.0, zorder=1)

# Mid-plane reference
ax.axhline(0.0, color="#888888", linestyle=":", linewidth=0.8, zorder=2)
ax.text(R0_PLASMA + A_PLASMA + 1.5, 0.0, "midplane", fontsize=8,
        color="#666666", va="center")

# Step face (visible separator between inner and outer divertor floors)
ax.plot([R_STEP, R_STEP], [Z_OUT_FLOOR, Z_IN_FLOOR],
        color="black", linewidth=2.0, zorder=3)

# ---- Compute world (R,Z) for each geom-passing arm pose ----
# Arm operates in the tilted chassis frame:
#   world = J2 + (-ee_x_arm) * chassis_x_hat + ee_z_arm * chassis_z_hat
# (arm reaches in chassis -X direction = inboard / toward column at orbit pose)
ee_R, ee_Z, margins_mm = [], [], []
j3_R_all, j3_Z_all = [], []
j4_R_all, j4_Z_all = [], []
for r in geom_rows:
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

    margin_m = None
    s_m = r.get("tip_worst_margin_m", "")
    if s_m:
        try:
            margin_m = float(s_m)
        except ValueError:
            margin_m = None
    if margin_m is None:
        tip = arm_ik.tipping_sweep_j1(j2, j3, j4, arm_params)
        margin_m = tip["worst_margin"]

    ee_R.append(R_ee); ee_Z.append(Z_ee)
    j3_R_all.append(R_j3); j3_Z_all.append(Z_j3)
    j4_R_all.append(R_j4); j4_Z_all.append(Z_j4)
    margins_mm.append(margin_m * 1000.0)

ee_R = np.array(ee_R); ee_Z = np.array(ee_Z)
j3_R_all = np.array(j3_R_all); j3_Z_all = np.array(j3_Z_all)
j4_R_all = np.array(j4_R_all); j4_Z_all = np.array(j4_Z_all)
margins_mm = np.array(margins_mm)

# Filter to poses where ALL linkage points lie inside the vessel polygon
# (so poses that visually hit the vessel wall in this view are dropped).
def all_inside(idx):
    pts = np.array([
        [J2_R, J2_Z],
        [j3_R_all[idx], j3_Z_all[idx]],
        [j4_R_all[idx], j4_Z_all[idx]],
        [ee_R[idx],     ee_Z[idx]],
    ])
    return bool(np.all(vessel_path.contains_points(pts)))

inside_mask = np.array([all_inside(i) for i in range(len(ee_R))])
n_total_raw = len(ee_R)
n_inside = int(inside_mask.sum())

ee_R_v   = ee_R[inside_mask];   ee_Z_v   = ee_Z[inside_mask]
j3_R_v   = j3_R_all[inside_mask]; j3_Z_v = j3_Z_all[inside_mask]
j4_R_v   = j4_R_all[inside_mask]; j4_Z_v = j4_Z_all[inside_mask]
margins_v = margins_mm[inside_mask]
n_stable_v = int(np.sum(margins_v > 0))

# ---- Arm linkage lines (faint background + sub-sampled foreground) ----
seg_R = np.column_stack([np.full(n_inside, J2_R), j3_R_v, j4_R_v, ee_R_v])
seg_Z = np.column_stack([np.full(n_inside, J2_Z), j3_Z_v, j4_Z_v, ee_Z_v])
for i in range(n_inside):
    ax.plot(seg_R[i], seg_Z[i], color="#5a7fb8",
            linewidth=0.35, alpha=0.10, zorder=4)
n_show = min(40, n_inside)
if n_inside > 0:
    sample_idx = np.linspace(0, n_inside - 1, n_show).astype(int)
    for i in sample_idx:
        ax.plot(seg_R[i], seg_Z[i], color="#1f3d6b",
                linewidth=0.9, alpha=0.55, zorder=4.5)

# ---- EE scatter ----
if n_inside > 0:
    vmax = max(40.0, np.percentile(np.abs(margins_v), 99))
    sc = ax.scatter(ee_R_v, ee_Z_v, c=margins_v, cmap="RdYlGn",
                    vmin=-vmax, vmax=vmax, s=14, alpha=0.85,
                    edgecolors="none", zorder=5)
    plt.colorbar(sc, ax=ax, label="Tipping margin (mm)", shrink=0.6,
                 pad=0.02)

# ---- Tilted chassis polygon ----
chassis_poly = MplPolygon(chassis_corners, closed=True,
                          facecolor="#9fbbe0", edgecolor="#1f3d6b",
                          linewidth=1.6, alpha=0.95, zorder=6,
                          joinstyle="round")
ax.add_patch(chassis_poly)
# J1 stack column (perpendicular to chassis top)
top_anchor = chassis_top_mid + chassis_x_hat * arm_mount_x_offset
ax.plot([top_anchor[0], J2_R], [top_anchor[1], J2_Z],
        color="#1f3d6b", linewidth=2.2, zorder=7)
# Wheels at the two pivots
for wp in (inner_wheel, outer_wheel):
    ax.add_patch(plt.Circle((wp[0], wp[1]), WHEEL_R_IN,
                            facecolor="#444", edgecolor="black",
                            linewidth=0.8, zorder=7))

# J2 marker
ax.scatter([J2_R], [J2_Z], marker="s", color="black", s=70, zorder=8)
ax.annotate("J2", (J2_R, J2_Z), xytext=(7, 4), textcoords="offset points",
            fontsize=10, fontweight="bold", zorder=9)

# Labels
ax.text(R_INBOARD - 1.0, 0.0, "centerstack",
        rotation=90, ha="right", va="center",
        fontsize=9, color="#444444")
ax.text(R0_PLASMA + A_PLASMA + 0.5, -10.0, "outer wall",
        rotation=-90, ha="left", va="center",
        fontsize=9, color="#444444")
ax.text(R_INBOARD + 12, Z_IN_FLOOR - 2.5,
        "lower divertor", ha="left", va="top",
        fontsize=9, color="#444444")

ax.set_xlabel("R (inches from reactor center)")
ax.set_ylabel("Z (inches)")
# Crop to lower half of vessel (where FORTIS operates) to keep the chassis
# and arm reach readable. Vessel extends to Z = +60" but the lower-half
# view is what matters here.
xlim_lo = R_INBOARD - 4
xlim_hi = R0_PLASMA + A_PLASMA + 8
ylim_lo = Z_IN_FLOOR - 4
ylim_hi = 22.0
ax.set_xlim(xlim_lo, xlim_hi)
ax.set_ylim(ylim_lo, ylim_hi)
ax.set_aspect("equal")
ax.grid(True, alpha=0.25)

stable_pct = 100.0 * n_stable_v / max(n_inside, 1)
ax.set_title(
    f'FORTIS at orbit pose (chassis tilt {tilt_deg:.1f} deg over step)  --  '
    f'30" CF loaded\n{n_inside} of {n_total_raw} arm poses inside vessel, '
    f'{n_stable_v} stable ({stable_pct:.0f}%)',
    fontsize=11)
plt.suptitle("DIII-D Poloidal Cross-Section", fontsize=14,
             fontweight="bold", y=0.97)
plt.tight_layout(rect=(0, 0, 1, 0.96))
out = os.path.join(out_dir, f"poloidal_v3{SUFFIX}.png")
plt.savefig(out, dpi=170)
plt.close(fig)
print(f"wrote {out}  ({n_inside}/{n_total_raw} inside, "
      f"{n_stable_v} stable, tilt={tilt_deg:.1f} deg)")


# ============================================================================
# Bar chart (kept for diagnostics)
# ============================================================================
fig, ax = plt.subplots(figsize=(10, 6))
joints = ("j2", "j3", "j4")
x = np.arange(len(joints))
w = 0.27
means = [stats[jn]["mean"] if stats[jn] else 0 for jn in joints]
p95s  = [stats[jn]["p95"]  if stats[jn] else 0 for jn in joints]
peaks = [stats[jn]["peak"] if stats[jn] else 0 for jn in joints]

ax.bar(x - w, means, w, label="Mean", color="#1f77b4")
ax.bar(x,     p95s,  w, label="P95",  color="#ff7f0e")
ax.bar(x + w, peaks, w, label="Peak", color="#d62728")
for i, jn in enumerate(joints):
    cont = TORQUE_LIMIT[jn]
    ax.hlines(cont, i - 1.5*w, i + 1.5*w, colors="gray",
              linestyles="--", linewidth=1.5,
              label="Continuous rating" if i == 0 else None)
for i, jn in enumerate(joints):
    if stats[jn]:
        ax.text(i - w, means[i] + 0.4, f"{means[i]:.2f}", ha="center", fontsize=9)
        ax.text(i,     p95s[i]  + 0.4, f"{p95s[i]:.2f}",  ha="center", fontsize=9)
        ax.text(i + w, peaks[i] + 0.4, f"{peaks[i]:.2f}", ha="center", fontsize=9)
ax.set_xticks(x)
ax.set_xticklabels([LABEL[j] for j in joints])
ax.set_ylabel("|Torque| (Nm)")
ax.set_title(f"FORTIS v3 arm torques -- 30\" CF, loaded, {len(valid_rows)} valid poses",
             fontweight="bold")
ax.legend(loc="upper right")
ax.grid(True, alpha=0.3)
plt.tight_layout()
out = os.path.join(out_dir, f"torques_v3{SUFFIX}.png")
plt.savefig(out, dpi=150)
plt.close(fig)
print(f"wrote {out}")


# ============================================================================
# Histograms (kept for diagnostics)
# ============================================================================
fig, axes = plt.subplots(1, 3, figsize=(15, 4))
for i, jn in enumerate(joints):
    ax = axes[i]
    if len(tau[jn]) == 0:
        ax.set_title(f"{LABEL[jn]} (no data)")
        continue
    ax.hist(tau[jn], bins=40, color="#1f77b4", edgecolor="black", alpha=0.7)
    ax.axvline(stats[jn]["mean"], color="black", linestyle="-", linewidth=1.5,
               label=f"mean {stats[jn]['mean']:.2f}")
    ax.axvline(stats[jn]["p95"], color="orange", linestyle="--", linewidth=1.5,
               label=f"p95 {stats[jn]['p95']:.2f}")
    ax.axvline(TORQUE_LIMIT[jn], color="gray", linestyle="--", linewidth=1.5,
               label=f"rating {TORQUE_LIMIT[jn]:.1f}")
    ax.set_xlabel("|Torque| (Nm)")
    ax.set_ylabel("Pose count")
    ax.set_title(LABEL[jn])
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
plt.tight_layout()
out = os.path.join(out_dir, f"torque_hist_v3{SUFFIX}.png")
plt.savefig(out, dpi=150)
plt.close(fig)
print(f"wrote {out}")
