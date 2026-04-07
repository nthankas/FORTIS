#!/usr/bin/env python3
"""
Analytical arm torque sweep — gravity torques + chassis stability.

Sweeps J2-J5 angles, computes gravity torque at each joint, filters for
arm-above-ground, checks chassis tipping at all J1 yaw angles.
No Isaac Sim dependency — pure numpy + matplotlib.

Usage:
    python tools/arm_torque_sweep.py
    (or IsaacSim\\python.bat tools/arm_torque_sweep.py)

Output → results/arm_torque_sweep/
"""
import numpy as np
import os, sys, time, json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
XDRIVE = os.path.dirname(SCRIPT_DIR)
OUT = os.path.join(XDRIVE, "results", "arm_torque_sweep")
os.makedirs(OUT, exist_ok=True)

# ---------------------------------------------------------------------------
# Constants — must match canonical/xdrive_reactor_arm.py & docs/arm_spec.md
# ---------------------------------------------------------------------------
IN = 0.0254          # inches → meters
g = 9.81             # m/s²

# Link lengths and stowed X-directions
LL = np.array([18.0, 19.0, 9.0, 4.0]) * IN       # L2, L3, L4, L5
DD = np.array([+1, -1, +1, -1], dtype=float)      # +X, -X, +X, -X

# Masses
MJ  = 0.629                             # joint mass (motor 0.58 + hw 0.049)
CF  = 0.0053 * 0.453592 / IN            # CF tube density kg/m
ML  = CF * LL                            # link masses [L2, L3, L4, L5]
MC  = 0.098                              # camera at L4 midpoint

# Per-body totals (for printout)
BODY = {
    "J1_base":     MJ,
    "J2_shoulder": MJ + ML[0],
    "J3_elbow":    MJ + ML[1],
    "J4_wrist":    MJ + ML[2] + MC,
    "J5_ee":       MJ + ML[3],
}
M_ARM = sum(BODY.values())

# Chassis
CL, CW, CH = 15.354 * IN, 9.353 * IN, 7.1 * IN
CHASSIS_KG = 13.6                        # 30 lb (worst case for tipping)
M_TOT = CHASSIS_KG + M_ARM

# Arm geometry
ARM_X  = -CL / 2.0                      # mount at back edge
J1_H   = 0.086                          # J1 motor stack height
WR     = 0.1015                          # wheel radius
Z_J2   = WR + CH + J1_H                 # J2 height above ground (~0.37 m)

# Support polygon — wheel contact XY in chassis frame
# 3" chamfers at 45° corners; wheel at chamfer midpoint
CHAM   = 3.0 * IN
C_OFF  = CHAM / (2 * np.sqrt(2))        # chamfer inset per axis
WPX    = CL / 2.0 - C_OFF               # ≈0.168 m (6.6")
WPY    = CW / 2.0 - C_OFF               # ≈0.092 m (3.6")

# Motor limits (for reference lines)
GB_RATED = 11.0                          # Cricket MK II rated Nm
MOT_MAX  = 0.80 * 25                    # motor torque × gear ratio = 20 Nm

# ---------------------------------------------------------------------------
# Sweep grid
# ---------------------------------------------------------------------------
T2D = np.arange(-30,  151, 10)          # 19 values
T3D = np.arange(-150, 151, 10)          # 31 values
T4D = np.arange(-150, 151, 15)          # 21 values
T5D = np.arange(-90,   91, 30)          # 7 values
T1D = np.arange(0, 360, 15)             # 24 values (tipping only)

NT = len(T2D) * len(T3D) * len(T4D) * len(T5D)

print(f"Torque grid : {len(T2D)}×{len(T3D)}×{len(T4D)}×{len(T5D)} = {NT:,} poses")
print(f"Tipping grid: ×{len(T1D)} J1 yaws = {NT * len(T1D):,} checks")
print(f"Arm mass    : {M_ARM:.3f} kg ({M_ARM * 2.205:.2f} lb)")
print(f"Chassis mass: {CHASSIS_KG:.1f} kg ({CHASSIS_KG * 2.205:.0f} lb)")
print(f"Estimated   : <5 s analytical, 0 % GPU\n")

t0 = time.time()

# ---------------------------------------------------------------------------
# Forward kinematics (vectorised)
# ---------------------------------------------------------------------------
T2, T3, T4, T5 = np.meshgrid(
    np.deg2rad(T2D), np.deg2rad(T3D),
    np.deg2rad(T4D), np.deg2rad(T5D), indexing="ij")

A2    = T2
A23   = T2 + T3
A234  = T2 + T3 + T4
A2345 = T2 + T3 + T4 + T5

# Joint X,Z positions relative to J2 in arm's XZ plane.
# R_y(α) * (d, 0, 0) = (d cos α, 0, −d sin α)
j3x =                     DD[0] * LL[0] * np.cos(A2)
j3z =                    -DD[0] * LL[0] * np.sin(A2)
j4x = j3x +               DD[1] * LL[1] * np.cos(A23)
j4z = j3z -               DD[1] * LL[1] * np.sin(A23)
j5x = j4x +               DD[2] * LL[2] * np.cos(A234)
j5z = j4z -               DD[2] * LL[2] * np.sin(A234)
eex = j5x +               DD[3] * LL[3] * np.cos(A2345)
eez = j5z -               DD[3] * LL[3] * np.sin(A2345)

# Link-midpoint X positions (mass centres)
xL2 =        DD[0] * LL[0] / 2 * np.cos(A2)
xL3 = j3x + DD[1] * LL[1] / 2 * np.cos(A23)
xL4 = j4x + DD[2] * LL[2] / 2 * np.cos(A234)    # = camera position
xL5 = j5x + DD[3] * LL[3] / 2 * np.cos(A2345)

# ---------------------------------------------------------------------------
# Above-ground filter — all joints & EE must have world Z > 0
# ---------------------------------------------------------------------------
minz = np.minimum(np.minimum(j3z, j4z), np.minimum(j5z, eez))
ok = (minz + Z_J2) > 0.0
nok = int(ok.sum())
print(f"Above ground: {nok:,} / {NT:,} ({100 * nok / NT:.1f}%)")

NAN = np.nan
def masked(a):
    return np.where(ok, a, NAN)

# ---------------------------------------------------------------------------
# Gravity torques: τ_Ji = g Σ m_k (x_k − x_Ji)  for outboard masses
# Motor at its own pivot ⇒ zero moment arm about that joint.
# ---------------------------------------------------------------------------
tau2 = masked(g * (
    ML[0] * xL2
    + MJ * j3x  + ML[1] * xL3
    + MJ * j4x  + (ML[2] + MC) * xL4
    + MJ * j5x  + ML[3] * xL5))

tau3 = masked(g * (
    ML[1] * (xL3 - j3x)
    + MJ * (j4x - j3x) + (ML[2] + MC) * (xL4 - j3x)
    + MJ * (j5x - j3x) + ML[3] * (xL5 - j3x)))

tau4 = masked(g * (
    (ML[2] + MC) * (xL4 - j4x)
    + MJ * (j5x - j4x) + ML[3] * (xL5 - j4x)))

tau5 = masked(g * (ML[3] * (xL5 - j5x)))

# ---------------------------------------------------------------------------
# Peak torques + worst-case poses
# ---------------------------------------------------------------------------
def peak(tau):
    pk = float(np.nanmax(np.abs(tau)))
    idx = np.unravel_index(np.nanargmax(np.abs(tau)), tau.shape)
    wp = (float(T2D[idx[0]]), float(T3D[idx[1]]),
          float(T4D[idx[2]]), float(T5D[idx[3]]))
    return {"peak_Nm": pk, "worst_pose_deg": wp,
            "worst_tau_Nm": float(tau[idx])}

peaks = {"J2": peak(tau2), "J3": peak(tau3),
         "J4": peak(tau4), "J5": peak(tau5)}

# Envelopes: max |τ| at each angle of that joint (collapse other axes)
e2 = np.nanmax(np.abs(tau2), axis=(1, 2, 3))
e3 = np.nanmax(np.abs(tau3), axis=(0, 2, 3))
e4 = np.nanmax(np.abs(tau4), axis=(0, 1, 3))
e5 = np.nanmax(np.abs(tau5), axis=(0, 1, 2))

# Signed envelopes (for shaded band)
e2hi, e2lo = np.nanmax(tau2, axis=(1,2,3)), np.nanmin(tau2, axis=(1,2,3))
e3hi, e3lo = np.nanmax(tau3, axis=(0,2,3)), np.nanmin(tau3, axis=(0,2,3))
e4hi, e4lo = np.nanmax(tau4, axis=(0,1,3)), np.nanmin(tau4, axis=(0,1,3))

# ---------------------------------------------------------------------------
# Tipping analysis
# ---------------------------------------------------------------------------
# Outboard mass (everything except J1 & J2 motors at the base)
M_base = 2 * MJ
M_out  = M_ARM - M_base

# Outboard COM x in arm frame (relative to J2, per pose)
out_cx = (ML[0] * xL2 + MJ * j3x + ML[1] * xL3
          + MJ * j4x + (ML[2] + MC) * xL4
          + MJ * j5x + ML[3] * xL5) / M_out

tips = []
for t1d in T1D:
    c, s = np.cos(np.deg2rad(t1d)), np.sin(np.deg2rad(t1d))
    # Combined COM in chassis XY
    cx = (CHASSIS_KG * 0 + M_base * ARM_X + M_out * (ARM_X + out_cx * c)) / M_TOT
    cy = (M_out * out_cx * s) / M_TOT
    # Margins to each edge of the rectangular support polygon
    mg = np.stack([WPX - cx, cx + WPX, WPY - cy, cy + WPY])
    mm = np.where(ok, mg.min(axis=0), NAN)
    nt = int(np.nansum(mm < 0))
    wm = float(np.nanmin(mm))
    tips.append({"j1_deg": float(t1d), "n_tip": nt, "n_ok": nok,
                 "pct": float(100 * nt / max(nok, 1)),
                 "worst_margin_m": wm, "worst_margin_in": wm / IN})

elapsed = time.time() - t0

# ---------------------------------------------------------------------------
# Print results
# ---------------------------------------------------------------------------
print(f"\nCompleted in {elapsed:.2f}s  |  GPU: 0%")

print(f"\n{'=' * 65}")
print("PEAK GRAVITY TORQUES  (above-ground poses only)")
print(f"{'=' * 65}")
for jn in ("J2", "J3", "J4", "J5"):
    p = peaks[jn]
    a = p["worst_pose_deg"]
    print(f"  {jn}: {p['peak_Nm']:7.2f} Nm   "
          f"worst=(t2={a[0]:+4.0f}  t3={a[1]:+4.0f}  t4={a[2]:+4.0f}  t5={a[3]:+4.0f})  "
          f"tau={p['worst_tau_Nm']:+.2f} Nm")
print(f"\n  Cricket MK II gearbox rated : {GB_RATED} Nm")
print(f"  Motor × ratio theoretical  : {MOT_MAX:.0f} Nm")
for jn in ("J2", "J3", "J4", "J5"):
    pk = peaks[jn]["peak_Nm"]
    if pk > GB_RATED:
        margin = pk / GB_RATED
        print(f"  ** {jn} peak ({pk:.1f} Nm) EXCEEDS gearbox rated "
              f"({GB_RATED} Nm) by {margin:.1f}x")

print(f"\n{'=' * 65}")
print(f"STABILITY  (chassis {CHASSIS_KG:.0f} kg / {CHASSIS_KG * 2.205:.0f} lb)")
print(f"{'=' * 65}")
print(f"  Support polygon: ±{WPX / IN:.2f}\" × ±{WPY / IN:.2f}\"")
any_tip = any(t["n_tip"] > 0 for t in tips)
if any_tip:
    for t in tips:
        if t["n_tip"] > 0:
            print(f"  J1={t['j1_deg']:5.0f}deg: {t['n_tip']:5d} tipped "
                  f"({t['pct']:.1f}%)  worst margin = {t['worst_margin_in']:+.2f}\"")
    tot_tip = sum(t["n_tip"] for t in tips)
    tot_chk = sum(t["n_ok"] for t in tips)
    print(f"\n  TOTAL: {tot_tip:,} / {tot_chk:,} pose-yaw combos tipped "
          f"({100 * tot_tip / tot_chk:.2f}%)")
else:
    wm = min(t["worst_margin_m"] for t in tips)
    print(f"  No tipping at any pose.  Min margin: "
          f"{wm / IN:.2f}\" ({wm * 100:.1f} cm)")

print(f"\n{'=' * 65}")
print("SIM-BASED TIMING ESTIMATE")
print(f"{'=' * 65}")
spp = 0.5  # seconds per pose, headless, ~120 frame settle
print(f"  Per pose (headless, 120-frame settle) : ~{spp}s")
print(f"  This grid ({NT:,} poses)              : ~{NT * spp / 3600:.1f} h")
print(f"  + J1 sweep ({NT * len(T1D):,} poses)  : ~{NT * len(T1D) * spp / 3600:.1f} h")
print(f"  Physics runs on CPU @ 360 Hz — GPU idle in headless mode.")
print(f"  GPU heating: negligible for torque sweeps.")

# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(3, 2, figsize=(14, 17))
fig.suptitle(
    f"FORTIS 5-DOF Arm — Gravity Torque Sweep\n"
    f"{nok:,} valid poses  |  Chassis {CHASSIS_KG:.0f} kg  |  "
    f"Arm {M_ARM:.3f} kg ({M_ARM * 2.205:.1f} lb)", fontsize=13)

# ---- J2 envelope ----
ax = axes[0, 0]
ax.fill_between(T2D, e2lo, e2hi, alpha=0.25, color="red")
ax.plot(T2D, e2, "r-o", ms=3, lw=1.5, label="|τ| max envelope")
ax.axhline(+GB_RATED, color="orange", ls="--", lw=1, alpha=0.7,
           label=f"Gearbox rated ±{GB_RATED} Nm")
ax.axhline(-GB_RATED, color="orange", ls="--", lw=1, alpha=0.7)
ax.set_title(f"J2 Shoulder  —  peak {peaks['J2']['peak_Nm']:.1f} Nm", fontsize=11)
ax.set_xlabel("θ₂ (deg)")
ax.set_ylabel("Torque (Nm)")
ax.legend(fontsize=7, loc="upper left")
ax.grid(True, alpha=0.3)

# ---- J3 envelope ----
ax = axes[0, 1]
ax.fill_between(T3D, e3lo, e3hi, alpha=0.25, color="green")
ax.plot(T3D, e3, "g-s", ms=2.5, lw=1.5, label="|τ| max envelope")
ax.axhline(+GB_RATED, color="orange", ls="--", lw=1, alpha=0.7,
           label=f"±{GB_RATED} Nm")
ax.axhline(-GB_RATED, color="orange", ls="--", lw=1, alpha=0.7)
ax.set_title(f"J3 Elbow  —  peak {peaks['J3']['peak_Nm']:.1f} Nm", fontsize=11)
ax.set_xlabel("θ₃ (deg)")
ax.set_ylabel("Torque (Nm)")
ax.legend(fontsize=7, loc="upper left")
ax.grid(True, alpha=0.3)

# ---- J4 envelope ----
ax = axes[1, 0]
ax.fill_between(T4D, e4lo, e4hi, alpha=0.25, color="blue")
ax.plot(T4D, e4, "b-^", ms=2.5, lw=1.5, label="|τ| max envelope")
ax.axhline(+GB_RATED, color="orange", ls="--", lw=1, alpha=0.7,
           label=f"±{GB_RATED} Nm")
ax.axhline(-GB_RATED, color="orange", ls="--", lw=1, alpha=0.7)
ax.set_title(f"J4 Wrist  —  peak {peaks['J4']['peak_Nm']:.1f} Nm", fontsize=11)
ax.set_xlabel("θ₄ (deg)")
ax.set_ylabel("Torque (Nm)")
ax.legend(fontsize=7, loc="upper left")
ax.grid(True, alpha=0.3)

# ---- J5 envelope (smaller, combined with peak bar) ----
ax = axes[1, 1]
e5hi = np.nanmax(tau5, axis=(0, 1, 2))
e5lo = np.nanmin(tau5, axis=(0, 1, 2))
ax.fill_between(T5D, e5lo, e5hi, alpha=0.25, color="purple")
ax.plot(T5D, e5, "m-d", ms=4, lw=1.5, label="|τ| max envelope")
ax.axhline(+GB_RATED, color="orange", ls="--", lw=1, alpha=0.7,
           label=f"±{GB_RATED} Nm")
ax.axhline(-GB_RATED, color="orange", ls="--", lw=1, alpha=0.7)
ax.set_title(f"J5 End-effector  —  peak {peaks['J5']['peak_Nm']:.1f} Nm",
             fontsize=11)
ax.set_xlabel("θ₅ (deg)")
ax.set_ylabel("Torque (Nm)")
ax.legend(fontsize=7, loc="upper left")
ax.grid(True, alpha=0.3)

# ---- Peak torques bar chart ----
ax = axes[2, 0]
jnames = ["J2", "J3", "J4", "J5"]
pvals = [peaks[j]["peak_Nm"] for j in jnames]
cols = ["#d32f2f", "#388e3c", "#1976d2", "#7b1fa2"]
bars = ax.bar(jnames, pvals, color=cols, alpha=0.85, edgecolor="black", lw=0.8)
ax.axhline(GB_RATED, color="orange", ls="--", lw=2,
           label=f"Gearbox rated ({GB_RATED} Nm)")
ax.axhline(MOT_MAX, color="red", ls=":", lw=1.5,
           label=f"Motor×ratio ({MOT_MAX:.0f} Nm)")
for bar, val in zip(bars, pvals):
    c = "red" if val > GB_RATED else "black"
    ax.text(bar.get_x() + bar.get_width() / 2, val + 0.3,
            f"{val:.1f}", ha="center", va="bottom", fontsize=10,
            fontweight="bold", color=c)
ax.set_ylabel("Peak |torque| (Nm)")
ax.set_title("Peak Gravity Torque per Joint", fontsize=11)
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3, axis="y")
ax.set_ylim(0, max(pvals) * 1.2)

# ---- Worst-case arm + tipping margin ----
ax = axes[2, 1]

# Draw worst-case J2 arm pose as stick figure
wa = peaks["J2"]["worst_pose_deg"]
r2, r3, r4, r5 = np.deg2rad(wa)
a2_ = r2; a23_ = r2 + r3; a234_ = r2 + r3 + r4; a2345_ = r2 + r3 + r4 + r5
px = [0.0]; pz = [0.0]
px.append(DD[0] * LL[0] * np.cos(a2_))
pz.append(-DD[0] * LL[0] * np.sin(a2_))
px.append(px[-1] + DD[1] * LL[1] * np.cos(a23_))
pz.append(pz[-1] - DD[1] * LL[1] * np.sin(a23_))
px.append(px[-1] + DD[2] * LL[2] * np.cos(a234_))
pz.append(pz[-1] - DD[2] * LL[2] * np.sin(a234_))
px.append(px[-1] + DD[3] * LL[3] * np.cos(a2345_))
pz.append(pz[-1] - DD[3] * LL[3] * np.sin(a2345_))
pxi = [x / IN for x in px]
pzi = [z / IN for z in pz]

ax.plot(pxi, pzi, "k-o", lw=2.5, ms=6, zorder=3)
for i, lbl in enumerate(["J2", "J3", "J4", "J5", "EE"]):
    ax.annotate(lbl, (pxi[i], pzi[i]), textcoords="offset points",
                xytext=(5, 5), fontsize=8, color="blue")
ax.axhline(-Z_J2 / IN, color="brown", ls="-", lw=2, alpha=0.5, label="Ground")
ax.plot(0, 0, "rs", ms=10, zorder=5, label="J2 pivot")
ax.set_xlabel("X (inches from J2)")
ax.set_ylabel("Z (inches from J2)")
ax.set_title(
    f"Worst J2 pose: ({wa[0]:+.0f}, {wa[1]:+.0f}, {wa[2]:+.0f}, {wa[3]:+.0f})°\n"
    f"τ_J2 = {peaks['J2']['worst_tau_Nm']:+.1f} Nm", fontsize=10)
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)
ax.legend(fontsize=7)

plt.tight_layout(rect=[0, 0, 1, 0.96])
plot_path = os.path.join(OUT, "torque_sweep.png")
plt.savefig(plot_path, dpi=150, bbox_inches="tight")
print(f"\nPlot saved : {os.path.relpath(plot_path, XDRIVE)}")

# ---- Tipping figure ----
fig2, ax2 = plt.subplots(figsize=(10, 5))
j1a = [t["j1_deg"] for t in tips]
wms = [t["worst_margin_in"] for t in tips]
ax2.plot(j1a, wms, "k-o", ms=4, lw=1.5)
ax2.axhline(0, color="red", ls="--", lw=2, label="Tipping threshold")
neg = [m < 0 for m in wms]
if any(neg):
    ax2.fill_between(j1a, wms, 0, where=neg, color="red", alpha=0.2,
                     label="Tipped region")
ax2.set_xlabel("J1 Yaw (deg)")
ax2.set_ylabel("Min stability margin (inches)")
ax2.set_title(f"Chassis Tipping Margin vs J1 Yaw\n"
              f"Chassis {CHASSIS_KG:.0f} kg ({CHASSIS_KG * 2.205:.0f} lb) + "
              f"arm {M_ARM:.2f} kg  |  support ±{WPX / IN:.1f}\" × ±{WPY / IN:.1f}\"",
              fontsize=11)
ax2.grid(True, alpha=0.3)
ax2.legend(fontsize=9)
stab_path = os.path.join(OUT, "stability.png")
plt.tight_layout()
plt.savefig(stab_path, dpi=150, bbox_inches="tight")
print(f"Stability  : {os.path.relpath(stab_path, XDRIVE)}")

# ---------------------------------------------------------------------------
# Save JSON
# ---------------------------------------------------------------------------
def conv(o):
    if isinstance(o, (np.integer,)):   return int(o)
    if isinstance(o, (np.floating,)):  return float(o)
    if isinstance(o, np.ndarray):      return o.tolist()
    if isinstance(o, tuple):           return list(o)
    return o

results = {
    "grid": {
        "T2_deg": T2D.tolist(), "T3_deg": T3D.tolist(),
        "T4_deg": T4D.tolist(), "T5_deg": T5D.tolist(),
        "T1_deg": T1D.tolist(),
        "n_total": NT, "n_valid": nok,
    },
    "peak_torques": peaks,
    "envelopes": {
        "J2": {"angle_deg": T2D.tolist(), "max_abs_Nm": e2.tolist(),
               "hi_Nm": e2hi.tolist(), "lo_Nm": e2lo.tolist()},
        "J3": {"angle_deg": T3D.tolist(), "max_abs_Nm": e3.tolist(),
               "hi_Nm": e3hi.tolist(), "lo_Nm": e3lo.tolist()},
        "J4": {"angle_deg": T4D.tolist(), "max_abs_Nm": e4.tolist(),
               "hi_Nm": e4hi.tolist(), "lo_Nm": e4lo.tolist()},
        "J5": {"angle_deg": T5D.tolist(), "max_abs_Nm": e5.tolist(),
               "hi_Nm": e5hi.tolist(), "lo_Nm": e5lo.tolist()},
    },
    "tipping": tips,
    "masses_kg": {"arm": M_ARM, "chassis": CHASSIS_KG, "total": M_TOT,
                  "bodies": {k: float(v) for k, v in BODY.items()}},
    "elapsed_s": elapsed,
}
json_path = os.path.join(OUT, "sweep_results.json")
with open(json_path, "w") as f:
    json.dump(results, f, indent=2, default=conv)
print(f"JSON saved : {os.path.relpath(json_path, XDRIVE)}")
print(f"\nDone ({elapsed:.2f}s).")
