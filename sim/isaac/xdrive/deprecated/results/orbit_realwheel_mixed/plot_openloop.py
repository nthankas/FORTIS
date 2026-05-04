"""Plot mean / P95 / peak torque per wheel vs commanded orbit speed.

Reads sweep_orbit_openloop.json and renders a 2x1 figure: top = mean+P95
torques per wheel, bottom = numeric table. M8325s continuous (3.32 Nm),
peak (4.98 Nm), and absolute peak (6.64 Nm) thresholds drawn.
"""
import json, os
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
data = json.load(open(os.path.join(HERE, "sweep_orbit_openloop.json")))
speeds = sorted(float(k) for k in data.keys())
wheels = ["FL", "FR", "BL", "BR"]
colors = {"FL": "tab:blue", "FR": "tab:orange", "BL": "tab:green", "BR": "tab:red"}

means = {w: [data[f"{s:.2f}"][w]["mean"] for s in speeds] for w in wheels}
p95s  = {w: [data[f"{s:.2f}"][w]["p95"]  for s in speeds] for w in wheels}
peaks = {w: [data[f"{s:.2f}"][w]["peak"] for s in speeds] for w in wheels}
trks  = [data[f"{s:.2f}"]["tracking_pct"] for s in speeds]

# M8325s thresholds
T_CONT = 3.32
T_PEAK = 4.98
T_ABS  = 6.64

fig, axes = plt.subplots(2, 1, figsize=(11, 11),
                          gridspec_kw={"height_ratios": [3, 2]})
ax = axes[0]
x = np.array(speeds)
width = 0.018
for i, w in enumerate(wheels):
    off = (i - 1.5) * width
    ax.bar(x + off, means[w], width=width, color=colors[w], alpha=0.8,
           label=f"{w} mean")
    ax.scatter(x + off, p95s[w], color=colors[w], marker="^", s=70,
               edgecolor="black", zorder=5,
               label=f"{w} P95" if i == 0 else None)
    ax.scatter(x + off, peaks[w], color=colors[w], marker="x", s=70,
               linewidth=2, zorder=6,
               label=f"{w} peak" if i == 0 else None)

for thr, lbl, col in [(T_CONT, "M8325s continuous (3.32)", "g"),
                       (T_PEAK, "M8325s peak (4.98)",       "y"),
                       (T_ABS,  "M8325s absolute (6.64)",   "r")]:
    ax.axhline(thr, linestyle="--", color=col, alpha=0.7)
    ax.text(x[-1] + 0.005, thr + 0.05, lbl, color=col, fontsize=8)

ax.set_xlabel("Commanded orbit radial speed (m/s)")
ax.set_ylabel("Wheel torque (Nm)")
ax.set_title("X-drive orbit-on-step torque sweep (open-loop FF, 7 Nm motor cap)")
ax.set_xticks(x)
ax.set_xticklabels([f"{s:.2f}" for s in speeds])
ax.grid(True, alpha=0.3)
ax.legend(ncol=4, fontsize=8, loc="upper left")

# Numeric table
ax2 = axes[1]
ax2.axis("off")
hdrs = ["v_cmd", "track%", "path m"] + sum(
    [[f"{w} mean", f"{w} P95", f"{w} pk"] for w in wheels], [])
rows = []
for i, s in enumerate(speeds):
    d = data[f"{s:.2f}"]
    row = [f"{s:.2f}", f"{d['tracking_pct']:.0f}", f"{d['path_len_m']:.2f}"]
    for w in wheels:
        row += [f"{d[w]['mean']:.2f}", f"{d[w]['p95']:.2f}", f"{d[w]['peak']:.2f}"]
    rows.append(row)
tbl = ax2.table(cellText=rows, colLabels=hdrs, loc="center", cellLoc="center")
tbl.auto_set_font_size(False)
tbl.set_fontsize(9)
tbl.scale(1, 1.4)

plt.tight_layout()
out = os.path.join(HERE, "torque_sweep_openloop.png")
plt.savefig(out, dpi=130)
print(f"Wrote {out}")
