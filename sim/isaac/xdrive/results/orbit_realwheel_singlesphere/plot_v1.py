"""Plot v1 realwheel orbit-sweep torques.

Reads sweep_orbit_realwheel.json and produces two charts:
  - Mean torque per wheel vs commanded speed
  - P95  torque per wheel vs commanded speed
Each chart annotates every data point with its numeric value, and overlays
ODrive M8325s 100 KV direct-drive thresholds (3.32 / 4.98 / 6.64 Nm).
"""
import os, sys, json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "sweep_orbit_realwheel.json")) as f:
    data = json.load(f)

speeds = sorted(float(k) for k in data.keys())
WHEELS = ["FL", "FR", "BL", "BR"]
COLORS = {"FL": "#1f77b4", "FR": "#d62728", "BL": "#2ca02c", "BR": "#9467bd"}

THRESH = [
    (3.32, "Cont. 40 A (free air)", "--"),
    (4.98, "Cont. 60 A (forced air)", "-."),
    (6.64, "Peak 80 A (3 s)", ":"),
]

fig, axes = plt.subplots(1, 2, figsize=(18, 9), sharey=True)

for ax, stat, label in zip(axes, ("mean", "p95"), ("Mean torque", "P95 torque")):
    for w in WHEELS:
        vals = [data[f"{s:.2f}"][f"torque_{w}"][stat] for s in speeds]
        ax.plot(speeds, vals, "o-", color=COLORS[w], linewidth=2.0,
                markersize=8, label=w)
        for s, v in zip(speeds, vals):
            ax.annotate(f"{v:.2f}", (s, v),
                        xytext=(0, 8), textcoords="offset points",
                        ha="center", fontsize=8, color=COLORS[w])
    for t, lab, ls in THRESH:
        ax.axhline(t, color="gray", linestyle=ls, linewidth=1.0, alpha=0.7)
        ax.text(speeds[0] - 0.018, t, f"{lab}\n{t:.2f} Nm",
                fontsize=8, color="#444", va="center", ha="right")
    ax.set_xlabel("Commanded orbit speed (m/s)", fontsize=11)
    ax.set_xticks(speeds)
    ax.set_xlim(speeds[0] - 0.05, speeds[-1] + 0.02)
    ax.set_ylim(0, 8)
    ax.set_title(f"{label} per wheel  --  v1 sphere-collider rollers",
                 fontweight="bold", fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", title="Wheel", framealpha=0.9)

axes[0].set_ylabel("|Torque| (Nm)")

# Embed numeric tables below each axes
def _table(ax, stat):
    cells = []
    for w in WHEELS:
        cells.append([f"{data[f'{s:.2f}'][f'torque_{w}'][stat]:.2f}"
                      for s in speeds])
    tbl = ax.table(cellText=cells,
                   rowLabels=WHEELS,
                   colLabels=[f"{s:.2f}" for s in speeds],
                   cellLoc="center", loc="bottom",
                   bbox=[0.06, -0.40, 0.94, 0.24])
    tbl.auto_set_font_size(False); tbl.set_fontsize(10)
    for col_i in range(len(speeds)):
        tbl[(0, col_i)].set_facecolor("#1f4e79")
        tbl[(0, col_i)].set_text_props(color="white", weight="bold")
    for row_i, w in enumerate(WHEELS, 1):
        tbl[(row_i, -1)].set_text_props(color=COLORS[w], weight="bold")
    return tbl

_table(axes[0], "mean")
_table(axes[1], "p95")

plt.suptitle(
    "FORTIS realwheel v1 -- orbit torque sweep "
    "(40 lb total, 14.144 kg chassis + 4x 1 kg wheels, R=1.59 m, sphere rollers)",
    fontsize=12, fontweight="bold")
plt.tight_layout(rect=(0, 0.18, 1, 0.95))

out = os.path.join(HERE, "orbit_realwheel_v1_torques.png")
plt.savefig(out, dpi=160, bbox_inches="tight")
plt.close(fig)
print(f"wrote {out}")
