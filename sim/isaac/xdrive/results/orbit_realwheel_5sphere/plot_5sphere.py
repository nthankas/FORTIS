"""Plot 5-sphere orbit torque sweep -- single panel with both mean and P95
per wheel, M8325s continuous-rating reference lines."""
import json, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
data = json.load(open(os.path.join(HERE, "sweep_orbit_realwheel.json")))

speeds = sorted(float(k) for k in data.keys())
WHEELS = ["FL", "FR", "BL", "BR"]
COLORS = {"FL": "#1f77b4", "FR": "#d62728",
          "BL": "#2ca02c", "BR": "#9467bd"}

# (stat_key, display name, linestyle, marker)
STATS = [
    ("p95",  "P95",  "-",  "o"),
    ("mean", "Mean", "--", "s"),
]

THRESH = [
    (3.32, "40 A cont. (3.32 Nm)", "--"),
    (4.98, "60 A cont. (4.98 Nm)", "-."),
    (5.81, "70 A cont. (5.81 Nm)", ":"),
]

fig, ax = plt.subplots(1, 1, figsize=(13, 9))

for stat_key, stat_name, ls, marker in STATS:
    for w in WHEELS:
        vals = [data[f"{s:.2f}"][f"torque_{w}"][stat_key] for s in speeds]
        ax.plot(speeds, vals, linestyle=ls, marker=marker,
                color=COLORS[w], linewidth=2.0, markersize=8,
                label=f"{w} {stat_name}")
        for s, v in zip(speeds, vals):
            ax.annotate(f"{v:.2f}", (s, v),
                        xytext=(0, 10), textcoords="offset points",
                        ha="center", fontsize=11, fontweight="bold",
                        color=COLORS[w])

for t, lab, ls in THRESH:
    ax.axhline(t, color="gray", linestyle=ls, linewidth=1.0, alpha=0.7)
    ax.text(speeds[0] - 0.018, t, f"{lab}",
            fontsize=9, color="#444", va="center", ha="right")

ax.set_xlabel("Commanded orbit speed (m/s)", fontsize=12)
ax.set_ylabel("|Torque| (Nm)", fontsize=12)
ax.set_xticks(speeds)
ax.set_xlim(speeds[0] - 0.05, speeds[-1] + 0.02)
ax.set_ylim(0, 5.81)
ax.set_title("Mean (dashed, square) and P95 (solid, circle) torque per wheel  "
             "--  5-sphere chain rollers",
             fontweight="bold", fontsize=13)
ax.grid(True, alpha=0.3)
ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5),
          framealpha=0.95, fontsize=10, title="Wheel x stat")

plt.suptitle(
    "FORTIS realwheel 5-sphere -- orbit torque sweep "
    "(45 lb total, 16.659 kg chassis+stowed-arm + 4x 1 kg wheels, R=1.59 m)",
    fontsize=12, fontweight="bold")
plt.tight_layout(rect=(0, 0, 1, 0.95))

out = os.path.join(HERE, "torque_sweep_5sphere_vs_baseline.png")
plt.savefig(out, dpi=160, bbox_inches="tight")
plt.close(fig)
print(f"wrote {out}")
