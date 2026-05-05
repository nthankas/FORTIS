"""Plot 5-sphere orbit torque sweep -- mean and P95 panels with M8325s
continuous-rating reference lines."""
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

THRESH = [
    (3.32, "40 A cont. (3.32 Nm)", "--"),
    (4.98, "60 A cont. (4.98 Nm)", "-."),
    (5.81, "70 A cont. (5.81 Nm)", ":"),
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
        ax.text(speeds[0] - 0.018, t, f"{lab}",
                fontsize=8, color="#444", va="center", ha="right")
    ax.set_xlabel("Commanded orbit speed (m/s)", fontsize=11)
    ax.set_xticks(speeds)
    ax.set_xlim(speeds[0] - 0.05, speeds[-1] + 0.02)
    ax.set_ylim(0, 5.81)
    ax.set_title(f"{label} per wheel  --  5-sphere chain rollers",
                 fontweight="bold", fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", title="Wheel", framealpha=0.9)

axes[0].set_ylabel("|Torque| (Nm)")

plt.suptitle(
    "FORTIS realwheel 5-sphere -- orbit torque sweep "
    "(45 lb total, 16.659 kg chassis+stowed-arm + 4x 1 kg wheels, R=1.59 m)",
    fontsize=12, fontweight="bold")
plt.tight_layout(rect=(0, 0, 1, 0.95))

out = os.path.join(HERE, "torque_sweep_5sphere_vs_baseline.png")
plt.savefig(out, dpi=160, bbox_inches="tight")
plt.close(fig)
print(f"wrote {out}")
