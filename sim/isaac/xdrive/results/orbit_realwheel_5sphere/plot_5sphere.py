"""Plot 5-sphere orbit-on-step torque sweep with M8325s thresholds, side-by-side
with the single-sphere baseline."""
import json, os
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
data_5 = json.load(open(os.path.join(HERE, "sweep_orbit_realwheel.json")))
data_1 = json.load(open(os.path.join(ROOT, "orbit_realwheel_singlesphere",
                                      "sweep_orbit_realwheel.json")))
speeds = sorted(float(k) for k in data_5.keys())
wheels = ["FL", "FR", "BL", "BR"]
colors = {"FL": "tab:blue", "FR": "tab:orange",
          "BL": "tab:green", "BR": "tab:red"}

T_CONT = 3.32
T_PEAK = 4.98
T_ABS  = 6.64

fig, axes = plt.subplots(2, 2, figsize=(15, 11),
                          gridspec_kw={"height_ratios": [3, 2]})

def plot_panel(ax, data, title):
    x = np.array(speeds)
    width = 0.018
    for i, w in enumerate(wheels):
        off = (i - 1.5) * width
        means = [data[f"{s:.2f}"][f"torque_{w}"]["mean"] for s in speeds]
        p95s  = [data[f"{s:.2f}"][f"torque_{w}"]["p95"]  for s in speeds]
        peaks = [data[f"{s:.2f}"][f"torque_{w}"]["peak"] for s in speeds]
        ax.bar(x + off, means, width=width, color=colors[w], alpha=0.8,
               label=f"{w} mean")
        ax.scatter(x + off, p95s, color=colors[w], marker="^", s=70,
                   edgecolor="black", zorder=5,
                   label=f"{w} P95" if i == 0 else None)
        ax.scatter(x + off, peaks, color=colors[w], marker="x", s=70,
                   linewidth=2, zorder=6,
                   label=f"{w} peak" if i == 0 else None)
    for thr, lbl, col in [(T_CONT, "M8325s cont (3.32)", "g"),
                           (T_PEAK, "M8325s peak (4.98)", "y"),
                           (T_ABS,  "M8325s abs (6.64)",  "r")]:
        ax.axhline(thr, linestyle="--", color=col, alpha=0.7)
        ax.text(x[-1] + 0.005, thr + 0.1, lbl, color=col, fontsize=8)
    ax.set_xlabel("Commanded orbit radial speed (m/s)")
    ax.set_ylabel("Wheel torque (Nm)")
    ax.set_title(title)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{s:.2f}" for s in speeds])
    ax.set_ylim(0, max(40, ax.get_ylim()[1]))
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=4, fontsize=7, loc="upper left")

plot_panel(axes[0, 0], data_1, "Single-sphere collider (baseline)")
plot_panel(axes[0, 1], data_5,
           "5-sphere chain collider (continuous-arc contact)")

def make_table(ax, data, title):
    ax.axis("off")
    hdrs = ["v_cmd", "track%", "path_m"] + sum(
        [[f"{w} mean", f"{w} P95", f"{w} pk"] for w in wheels], [])
    rows = []
    for s in speeds:
        d = data[f"{s:.2f}"]
        row = [f"{s:.2f}", f"{d['tracking_pct']:.0f}",
               f"{d['path_len_m']:.2f}"]
        for w in wheels:
            t = d[f"torque_{w}"]
            row += [f"{t['mean']:.2f}", f"{t['p95']:.2f}", f"{t['peak']:.2f}"]
        rows.append(row)
    tbl = ax.table(cellText=rows, colLabels=hdrs, loc="center",
                   cellLoc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8)
    tbl.scale(1, 1.4)
    ax.set_title(title, fontsize=10, y=0.92)

make_table(axes[1, 0], data_1, "Single-sphere baseline numbers")
make_table(axes[1, 1], data_5, "5-sphere chain numbers")

plt.tight_layout()
out = os.path.join(HERE, "torque_sweep_5sphere_vs_baseline.png")
plt.savefig(out, dpi=130)
print(f"Wrote {out}")
