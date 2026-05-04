"""Generate the canonical orbit-torque plot for the OLD-chassis sweep.

Loads every orbit_torque_*.csv in this directory, drops the 1.00 m/s
file (incomplete -- only static pre-roll), and plots per-wheel
mean / P95 / peak torque vs commanded speed plus the duration and
path length each orbit actually ran.
"""
import csv, glob, math, os
import matplotlib.pyplot as plt

BASE = os.path.dirname(os.path.abspath(__file__))
WHEELS = ["FL", "FR", "BL", "BR"]
COLORS = {"FL": "#1f77b4", "FR": "#d62728", "BL": "#2ca02c", "BR": "#ff7f0e"}

rows_by_speed = {}
for f in sorted(glob.glob(os.path.join(BASE, "orbit_torque_*mps.csv"))):
    speed = float(os.path.basename(f).replace("orbit_torque_", "").replace("mps.csv", ""))
    rows = []
    with open(f) as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append({k: float(v) for k, v in r.items()})
            except ValueError:
                pass
    if speed < 0.10 - 1e-9 or speed > 0.30 + 1e-9:
        continue
    rows = [r for r in rows if r["sim_time"] >= 2.0]
    if len(rows) < 100:
        print(f"skipping {speed} m/s (only {len(rows)} usable rows)")
        continue
    path = sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"])
               for a, b in zip(rows[:-1], rows[1:]))
    if path < 1.0:
        print(f"skipping {speed} m/s (robot did not move; path = {path:.3f} m)")
        continue
    rows_by_speed[speed] = rows

speeds = sorted(rows_by_speed.keys())

stats = {}
for s in speeds:
    rows = rows_by_speed[s]
    duration = rows[-1]["sim_time"] - rows[0]["sim_time"]
    path = 0.0
    for a, b in zip(rows[:-1], rows[1:]):
        path += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    per = {}
    for wn in WHEELS:
        ts = sorted(abs(r[f"torque_{wn}"]) for r in rows)
        per[wn] = {
            "mean": sum(ts) / len(ts),
            "p95": ts[int(len(ts) * 0.95)],
            "peak": ts[-1],
        }
    stats[s] = {"duration": duration, "path": path, "per": per}

fig, axes = plt.subplots(2, 2, figsize=(13, 9))
fig.suptitle(
    "FORTIS X-Drive Orbit Torque Sweep -- Old Chassis (15.354 x 9.353 x 7.1 in, 18.14 kg, mu=0.5)",
    fontsize=12, fontweight="bold")

for ax, metric, title in [
    (axes[0, 0], "mean", "Mean |tau| per wheel"),
    (axes[0, 1], "p95",  "P95 |tau| per wheel"),
    (axes[1, 0], "peak", "Peak |tau| per wheel"),
]:
    for wn in WHEELS:
        ys = [stats[s]["per"][wn][metric] for s in speeds]
        ax.plot(speeds, ys, "-o", color=COLORS[wn], label=wn, linewidth=1.8, markersize=6)
    ax.set_xlabel("Commanded orbit speed (m/s)")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=9)
    ax.set_xlim(0.08, 0.32)

# bottom-right: duration (left axis) + path length (right axis)
ax = axes[1, 1]
durations = [stats[s]["duration"] for s in speeds]
paths     = [stats[s]["path"]     for s in speeds]
l1 = ax.plot(speeds, durations, "-o", color="#444444", label="Duration (s)",
             linewidth=1.8, markersize=6)
ax.set_xlabel("Commanded orbit speed (m/s)")
ax.set_ylabel("Run duration (s)", color="#444444")
ax.tick_params(axis="y", labelcolor="#444444")
ax.set_xlim(0.08, 0.32)
ax.grid(True, alpha=0.3)

ax2 = ax.twinx()
l2 = ax2.plot(speeds, paths, "-s", color="#7c3aed", label="Path length (m)",
              linewidth=1.8, markersize=6)
ax2.set_ylabel("Path length (m)", color="#7c3aed")
ax2.tick_params(axis="y", labelcolor="#7c3aed")

ax.set_title("Orbit duration & path length per run")
lines = l1 + l2
ax.legend(lines, [l.get_label() for l in lines], loc="upper right", fontsize=9)

plt.tight_layout(rect=[0, 0, 1, 0.96])
out = os.path.join(BASE, "orbit_torque_summary.png")
plt.savefig(out, dpi=150)
print(f"wrote {out}")

print(f"\n{'speed':>6} {'dur(s)':>8} {'path(m)':>9} {'FL_mean':>8} {'FR_mean':>8} {'BL_mean':>8} {'BR_mean':>8} "
      f"{'FL_p95':>7} {'FR_p95':>7} {'BL_p95':>7} {'BR_p95':>7} "
      f"{'FL_pk':>7} {'FR_pk':>7} {'BL_pk':>7} {'BR_pk':>7}")
for s in speeds:
    st = stats[s]
    p = st["per"]
    print(f"{s:>6.2f} {st['duration']:>8.2f} {st['path']:>9.2f} "
          f"{p['FL']['mean']:>8.3f} {p['FR']['mean']:>8.3f} {p['BL']['mean']:>8.3f} {p['BR']['mean']:>8.3f} "
          f"{p['FL']['p95']:>7.3f} {p['FR']['p95']:>7.3f} {p['BL']['p95']:>7.3f} {p['BR']['p95']:>7.3f} "
          f"{p['FL']['peak']:>7.2f} {p['FR']['peak']:>7.2f} {p['BL']['peak']:>7.2f} {p['BR']['peak']:>7.2f}")
