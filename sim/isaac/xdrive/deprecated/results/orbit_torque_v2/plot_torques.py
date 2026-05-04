"""Mean & P95 per-wheel torque graphs and tables for the V2 chassis sweep.

Loads every orbit_torque_*.csv in this directory, drops the warmup
(sim_time < 2.0 s), produces two figures:

  orbit_torque_v2_mean.png  -- per-wheel mean |tau| vs commanded speed
  orbit_torque_v2_p95.png   -- per-wheel P95  |tau| vs commanded speed

Also prints two tables and writes them to mean_table.md / p95_table.md.
"""
import csv, glob, math, os
import matplotlib.pyplot as plt

BASE = os.path.dirname(os.path.abspath(__file__))
WHEELS = ["FL", "FR", "BL", "BR"]
COLORS = {"FL": "#1f77b4", "FR": "#d62728", "BL": "#2ca02c", "BR": "#ff7f0e"}

CHASSIS_LABEL = "V2 Chassis (13.082 x 8.54 x 6.0 in arched, 20.4 kg chassis + 4x1 kg wheels = 24.4 kg total, mu=0.5)"

rows_by_speed = {}
for f in sorted(glob.glob(os.path.join(BASE, "orbit_torque_*mps.csv"))):
    speed = float(os.path.basename(f).replace("orbit_torque_", "").replace("mps.csv", ""))
    if speed < 0.10 - 1e-9 or speed > 0.30 + 1e-9:
        continue
    rows = []
    with open(f) as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append({k: float(v) for k, v in r.items()})
            except ValueError:
                pass
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
if not speeds:
    raise SystemExit("no usable CSVs found in this directory")

stats = {}
for s in speeds:
    rows = rows_by_speed[s]
    duration = rows[-1]["sim_time"] - rows[0]["sim_time"]
    path = sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"])
               for a, b in zip(rows[:-1], rows[1:]))
    per = {}
    for wn in WHEELS:
        ts = sorted(abs(r[f"torque_{wn}"]) for r in rows)
        per[wn] = {
            "mean": sum(ts) / len(ts),
            "p95": ts[int(len(ts) * 0.95)],
            "peak": ts[-1],
        }
    stats[s] = {"duration": duration, "path": path, "per": per}


def plot_metric(metric, title, fname):
    fig, ax = plt.subplots(figsize=(9, 6))
    fig.suptitle(CHASSIS_LABEL, fontsize=10)
    for wn in WHEELS:
        ys = [stats[s]["per"][wn][metric] for s in speeds]
        ax.plot(speeds, ys, "-o", color=COLORS[wn], label=wn, linewidth=2, markersize=7)
    ax.set_xlabel("Commanded orbit speed (m/s)")
    ax.set_ylabel(f"{title} (Nm)")
    ax.set_title(title + " per wheel")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=10)
    ax.set_xlim(0.08, 0.32)
    ax.set_ylim(bottom=0)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    out = os.path.join(BASE, fname)
    plt.savefig(out, dpi=150)
    plt.close(fig)
    print(f"wrote {out}")


plot_metric("mean", "Mean |tau|", "orbit_torque_v2_mean.png")
plot_metric("p95",  "P95 |tau|",  "orbit_torque_v2_p95.png")


def write_table(metric, fname, header_label):
    lines = []
    lines.append(f"# V2 Chassis Orbit Torque -- {header_label}")
    lines.append("")
    lines.append(f"{CHASSIS_LABEL}")
    lines.append("")
    lines.append("| Speed (m/s) | Duration (s) | Path (m) | FL | FR | BL | BR | All-wheels |")
    lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|")
    for s in speeds:
        st = stats[s]
        p = st["per"]
        all_vals = [p[wn][metric] for wn in WHEELS]
        if metric == "mean":
            allw = sum(all_vals) / len(all_vals)
        else:
            allw = max(all_vals)
        lines.append(f"| {s:.2f} | {st['duration']:.2f} | {st['path']:.2f} | "
                     f"{p['FL'][metric]:.3f} | {p['FR'][metric]:.3f} | "
                     f"{p['BL'][metric]:.3f} | {p['BR'][metric]:.3f} | {allw:.3f} |")
    lines.append("")
    lines.append("All values in Nm.")
    if metric == "mean":
        lines.append("All-wheels = average of per-wheel means.")
    else:
        lines.append("All-wheels = max of per-wheel P95.")
    out = os.path.join(BASE, fname)
    with open(out, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"wrote {out}")
    return lines


print()
for line in write_table("mean", "mean_table.md", "Mean |tau| per wheel"):
    print(line)
print()
for line in write_table("p95", "p95_table.md", "P95 |tau| per wheel"):
    print(line)
