"""Run a full-orbit sweep of xdrive_realwheel.py at multiple speeds.

Runs each speed as a subprocess (1 full orbit), writes per-speed CSVs into
results/orbit_realwheel/, then aggregates path-length, net-displacement,
and torque stats into sweep_orbit_realwheel.json.
"""
import os, sys, subprocess, csv, math, json, time

XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ISAAC_PYTHON = os.path.normpath(os.path.join(
    XDRIVE_ROOT, "..", "..", "..", "..", "..", "IsaacSim", "python.bat"))
SCRIPT = os.path.join(XDRIVE_ROOT, "canonical", "xdrive_realwheel.py")
OUT_DIR = os.path.join(XDRIVE_ROOT, "results", "orbit_realwheel")
os.makedirs(OUT_DIR, exist_ok=True)

SPEEDS = [0.10, 0.15, 0.20, 0.25, 0.30]
ORBITS = 1.0  # full 360 deg per speed

results = {}
for speed in SPEEDS:
    csv_path = os.path.join(OUT_DIR, f"orbit_realwheel_{speed:.2f}mps.csv")
    cmd = [ISAAC_PYTHON, SCRIPT,
           "--orbit-speed", str(speed),
           "--orbit-orbits", str(ORBITS),
           "--orbit-csv", csv_path]
    print(f"\n=== {speed:.2f} m/s ({ORBITS} orbit) ===", flush=True)
    print(f"cmd: {' '.join(cmd)}", flush=True)
    t0 = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    elapsed = time.time() - t0
    print(f"  done in {elapsed:.1f}s, exit {r.returncode}", flush=True)
    if r.returncode != 0:
        print("  STDERR tail:")
        print(r.stderr[-600:])
        continue
    # Parse the CSV
    rows = []
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            for d in csv.DictReader(f):
                rows.append({k: float(v) for k, v in d.items()})
    if not rows:
        print(f"  WARN: empty CSV {csv_path}")
        continue
    # Skip first 2 s as transient
    rows = [r_ for r_ in rows if r_["sim_time"] >= 2.0]
    if not rows:
        continue
    xs = [r_["x"] for r_ in rows]
    ys = [r_["y"] for r_ in rows]
    path_len = sum(math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i])
                   for i in range(len(xs)-1))
    net = math.hypot(xs[-1]-xs[0], ys[-1]-ys[0])
    duration = rows[-1]["sim_time"] - rows[0]["sim_time"]
    Rs = [r_["R_m"] for r_ in rows]
    yaws = [r_["yaw_deg"] for r_ in rows]

    def stats(key):
        v = [abs(r_[key]) for r_ in rows]
        return {"mean": sum(v)/len(v), "p95": sorted(v)[int(0.95*len(v))], "peak": max(v)}

    results[f"{speed:.2f}"] = {
        "speed_cmd": speed,
        "orbits": ORBITS,
        "duration_s": duration,
        "path_len_m": path_len,
        "net_disp_m": net,
        "effective_v_mps": net / duration if duration > 0 else 0,
        "tracking_pct": 100 * (net / duration) / speed if duration > 0 and speed > 0 else 0,
        "R_min": min(Rs), "R_max": max(Rs),
        "R_mean": sum(Rs)/len(Rs),
        "yaw_min": min(yaws), "yaw_max": max(yaws),
        "torque_FL": stats("torque_FL"),
        "torque_FR": stats("torque_FR"),
        "torque_BL": stats("torque_BL"),
        "torque_BR": stats("torque_BR"),
    }
    r_ = results[f"{speed:.2f}"]
    print(f"  net={r_['net_disp_m']:.2f}m  "
          f"v_eff={r_['effective_v_mps']:.3f}m/s ({r_['tracking_pct']:.0f}%)  "
          f"R={r_['R_mean']:.3f}m [{r_['R_min']:.3f}-{r_['R_max']:.3f}]")
    print(f"  torque mean (Nm): "
          f"FL={r_['torque_FL']['mean']:.2f} "
          f"FR={r_['torque_FR']['mean']:.2f} "
          f"BL={r_['torque_BL']['mean']:.2f} "
          f"BR={r_['torque_BR']['mean']:.2f}")

with open(os.path.join(OUT_DIR, "sweep_orbit_realwheel.json"), "w") as f:
    json.dump(results, f, indent=2)
print(f"\nWrote {OUT_DIR}/sweep_orbit_realwheel.json")
