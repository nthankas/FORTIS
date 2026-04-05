import csv, math, os

BASE = os.path.dirname(os.path.abspath(__file__))

for speed_str in ["0.10", "0.15", "0.20", "0.25", "0.30"]:
    csv_path = os.path.join(BASE, f"orbit_torque_{speed_str}mps.csv")
    if not os.path.exists(csv_path):
        continue
    rows = []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) for k, v in row.items()})
    rows = [r for r in rows if r["sim_time"] >= 2.0]
    if not rows:
        continue

    speeds = [r["actual_speed_mps"] for r in rows]
    torques_all = []
    for r in rows:
        for wn in ["FL", "FR", "BL", "BR"]:
            torques_all.append(abs(r[f"torque_{wn}"]))
    torques_all.sort()
    n = len(torques_all)

    print(f"\n=== {speed_str} m/s commanded ===")
    print(f"  Rows: {len(rows)}")
    print(f"  Speed: mean={sum(speeds)/len(speeds):.3f} min={min(speeds):.3f} max={max(speeds):.3f}")
    print(f"  Torque: mean={sum(torques_all)/n:.3f} median={torques_all[n//2]:.3f}")
    print(f"    P95={torques_all[int(n*0.95)]:.3f}  P99={torques_all[int(n*0.99)]:.3f}  "
          f"P99.9={torques_all[int(n*0.999)]:.3f}  Max={torques_all[-1]:.3f}")
    over = sum(1 for t in torques_all if t > 12.6)
    print(f"    Samples > 12.6 Nm: {over} ({100*over/n:.2f}%)")

    # Radius
    radii = [math.sqrt(r["x"]**2 + r["y"]**2) / 0.0254 for r in rows]
    print(f"  Radius: mean={sum(radii)/len(radii):.1f}\" min={min(radii):.1f}\" max={max(radii):.1f}\"")

    # Top spikes
    spikes = []
    for r in rows:
        for wn in ["FL", "FR", "BL", "BR"]:
            t = abs(r[f"torque_{wn}"])
            if t > 12.6:
                spikes.append((r["sim_time"], wn, t))
    spikes.sort(key=lambda x: -x[2])
    if spikes:
        print(f"  Top 5 spikes:")
        for t, wn, torque in spikes[:5]:
            print(f"    t={t:.3f}s {wn} {torque:.2f} Nm")
