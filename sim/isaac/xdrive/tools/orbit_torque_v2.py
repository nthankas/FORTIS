"""
FORTIS X-Drive Orbit Torque Profiling — V2 chassis (2026-05-01).

Rectangular skeleton 13.082x8.54x6", wheels flush at corners, 2" belly.
Same orbit test as orbit_torque.py but with the updated chassis geometry.

Two modes:
  --speed X   : Single test at X m/s (launched as subprocess by sweep mode)
  (no --speed): Sweep mode — runs all speeds via subprocesses, aggregates, plots

Usage:
  Sweep (orchestrator, regular python):
    python orbit_torque_v2.py

  Single test (Isaac Sim python):
    IsaacSim\\python.bat orbit_torque_v2.py --speed 1.0 --gui
    IsaacSim\\python.bat orbit_torque_v2.py --speed 1.0 --headless
"""
import os, sys, math, argparse, csv, json

os.environ["PYTHONIOENCODING"] = "utf-8"

# --- xdrive path bootstrap (reorg 2026-04-04) ---
BASE_DIR    = os.path.dirname(os.path.abspath(__file__))   # xdrive/tools/
XDRIVE_ROOT = os.path.abspath(os.path.join(BASE_DIR, ".."))
ASSETS_DIR  = os.path.join(XDRIVE_ROOT, "assets")
RESULTS_DIR = os.path.join(XDRIVE_ROOT, "results", "orbit_torque_v2")
sys.path.insert(0, os.path.join(XDRIVE_ROOT, "lib"))
# --- end bootstrap ---
ISAAC_PYTHON = os.path.normpath(os.path.join(
    BASE_DIR, "..", "..", "..", "..", "..", "..", "IsaacSim", "python.bat"))

SPEEDS = [0.1, 0.15, 0.2, 0.25, 0.3]

# ============================================================================
# Argument parsing
# ============================================================================
parser = argparse.ArgumentParser(description="FORTIS orbit torque profiling (v2 chassis)")
parser.add_argument("--speed", type=float, default=None,
                    help="Single-test mode: orbit speed in m/s")
parser.add_argument("--gui", action="store_true")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--hz", type=int, default=360,
                    help="Physics Hz (default 360)")
parser.add_argument("--belly", type=float, default=2.0,
                    help="Belly height in inches (default 2.0)")
parser.add_argument("--out-dir", type=str, default=RESULTS_DIR)
args, _ = parser.parse_known_args()


# ============================================================================
# SWEEP MODE — no Isaac Sim imports, just subprocess + aggregation
# ============================================================================
if args.speed is None:
    import subprocess

    def _mean(lst):
        return sum(lst) / len(lst) if lst else 0.0

    os.makedirs(args.out_dir, exist_ok=True)
    script_path = os.path.abspath(__file__)

    print("=" * 70)
    print("FORTIS X-Drive Orbit Torque Sweep (V2 Chassis)")
    print(f"Speeds: {SPEEDS} m/s")
    print(f"Output: {args.out_dir}")
    print(f"Isaac Python: {ISAAC_PYTHON}")
    print("=" * 70)

    results = {}
    for speed in SPEEDS:
        csv_name = f"orbit_torque_{speed:.2f}mps.csv"
        csv_path = os.path.join(args.out_dir, csv_name)
        cmd = [ISAAC_PYTHON, script_path,
               "--speed", str(speed), "--headless",
               "--hz", str(args.hz), "--belly", str(args.belly),
               "--out-dir", args.out_dir]
        print(f"\n--- Running speed={speed:.2f} m/s ---")
        print(f"  CMD: {' '.join(cmd)}")
        try:
            result = subprocess.run(cmd, capture_output=True, text=True,
                                    timeout=600, cwd=BASE_DIR)
            if result.returncode != 0:
                print(f"  FAILED (exit code {result.returncode})")
                if result.stderr:
                    print(f"  STDERR (last 800 chars):\n{result.stderr[-800:]}")
                continue
            print(f"  OK")
            lines = result.stdout.strip().split("\n")
            for line in lines[-5:]:
                print(f"  {line}")
        except subprocess.TimeoutExpired:
            print(f"  TIMEOUT (>600s)")
            continue

        # Read CSV
        if not os.path.exists(csv_path):
            print(f"  WARNING: CSV not found at {csv_path}")
            continue
        rows = []
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append({k: float(v) for k, v in row.items()})
        rows = [r for r in rows if r["sim_time"] >= 2.0]
        if len(rows) < 50:
            print(f"  WARNING: Only {len(rows)} rows after transient removal")
            continue

        torques_fl = [abs(r["torque_FL"]) for r in rows]
        torques_fr = [abs(r["torque_FR"]) for r in rows]
        torques_bl = [abs(r["torque_BL"]) for r in rows]
        torques_br = [abs(r["torque_BR"]) for r in rows]
        actual_speeds = [r["actual_speed_mps"] for r in rows]

        results[speed] = {
            "mean_FL": _mean(torques_fl), "peak_FL": max(torques_fl),
            "mean_FR": _mean(torques_fr), "peak_FR": max(torques_fr),
            "mean_BL": _mean(torques_bl), "peak_BL": max(torques_bl),
            "mean_BR": _mean(torques_br), "peak_BR": max(torques_br),
            "peak_all": max(max(torques_fl), max(torques_fr),
                           max(torques_bl), max(torques_br)),
            "actual_speed": _mean(actual_speeds),
            "rows": len(rows),
            "duration": rows[-1]["sim_time"] - rows[0]["sim_time"],
        }

    # Summary table
    print("\n" + "=" * 100)
    print(f"{'Speed':>6} {'Actual':>7} {'Dur(s)':>7} {'FL_mean':>8} {'FR_mean':>8} "
          f"{'BL_mean':>8} {'BR_mean':>8} {'Peak':>8}")
    print("-" * 100)
    for speed in SPEEDS:
        if speed not in results:
            print(f"{speed:6.2f}    FAILED")
            continue
        r = results[speed]
        print(f"{speed:6.2f} {r['actual_speed']:7.3f} {r['duration']:7.1f} "
              f"{r['mean_FL']:8.3f} {r['mean_FR']:8.3f} "
              f"{r['mean_BL']:8.3f} {r['mean_BR']:8.3f} "
              f"{r['peak_all']:8.3f}")
    print("=" * 100)
    print(f"Motor available: NEO 2.0 + 9:1 = 12.6 Nm")

    # Generate summary plot
    if results:
        results_json = os.path.join(args.out_dir, "sweep_results.json")
        with open(results_json, "w") as f:
            json.dump(results, f, indent=2)
        plot_script = os.path.join(args.out_dir, "_plot.py")
        with open(plot_script, "w") as f:
            f.write(f'''import json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

with open(r"{results_json}") as f:
    results = json.load(f)
results = {{float(k): v for k, v in results.items()}}

fig, ax = plt.subplots(figsize=(12, 7))
speeds_ok = sorted(results.keys())
wheel_names = ["FL", "FR", "BL", "BR"]
colors = {{"FL": "#2ca02c", "FR": "#d62728", "BL": "#1f77b4", "BR": "#ff7f0e"}}

for wn in wheel_names:
    means = [results[s][f"mean_{{wn}}"] for s in speeds_ok]
    peaks = [results[s][f"peak_{{wn}}"] for s in speeds_ok]
    ax.plot(speeds_ok, means, "-o", color=colors[wn], label=f"{{wn}} mean",
            linewidth=2, markersize=6)
    ax.plot(speeds_ok, peaks, "--s", color=colors[wn], label=f"{{wn}} peak",
            linewidth=1.5, markersize=5, alpha=0.7)

peak_all = [results[s]["peak_all"] for s in speeds_ok]
ax.plot(speeds_ok, peak_all, "-D", color="black", linewidth=3,
        markersize=8, label="Max peak (all wheels)", zorder=10)
ax.axhline(12.6, color="gray", linestyle="-.", linewidth=2,
           label="NEO 2.0 + 9:1 available (12.6 Nm)")

ax.set_xlabel("Commanded Orbit Speed (m/s)", fontsize=12)
ax.set_ylabel("Torque (Nm)", fontsize=12)
ax.set_title("FORTIS X-Drive Orbit Torque vs Speed (v2 chassis, mu=0.5, reactor floor)",
             fontsize=13, fontweight="bold")
ax.legend(loc="upper left", fontsize=9, ncol=2)
ax.grid(True, alpha=0.3)
ax.set_xlim(speeds_ok[0] - 0.1, speeds_ok[-1] + 0.1)
ax.set_ylim(bottom=0)
plt.tight_layout()
plot_path = r"{os.path.join(args.out_dir, 'orbit_torque_summary.png')}"
fig.savefig(plot_path, dpi=150)
plt.close(fig)
print(f"Plot saved: {{plot_path}}")
''')
        print("\nGenerating plot...")
        plot_result = subprocess.run([ISAAC_PYTHON, plot_script],
                                     capture_output=True, text=True, timeout=120)
        if plot_result.returncode == 0:
            print(plot_result.stdout.strip())
        else:
            print(f"WARNING: Plot generation failed: {plot_result.stderr[-400:]}")
        try:
            os.remove(plot_script)
        except OSError:
            pass

    sys.exit(0)


# ============================================================================
# SINGLE TEST MODE — full Isaac Sim session
# ============================================================================
headless = args.headless or not args.gui

from isaacsim import SimulationApp
app = SimulationApp({"headless": headless, "width": 1920, "height": 1080})

import omni.usd
import omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema, Vt
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
import numpy as np

import sim_config as cfg

# ============================================================================
# Constants
# ============================================================================
IN = 0.0254
MM = 0.001
OMNIWHEEL_USD = os.path.join(ASSETS_DIR, "omniwheels.usd")
REACTOR_USD = os.path.join(ASSETS_DIR, "diiid_reactor.usd")

# Chassis geometry — octagonal with 3" chamfer faces
CHASSIS_L = 13.082 * IN     # bounding box length
CHASSIS_W = 8.54 * IN       # bounding box width
CHASSIS_H = 6.0 * IN        # height
CHAMFER_FACE = 3.0 * IN
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)
MOTOR_MOUNT_LEN = 1.272 * IN
BELLY_HEIGHT = args.belly * IN
ARCH_FLAT_WIDTH = 2.059 * IN

# Wheel (AndyMark 8" Dualie)
TARGET_DIA_MM = 203.0
TARGET_WIDTH_MM = 51.8
WHEEL_RADIUS = TARGET_DIA_MM / 2.0 * MM
WHEEL_WIDTH = TARGET_WIDTH_MM * MM

# Source wheel dimensions
SRC_DIA_MM = 82.5
SRC_WIDTH_MM = 44.87
SRC_CENTER_Y_MM = -17.43
SCALE_XZ = TARGET_DIA_MM / SRC_DIA_MM
SCALE_Y = TARGET_WIDTH_MM / SRC_WIDTH_MM
SCALE_UNIFORM = SCALE_XZ

# Mass — 20.4 kg chassis (matches canonical scripts)
WHEEL_MASS = 1.0
CHASSIS_MASS = 20.4
ROLLER_MASS_FRAC = 0.3
NUM_ROLLERS = 10
HUB_MASS = WHEEL_MASS * (1.0 - ROLLER_MASS_FRAC)
ROLLER_MASS = WHEEL_MASS * ROLLER_MASS_FRAC / NUM_ROLLERS

# Physics
PHYSICS_HZ = max(360, min(480, args.hz))
FRICTION_MU = 0.5
DRIVE_DAMPING = 100.0
DRIVE_MAX_FORCE = 200.0

# Chassis derived
SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0
C = CHAMFER_CUT

OCT_XY = [
    ( SL,       -(SW - C)),  ( SL,        (SW - C)),
    ( (SL - C),  SW      ),  (-(SL - C),  SW      ),
    (-SL,        (SW - C)),  (-SL,       -(SW - C)),
    (-(SL - C), -SW      ),  ( (SL - C), -SW      ),
]

def mid(a, b):
    return ((a[0]+b[0])/2, (a[1]+b[1])/2)

WHEEL_XY = {
    "FR": mid(OCT_XY[0], OCT_XY[7]), "FL": mid(OCT_XY[1], OCT_XY[2]),
    "BL": mid(OCT_XY[3], OCT_XY[4]), "BR": mid(OCT_XY[5], OCT_XY[6]),
}
AXLE_ANGLES = {"FR": 45.0, "FL": 135.0, "BL": 45.0, "BR": 135.0}
WHEEL_ORDER = ["FR", "FL", "BL", "BR"]

# Orbit parameters
ORBIT_SPEED = args.speed
ORBIT_RADIUS = 1.59  # m (spawn radius on outer floor)
ORBIT_CIRCUMFERENCE = 2.0 * math.pi * ORBIT_RADIUS
TRANSIENT_S = 2.0
ORBIT_DURATION = 2.0 * ORBIT_CIRCUMFERENCE / ORBIT_SPEED + TRANSIENT_S + 1.0
LOG_HZ = 100
LOG_INTERVAL = max(1, PHYSICS_HZ // LOG_HZ)

# Abort thresholds
Z_DROP_ABORT = 0.5


# ============================================================================
# Robot build functions
# ============================================================================

def classify_part(name):
    if "Kaya_Wheel_Hub" in name:
        return "hub"
    if "_17_4775_001_" in name or "_17_4775_003_" in name:
        return "sideplate"
    if "_17_4775_004_" in name:
        return "roller_barrel"
    if "_17_4775_005_" in name:
        return "roller_cap"
    if "_17_4775_007_" in name:
        return "roller_pin"
    if "_17_2584_007_" in name:
        return "screw_large"
    if "_17_2584_009_" in name:
        return "screw_small"
    return "unknown"


def read_source_wheel():
    print(f"Reading wheel source: {OMNIWHEEL_USD}", flush=True)
    src = Usd.Stage.Open(OMNIWHEEL_USD)
    xfc = UsdGeom.XformCache()
    parts = {"hub": [], "sideplate": [], "roller_barrel": [], "roller_cap": [],
             "roller_pin": [], "screw_large": [], "screw_small": []}
    for prim in src.Traverse():
        if prim.GetTypeName() != "Mesh":
            continue
        mesh = UsdGeom.Mesh(prim)
        raw_pts = mesh.GetPointsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        if not raw_pts or len(raw_pts) == 0:
            continue
        wxf = xfc.GetLocalToWorldTransform(prim)
        world_pts = []
        for p in raw_pts:
            wp = wxf.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
            world_pts.append([wp[0], wp[1], wp[2]])
        cat = classify_part(prim.GetName())
        if cat == "unknown":
            continue
        parts[cat].append({
            "name": prim.GetName(),
            "points": np.array(world_pts),
            "indices": list(indices) if indices else [],
            "counts": list(counts) if counts else [],
        })
    barrel_centers = [p["points"].mean(axis=0) for p in parts["roller_barrel"]]
    wheel_center = np.mean(barrel_centers, axis=0)
    for cat, lst in parts.items():
        print(f"  {cat}: {len(lst)} meshes", flush=True)
    return parts, wheel_center


def create_mesh_prim(stage, path, part, src_center, sx, sy, sz,
                     body_offset=None, color=None):
    scaled = []
    for p in part["points"]:
        x = float((p[0] - src_center[0]) * sx)
        y = float((p[1] - src_center[1]) * sy)
        z = float((p[2] - src_center[2]) * sz)
        if body_offset is not None:
            x -= body_offset[0]
            y -= body_offset[1]
            z -= body_offset[2]
        scaled.append(Gf.Vec3f(x, y, z))
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(scaled))
    if part["indices"]:
        mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(part["indices"]))
    if part["counts"]:
        mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(part["counts"]))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    if color:
        mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    return mesh


def build_physics_scene(stage):
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(True)
    px.CreateSolverTypeAttr("TGS")
    px.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
    print(f"Physics: {PHYSICS_HZ}Hz, CPU dynamics, TGS solver", flush=True)


def build_arched_chassis(stage, path, half_h, color):
    arch_flat_half = ARCH_FLAT_WIDTH / 2.0
    ramp_start = arch_flat_half
    ramp_end = SL - MOTOR_MOUNT_LEN
    z_low = -half_h
    z_high = -half_h + BELLY_HEIGHT
    x_stations = sorted(set([
        -SL, -(SL - C), -ramp_end, -ramp_start, 0.0,
        ramp_start, ramp_end, (SL - C), SL,
    ]))
    def bottom_z(x):
        ax = abs(x)
        if ax <= ramp_start:
            return z_high
        elif ax <= ramp_end:
            t = (ax - ramp_start) / (ramp_end - ramp_start)
            return z_high + t * (z_low - z_high)
        else:
            return z_low
    def half_width_at(x):
        ax = abs(x)
        if ax <= (SL - C):
            return SW
        else:
            t = (ax - (SL - C)) / C
            return SW - t * C
    verts, face_indices, face_counts, sections = [], [], [], []
    for x in x_stations:
        hw = half_width_at(x)
        bz = bottom_z(x)
        tl = Gf.Vec3f(x, hw, half_h)
        tr = Gf.Vec3f(x, -hw, half_h)
        br = Gf.Vec3f(x, -hw, bz)
        bl = Gf.Vec3f(x, hw, bz)
        idx_start = len(verts)
        verts.extend([tl, tr, br, bl])
        sections.append((idx_start, idx_start + 1, idx_start + 2, idx_start + 3))
    for i in range(len(sections) - 1):
        tl0, tr0, br0, bl0 = sections[i]
        tl1, tr1, br1, bl1 = sections[i + 1]
        face_indices.extend([tl0, tl1, tr1, tr0]); face_counts.append(4)
        face_indices.extend([bl0, br0, br1, bl1]); face_counts.append(4)
        face_indices.extend([tl0, bl0, bl1, tl1]); face_counts.append(4)
        face_indices.extend([tr0, tr1, br1, br0]); face_counts.append(4)
    tl, tr, br, bl = sections[-1]
    face_indices.extend([tl, tr, br, bl]); face_counts.append(4)
    tl, tr, br, bl = sections[0]
    face_indices.extend([tr, tl, bl, br]); face_counts.append(4)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    return mesh


def build_chassis(stage, chassis_path):
    half_h = CHASSIS_H / 2.0
    build_arched_chassis(stage, chassis_path + "/body", half_h, (0.25, 0.25, 0.35))
    INSET = 1.0 * IN
    col = UsdGeom.Cube.Define(stage, chassis_path + "/collider")
    col.GetSizeAttr().Set(1.0)
    cxf = UsdGeom.Xformable(col.GetPrim())
    cxf.ClearXformOpOrder()
    cxf.AddScaleOp().Set(Gf.Vec3d((SL - INSET) * 2.0, (SW - INSET) * 2.0, CHASSIS_H))
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())
    UsdGeom.Imageable(col.GetPrim()).CreatePurposeAttr("guide")
    arrow = UsdGeom.Cube.Define(stage, chassis_path + "/fwd")
    arrow.GetSizeAttr().Set(1.0)
    axf = UsdGeom.Xformable(arrow.GetPrim())
    axf.ClearXformOpOrder()
    axf.AddTranslateOp().Set(Gf.Vec3d(SL * 0.6, 0, half_h + 0.003))
    axf.AddScaleOp().Set(Gf.Vec3d(SL * 0.4, SW * 0.15, 0.005))
    arrow.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.15, 0.15)]))


def build_omniwheel(stage, wheel_path, src_parts, src_center, wheel_mat, wname):
    hub_path = wheel_path + "/Hub"
    hub_xf = UsdGeom.Xform.Define(stage, hub_path)
    UsdPhysics.RigidBodyAPI.Apply(hub_xf.GetPrim())
    UsdPhysics.MassAPI.Apply(hub_xf.GetPrim()).CreateMassAttr(HUB_MASS)
    hub_color = (0.3, 0.3, 0.35)
    mi = 0
    for cat in ["hub", "sideplate", "screw_large", "screw_small"]:
        for part in src_parts[cat]:
            mp = f"{hub_path}/vis_{mi}"
            create_mesh_prim(stage, mp, part, src_center,
                             SCALE_XZ, SCALE_Y, SCALE_XZ, color=hub_color)
            mi += 1
    cap_data = [(p, p["points"].mean(axis=0)) for p in src_parts["roller_cap"]]
    pin_data = [(p, p["points"].mean(axis=0)) for p in src_parts["roller_pin"]]
    used_caps = set()
    used_pins = set()
    roller_color = (0.15, 0.15, 0.15)
    all_scaled_centers = []
    for barrel in src_parts["roller_barrel"]:
        bc = barrel["points"].mean(axis=0)
        sc = np.array([
            float((bc[0] - src_center[0]) * SCALE_XZ),
            float((bc[1] - src_center[1]) * SCALE_Y),
            float((bc[2] - src_center[2]) * SCALE_XZ),
        ])
        all_scaled_centers.append(sc)
    min_pair_dist = float('inf')
    for i in range(len(all_scaled_centers)):
        for j in range(i + 1, len(all_scaled_centers)):
            d = float(np.linalg.norm(all_scaled_centers[i] - all_scaled_centers[j]))
            if d < min_pair_dist:
                min_pair_dist = d
    safe_roller_r = min_pair_dist / 2.0 - 0.003
    max_surface_r = float(WHEEL_RADIUS - np.sqrt(
        all_scaled_centers[0][0]**2 + all_scaled_centers[0][2]**2))
    roller_coll_r_global = max(min(safe_roller_r, max_surface_r), 0.005)

    for ri, barrel in enumerate(src_parts["roller_barrel"]):
        bc = barrel["points"].mean(axis=0)
        scaled_center = all_scaled_centers[ri]
        rp = wheel_path + f"/Roller_{ri}"
        rxf = UsdGeom.Xform.Define(stage, rp)
        rprim = rxf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(rprim)
        UsdPhysics.MassAPI.Apply(rprim).CreateMassAttr(ROLLER_MASS)
        prb = PhysxSchema.PhysxRigidBodyAPI.Apply(rprim)
        prb.CreateSolverPositionIterationCountAttr(16)
        prb.CreateMaxDepenetrationVelocityAttr(5.0)
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(
            float(scaled_center[0]), float(scaled_center[1]), float(scaled_center[2])))
        bmp = rp + "/collider"
        sphere = UsdGeom.Sphere.Define(stage, bmp)
        sphere.GetRadiusAttr().Set(roller_coll_r_global)
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(
            wheel_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        UsdGeom.Imageable(sphere.GetPrim()).CreatePurposeAttr("guide")
        pcoll = PhysxSchema.PhysxCollisionAPI.Apply(sphere.GetPrim())
        pcoll.CreateContactOffsetAttr(0.001)
        pcoll.CreateRestOffsetAttr(0.0005)
        bvp = rp + "/barrel_vis"
        create_mesh_prim(stage, bvp, barrel, src_center,
                         SCALE_UNIFORM, SCALE_UNIFORM, SCALE_UNIFORM,
                         body_offset=scaled_center, color=roller_color)
        for label, data_list, used_set in [("cap", cap_data, used_caps),
                                            ("pin", pin_data, used_pins)]:
            best_idx, best_dist = -1, float('inf')
            for idx, (part, center) in enumerate(data_list):
                if idx in used_set:
                    continue
                d = np.linalg.norm(bc - center)
                if d < best_dist:
                    best_dist = d
                    best_idx = idx
            if best_idx >= 0:
                used_set.add(best_idx)
                part = data_list[best_idx][0]
                vp = rp + f"/{label}_vis"
                create_mesh_prim(stage, vp, part, src_center,
                                 SCALE_UNIFORM, SCALE_UNIFORM, SCALE_UNIFORM,
                                 body_offset=scaled_center, color=hub_color)
        dx = bc[0] - src_center[0]
        dz = bc[2] - src_center[2]
        theta = math.atan2(dz, dx)
        jp = rp + "/joint"
        joint = UsdPhysics.RevoluteJoint.Define(stage, jp)
        joint.CreateBody0Rel().SetTargets([hub_path])
        joint.CreateBody1Rel().SetTargets([rp])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(
            float(scaled_center[0]), float(scaled_center[1]), float(scaled_center[2])))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        half_a = float(-theta / 2.0)
        qy = Gf.Quatf(float(math.cos(half_a)), 0.0, float(math.sin(half_a)), 0.0)
        joint.CreateLocalRot0Attr().Set(qy)
        joint.CreateLocalRot1Attr().Set(qy)
        joint.CreateAxisAttr("Z")
        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jp), "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.001)

    return hub_path


def build_robot(stage, src_parts, src_center):
    robot_path = "/World/Robot"
    chassis_path = robot_path + "/Chassis"
    UsdGeom.Xform.Define(stage, robot_path)
    UsdGeom.Xform.Define(stage, chassis_path)
    cp = stage.GetPrimAtPath(chassis_path)
    UsdPhysics.RigidBodyAPI.Apply(cp)
    UsdPhysics.MassAPI.Apply(cp).CreateMassAttr(CHASSIS_MASS)
    UsdPhysics.MassAPI(cp).CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    UsdPhysics.ArticulationRootAPI.Apply(cp)
    PhysxSchema.PhysxArticulationAPI.Apply(cp)
    build_chassis(stage, chassis_path)

    wmat = UsdShade.Material.Define(stage, "/World/WheelMat")
    wpm = UsdPhysics.MaterialAPI.Apply(wmat.GetPrim())
    wpm.CreateStaticFrictionAttr(FRICTION_MU)
    wpm.CreateDynamicFrictionAttr(FRICTION_MU)
    wpm.CreateRestitutionAttr(0.1)
    wheel_mat = UsdShade.Material(stage.GetPrimAtPath("/World/WheelMat"))

    wheel_z = WHEEL_RADIUS - (BELLY_HEIGHT + CHASSIS_H / 2.0)
    wheel_offset = WHEEL_WIDTH / 2.0 + 0.005
    drive_joint_paths = []

    for wname in WHEEL_ORDER:
        cx, cy = WHEEL_XY[wname]
        angle_deg = AXLE_ANGLES[wname]
        angle_rad = math.radians(angle_deg)
        norm_len = math.sqrt(cx * cx + cy * cy)
        nx, ny = cx / norm_len, cy / norm_len
        wx = cx + nx * wheel_offset
        wy = cy + ny * wheel_offset

        # Visual bracket from chamfer face to wheel hub
        bp = chassis_path + f"/bracket_{wname}"
        bk = UsdGeom.Cube.Define(stage, bp)
        bk.GetSizeAttr().Set(1.0)
        bxf = UsdGeom.Xformable(bk.GetPrim())
        bxf.ClearXformOpOrder()
        bxf.AddTranslateOp().Set(Gf.Vec3d(
            (cx + wx) / 2.0, (cy + wy) / 2.0, float(wheel_z)))
        bxf.AddRotateZOp().Set(float(math.degrees(math.atan2(ny, nx))))
        bxf.AddScaleOp().Set(Gf.Vec3d(float(wheel_offset), 0.025, 0.025))
        bk.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.35, 0.35, 0.40)]))

        wp = robot_path + f"/Wheel_{wname}"
        wxf = UsdGeom.Xform.Define(stage, wp)
        wxf.ClearXformOpOrder()
        wxf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wheel_z))
        wxf.AddRotateZOp().Set(float(angle_deg))
        hub_path = build_omniwheel(stage, wp, src_parts, src_center, wheel_mat, wname)
        djp = robot_path + f"/DriveJoint_{wname}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, djp)
        joint.CreateBody0Rel().SetTargets([chassis_path])
        joint.CreateBody1Rel().SetTargets([hub_path])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(float(wx), float(wy), float(wheel_z)))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        half_a = angle_rad / 2.0
        rot_q = Gf.Quatf(float(math.cos(half_a)), 0.0, 0.0, float(math.sin(half_a)))
        joint.CreateLocalRot0Attr().Set(rot_q)
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateAxisAttr("Y")
        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(djp), "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(DRIVE_DAMPING)
        drive.CreateMaxForceAttr(DRIVE_MAX_FORCE)
        drive.CreateTargetVelocityAttr(0.0)
        jprim = stage.GetPrimAtPath(djp)
        jprim.CreateAttribute("isaacmecanumwheel:radius",
                              Sdf.ValueTypeNames.Float).Set(float(WHEEL_RADIUS))
        jprim.CreateAttribute("isaacmecanumwheel:angle",
                              Sdf.ValueTypeNames.Float).Set(90.0)
        drive_joint_paths.append(djp)

    # Spawn on outer floor — matches canonical/xdrive_realwheel.py
    SPAWN_X = 0.0
    SPAWN_Y = -1.59
    SPAWN_FLOOR_Z = cfg.Z_OUTER_IN * IN
    spawn_z = SPAWN_FLOOR_Z + BELLY_HEIGHT + CHASSIS_H / 2.0 + 0.02
    spawn_yaw = math.degrees(math.atan2(0 - SPAWN_Y, 0 - SPAWN_X))  # 90 deg
    rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
    rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(SPAWN_X, SPAWN_Y, spawn_z))
    rxf.AddRotateZOp().Set(spawn_yaw)
    r_from_origin = math.sqrt(SPAWN_X**2 + SPAWN_Y**2)

    total_mass = CHASSIS_MASS + 4 * WHEEL_MASS
    print(f"\nMass: chassis={CHASSIS_MASS:.2f}kg + 4x{WHEEL_MASS}kg wheels = "
          f"{total_mass:.2f}kg ({total_mass*2.205:.1f} lbs)", flush=True)
    print(f"Chassis: {CHASSIS_L/IN:.3f}x{CHASSIS_W/IN:.2f}x{CHASSIS_H/IN:.1f}\" "
          f"(skeleton, no chamfer)", flush=True)
    print(f"Spawn: ({SPAWN_X/IN:.1f}\", {SPAWN_Y/IN:.1f}\", {spawn_z/IN:.1f}\") "
          f"R={r_from_origin/IN:.1f}\" yaw={spawn_yaw:.1f}deg", flush=True)

    return robot_path, chassis_path, drive_joint_paths, spawn_z


# ============================================================================
# X-drive IK
# ============================================================================

def xdrive_ik(vx, vy, omega):
    r = WHEEL_RADIUS
    vels = []
    for wn in WHEEL_ORDER:
        wx, wy = WHEEL_XY[wn]
        vwx = vx - omega * wy
        vwy = vy + omega * wx
        angle_rad = math.radians(AXLE_ANGLES[wn])
        drive_x = math.cos(angle_rad)
        drive_y = math.sin(angle_rad)
        v_drive = vwx * drive_x + vwy * drive_y
        vels.append(v_drive / r)
    return np.array(vels)


# ============================================================================
# State helpers
# ============================================================================

def get_state(art, stage, chassis_path):
    prim = stage.GetPrimAtPath(chassis_path)
    if not prim.IsValid():
        return None
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = mat.ExtractTranslation()
    jv, je = None, None
    try: jv = art.get_joint_velocities()
    except: pass
    try: je = art.get_measured_joint_efforts()
    except:
        try: je = art.get_applied_joint_efforts()
        except: pass
    return {"pos": np.array([t[0], t[1], t[2]]), "jv": jv, "je": je, "mat": mat}


def get_yaw(mat):
    return math.degrees(math.atan2(mat[1][0], mat[0][0]))


def normalize_angle(deg):
    while deg > 180: deg -= 360
    while deg < -180: deg += 360
    return deg


# ============================================================================
# MAIN — single orbit test
# ============================================================================
print("=" * 60, flush=True)
print(f"FORTIS Orbit Torque Test (V2 Chassis) — {ORBIT_SPEED:.2f} m/s", flush=True)
print(f"Duration: {ORBIT_DURATION:.1f}s, Orbit circumference: {ORBIT_CIRCUMFERENCE:.2f}m", flush=True)
print("=" * 60, flush=True)

src_parts, src_center = read_source_wheel()

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(10): app.update()
stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

build_physics_scene(stage)

# Load reactor
print(f"Loading reactor: {REACTOR_USD}", flush=True)
rp = stage.DefinePrim("/World/Reactor", "Xform")
rp.GetReferences().AddReference(REACTOR_USD)
dl = stage.DefinePrim("/World/Light", "DistantLight")
dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)

robot_path, chassis_path, drive_joint_paths, spawn_z = build_robot(
    stage, src_parts, src_center)

for _ in range(20): app.update()

world = World(stage_units_in_meters=1.0)
omni.timeline.get_timeline_interface().play()
for _ in range(10): app.update()

art = Articulation(chassis_path)
art.initialize()
for _ in range(10): world.step(render=not headless)

if not art.is_physics_handle_valid():
    print("FATAL: Articulation physics handle invalid", flush=True)
    app.close()
    sys.exit(1)

ndof = art.num_dof
dof_names = art.dof_names
print(f"Articulation: {ndof} DOFs", flush=True)

# Find drive DOF indices
drive_dof_indices = []
for djp in drive_joint_paths:
    joint_name = djp.split("/")[-1]
    for i, dn in enumerate(dof_names):
        if joint_name in dn or dn == joint_name:
            drive_dof_indices.append(i)
            break
if len(drive_dof_indices) != 4:
    drive_dof_indices = []
    for i, dn in enumerate(dof_names):
        if "Drive" in dn or "drive" in dn:
            drive_dof_indices.append(i)
    if len(drive_dof_indices) != 4:
        print(f"WARNING: Found {len(drive_dof_indices)} drive DOFs, using first 4", flush=True)
        drive_dof_indices = list(range(4))
print(f"Drive DOF indices: {drive_dof_indices} "
      f"({[dof_names[i] for i in drive_dof_indices]})", flush=True)

art.switch_control_mode("velocity", joint_indices=np.arange(ndof))

# Settle
print("Settling 3s...", flush=True)
for i in range(3 * PHYSICS_HZ):
    world.step(render=not headless)
    if i % (PHYSICS_HZ // 2) == 0:
        s = get_state(art, stage, chassis_path)
        if s:
            p = s["pos"]
            yaw_s = get_yaw(s["mat"])
            print(f"  [{i/PHYSICS_HZ:.1f}s] pos=({p[0]/IN:.1f},{p[1]/IN:.1f},{p[2]/IN:.1f})\" "
                  f"yaw={yaw_s:.1f}", flush=True)

# Initial state
s0 = get_state(art, stage, chassis_path)
if not s0:
    print("FATAL: no initial state", flush=True)
    app.close()
    sys.exit(1)
p0 = s0["pos"]
initial_z = p0[2]
print(f"\nInitial pos: ({p0[0]/IN:.1f}, {p0[1]/IN:.1f}, {p0[2]/IN:.1f})\"", flush=True)
print(f"Initial yaw: {get_yaw(s0['mat']):.1f} deg", flush=True)

# Prepare CSV
os.makedirs(args.out_dir, exist_ok=True)
csv_path = os.path.join(args.out_dir, f"orbit_torque_{ORBIT_SPEED:.2f}mps.csv")
csv_file = open(csv_path, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "sim_time", "torque_FL", "torque_FR", "torque_BL", "torque_BR",
    "x", "y", "z", "yaw_deg", "actual_speed_mps", "heading_correction_rads"
])

# Orbit loop
total_frames = int(ORBIT_DURATION * PHYSICS_HZ)
prev_pos = p0.copy()
prev_log_time = 0.0
aborted = False

r0 = math.sqrt(p0[0]**2 + p0[1]**2)
orbit_omega = ORBIT_SPEED / r0

tv = xdrive_ik(0, ORBIT_SPEED, orbit_omega)
va = np.zeros(ndof)
for ii, di in enumerate(drive_dof_indices):
    va[di] = tv[ii]

print(f"\nActual orbit radius: {r0:.4f}m ({r0/IN:.1f}\")", flush=True)
print(f"Orbit omega: {orbit_omega:.3f} rad/s", flush=True)
print(f"Wheel vel targets: [{', '.join(f'{v:.2f}' for v in tv)}] rad/s", flush=True)
print(f"Starting orbit: speed={ORBIT_SPEED:.2f}m/s, duration={ORBIT_DURATION:.1f}s, "
      f"frames={total_frames}", flush=True)

for frame in range(total_frames):
    t = frame / PHYSICS_HZ

    if art.is_physics_handle_valid():
        art.set_joint_velocity_targets(va.reshape(1, -1))
    world.step(render=not headless)

    if frame % LOG_INTERVAL == 0 or frame % (2 * PHYSICS_HZ) == 0:
        s = get_state(art, stage, chassis_path)
        if not s:
            continue
        pos = s["pos"]
        mat = s["mat"]
        actual_yaw = get_yaw(mat)
        r_actual = math.sqrt(pos[0]**2 + pos[1]**2)
        r_error = r_actual - r0

    if frame % LOG_INTERVAL == 0 and s:
        dt_log = t - prev_log_time if prev_log_time > 0 else 1.0 / LOG_HZ
        if dt_log <= 0: dt_log = 1.0 / LOG_HZ
        ddx = pos[0] - prev_pos[0]
        ddy = pos[1] - prev_pos[1]
        actual_speed = math.sqrt(ddx**2 + ddy**2) / dt_log

        je = s["je"]
        torques = [0.0, 0.0, 0.0, 0.0]
        if je is not None:
            je_flat = je.flatten()
            for ii, di in enumerate(drive_dof_indices):
                if di < len(je_flat):
                    torques[ii] = float(je_flat[di])

        csv_writer.writerow([
            f"{t:.4f}",
            f"{torques[0]:.4f}", f"{torques[1]:.4f}",
            f"{torques[2]:.4f}", f"{torques[3]:.4f}",
            f"{pos[0]:.5f}", f"{pos[1]:.5f}", f"{pos[2]:.5f}",
            f"{actual_yaw:.2f}", f"{actual_speed:.4f}",
            f"0.0000"
        ])
        prev_pos = pos.copy()
        prev_log_time = t

    if frame % (2 * PHYSICS_HZ) == 0 and frame > 0 and s:
        orbit_angle = math.degrees(math.atan2(pos[1], pos[0]))
        print(f"[{t:.0f}s] r={r_actual/IN:.1f}\" yaw={actual_yaw:.1f} "
              f"r_err={r_error/IN:+.1f}\" orbit_ang={orbit_angle:.0f}", flush=True)

    if t > 5.0 and frame % LOG_INTERVAL == 0 and s:
        if pos[2] < initial_z - Z_DROP_ABORT:
            print(f"ABORT: Robot fell off floor at t={t:.2f}s "
                  f"(Z={pos[2]/IN:.1f}\" vs initial {initial_z/IN:.1f}\")", flush=True)
            aborted = True
            break

csv_file.close()
print(f"\nCSV saved: {csv_path}", flush=True)

# Final state
sf = get_state(art, stage, chassis_path)
if sf:
    pf = sf["pos"]
    rf = math.sqrt(pf[0]**2 + pf[1]**2)
    print(f"Final pos: ({pf[0]/IN:.1f}, {pf[1]/IN:.1f}, {pf[2]/IN:.1f})\" R={rf/IN:.1f}\"",
          flush=True)
    print(f"Final yaw: {get_yaw(sf['mat']):.1f} deg", flush=True)

omni.timeline.get_timeline_interface().stop()
app.close()
sys.exit(1 if aborted else 0)
