"""
Phase 2 Step 4: Automated Toroidal Arc Driving Sweep

Tests straight driving, arc following, and tight turns at various speeds.
Uses live URDF import into reactor scene (no saved USD).

Run:
  E:\FORTIS\IsaacSim\python.bat E:\FORTIS\Optimizer\phase2\test_driving.py
  E:\FORTIS\IsaacSim\python.bat E:\FORTIS\Optimizer\phase2\test_driving.py --gui
"""
import os, sys, math, json
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--torque", type=float, default=5.0)
cli, _ = parser.parse_known_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": not cli.gui, "width": 1920, "height": 1080})

import omni.usd
import omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

from fortis_config import R_CENTER_POST, R_OUTER_WALL
from setup_phase2_scene import prepare_scene, ES

DT = 1.0 / 60.0
WHEEL_RADIUS = 0.072
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

DAMPING = 1000.0
VELOCITIES = [0.05, 0.1, 0.15, 0.2, 0.3]
TESTS = [
    ("straight",   1.0,  1.0,  10.0),
    ("arc_gentle",  0.8,  1.0,  20.0),
    ("arc_tight",   0.0,  1.0,  10.0),
]

print("=" * 60)
print("FORTIS Phase 2: Driving Test Sweep")
print("=" * 60)


def run_driving_test(vel_ms, inner_ratio, outer_ratio, duration_s, max_torque):
    scene = prepare_scene(app)
    stage = scene["stage"]
    joint_paths = scene["joint_paths"]
    wheel_joint_names = scene["wheel_joint_names"]

    for wn in wheel_joint_names:
        path = joint_paths.get(wn)
        if path:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                d = UsdPhysics.DriveAPI.Get(prim, "angular")
                if d:
                    d.GetStiffnessAttr().Set(0.0)
                    d.GetDampingAttr().Set(float(DAMPING))
                    d.GetMaxForceAttr().Set(float(max_torque))

    for _ in range(5):
        app.update()

    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation

    world = World(stage_units_in_meters=1.0)
    omni.timeline.get_timeline_interface().play()

    for _ in range(10):
        app.update()

    art = None
    art_root = scene["art_root_path"]
    robot_path = scene["robot_prim_path"]
    for try_path in [art_root, robot_path, str(Sdf.Path(robot_path).GetParentPath())]:
        if not try_path or try_path == "/":
            continue
        prim = stage.GetPrimAtPath(try_path)
        if not prim.IsValid():
            continue
        try:
            candidate = Articulation(try_path)
            candidate.initialize()
            if candidate.is_physics_handle_valid():
                art = candidate
                break
        except Exception:
            continue

    if art is None:
        raise RuntimeError("Articulation not found")

    dof_names = art.dof_names
    chassis_prim = stage.GetPrimAtPath(str(art.prim_path))

    for _ in range(int(2.0 / DT)):
        world.step(render=cli.gui)

    def get_state():
        tf = UsdGeom.Xformable(chassis_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = tf.ExtractTranslation()
        return pos

    pos0 = get_state()
    init_r = math.sqrt(pos0[0]**2 + pos0[1]**2) / ES
    init_angle = math.atan2(pos0[1], pos0[0])
    prev_angle = init_angle
    angle_acc = 0.0
    collision = False
    max_zdrop = 0.0
    init_z = pos0[2]

    inner_radps = vel_ms * inner_ratio / WHEEL_RADIUS
    outer_radps = vel_ms * outer_ratio / WHEEL_RADIUS
    vel_targets = np.zeros(art.num_dof, dtype=np.float64)
    for wn in wheel_joint_names:
        if wn in dof_names:
            idx = dof_names.index(wn)
            if "fl" in wn or "rl" in wn:
                vel_targets[idx] = inner_radps
            else:
                vel_targets[idx] = outer_radps

    radii = []
    steps = int(duration_s / DT)

    for s in range(steps):
        art.set_joint_velocity_targets(vel_targets)
        world.step(render=cli.gui)

        if s % 6 == 0:
            pos = get_state()
            r = math.sqrt(pos[0]**2 + pos[1]**2) / ES
            cur_angle = math.atan2(pos[1], pos[0])
            da = cur_angle - prev_angle
            if da > math.pi: da -= 2 * math.pi
            elif da < -math.pi: da += 2 * math.pi
            angle_acc += da
            prev_angle = cur_angle

            zdrop = abs(init_z - pos[2])
            max_zdrop = max(max_zdrop, zdrop)

            if r < R_CENTER_POST + 2 or r > R_OUTER_WALL - 2:
                collision = True

            radii.append(r)
            if collision:
                break

    lateral_drift = abs(np.mean(radii) - init_r) if radii else 0.0

    result = {
        "vel_ms": vel_ms,
        "inner_ratio": inner_ratio,
        "outer_ratio": outer_ratio,
        "max_torque_nm": max_torque,
        "orbit_deg": round(math.degrees(angle_acc), 1),
        "collision": collision,
        "max_zdrop_in": round(max_zdrop / ES, 2),
        "lateral_drift_in": round(lateral_drift, 1),
        "avg_radius_in": round(np.mean(radii), 1) if radii else 0,
    }

    omni.timeline.get_timeline_interface().stop()
    world.clear()
    del world
    return result


# -- Run sweep --
all_results = {}
for test_name, ir, or_, dur in TESTS:
    all_results[test_name] = []
    print(f"\n--- {test_name} (inner={ir}, outer={or_}, dur={dur}s) ---")
    for vel in VELOCITIES:
        print(f"  vel={vel:.2f} ... ", end="", flush=True)
        try:
            result = run_driving_test(vel, ir, or_, dur, cli.torque)
            result["test"] = test_name
            all_results[test_name].append(result)
            coll = " COLLISION" if result["collision"] else ""
            print(f"orbit={result['orbit_deg']}deg drift={result['lateral_drift_in']}\"{coll}")
        except Exception as e:
            print(f"ERROR: {e}")
            all_results[test_name].append({"vel_ms": vel, "test": test_name, "error": str(e)})

# -- Save --
os.makedirs(DATA_DIR, exist_ok=True)
out_path = os.path.join(DATA_DIR, "driving_results.json")
with open(out_path, "w") as f:
    json.dump({"max_torque": cli.torque, "damping": DAMPING, "tests": all_results}, f, indent=2)
print(f"\nResults saved: {out_path}")

# -- Summary --
print(f"\n{'='*80}")
print(f"{'Test':>12} {'Vel':>8} {'Orbit':>8} {'Drift':>8} {'Avg R':>8} {'Coll':>6}")
print(f"{'-'*80}")
for tn in ["straight", "arc_gentle", "arc_tight"]:
    for r in all_results.get(tn, []):
        if "error" in r:
            print(f"{tn:>12} {r['vel_ms']:>8.2f} {'ERROR':>8}")
            continue
        print(f"{tn:>12} {r['vel_ms']:>8.2f} {r['orbit_deg']:>8.1f} "
              f"{r['lateral_drift_in']:>8.1f} {r['avg_radius_in']:>8.1f} "
              f"{'YES' if r['collision'] else 'no':>6}")
print(f"{'='*80}")

for tn in ["straight", "arc_gentle", "arc_tight"]:
    safe = [r for r in all_results.get(tn, []) if not r.get("collision") and not r.get("error")]
    if safe:
        fastest = max(safe, key=lambda r: r["vel_ms"])
        print(f"{tn}: max safe speed = {fastest['vel_ms']} m/s")

app.close()
