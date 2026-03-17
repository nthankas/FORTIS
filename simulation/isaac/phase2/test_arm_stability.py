"""
Phase 2 Step 5: Arm Stability Tests

Tests chassis stability with the arm in various coverage poses.
Wheels are braked (zero velocity). Measures tilt, sliding, joint torques.
Uses live URDF import into reactor scene (no saved USD).

Run:
  E:\FORTIS\IsaacSim\python.bat E:\FORTIS\Optimizer\phase2\test_arm_stability.py
  E:\FORTIS\IsaacSim\python.bat E:\FORTIS\Optimizer\phase2\test_arm_stability.py --gui
"""
import os, sys, math, json
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
cli, _ = parser.parse_known_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": not cli.gui, "width": 1920, "height": 1080})

import omni.usd
import omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

from setup_phase2_scene import prepare_scene, ES

DT = 1.0 / 60.0
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
SETTLE_TIME = 3.0

WHEEL_JOINT_NAMES = ["fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"]
ARM_JOINT_NAMES = ["j1_yaw", "j2_shoulder", "j3_elbow", "j4_wrist"]

# Arm poses: [j1_yaw, j2_shoulder, j3_elbow, j4_wrist] in degrees
ARM_POSES = {
    "stowed":          [0,    0,   -150,  120],
    "scorpion_near":   [0,    100, -120,   60],
    "extended_far":    [0,    30,   -30,    0],
    "upward_reach":    [0,    90,   -60,    0],
    "lateral_left":    [90,   60,   -60,    0],
    "lateral_right":   [-90,  60,   -60,    0],
}

print("=" * 60)
print("FORTIS Phase 2: Arm Stability Tests")
print("=" * 60)


def run_arm_test(pose_name, joint_angles_deg):
    scene = prepare_scene(app)
    stage = scene["stage"]
    joint_paths = scene["joint_paths"]
    wheel_joint_names = scene["wheel_joint_names"]
    arm_joint_names = scene["arm_joint_names"]

    # Brake wheels
    for wn in wheel_joint_names:
        path = joint_paths.get(wn)
        if path:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                d = UsdPhysics.DriveAPI.Get(prim, "angular")
                if d:
                    d.GetStiffnessAttr().Set(0.0)
                    d.GetDampingAttr().Set(10000.0)
                    d.GetMaxForceAttr().Set(50.0)
                    d.GetTargetVelocityAttr().Set(0.0)

    # Set arm joint targets
    for i, jname in enumerate(arm_joint_names):
        path = joint_paths.get(jname)
        if path:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                d = UsdPhysics.DriveAPI.Get(prim, "angular")
                if d:
                    d.GetTargetPositionAttr().Set(float(joint_angles_deg[i]))

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

    # Brake wheels via articulation too
    vel_targets = np.zeros(art.num_dof, dtype=np.float64)

    # Settle with brakes on
    for _ in range(int(2.0 / DT)):
        art.set_joint_velocity_targets(vel_targets)
        world.step(render=cli.gui)

    def get_state():
        tf = UsdGeom.Xformable(chassis_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = tf.ExtractTranslation()
        rot = tf.ExtractRotation()
        q = rot.GetQuat()
        w = q.GetReal()
        img = q.GetImaginary()
        roll = math.degrees(2.0 * math.atan2(img[0], w))
        pitch = math.degrees(2.0 * math.atan2(img[1], w))
        yaw = math.degrees(2.0 * math.atan2(img[2], w))
        return {"x": pos[0], "y": pos[1], "z": pos[2],
                "roll": roll, "pitch": pitch, "yaw": yaw}

    state0 = get_state()

    # Set arm position targets through articulation
    pos_targets = np.zeros(art.num_dof, dtype=np.float64)
    for i, jname in enumerate(arm_joint_names):
        if jname in dof_names:
            idx = dof_names.index(jname)
            pos_targets[idx] = math.radians(joint_angles_deg[i])

    for s in range(int(SETTLE_TIME / DT)):
        art.set_joint_velocity_targets(vel_targets)
        art.set_joint_position_targets(pos_targets)
        world.step(render=cli.gui)

    state_final = get_state()

    # Read actual arm joint positions
    jp = art.get_joint_positions()
    arm_actual_deg = {}
    for jname in arm_joint_names:
        if jname in dof_names:
            idx = dof_names.index(jname)
            arm_actual_deg[jname] = round(math.degrees(jp[idx]), 1) if jp is not None else None

    roll_change = abs(state_final["roll"] - state0["roll"])
    pitch_change = abs(state_final["pitch"] - state0["pitch"])
    z_change = abs(state_final["z"] - state0["z"]) / ES
    x_drift = abs(state_final["x"] - state0["x"]) / ES
    y_drift = abs(state_final["y"] - state0["y"]) / ES

    tipped = roll_change > 15 or pitch_change > 15 or z_change > 2.0
    slid = x_drift > 1.0 or y_drift > 1.0

    result = {
        "pose": pose_name,
        "target_angles_deg": joint_angles_deg,
        "actual_angles_deg": arm_actual_deg,
        "initial_state": {
            "roll": round(state0["roll"], 2),
            "pitch": round(state0["pitch"], 2),
            "z_in": round(state0["z"] / ES, 2),
        },
        "final_state": {
            "roll": round(state_final["roll"], 2),
            "pitch": round(state_final["pitch"], 2),
            "z_in": round(state_final["z"] / ES, 2),
        },
        "roll_change_deg": round(roll_change, 2),
        "pitch_change_deg": round(pitch_change, 2),
        "z_change_in": round(z_change, 2),
        "x_drift_in": round(x_drift, 2),
        "y_drift_in": round(y_drift, 2),
        "tipped": tipped,
        "slid": slid,
        "stable": not tipped and not slid,
    }

    omni.timeline.get_timeline_interface().stop()
    world.clear()
    del world
    return result


# -- Run tests --
all_results = []
print(f"\nTesting {len(ARM_POSES)} arm poses...\n")

for pose_name, angles in ARM_POSES.items():
    print(f"  {pose_name}: [{', '.join(str(a) for a in angles)}] deg ... ", end="", flush=True)
    try:
        result = run_arm_test(pose_name, angles)
        all_results.append(result)
        status = "STABLE" if result["stable"] else "UNSTABLE"
        if result["tipped"]: status = "TIPPED"
        if result["slid"]: status += "+SLID"
        print(f"{status} | dRoll={result['roll_change_deg']:.1f} "
              f"dPitch={result['pitch_change_deg']:.1f} dZ={result['z_change_in']:.2f}\"")
        if result.get("actual_angles_deg"):
            print(f"           actual: {result['actual_angles_deg']}")
    except Exception as e:
        print(f"ERROR: {e}")
        all_results.append({"pose": pose_name, "error": str(e)})

# -- Save --
os.makedirs(DATA_DIR, exist_ok=True)
out_path = os.path.join(DATA_DIR, "arm_stability_results.json")
with open(out_path, "w") as f:
    json.dump({"results": all_results}, f, indent=2)
print(f"\nResults saved: {out_path}")

# -- Summary --
print(f"\n{'='*80}")
print(f"{'Pose':>18} {'Stable':>7} {'dRoll':>7} {'dPitch':>8} {'dZ':>8} {'Tipped':>7} {'Slid':>6}")
print(f"{'-'*80}")
for r in all_results:
    if "error" in r:
        print(f"{r['pose']:>18} {'ERROR':>7}")
        continue
    print(f"{r['pose']:>18} {'YES' if r['stable'] else 'NO':>7} "
          f"{r['roll_change_deg']:>7.1f} {r['pitch_change_deg']:>8.1f} "
          f"{r['z_change_in']:>8.2f} "
          f"{'YES' if r['tipped'] else 'no':>7} "
          f"{'YES' if r['slid'] else 'no':>6}")
print(f"{'='*80}")

unstable = [r for r in all_results if not r.get("stable") and not r.get("error")]
if unstable:
    worst = max(unstable, key=lambda r: r.get("roll_change_deg", 0) + r.get("pitch_change_deg", 0))
    print(f"\nWorst-case pose: {worst['pose']}")
else:
    print(f"\nAll poses are stable!")

app.close()
