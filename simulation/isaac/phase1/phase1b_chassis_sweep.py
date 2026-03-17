"""
Phase 1b: Chassis stability sweep via quasi-static PhysX simulation.

For each chassis configuration and azimuthal angle:
  1. Place chassis straddling the step at that angle
  2. Let it settle under gravity (PhysX handles collision with reactor mesh)
  3. Check if it tipped (Z drop) or collided with walls
  4. Repeat at 15-degree increments around the full circle

This tests whether the chassis is stable (doesn't tip) at all rotation angles
without needing working wheel drives - the key Phase 1b question.

Run with:
    E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1b_chassis_sweep.py --single
    E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1b_chassis_sweep.py
"""

import os
import sys
import json
import math
import argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="Show GUI viewport")
parser.add_argument("--single", action="store_true", help="Test single recommended config")
cli_args, _ = parser.parse_known_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": not cli_args.gui,
    "width": 1920,
    "height": 1080,
})

import omni.usd
from pxr import (
    Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf,
    PhysxSchema, Vt
)

from fortis_config import (
    INCHES_TO_METERS, SCENE_REACTOR, RESULTS_DIR,
    REACTOR_MESH_PRIM_PATH,
    Z_INNER, Z_OUTER, STEP_HEIGHT, STEP_R,
    R_INNER_FLOOR_MIN, R_INNER_FLOOR_MAX,
    R_OUTER_FLOOR_MIN, R_OUTER_FLOOR_MAX,
    R_CENTER_POST, R_OUTER_WALL,
    GRAPHITE_MU_LOW, GRAPHITE_MU_HIGH,
    RECOMMENDED_CHASSIS,
)

LBS_TO_KG = 0.453592
PHYSICS_DT = 1.0 / 60.0

# reactor.usd effective scale: parent unitsResolve(0.01) * mesh scale(2.4) = 0.024
EFFECTIVE_SCALE = 0.024

def i2w(inches):
    """Convert inches to world units."""
    return inches * EFFECTIVE_SCALE

# Azimuthal angles to test (every 15 degrees)
AZIMUTH_STEPS = list(range(0, 360, 15))


def prepare_scene(stage):
    """Add PhysicsScene if needed."""
    ps = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(ps).IsValid():
        p = UsdPhysics.Scene.Define(stage, ps)
        p.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        p.CreateGravityMagnitudeAttr(9.81)
        px = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(ps))
        px.CreateEnableCCDAttr(True)
        px.CreateEnableStabilizationAttr(True)
        px.CreateSolverTypeAttr("TGS")
        print("  Added PhysicsScene")
    return True


def delete_prim(stage, path):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(Sdf.Path(path))


def place_chassis(stage, params, azimuth_deg):
    """
    Place chassis at given azimuthal angle, straddling the step.
    Returns the prim path and expected Z after settling.
    """
    L = i2w(params["length"])
    W = i2w(params["width"])
    H = i2w(params["height"])
    M = params["weight"] * LBS_TO_KG
    hl, hw, hh = L/2, W/2, H/2

    azimuth = math.radians(azimuth_deg)
    center_r = i2w(STEP_R)
    cx = center_r * math.cos(azimuth)
    cy = center_r * math.sin(azimuth)

    # Chassis long axis is tangent to the circle
    yaw = azimuth + math.pi / 2

    # Compute approximate floor Z under chassis center
    # Inner wheels (~49.5") and outer wheels (~62.4") based on 13" track width
    inner_floor_z = i2w(Z_INNER)
    outer_floor_z = i2w(Z_OUTER)
    avg_floor_z = (inner_floor_z + outer_floor_z) / 2

    # Place chassis so bottom is slightly above the average floor
    cz = avg_floor_z + hh + i2w(2.0)  # 2" above expected position, will settle

    chassis_path = "/World/Robot"
    delete_prim(stage, chassis_path)

    # Create mesh box (no scale op)
    v = [Gf.Vec3f(-hl,-hw,-hh), Gf.Vec3f(hl,-hw,-hh),
         Gf.Vec3f(hl,hw,-hh), Gf.Vec3f(-hl,hw,-hh),
         Gf.Vec3f(-hl,-hw,hh), Gf.Vec3f(hl,-hw,hh),
         Gf.Vec3f(hl,hw,hh), Gf.Vec3f(-hl,hw,hh)]
    fi = [0,1,2,3, 4,5,6,7, 0,4,5,1, 2,6,7,3, 0,3,7,4, 1,5,6,2]
    fc = [4,4,4,4,4,4]
    mesh = UsdGeom.Mesh.Define(stage, chassis_path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(v))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(fi))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(fc))
    mesh.GetSubdivisionSchemeAttr().Set("none")

    xf = UsdGeom.Xformable(mesh)
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    half_yaw = yaw / 2
    xf.AddOrientOp().Set(Gf.Quatf(
        float(math.cos(half_yaw)), 0, 0, float(math.sin(half_yaw))
    ))

    prim = mesh.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).GetApproximationAttr().Set("convexHull")
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(M))

    # Low damping for settling
    prb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    prb.CreateLinearDampingAttr(0.5)
    prb.CreateAngularDampingAttr(0.5)

    return chassis_path, cz


def test_stability_at_angle(stage, world, params, azimuth_deg, settle_s=3.0, headless=True):
    """
    Place chassis at given angle, let settle, check if it tipped.
    Returns dict with stability metrics.
    """
    chassis_path, init_z = place_chassis(stage, params, azimuth_deg)

    # Step a few frames so the new prim is picked up
    for _ in range(5):
        simulation_app.update()

    world.reset()

    prim = stage.GetPrimAtPath(chassis_path)

    # Settle
    settle_steps = int(settle_s / PHYSICS_DT)
    for _ in range(settle_steps):
        world.step(render=not headless)

    # Read final pose
    tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = tf.ExtractTranslation()
    rot = tf.ExtractRotation()

    final_z = pos[2]
    avg_floor_z = (i2w(Z_INNER) + i2w(Z_OUTER)) / 2

    # Check tipping: if the chassis dropped significantly or tilted too much
    z_drop = init_z - final_z
    expected_z = avg_floor_z + i2w(params["height"]) / 2

    # Compute tilt from rotation (how far from horizontal)
    rot_angle = rot.GetAngle()
    rot_axis = rot.GetAxis()
    # Tilt = rotation component NOT about Z (roll/pitch)
    tilt_deg = rot_angle * math.sqrt(rot_axis[0]**2 + rot_axis[1]**2)

    # Fell through floor?
    fell_through = final_z < avg_floor_z - i2w(12)

    # Tipped over? (tilt > 30 degrees or Z dropped significantly)
    tipped = tilt_deg > 30.0 or z_drop > i2w(params["height"])

    # Radial position check: did it slide off the floor?
    r_final = math.sqrt(pos[0]**2 + pos[1]**2) / EFFECTIVE_SCALE
    in_bounds = R_CENTER_POST < r_final < R_OUTER_WALL

    # Clearance check: compute rotation diagonal
    diag = math.sqrt(params["length"]**2 + params["width"]**2)
    floor_annulus_width = R_OUTER_WALL - R_CENTER_POST  # ~32.3"

    return {
        "azimuth_deg": azimuth_deg,
        "final_z": final_z,
        "z_drop_inches": z_drop / EFFECTIVE_SCALE,
        "tilt_deg": tilt_deg,
        "tipped": tipped,
        "fell_through": fell_through,
        "r_final_inches": r_final,
        "in_bounds": in_bounds,
        "rotation_diagonal_inches": diag,
        "floor_width_inches": floor_annulus_width,
        "clearance_ok": diag < floor_annulus_width,
    }


def test_chassis_config(stage, world, params, headless=True):
    """Test a chassis config at all azimuthal angles."""
    print(f"\n  Config: {params['length']}L x {params['width']}W x {params['height']}H, "
          f"{params['weight']}lbs")

    # Pre-check: rotation diagonal clearance
    diag = math.sqrt(params["length"]**2 + params["width"]**2)
    floor_width = R_OUTER_WALL - R_CENTER_POST
    if diag > floor_width - 2:  # 2" safety margin
        print(f"    SKIP: diagonal {diag:.1f}\" > floor width {floor_width:.1f}\"")
        return {
            "params": params,
            "pass_stability": False,
            "pass_clearance": False,
            "reason": f"diagonal {diag:.1f}\" exceeds floor width {floor_width:.1f}\"",
        }

    # Stowed width check
    stowed_w = params.get("stowed_width", params["width"] + 4.0)  # +4" for wheel overhang
    if stowed_w > 14.75:
        print(f"    SKIP: stowed width {stowed_w:.1f}\" > 14.75\"")
        return {
            "params": params,
            "pass_stability": False,
            "pass_clearance": False,
            "reason": f"stowed width {stowed_w:.1f}\" > 14.75\"",
        }

    angle_results = []
    all_stable = True
    worst_tilt = 0
    worst_z_drop = 0

    for az_deg in AZIMUTH_STEPS:
        result = test_stability_at_angle(stage, world, params, az_deg, headless=headless)
        angle_results.append(result)

        if result["tipped"] or result["fell_through"]:
            all_stable = False

        worst_tilt = max(worst_tilt, result["tilt_deg"])
        worst_z_drop = max(worst_z_drop, result["z_drop_inches"])

        status = "OK" if not result["tipped"] and not result["fell_through"] else "FAIL"
        if az_deg % 90 == 0:  # Print every 90 degrees
            print(f"    az={az_deg:3d}deg: z_drop={result['z_drop_inches']:.1f}\" "
                  f"tilt={result['tilt_deg']:.1f}deg r={result['r_final_inches']:.1f}\" {status}")

    pass_stability = all_stable
    pass_clearance = diag < floor_width - 2

    overall = "PASS" if (pass_stability and pass_clearance) else "FAIL"
    print(f"    Result: {overall} | worst_tilt={worst_tilt:.1f}deg "
          f"worst_z_drop={worst_z_drop:.1f}\" clearance={'OK' if pass_clearance else 'FAIL'}")

    return {
        "params": params,
        "pass_stability": pass_stability,
        "pass_clearance": pass_clearance,
        "worst_tilt_deg": worst_tilt,
        "worst_z_drop_inches": worst_z_drop,
        "rotation_diagonal_inches": diag,
        "angle_results": angle_results,
    }


def get_sweep_configs():
    """Generate sweep configurations."""
    configs = []
    for length in [12, 14, 16]:
        for width in [10, 12, 13, 14]:
            for height in [5, 7, 9]:
                for weight in [25, 35, 50]:
                    configs.append({
                        "length": length, "width": width,
                        "height": height, "weight": weight,
                    })
    return configs


def get_single_config():
    return [{
        "length": RECOMMENDED_CHASSIS["length"],
        "width": RECOMMENDED_CHASSIS["width"],
        "height": RECOMMENDED_CHASSIS["height"],
        "weight": RECOMMENDED_CHASSIS["weight"],
    }]


if __name__ == "__main__":
    print("=" * 60)
    print("FORTIS Phase 1b: Chassis Stability Sweep (Isaac Sim)")
    print("=" * 60)
    print(f"  Approach: Quasi-static stability at {len(AZIMUTH_STEPS)} azimuthal angles")
    print(f"  Effective scale: {EFFECTIVE_SCALE}")

    print(f"\nLoading: {SCENE_REACTOR}")
    if not os.path.exists(SCENE_REACTOR):
        print(f"ERROR: Not found: {SCENE_REACTOR}")
        simulation_app.close()
        sys.exit(1)

    omni.usd.get_context().open_stage(SCENE_REACTOR)
    for _ in range(10):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("ERROR: No stage")
        simulation_app.close()
        sys.exit(1)
    print("  Stage loaded.")

    prepare_scene(stage)

    for _ in range(5):
        simulation_app.update()

    from isaacsim.core.api import World
    world = World(stage_units_in_meters=1.0)

    if cli_args.single:
        configs = get_single_config()
        print(f"\nSingle config test")
    else:
        configs = get_sweep_configs()
        print(f"\nSweep: {len(configs)} configs x {len(AZIMUTH_STEPS)} angles")

    results = []
    for i, cfg in enumerate(configs):
        print(f"\n[{i+1}/{len(configs)}]", end="")
        try:
            r = test_chassis_config(stage, world, cfg, not cli_args.gui)
            results.append(r)
        except Exception as e:
            import traceback
            print(f"    ERROR: {e}\n{traceback.format_exc()}")
            results.append({"params": cfg, "error": str(e)})

    os.makedirs(RESULTS_DIR, exist_ok=True)
    out = os.path.join(RESULTS_DIR, "skid_steer_isaac.json")

    summary = {
        "total_tested": len(results),
        "pass_stability": sum(1 for r in results if r.get("pass_stability")),
        "pass_clearance": sum(1 for r in results if r.get("pass_clearance")),
        "pass_both": sum(1 for r in results
                         if r.get("pass_stability") and r.get("pass_clearance")),
        "azimuth_steps_deg": AZIMUTH_STEPS,
        "effective_scale": EFFECTIVE_SCALE,
        "results": results,
    }

    with open(out, "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\n{'='*60}")
    print("Phase 1b Results")
    print(f"{'='*60}")
    ps = summary["pass_stability"]
    pc = summary["pass_clearance"]
    pb = summary["pass_both"]
    print(f"  Tested: {len(results)}")
    print(f"  Pass stability: {ps}")
    print(f"  Pass clearance: {pc}")
    print(f"  Pass both: {pb}")
    print(f"  Saved: {out}")
    print(f"{'='*60}")

    if cli_args.gui:
        print("\nGUI open. Close to exit.")
        while simulation_app.is_running():
            simulation_app.update()

    simulation_app.close()
