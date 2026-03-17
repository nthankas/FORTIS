"""
Phase 2 Setup: Import FORTIS URDF into reactor scene and run physics.

Does NOT save a combined USD (sublayer references break on reopen).
Instead, this script is the entry point: loads reactor, imports URDF,
configures drives, runs physics to settle, then optionally stays in GUI.

Other scripts (manual_drive, test_rotation, etc.) call setup_and_settle()
or prepare_scene() to get a ready-to-use simulation.

Run standalone:
  E:\FORTIS\IsaacSim\python.bat E:\FORTIS\Optimizer\phase2\setup_phase2_scene.py --gui
"""
import os, sys, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

ES = 0.024
FLOOR_OFFSET = 0.3

def i2w(inches):
    return inches * ES

PHASE2_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(PHASE2_DIR, "fortis_robot.urdf")


def make_annular_ring(stage, path, z_in, r_inner_in, r_outer_in, mu=0.3, segments=72):
    from pxr import UsdGeom, UsdPhysics, UsdShade, Gf, Vt
    if stage.GetPrimAtPath(path).IsValid():
        return
    z = i2w(z_in)
    r_in = i2w(r_inner_in)
    r_out = i2w(r_outer_in)
    verts = []
    for i in range(segments + 1):
        angle = 2 * math.pi * i / segments
        c, s = math.cos(angle), math.sin(angle)
        verts.append(Gf.Vec3f(r_in * c, r_in * s, z))
        verts.append(Gf.Vec3f(r_out * c, r_out * s, z))
    faces_idx = []
    faces_cnt = []
    for i in range(segments):
        j = i * 2
        faces_idx.extend([j, j+1, j+3, j+2])
        faces_cnt.append(4)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(faces_idx))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(faces_cnt))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetVisibilityAttr().Set("invisible")
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).GetApproximationAttr().Set("none")
    mat_path = path + "_Mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(float(mu))
    pm.CreateDynamicFrictionAttr(float(mu))
    pm.CreateRestitutionAttr(0.05)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    print(f"  Floor ring: {path} Z={z_in}\" r=[{r_inner_in},{r_outer_in}]\"")


def prepare_scene(app):
    """
    Load reactor scene, import URDF robot, configure drives.
    Does NOT start physics or create Articulation.

    Returns dict with: stage, robot_prim_path, joint_paths,
    wheel_joint_names, arm_joint_names, art_root_path
    """
    import omni.usd
    import omni.kit.commands
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema, Vt

    from fortis_config import (
        SCENE_REACTOR,
        Z_INNER, Z_OUTER,
        R_INNER_FLOOR_MIN, R_INNER_FLOOR_MAX,
        R_OUTER_FLOOR_MIN, R_OUTER_FLOOR_MAX,
    )

    # -- Load reactor --
    print(f"  Loading reactor: {SCENE_REACTOR}")
    omni.usd.get_context().open_stage(SCENE_REACTOR)
    for _ in range(10):
        app.update()
    stage = omni.usd.get_context().get_stage()

    # -- Remove ghost /fortis prim if it exists (reactor.usd variant reference artifact) --
    fortis_ghost = stage.GetPrimAtPath("/fortis")
    if fortis_ghost.IsValid():
        stage.RemovePrim("/fortis")
        print("  Removed ghost /fortis prim from reactor.usd")

    # -- Smooth floors --
    make_annular_ring(stage, "/World/InnerFloor",
                      Z_INNER + FLOOR_OFFSET, R_INNER_FLOOR_MIN, R_INNER_FLOOR_MAX, 0.3)
    make_annular_ring(stage, "/World/OuterFloor",
                      Z_OUTER + FLOOR_OFFSET, R_OUTER_FLOOR_MIN, R_OUTER_FLOOR_MAX, 0.3)

    # -- Physics scene --
    ps = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(ps).IsValid():
        p = UsdPhysics.Scene.Define(stage, ps)
        p.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        p.CreateGravityMagnitudeAttr(9.81)
        px = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(ps))
        px.CreateEnableCCDAttr(True)
        px.CreateEnableStabilizationAttr(True)
        px.CreateSolverTypeAttr("TGS")
        print("  PhysicsScene added")

    # -- Import URDF --
    print(f"  Importing URDF: {URDF_PATH}")
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.distance_scale = 1.0
    import_config.create_physics_scene = False
    import_config.make_default_prim = False

    status, robot_prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=URDF_PATH,
        import_config=import_config,
        get_articulation_root=True,
    )
    print(f"  Robot prim: {robot_prim_path}")

    for _ in range(10):
        app.update()

    # -- Position robot --
    z_outer_floor = i2w(Z_OUTER + FLOOR_OFFSET)
    tire_r = 0.072
    half_height = 0.108
    robot_x = i2w(55.5)
    robot_z = z_outer_floor + tire_r + half_height + 0.05

    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    check_path = robot_prim_path
    while check_path and check_path != "/":
        p = stage.GetPrimAtPath(check_path)
        if p.IsValid() and p.GetTypeName() in ("Xform", ""):
            xf = UsdGeom.Xformable(p)
            if xf:
                xf.ClearXformOpOrder()
                xf.AddTranslateOp().Set(Gf.Vec3d(robot_x, 0.0, robot_z))
                print(f"  Positioned robot at ({robot_x:.4f}, 0, {robot_z:.4f}) on {check_path}")
                break
        check_path = str(Sdf.Path(check_path).GetParentPath())

    for _ in range(5):
        app.update()

    # -- Discover joints --
    wheel_joint_names = ["fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"]
    arm_joint_names = ["j1_yaw", "j2_shoulder", "j3_elbow", "j4_wrist"]
    joint_paths = {}

    for prim in stage.Traverse():
        name = prim.GetName()
        if name in wheel_joint_names or name in arm_joint_names:
            joint_paths[name] = str(prim.GetPath())

    print(f"  Found joints:")
    for n, p in joint_paths.items():
        print(f"    {n}: {p}")

    missing = [n for n in wheel_joint_names + arm_joint_names if n not in joint_paths]
    if missing:
        print(f"  WARNING: Missing joints: {missing}")

    # -- Configure wheel drives --
    for jname in wheel_joint_names:
        path = joint_paths.get(jname)
        if not path:
            continue
        prim = stage.GetPrimAtPath(path)
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetDampingAttr().Set(1000.0)
        drive.GetMaxForceAttr().Set(5.0)
        drive.GetTargetVelocityAttr().Set(0.0)
    print(f"  Wheel drives: stiffness=0, damping=1000, maxForce=5 Nm")

    # -- Configure arm drives --
    arm_drive_cfg = {
        "j1_yaw":      {"stiffness": 500.0, "damping": 100.0, "max_force": 20.0},
        "j2_shoulder":  {"stiffness": 500.0, "damping": 100.0, "max_force": 20.0},
        "j3_elbow":     {"stiffness": 300.0, "damping": 80.0,  "max_force": 15.0},
        "j4_wrist":     {"stiffness": 200.0, "damping": 50.0,  "max_force": 10.0},
    }
    for jname in arm_joint_names:
        path = joint_paths.get(jname)
        if not path:
            continue
        prim = stage.GetPrimAtPath(path)
        cfg = arm_drive_cfg[jname]
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetStiffnessAttr().Set(cfg["stiffness"])
        drive.GetDampingAttr().Set(cfg["damping"])
        drive.GetMaxForceAttr().Set(cfg["max_force"])
        drive.GetTargetPositionAttr().Set(0.0)
    print(f"  Arm drives configured")

    # -- Wheel friction material --
    wmp = "/World/WheelMat"
    if not stage.GetPrimAtPath(wmp).IsValid():
        wm = UsdShade.Material.Define(stage, wmp)
        wpm = UsdPhysics.MaterialAPI.Apply(wm.GetPrim())
        wpm.CreateStaticFrictionAttr(0.5)
        wpm.CreateDynamicFrictionAttr(0.5)
        wpm.CreateRestitutionAttr(0.2)
    wmat = UsdShade.Material(stage.GetPrimAtPath(wmp))
    for wname in ["fl_wheel_link", "fr_wheel_link", "rl_wheel_link", "rr_wheel_link"]:
        for prim in stage.Traverse():
            if prim.GetName() == wname:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    wmat, UsdShade.Tokens.weakerThanDescendants, "physics")
                break
    print(f"  Wheel friction: mu=0.5")

    for _ in range(5):
        app.update()

    # -- Find actual articulation root --
    art_root_path = None
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            ppath = str(prim.GetPath())
            # Only match robot prims, not reactor prims
            if "fortis" in ppath.lower() or "chassis" in ppath.lower():
                art_root_path = ppath
                print(f"  Found ArticulationRootAPI at: {art_root_path}")
                break

    if art_root_path is None:
        parent_path = str(Sdf.Path(robot_prim_path).GetParentPath())
        if parent_path != "/":
            art_root_path = parent_path
        else:
            art_root_path = robot_prim_path
        print(f"  No ArticulationRootAPI found, trying: {art_root_path}")

    return {
        "stage": stage,
        "robot_prim_path": robot_prim_path,
        "art_root_path": art_root_path,
        "joint_paths": joint_paths,
        "wheel_joint_names": wheel_joint_names,
        "arm_joint_names": arm_joint_names,
    }


def start_physics_and_settle(app, scene_info, headless=True, settle_seconds=2.0):
    """
    Start physics, create Articulation, and settle.
    Call after prepare_scene() and any drive parameter adjustments.
    """
    import omni.timeline
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation

    stage = scene_info["stage"]
    robot_prim_path = scene_info["robot_prim_path"]
    art_root_path = scene_info["art_root_path"]
    wheel_joint_names = scene_info["wheel_joint_names"]
    arm_joint_names = scene_info["arm_joint_names"]

    world = World(stage_units_in_meters=1.0)
    omni.timeline.get_timeline_interface().play()

    for _ in range(10):
        app.update()

    # Try articulation at multiple paths
    art = None
    final_path = None
    candidates = []
    if art_root_path:
        candidates.append(art_root_path)
    candidates.append(robot_prim_path)
    parent_path = str(Sdf.Path(robot_prim_path).GetParentPath())
    if parent_path != "/" and parent_path not in candidates:
        candidates.append(parent_path)

    for try_path in candidates:
        prim = stage.GetPrimAtPath(try_path)
        if not prim.IsValid():
            continue
        print(f"  Trying Articulation at: {try_path}")
        try:
            candidate = Articulation(try_path)
            candidate.initialize()
            if candidate.is_physics_handle_valid():
                art = candidate
                final_path = try_path
                print(f"  OK: Articulation valid at {try_path}")
                break
            else:
                print(f"  Not valid at {try_path}")
        except Exception as e:
            print(f"  Failed at {try_path}: {e}")

    if art is None:
        print(f"  FATAL: No valid articulation found")
        return None

    dof_names = art.dof_names
    print(f"  Articulation: {final_path}")
    print(f"  DOFs ({art.num_dof}): {dof_names}")

    wheel_dof_indices = []
    for wn in wheel_joint_names:
        if wn in dof_names:
            wheel_dof_indices.append(dof_names.index(wn))
    arm_dof_indices = []
    for an in arm_joint_names:
        if an in dof_names:
            arm_dof_indices.append(dof_names.index(an))

    print(f"  Wheel DOF indices: {wheel_dof_indices}")
    print(f"  Arm DOF indices: {arm_dof_indices}")

    # -- Settle --
    render = not headless
    settle_steps = int(settle_seconds / (1.0/60.0))
    print(f"  Settling {settle_seconds}s ({settle_steps} steps)...")
    for _ in range(settle_steps):
        world.step(render=render)

    chassis_prim = stage.GetPrimAtPath(final_path)
    tf = UsdGeom.Xformable(chassis_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = tf.ExtractTranslation()
    r = math.sqrt(pos[0]**2 + pos[1]**2) / ES
    print(f"  Settled at: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}), r={r:.1f}\"")

    return {
        "stage": stage,
        "world": world,
        "art": art,
        "robot_prim_path": final_path,
        "dof_names": dof_names,
        "wheel_joint_names": wheel_joint_names,
        "arm_joint_names": arm_joint_names,
        "wheel_dof_indices": wheel_dof_indices,
        "arm_dof_indices": arm_dof_indices,
        "joint_paths": scene_info["joint_paths"],
    }


def setup_and_settle(app, headless=True, settle_seconds=2.0):
    """
    Full setup: load reactor, import URDF, configure drives, settle physics.
    Convenience wrapper that calls prepare_scene() + start_physics_and_settle().
    """
    scene_info = prepare_scene(app)
    return start_physics_and_settle(app, scene_info, headless=headless,
                                     settle_seconds=settle_seconds)


# -- Standalone mode --
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true")
    cli, _ = parser.parse_known_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not cli.gui, "width": 1920, "height": 1080})

    print("=" * 60)
    print("FORTIS Phase 2: Scene Setup (URDF Import)")
    print("=" * 60)

    result = setup_and_settle(app, headless=not cli.gui, settle_seconds=3.0)

    if result is None:
        print("Setup failed!")
        app.close()
        sys.exit(1)

    print(f"\n{'='*60}")
    print("Setup complete. Robot settled in reactor.")
    print(f"{'='*60}")

    if cli.gui:
        print("\nGUI mode. Robot is live with physics running.")
        print("Observe the chassis straddling the step.")
        print("Close the window to exit.")
        while app.is_running():
            result["world"].step(render=True)

    import omni.timeline
    omni.timeline.get_timeline_interface().stop()
    app.close()
