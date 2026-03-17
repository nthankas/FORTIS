"""
Phase 1a: Import reactor floor into Isaac Sim as static collider.

Run with:
    E:\\FORTIS\\IsaacSim\\python.bat E:\\FORTIS\\Optimizer\\isaac\\phase1a_setup_scene.py

Creates USD scene with:
- Reactor floor mesh as triangle-mesh static collider
- PhysX materials for graphite (mu=0.3 and mu=0.5) and rubber (mu=0.3 and mu=0.5)
- Physics scene with gravity (Z-up)
- Lighting
- Saves to: E:\\FORTIS\\Optimizer\\isaac\\scenes\\reactor_floor.usd

Also drops a test cube on the floor to verify collision works.
"""

import os
import sys
import struct
import numpy as np

# Add parent dir so we can import config
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from isaacsim import SimulationApp

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", default=True,
                    help="Run headless (no GUI window)")
parser.add_argument("--gui", action="store_true",
                    help="Run with GUI viewport for visual verification")
cli_args, _ = parser.parse_known_args()

simulation_app = SimulationApp({
    "headless": not cli_args.gui,
    "width": 1920,
    "height": 1080,
    "anti_aliasing": 0,
})

import omni.usd
from pxr import (
    Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf,
    PhysxSchema, UsdLux, Vt
)

from fortis_config import (
    INCHES_TO_METERS, STL_FLOOR_DECIMATED, STL_HALF_REACTOR,
    SCENE_DIR, SCENE_REACTOR_FLOOR,
    Z_INNER, Z_OUTER, STEP_HEIGHT,
    R_INNER_FLOOR_MIN, R_INNER_FLOOR_MAX, R_OUTER_FLOOR_MIN, R_OUTER_FLOOR_MAX,
    R_CENTER_POST, R_OUTER_WALL,
    GRAPHITE_MU_LOW, GRAPHITE_MU_HIGH,
    GRAPHITE_RESTITUTION, RUBBER_RESTITUTION,
)


# =============================================================================
# Binary STL Reader (no trimesh dependency needed in Isaac Sim Python)
# =============================================================================

def read_stl_binary(path):
    """
    Read binary STL file.
    Returns (vertices, faces) where vertices is Nx3 float64 and faces is Mx3 int.
    Vertices are in the original STL units (inches for our reactor mesh).
    """
    with open(path, 'rb') as f:
        raw = f.read()

    header = raw[:80]
    num_triangles = struct.unpack('<I', raw[80:84])[0]

    # Each triangle record: 12 bytes normal + 36 bytes verts + 2 bytes attr = 50 bytes
    expected_size = 84 + num_triangles * 50
    if len(raw) < expected_size:
        raise ValueError(
            f"STL file too small: expected {expected_size} bytes for "
            f"{num_triangles} triangles, got {len(raw)}"
        )

    dt = np.dtype([
        ('normal', '<f4', (3,)),
        ('v0', '<f4', (3,)),
        ('v1', '<f4', (3,)),
        ('v2', '<f4', (3,)),
        ('attr', '<u2')
    ])
    data = np.frombuffer(raw[84:84 + num_triangles * 50], dtype=dt)

    # Stack all vertices
    all_verts = np.vstack([data['v0'], data['v1'], data['v2']])  # (3N, 3)

    # Deduplicate vertices (round to avoid float precision mismatch)
    unique_verts, inverse = np.unique(
        np.round(all_verts, decimals=5), axis=0, return_inverse=True
    )
    faces = inverse.reshape(-1, 3)

    print(f"  Read {num_triangles} triangles from {os.path.basename(path)}")
    print(f"  Unique vertices: {len(unique_verts)}, Faces: {len(faces)}")
    bounds_min = unique_verts.min(axis=0)
    bounds_max = unique_verts.max(axis=0)
    print(f"  Bounds (inches): [{bounds_min[0]:.1f}, {bounds_min[1]:.1f}, {bounds_min[2]:.1f}]"
          f" to [{bounds_max[0]:.1f}, {bounds_max[1]:.1f}, {bounds_max[2]:.1f}]")

    return unique_verts.astype(np.float64), faces.astype(np.int32)


# =============================================================================
# Scene Builder
# =============================================================================

def create_reactor_scene():
    """Build the complete reactor floor scene."""

    stage = omni.usd.get_context().get_stage()

    # Stage metadata
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Root prim
    world_xform = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_xform.GetPrim())

    # ---- Physics Scene ----
    print("\nSetting up physics scene...")
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics_scene.CreateGravityMagnitudeAttr(9.81)

    physx_api = PhysxSchema.PhysxSceneAPI.Apply(
        stage.GetPrimAtPath("/World/PhysicsScene")
    )
    physx_api.CreateEnableCCDAttr(True)
    physx_api.CreateEnableStabilizationAttr(True)
    physx_api.CreateSolverTypeAttr("TGS")
    physx_api.CreateBroadphaseTypeAttr("GPU")
    physx_api.CreateGpuFoundLostPairsCapacityAttr(1024 * 256)
    physx_api.CreateGpuTotalAggregatePairsCapacityAttr(1024 * 256)

    # ---- Load Reactor Floor Mesh ----
    print("\nLoading reactor floor mesh...")
    stl_path = STL_FLOOR_DECIMATED
    if not os.path.exists(stl_path):
        print(f"ERROR: Floor mesh not found at {stl_path}")
        print(f"  Trying full reactor mesh at {STL_HALF_REACTOR}")
        stl_path = STL_HALF_REACTOR
        if not os.path.exists(stl_path):
            print("ERROR: No reactor mesh found. Cannot proceed.")
            return None

    vertices_in, faces = read_stl_binary(stl_path)
    vertices_m = vertices_in * INCHES_TO_METERS

    print(f"\n  Converted to meters:")
    print(f"  Bounds: [{vertices_m.min(0)[0]:.3f}, {vertices_m.min(0)[1]:.3f}, {vertices_m.min(0)[2]:.3f}]"
          f" to [{vertices_m.max(0)[0]:.3f}, {vertices_m.max(0)[1]:.3f}, {vertices_m.max(0)[2]:.3f}]")

    # Create reactor Xform
    UsdGeom.Xform.Define(stage, "/World/Reactor")

    # Create mesh prim
    mesh_path = "/World/Reactor/FloorMesh"
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)

    # Set geometry
    points = Vt.Vec3fArray(len(vertices_m))
    for i, v in enumerate(vertices_m):
        points[i] = Gf.Vec3f(float(v[0]), float(v[1]), float(v[2]))
    mesh.GetPointsAttr().Set(points)

    face_counts = Vt.IntArray(len(faces))
    for i in range(len(faces)):
        face_counts[i] = 3
    mesh.GetFaceVertexCountsAttr().Set(face_counts)

    face_indices = Vt.IntArray(len(faces) * 3)
    flat_faces = faces.flatten()
    for i in range(len(flat_faces)):
        face_indices[i] = int(flat_faces[i])
    mesh.GetFaceVertexIndicesAttr().Set(face_indices)

    mesh.GetSubdivisionSchemeAttr().Set("none")

    mesh_prim = stage.GetPrimAtPath(mesh_path)

    # ---- Visual Material (graphite look) ----
    print("  Creating visual material...")
    vis_mat = UsdShade.Material.Define(stage, "/World/Reactor/GraphiteLook")
    shader = UsdShade.Shader.Define(stage, "/World/Reactor/GraphiteLook/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.12, 0.12, 0.15)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.05)
    vis_mat.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    UsdShade.MaterialBindingAPI.Apply(mesh_prim).Bind(vis_mat)

    # ---- Static Collision (triangle mesh, NO convex hull) ----
    print("  Applying triangle mesh collision...")
    UsdPhysics.CollisionAPI.Apply(mesh_prim)
    mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
    mesh_coll.GetApproximationAttr().Set("none")  # Exact triangle mesh

    # ---- Physics Materials ----
    print("  Creating physics materials...")

    def make_physics_material(path, static_mu, dynamic_mu, restitution):
        mat = UsdShade.Material.Define(stage, path)
        phys = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        phys.CreateStaticFrictionAttr(static_mu)
        phys.CreateDynamicFrictionAttr(dynamic_mu)
        phys.CreateRestitutionAttr(restitution)
        return mat

    # Graphite floor materials
    mat_graphite_03 = make_physics_material(
        "/World/PhysicsMaterials/Graphite_mu03",
        GRAPHITE_MU_LOW, GRAPHITE_MU_LOW, GRAPHITE_RESTITUTION
    )
    mat_graphite_05 = make_physics_material(
        "/World/PhysicsMaterials/Graphite_mu05",
        GRAPHITE_MU_HIGH, GRAPHITE_MU_HIGH, GRAPHITE_RESTITUTION
    )

    # Rubber tire materials (for later use by chassis)
    make_physics_material(
        "/World/PhysicsMaterials/Rubber50A_mu03",
        GRAPHITE_MU_LOW, GRAPHITE_MU_LOW, RUBBER_RESTITUTION
    )
    make_physics_material(
        "/World/PhysicsMaterials/Rubber50A_mu05",
        GRAPHITE_MU_HIGH, GRAPHITE_MU_HIGH, RUBBER_RESTITUTION
    )

    # Bind conservative graphite to floor (default)
    phys_binding = UsdShade.MaterialBindingAPI(mesh_prim)
    phys_binding.Bind(mat_graphite_03, UsdShade.Tokens.weakerThanDescendants, "physics")

    # ---- Lighting ----
    print("  Setting up lighting...")
    dist_light = UsdLux.DistantLight.Define(stage, "/World/Lighting/DistantLight")
    dist_light.CreateIntensityAttr(500)
    dist_light.CreateAngleAttr(0.53)
    UsdGeom.Xformable(dist_light).AddRotateXYZOp().Set(Gf.Vec3f(-45, 30, 0))

    dome_light = UsdLux.DomeLight.Define(stage, "/World/Lighting/DomeLight")
    dome_light.CreateIntensityAttr(200)

    # ---- Test Cube (collision verification) ----
    # Drop a small cube onto the floor to verify collision works
    print("  Adding test cube for collision verification...")
    test_cube_path = "/World/TestCube"
    cube_size = 0.05  # 5cm = ~2 inches
    drop_r = (R_OUTER_FLOOR_MIN + R_OUTER_FLOOR_MAX) / 2  # Middle of outer floor
    drop_x = drop_r * INCHES_TO_METERS
    drop_z = Z_OUTER * INCHES_TO_METERS + 0.3  # 30cm above floor

    cube_mesh = UsdGeom.Cube.Define(stage, test_cube_path)
    cube_mesh.GetSizeAttr().Set(cube_size)
    xform = UsdGeom.Xformable(cube_mesh)
    xform.AddTranslateOp().Set(Gf.Vec3d(drop_x, 0, drop_z))

    cube_prim = stage.GetPrimAtPath(test_cube_path)

    # Make it a dynamic rigid body
    UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    UsdPhysics.CollisionAPI.Apply(cube_prim)
    mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
    mass_api.CreateMassAttr(0.5)  # 0.5 kg

    # Color it orange
    cube_vis = UsdShade.Material.Define(stage, "/World/TestCube/OrangeMat")
    cube_shader = UsdShade.Shader.Define(stage, "/World/TestCube/OrangeMat/Shader")
    cube_shader.CreateIdAttr("UsdPreviewSurface")
    cube_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(1.0, 0.5, 0.0)
    )
    cube_vis.CreateSurfaceOutput().ConnectToSource(
        cube_shader.ConnectableAPI(), "surface"
    )
    UsdShade.MaterialBindingAPI.Apply(cube_prim).Bind(cube_vis)

    # ---- Save Scene ----
    os.makedirs(SCENE_DIR, exist_ok=True)
    stage.Export(SCENE_REACTOR_FLOOR)
    print(f"\n  Scene saved: {SCENE_REACTOR_FLOOR}")

    # ---- Summary ----
    print("\n" + "=" * 60)
    print("Phase 1a Summary")
    print("=" * 60)
    print(f"  Mesh: {len(faces)} triangles, {len(vertices_m)} unique vertices")
    print(f"  Floor Z: inner={Z_INNER*INCHES_TO_METERS:.4f}m, outer={Z_OUTER*INCHES_TO_METERS:.4f}m")
    print(f"  Step height: {STEP_HEIGHT:.1f}\" = {STEP_HEIGHT*INCHES_TO_METERS*1000:.1f}mm")
    print(f"  Radial range: {R_CENTER_POST*INCHES_TO_METERS:.3f}m to {R_OUTER_WALL*INCHES_TO_METERS:.3f}m")
    print(f"  Collision: Exact triangle mesh (static)")
    print(f"  Materials: Graphite mu=0.3 (default), mu=0.5 (available)")
    print(f"  Rubber materials: mu=0.3 and mu=0.5 (for wheels)")
    print(f"  Test cube: dropping onto outer floor for collision check")
    print(f"  Scene USD: {SCENE_REACTOR_FLOOR}")
    print("=" * 60)

    return stage


# =============================================================================
# Main
# =============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("FORTIS Phase 1a: Reactor Floor Scene Setup (Isaac Sim)")
    print("=" * 60)

    stage = create_reactor_scene()

    if stage is None:
        print("Failed to create scene. Exiting.")
        simulation_app.close()
        sys.exit(1)

    if cli_args.gui:
        # Set camera to view the floor
        cam_r = 55.0 * INCHES_TO_METERS
        cam_z = Z_OUTER * INCHES_TO_METERS

        camera_path = "/World/Camera"
        camera = UsdGeom.Camera.Define(stage, camera_path)
        xform = UsdGeom.Xformable(camera)
        xform.AddTranslateOp().Set(Gf.Vec3d(
            cam_r + 1.0, 1.0, cam_z + 1.5
        ))

        print("\nIsaac Sim viewport is open.")
        print("The orange test cube should fall and land on the reactor floor.")
        print("If it passes through, collision setup needs debugging.")
        print("Press PLAY to start physics. Close window to exit.")

        while simulation_app.is_running():
            simulation_app.update()
    else:
        # Headless: just step a few frames to verify scene loads, then exit
        print("\nHeadless mode: stepping a few frames to verify...")
        for i in range(5):
            simulation_app.update()
        print("Scene verified. Exiting.")

    simulation_app.close()
    print("\nPhase 1a complete. To view the scene with GUI:")
    print(f'  E:\\FORTIS\\IsaacSim\\python.bat {os.path.abspath(__file__)} --gui')
