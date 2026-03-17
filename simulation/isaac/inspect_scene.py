"""Quick scene inspector - prints structure of reactor.usd"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd
from pxr import UsdPhysics, UsdGeom, Sdf, PhysxSchema

SCENE = r"E:\FORTIS\Optimizer\isaac\scenes\reactor.usd"
print(f"\n{'='*60}")
print(f"Inspecting: {SCENE}")
print(f"{'='*60}")

omni.usd.get_context().open_stage(SCENE)
for _ in range(10):
    app.update()

stage = omni.usd.get_context().get_stage()
if stage is None:
    print("ERROR: stage is None")
    app.close()
    sys.exit(1)

print(f"\nUpAxis: {UsdGeom.GetStageUpAxis(stage)}")
print(f"MetersPerUnit: {UsdGeom.GetStageMetersPerUnit(stage)}")

print("\n--- Prim Hierarchy ---")
count = 0
for prim in stage.Traverse():
    count += 1
    t = prim.GetTypeName()
    apis = []
    if prim.HasAPI(UsdPhysics.CollisionAPI): apis.append("Collision")
    if prim.HasAPI(UsdPhysics.RigidBodyAPI): apis.append("RigidBody")
    if prim.HasAPI(UsdPhysics.MaterialAPI): apis.append("PhysMaterial")
    if prim.HasAPI(UsdPhysics.MeshCollisionAPI): apis.append("MeshCollision")
    try:
        if prim.IsA(UsdPhysics.Scene):
            scene = UsdPhysics.Scene(prim)
            apis.append(f"PhysScene(g={scene.GetGravityMagnitudeAttr().Get()})")
    except:
        pass

    s = f" [{', '.join(apis)}]" if apis else ""

    # Check for mesh data
    mesh_info = ""
    if t == "Mesh":
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        if pts:
            mesh_info = f" verts={len(pts)}"

    # Check xform scale
    scale_info = ""
    if prim.HasAttribute("xformOp:scale"):
        sc = prim.GetAttribute("xformOp:scale").Get()
        if sc:
            scale_info = f" scale={sc}"

    print(f"  {prim.GetPath()} ({t}){s}{mesh_info}{scale_info}")

print(f"\nTotal prims: {count}")
print(f"{'='*60}")

app.close()
