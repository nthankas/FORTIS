"""Quick bounds check - get world-space coordinates of reactor.usd mesh"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd
import numpy as np
from pxr import UsdPhysics, UsdGeom, Sdf, PhysxSchema, Gf, UsdShade

SCENE = r"E:\FORTIS\Optimizer\isaac\scenes\reactor.usd"
omni.usd.get_context().open_stage(SCENE)
for _ in range(10):
    app.update()

stage = omni.usd.get_context().get_stage()

# Check stage metadata
print(f"UpAxis: {UsdGeom.GetStageUpAxis(stage)}")
print(f"MetersPerUnit: {UsdGeom.GetStageMetersPerUnit(stage)}")

# Check mesh
mesh_prim = stage.GetPrimAtPath("/World/halfReactor/mesh")
mesh = UsdGeom.Mesh(mesh_prim)

# Raw vertex bounds (first 100 to be fast)
pts = mesh.GetPointsAttr().Get()
pts_np = np.array(pts)
print(f"\nRaw mesh vertex bounds:")
print(f"  X: [{pts_np[:,0].min():.3f}, {pts_np[:,0].max():.3f}]")
print(f"  Y: [{pts_np[:,1].min():.3f}, {pts_np[:,1].max():.3f}]")
print(f"  Z: [{pts_np[:,2].min():.3f}, {pts_np[:,2].max():.3f}]")

# Get scale from xform
scale_attr = mesh_prim.GetAttribute("xformOp:scale")
if scale_attr:
    scale = scale_attr.Get()
    print(f"\nMesh scale: {scale}")
    sx, sy, sz = float(scale[0]), float(scale[1]), float(scale[2])
    print(f"\nScaled mesh bounds (world coords):")
    print(f"  X: [{pts_np[:,0].min()*sx:.3f}, {pts_np[:,0].max()*sx:.3f}]")
    print(f"  Y: [{pts_np[:,1].min()*sy:.3f}, {pts_np[:,1].max()*sy:.3f}]")
    print(f"  Z: [{pts_np[:,2].min()*sz:.3f}, {pts_np[:,2].max()*sz:.3f}]")

# Also check parent xform
parent_prim = stage.GetPrimAtPath("/World/halfReactor")
if parent_prim.HasAttribute("xformOp:scale"):
    print(f"\nParent scale: {parent_prim.GetAttribute('xformOp:scale').Get()}")
if parent_prim.HasAttribute("xformOp:translate"):
    print(f"Parent translate: {parent_prim.GetAttribute('xformOp:translate').Get()}")

# Check the collision approximation
mesh_coll = UsdPhysics.MeshCollisionAPI(mesh_prim)
if mesh_coll:
    approx = mesh_coll.GetApproximationAttr().Get()
    print(f"\nCollision approximation: {approx}")

# Check physics material binding
mat_prim = stage.GetPrimAtPath("/World/PhysicsMaterial")
if mat_prim:
    phys_mat = UsdPhysics.MaterialAPI(mat_prim)
    print(f"\nPhysicsMaterial:")
    print(f"  staticFriction: {phys_mat.GetStaticFrictionAttr().Get()}")
    print(f"  dynamicFriction: {phys_mat.GetDynamicFrictionAttr().Get()}")
    print(f"  restitution: {phys_mat.GetRestitutionAttr().Get()}")

# Check for xformOpOrder
order_attr = mesh_prim.GetAttribute("xformOpOrder")
if order_attr:
    print(f"\nMesh xformOpOrder: {order_attr.Get()}")

parent_order = parent_prim.GetAttribute("xformOpOrder")
if parent_order:
    print(f"Parent xformOpOrder: {parent_order.Get()}")

# Check world xform
world_prim = stage.GetPrimAtPath("/World")
if world_prim.HasAttribute("xformOp:scale"):
    print(f"\nWorld scale: {world_prim.GetAttribute('xformOp:scale').Get()}")

# Compute effective floor Z values
# STL is in inches, we need to find the scale factor
print("\n--- Floor Analysis ---")
# Find Z values around the known floor Z ranges
inner_z_raw = -53.8  # inches in original STL
outer_z_raw = -49.3
if scale_attr:
    inner_z_world = inner_z_raw * sz
    outer_z_world = outer_z_raw * sz
    print(f"Expected inner floor Z (world): {inner_z_world:.3f}")
    print(f"Expected outer floor Z (world): {outer_z_world:.3f}")
    print(f"Step height (world): {(outer_z_world - inner_z_world):.3f}")

    # If we want the robot at the step, radial position ~ 55.5" from center
    step_r_raw = 55.5  # inches
    step_r_world = step_r_raw * sx
    print(f"Step radius (world): {step_r_world:.3f}")

app.close()
