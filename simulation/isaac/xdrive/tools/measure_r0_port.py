"""
Extract R0 port geometry from reactor USD.
Measures the lip profile, opening dimensions, edge radius, and slope angles
needed to compute underbelly clearance during the entry pivot maneuver.
"""
import os, sys, math
import numpy as np
os.environ["PYTHONIOENCODING"] = "utf-8"

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
XDRIVE_ROOT = os.path.abspath(os.path.join(_TOOLS_DIR, ".."))
REACTOR_USD = os.path.join(XDRIVE_ROOT, "assets", "diiid_reactor.usd")

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 1280, "height": 720})

import omni.usd
from pxr import Usd, UsdGeom, Gf

IN = 0.0254

ctx = omni.usd.get_context()
ctx.open_stage(REACTOR_USD)
for _ in range(30):
    app.update()
stage = ctx.get_stage()

print("=" * 70, flush=True)
print("R0 Port Geometry Extraction", flush=True)
print("=" * 70, flush=True)

mpu = UsdGeom.GetStageMetersPerUnit(stage)
print(f"metersPerUnit: {mpu}", flush=True)

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

# Find R0 and port-related prims
print(f"\n--- Port-related prims ---", flush=True)
port_prims = []
for prim in stage.Traverse():
    name = prim.GetName()
    if any(kw in name for kw in ["Port", "R_0", "R0", "port"]):
        bb = cache.ComputeWorldBound(prim)
        rng = bb.ComputeAlignedRange()
        if not rng.IsEmpty():
            mn, mx = rng.GetMin(), rng.GetMax()
            sz = [mx[i]-mn[i] for i in range(3)]
            ct = [(mn[i]+mx[i])/2 for i in range(3)]
            print(f"  {prim.GetPath()} ({prim.GetTypeName()})", flush=True)
            print(f"    BBox: ({mn[0]/IN:.1f}\",{mn[1]/IN:.1f}\",{mn[2]/IN:.1f}\") to "
                  f"({mx[0]/IN:.1f}\",{mx[1]/IN:.1f}\",{mx[2]/IN:.1f}\")", flush=True)
            print(f"    Size: ({sz[0]/IN:.1f}\",{sz[1]/IN:.1f}\",{sz[2]/IN:.1f}\")", flush=True)
            print(f"    Center: ({ct[0]/IN:.1f}\",{ct[1]/IN:.1f}\",{ct[2]/IN:.1f}\")", flush=True)
            port_prims.append((prim, mn, mx, ct, sz))

# Find the R0 port specifically (it's the large port opening, usually the biggest "Port" or "R_0" prim)
# R0 is typically oriented radially, extending outward from the vessel
print(f"\n--- Identifying R0 port ---", flush=True)
r0_candidates = []
for prim, mn, mx, ct, sz in port_prims:
    name = prim.GetName()
    if "R_0" in name or "R0" in name or "Port_Assembly" in name:
        # R0 extends radially outward -- it should be the one furthest from center
        radial_dist = math.sqrt(ct[0]**2 + ct[1]**2)
        r0_candidates.append((prim, mn, mx, ct, sz, radial_dist))

r0_candidates.sort(key=lambda x: x[5], reverse=True)
if r0_candidates:
    r0_prim, r0_mn, r0_mx, r0_ct, r0_sz, r0_dist = r0_candidates[0]
    print(f"  R0 port: {r0_prim.GetPath()}", flush=True)
    print(f"    Radial distance from center: {r0_dist/IN:.1f}\"", flush=True)
    print(f"    Center: ({r0_ct[0]/IN:.1f}\", {r0_ct[1]/IN:.1f}\", {r0_ct[2]/IN:.1f}\")", flush=True)
    print(f"    Size: ({r0_sz[0]/IN:.1f}\" x {r0_sz[1]/IN:.1f}\" x {r0_sz[2]/IN:.1f}\")", flush=True)

    # R0 opening direction (radial outward from center)
    r0_angle = math.degrees(math.atan2(r0_ct[1], r0_ct[0]))
    print(f"    Azimuthal angle: {r0_angle:.1f} deg from +X", flush=True)

# Find the vessel outer wall near R0 to determine the lip edge
print(f"\n--- Outer wall and slopes near R0 ---", flush=True)
for prim, mn, mx, ct, sz in port_prims:
    # Look for prims near the R0 azimuthal position
    if r0_candidates:
        r0_angle_rad = math.atan2(r0_ct[1], r0_ct[0])
        prim_angle = math.atan2(ct[1], ct[0])
        angle_diff = abs(prim_angle - r0_angle_rad)
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        if angle_diff < math.radians(30):
            prim_r = math.sqrt(ct[0]**2 + ct[1]**2)
            print(f"  Near R0: {prim.GetName()} r={prim_r/IN:.1f}\" "
                  f"Z=({mn[2]/IN:.1f}\" to {mx[2]/IN:.1f}\")", flush=True)

# Measure the floor geometry to understand the slope from R0 down to the flat floor
print(f"\n--- Floor and slope geometry ---", flush=True)
for prim in stage.Traverse():
    name = prim.GetName()
    if any(kw in name for kw in ["LOWER_DIVERTOR", "Outer_UCSC", "CP_UCSC"]):
        bb = cache.ComputeWorldBound(prim)
        rng = bb.ComputeAlignedRange()
        if not rng.IsEmpty():
            mn, mx = rng.GetMin(), rng.GetMax()
            r_min = math.sqrt(mn[0]**2 + mn[1]**2)
            r_max = math.sqrt(mx[0]**2 + mx[1]**2)
            print(f"  {name}: Z=({mn[2]/IN:.1f}\" to {mx[2]/IN:.1f}\") "
                  f"r=({r_min/IN:.1f}\" to {r_max/IN:.1f}\")", flush=True)

# Try to get actual mesh vertex data from R0 port prims to measure the lip profile
print(f"\n--- R0 lip edge profile (mesh vertices) ---", flush=True)
if r0_candidates:
    r0_path = str(r0_prim.GetPath())
    for prim in Usd.PrimRange(r0_prim):
        if prim.GetTypeName() == "Mesh":
            mesh = UsdGeom.Mesh(prim)
            points = mesh.GetPointsAttr().Get()
            if points and len(points) > 0:
                pts = np.array([[p[0], p[1], p[2]] for p in points])
                # Transform to world space
                mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                world_pts = []
                for p in pts:
                    wp = mat.Transform(Gf.Vec3d(p[0], p[1], p[2]))
                    world_pts.append([wp[0], wp[1], wp[2]])
                world_pts = np.array(world_pts)

                print(f"  Mesh: {prim.GetPath()} ({len(world_pts)} vertices)", flush=True)
                print(f"    X: {world_pts[:,0].min()/IN:.1f}\" to {world_pts[:,0].max()/IN:.1f}\"", flush=True)
                print(f"    Y: {world_pts[:,1].min()/IN:.1f}\" to {world_pts[:,1].max()/IN:.1f}\"", flush=True)
                print(f"    Z: {world_pts[:,2].min()/IN:.1f}\" to {world_pts[:,2].max()/IN:.1f}\"", flush=True)

                # Find the inner edge of the port (where it meets the vessel interior)
                # This is the lip the robot drives over. It's at the smallest radial distance.
                radii = np.sqrt(world_pts[:,0]**2 + world_pts[:,1]**2)
                inner_mask = radii < np.percentile(radii, 20)
                inner_pts = world_pts[inner_mask]
                if len(inner_pts) > 0:
                    print(f"    Inner edge (lip):", flush=True)
                    print(f"      Z range: {inner_pts[:,2].min()/IN:.1f}\" to {inner_pts[:,2].max()/IN:.1f}\"", flush=True)
                    print(f"      R range: {radii[inner_mask].min()/IN:.1f}\" to {radii[inner_mask].max()/IN:.1f}\"", flush=True)
                    lip_bottom_z = inner_pts[:,2].min()
                    lip_top_z = inner_pts[:,2].max()
                    lip_height = lip_top_z - lip_bottom_z
                    print(f"      Lip height: {lip_height/IN:.1f}\"", flush=True)

# Overall reactor dimensions for reference
print(f"\n--- Reactor overall ---", flush=True)
reactor_bb = cache.ComputeWorldBound(stage.GetPrimAtPath("/World"))
rng = reactor_bb.ComputeAlignedRange()
if not rng.IsEmpty():
    mn, mx = rng.GetMin(), rng.GetMax()
    print(f"  Total height: {(mx[2]-mn[2])/IN:.1f}\"", flush=True)
    print(f"  Z range: {mn[2]/IN:.1f}\" to {mx[2]/IN:.1f}\"", flush=True)
    print(f"  Max radial: {max(abs(mn[0]), abs(mx[0]), abs(mn[1]), abs(mx[1]))/IN:.1f}\"", flush=True)

print(f"\nDone.", flush=True)
app.close()
