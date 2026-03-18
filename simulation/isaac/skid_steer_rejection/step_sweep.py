#!/usr/bin/env python3
"""
step_sweep.py -- Parametric sweep for 90-degree straddle turn.

The robot drops straddling the 4.5" step (front/rear axle on different floors).
It must rotate 90 degrees to straddle the step tangentially (left/right sides).

Swept parameters:
  - Wheel radius (3.0" to 4.5")
  - Wheel width (2.0" to 4.0")
  - Wheelbase (9" to 14" -- front-rear axle spacing)
  - Track width (8" to 13" -- left-right wheel center spacing)
  - Motor torque (30 to 100 Nm)

The step edge has a realistic chamfer (1" x 1" 45-degree ramp at the top
of the inner-facing wall). This reduces the effective climb height.

Configs are pre-sorted by geometric "crossing angle" (how far the robot
can rotate before any wheel reaches the step). Higher = better.

Results saved to phase2/results/step_sweep.json

Run: E:\\FORTIS\\IsaacSim\\python.bat step_sweep.py [--gui] [--quick]
"""

import sys
import os
import math
import json
import time
import tempfile
import itertools

os.environ["PYTHONUNBUFFERED"] = "1"

import builtins
_orig_print = builtins.print
def print(*a, **kw):
    kw["flush"] = True
    _orig_print(*a, **kw)
builtins.print = print

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--quick", action="store_true", help="Reduced sweep for quick validation")
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": not args.gui})

import omni.usd
import omni.timeline
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
ES = 0.024
def i2w(inches):
    return inches * ES

Z_INNER = -53.8
Z_OUTER = -49.3
FLOOR_OFFSET = 0.3
STEP_R = 55.5          # radial position of step center
STEP_HEIGHT = Z_OUTER - Z_INNER  # 4.5 inches
R_INNER_MIN = 45.8     # inner floor inner edge
R_INNER_MAX = 54.8     # inner floor outer edge
R_OUTER_MIN = 56.2     # outer floor inner edge
R_OUTER_MAX = 70.3     # outer floor outer edge

TUNNEL_SIZE = 14.75    # usable tunnel dimension in inches
CHASSIS_HEIGHT = 5.0   # inches (fixed)
CHAMFER_SIZE = 1.0     # inches -- 45-degree ramp at step top edge

REACTOR_USD = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "xdrive", "diiid_reactor.usd"))
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

# ---------------------------------------------------------------------------
# Sweep parameter space
# ---------------------------------------------------------------------------
if args.quick:
    WHEEL_RADII   = [3.5, 4.0, 4.5]
    WHEEL_WIDTHS  = [2.0, 3.0]
    WHEELBASES    = [9, 10, 12]      # front-rear axle distance, inches
    TRACK_WIDTHS  = [8, 10, 12]      # left-right wheel center distance, inches
    MOTOR_TORQUES = [50, 100]        # Nm per wheel
    DRIVE_VELS    = [15]             # rad/s
    CHASSIS_MASSES = [40]            # lbs
    print("QUICK MODE: reduced parameter space")
else:
    WHEEL_RADII   = [3.0, 3.5, 4.0, 4.5]
    WHEEL_WIDTHS  = [2.0, 2.5, 3.0, 3.5, 4.0]
    WHEELBASES    = [8, 9, 10, 11, 12, 13, 14]
    TRACK_WIDTHS  = [7, 8, 9, 10, 11, 12, 13]
    MOTOR_TORQUES = [30, 50, 100]
    DRIVE_VELS    = [10, 15, 20]
    CHASSIS_MASSES = [35, 50]

TEST_DURATION = 15.0   # seconds per trial (longer to give time)
SETTLE_TIME = 1.5      # seconds to settle

# ---------------------------------------------------------------------------
# Geometric pre-filter: compute crossing angle analytically
# ---------------------------------------------------------------------------
def compute_crossing_angle(wx_in, wy_in):
    """
    Compute the rotation angle at which the FIRST wheel reaches the step circle.

    The robot center is at r=STEP_R on the step circle. Wheels trace circles
    of radius d = sqrt(wx^2 + wy^2) around the center. The step is a circle
    at r=STEP_R. First wheel to cross (RR at (-wx, -wy) for CCW rotation):

    -wx*cos(θ) + wy*sin(θ) = -(wx² + wy²) / (2*STEP_R)

    Returns the crossing angle in degrees for the RR wheel.
    """
    d2 = wx_in**2 + wy_in**2
    rhs = -d2 / (2 * STEP_R)

    # Solve: -wx*cos(θ) + wy*sin(θ) = rhs
    # Rewrite: R*sin(θ - φ) = rhs, where R = sqrt(d2), φ = arctan(wx/wy)
    R = math.sqrt(d2)
    phi = math.atan2(wx_in, wy_in)  # arctan(wx/wy)
    sin_val = rhs / R
    if abs(sin_val) > 1:
        return 0  # no crossing (impossible config)
    theta = math.degrees(phi + math.asin(sin_val))
    # We want the positive angle solution
    if theta < 0:
        theta += 360
    return theta


def check_wheel_floor_fit(wheel_radius_in, wheelbase_in):
    """
    Check if wheels fit on their respective floors.
    Returns (fits, rear_r, front_r, reason).
    """
    wx = wheelbase_in / 2
    rear_r = STEP_R - wx   # rear axle radial position (inner floor)
    front_r = STEP_R + wx  # front axle radial position (outer floor)

    # Rear wheel on inner floor
    if rear_r - wheel_radius_in < R_INNER_MIN:
        return False, rear_r, front_r, f"rear inner edge {rear_r - wheel_radius_in:.1f}\" < {R_INNER_MIN}\""
    if rear_r + wheel_radius_in > R_INNER_MAX + 0.5:  # allow 0.5" overhang toward step
        return False, rear_r, front_r, f"rear outer edge {rear_r + wheel_radius_in:.1f}\" > step"

    # Front wheel on outer floor
    if front_r - wheel_radius_in < R_OUTER_MIN - 0.5:  # allow 0.5" overhang
        return False, rear_r, front_r, f"front inner edge too close to step"
    if front_r + wheel_radius_in > R_OUTER_MAX:
        return False, rear_r, front_r, f"front outer edge {front_r + wheel_radius_in:.1f}\" > wall"

    return True, rear_r, front_r, "OK"


# ---------------------------------------------------------------------------
# Generate and filter configs
# ---------------------------------------------------------------------------
all_configs = []
skipped_reasons = {}

for wr_in, ww_in, wb_in, tw_in, torque, dvel, mass_lbs in itertools.product(
    WHEEL_RADII, WHEEL_WIDTHS, WHEELBASES, TRACK_WIDTHS,
    MOTOR_TORQUES, DRIVE_VELS, CHASSIS_MASSES
):
    # Tunnel height constraint
    stowed_h = CHASSIS_HEIGHT + wr_in + 2.0
    if stowed_h > TUNNEL_SIZE:
        skipped_reasons["height"] = skipped_reasons.get("height", 0) + 1
        continue

    # Chassis width = space between wheel inner faces
    chassis_w = tw_in - ww_in - 0.5
    if chassis_w < 5.0:  # minimum for electronics
        skipped_reasons["chassis_narrow"] = skipped_reasons.get("chassis_narrow", 0) + 1
        continue

    # Check wheel fits on floors
    fits, rear_r, front_r, reason = check_wheel_floor_fit(wr_in, wb_in)
    if not fits:
        skipped_reasons[reason[:20]] = skipped_reasons.get(reason[:20], 0) + 1
        continue

    # Compute crossing angle
    wx = wb_in / 2
    wy = tw_in / 2  # wheel center offset (track_width/2, not track/2 + wheel_width/2)
    cross_angle = compute_crossing_angle(wx, wy)

    # Skip configs with very low crossing angle (won't work)
    if cross_angle < 25:
        skipped_reasons["low_cross_angle"] = skipped_reasons.get("low_cross_angle", 0) + 1
        continue

    # Can the wheel potentially climb the chamfered step?
    effective_step = STEP_HEIGHT - CHAMFER_SIZE  # 3.5" with 1" chamfer
    can_climb = wr_in >= effective_step * 0.9  # allow 10% margin

    all_configs.append({
        "wheel_radius_in": wr_in,
        "wheel_width_in": ww_in,
        "wheelbase_in": wb_in,
        "track_width_in": tw_in,
        "motor_torque_nm": torque,
        "drive_vel": dvel,
        "chassis_mass_lbs": mass_lbs,
        "chassis_width_in": round(chassis_w, 1),
        "crossing_angle_deg": round(cross_angle, 1),
        "can_climb_step": can_climb,
        "rear_r": round(rear_r, 1),
        "front_r": round(front_r, 1),
    })

# Sort by crossing angle (highest first) -- test most promising configs first
all_configs.sort(key=lambda c: (c["can_climb_step"], c["crossing_angle_deg"]), reverse=True)

print(f"\n{'='*70}")
print(f"Step Straddle Turn Parametric Sweep")
print(f"{'='*70}")
print(f"Total configs after filtering: {len(all_configs)}")
print(f"Step height: {STEP_HEIGHT}\"  Chamfer: {CHAMFER_SIZE}\"  Effective: {STEP_HEIGHT - CHAMFER_SIZE}\"")
print(f"Tunnel limit: {TUNNEL_SIZE}\"")
print(f"Skipped reasons: {skipped_reasons}")

# Show top 10 by crossing angle
print(f"\nTop 10 configs by crossing angle:")
for c in all_configs[:10]:
    print(f"  R={c['wheel_radius_in']}\" W={c['wheel_width_in']}\" "
          f"WB={c['wheelbase_in']}\" TW={c['track_width_in']}\" "
          f"cross={c['crossing_angle_deg']}° climb={c['can_climb_step']}")
print(f"{'='*70}\n")

# Limit number of configs to test
MAX_TESTS = 80 if args.quick else 200
configs_to_test = all_configs[:MAX_TESTS]
print(f"Testing top {len(configs_to_test)} configs (sorted by crossing angle)\n")

# ---------------------------------------------------------------------------
# Load reactor scene once
# ---------------------------------------------------------------------------
print(f"Reactor USD: {REACTOR_USD}")
omni.usd.get_context().open_stage(REACTOR_USD)
for _ in range(10):
    app.update()

stage = omni.usd.get_context().get_stage()

# Remove any stale prims
for stale_path in ["/fortis", "/fortis_robot", "/Robot", "/StepWall"]:
    p = stage.GetPrimAtPath(stale_path)
    if p and p.IsValid():
        stage.RemovePrim(stale_path)

for _ in range(5):
    app.update()

# ---------------------------------------------------------------------------
# Add step wall with chamfer
# ---------------------------------------------------------------------------
def add_step_wall_chamfered(stage, chamfer_size_in=CHAMFER_SIZE):
    """
    Add step wall with a chamfered top edge.
    Main wall: (STEP_HEIGHT - chamfer) tall, vertical
    Chamfer ramp: chamfer_size at 45 degrees on the inner side
    """
    step_prim = UsdGeom.Xform.Define(stage, "/StepWall")

    wall_thickness = i2w(1.4)
    main_wall_height = i2w(STEP_HEIGHT - chamfer_size_in)
    chamfer_h = i2w(chamfer_size_in)
    wall_r = i2w((R_INNER_MAX + R_OUTER_MIN) / 2)

    # Main wall: shorter, bottom at inner floor level
    wall_z = i2w(Z_INNER + FLOOR_OFFSET) + main_wall_height / 2

    # Chamfer ramp extends inward from wall top
    ramp_length = chamfer_h * math.sqrt(2)  # 45-degree ramp
    ramp_z = i2w(Z_INNER + FLOOR_OFFSET) + main_wall_height + chamfer_h / 2
    ramp_r_offset = chamfer_h / 2  # shift inward

    n_segments = 36
    for i in range(n_segments):
        angle = 2 * math.pi * i / n_segments
        seg_width = 2 * math.pi * wall_r / n_segments * 1.1

        # --- Main wall segment ---
        seg_path = f"/StepWall/Wall_{i:02d}"
        seg = UsdGeom.Cube.Define(stage, seg_path)
        seg.GetSizeAttr().Set(1.0)
        UsdGeom.XformCommonAPI(seg).SetScale(Gf.Vec3f(
            wall_thickness, seg_width, main_wall_height
        ))
        cx = wall_r * math.cos(angle)
        cy = wall_r * math.sin(angle)
        UsdGeom.XformCommonAPI(seg).SetTranslate(Gf.Vec3d(cx, cy, wall_z))
        rot_deg = math.degrees(angle)
        UsdGeom.XformCommonAPI(seg).SetRotate(Gf.Vec3f(0, 0, rot_deg))
        UsdPhysics.CollisionAPI.Apply(seg.GetPrim())
        seg.GetPurposeAttr().Set("guide")

        # --- Chamfer ramp segment ---
        # A thin box rotated 45 degrees, at the top-inner edge of the wall
        ramp_path = f"/StepWall/Ramp_{i:02d}"
        ramp = UsdGeom.Cube.Define(stage, ramp_path)
        ramp.GetSizeAttr().Set(1.0)
        # Ramp box: length along slope x width x thin
        ramp_thick = i2w(0.3)  # thin collision surface
        UsdGeom.XformCommonAPI(ramp).SetScale(Gf.Vec3f(
            ramp_length, seg_width, ramp_thick
        ))
        # Position: shifted inward from wall center by ramp_r_offset
        # and up to the top of the main wall
        ramp_cx = (wall_r - ramp_r_offset) * math.cos(angle)
        ramp_cy = (wall_r - ramp_r_offset) * math.sin(angle)
        UsdGeom.XformCommonAPI(ramp).SetTranslate(Gf.Vec3d(ramp_cx, ramp_cy, ramp_z))
        # Rotate: face center (Z rotation) + tilt 45 degrees (Y rotation for ramp)
        # The ramp tilts inward-upward: rotate around the tangential axis
        # After Z rotation to face center, the local X axis points radially inward
        # We want to tilt around the local Y axis (tangential) by -45 degrees
        UsdGeom.XformCommonAPI(ramp).SetRotate(Gf.Vec3f(0, -45, rot_deg))
        UsdPhysics.CollisionAPI.Apply(ramp.GetPrim())
        ramp.GetPurposeAttr().Set("guide")

    # Graphite material
    mat_path = "/StepWall/GraphiteMat"
    mat = UsdShade.Material.Define(stage, mat_path)
    phys_mat = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    phys_mat.GetStaticFrictionAttr().Set(0.3)
    phys_mat.GetDynamicFrictionAttr().Set(0.3)
    phys_mat.GetRestitutionAttr().Set(0.05)

    for i in range(n_segments):
        for prefix in ["Wall", "Ramp"]:
            seg_prim = stage.GetPrimAtPath(f"/StepWall/{prefix}_{i:02d}")
            if seg_prim and seg_prim.IsValid():
                UsdShade.MaterialBindingAPI.Apply(seg_prim).Bind(
                    mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    print(f"Added chamfered step wall: {n_segments} segments at r={wall_r/ES:.1f}\", "
          f"main_height={main_wall_height/ES:.1f}\", chamfer={chamfer_size_in}\"")

add_step_wall_chamfered(stage)
for _ in range(5):
    app.update()

# ---------------------------------------------------------------------------
# Robot builder (parametric)
# ---------------------------------------------------------------------------
def build_parametric_robot(output_path, wheel_radius_in, wheel_width_in,
                            motor_torque, chassis_mass_lbs,
                            wheelbase_in=12, track_width_in=10):
    """Build a robot USD with given parameters."""

    chassis_mass_kg = chassis_mass_lbs * 0.4536
    wheel_mass_kg = 1.13

    chassis_width_in = track_width_in - wheel_width_in - 0.5
    chassis_length_in = max(wheelbase_in + 1.0, 10.0)

    # Remove old layer cache
    old_layer = Sdf.Layer.Find(output_path)
    if old_layer:
        old_layer.Clear()
    if os.path.exists(output_path):
        os.remove(output_path)

    s = Usd.Stage.CreateInMemory()
    s.SetMetadata("metersPerUnit", 1.0)
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    s.SetDefaultPrim(s.DefinePrim("/Robot"))

    def box_inertia(mass, lx, ly, lz):
        return (mass/12*(ly**2+lz**2), mass/12*(lx**2+lz**2), mass/12*(lx**2+ly**2))

    def cyl_inertia(mass, r, h):
        Iyy = 0.5*mass*r**2
        Ixx = mass/12*(3*r**2+h**2)
        return (Ixx, Iyy, Ixx)

    def make_box(path, hx, hy, hz, mass, color, collision=True):
        prim = UsdGeom.Xform.Define(s, path)
        UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
        mapi = UsdPhysics.MassAPI.Apply(prim.GetPrim())
        mapi.GetMassAttr().Set(mass)
        ix, iy, iz = box_inertia(mass, hx*2, hy*2, hz*2)
        mapi.GetDiagonalInertiaAttr().Set(Gf.Vec3f(ix, iy, iz))
        if collision:
            c = UsdGeom.Cube.Define(s, path+"/Col")
            c.GetSizeAttr().Set(1.0)
            c.GetPurposeAttr().Set("guide")
            UsdGeom.XformCommonAPI(c).SetScale(Gf.Vec3f(hx*2, hy*2, hz*2))
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        v = UsdGeom.Cube.Define(s, path+"/Vis")
        v.GetSizeAttr().Set(1.0)
        UsdGeom.XformCommonAPI(v).SetScale(Gf.Vec3f(hx*2, hy*2, hz*2))
        v.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        return prim

    def make_cyl(path, r, hh, mass, axis, color, collision=True):
        prim = UsdGeom.Xform.Define(s, path)
        UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
        mapi = UsdPhysics.MassAPI.Apply(prim.GetPrim())
        mapi.GetMassAttr().Set(mass)
        ix, iy, iz = cyl_inertia(mass, r, hh*2)
        mapi.GetDiagonalInertiaAttr().Set(Gf.Vec3f(ix, iy, iz))
        if collision:
            c = UsdGeom.Cylinder.Define(s, path+"/Col")
            c.GetRadiusAttr().Set(r)
            c.GetHeightAttr().Set(hh*2)
            c.GetAxisAttr().Set(axis)
            c.GetPurposeAttr().Set("guide")
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        v = UsdGeom.Cylinder.Define(s, path+"/Vis")
        v.GetRadiusAttr().Set(r)
        v.GetHeightAttr().Set(hh*2)
        v.GetAxisAttr().Set(axis)
        v.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        return prim

    def make_joint(path, parent, child, axis, anchor, stiffness=0, damping=1000,
                   max_force=5, lo=None, hi=None, target=None):
        j = UsdPhysics.RevoluteJoint.Define(s, path)
        j.GetAxisAttr().Set(axis)
        j.GetBody0Rel().SetTargets([parent])
        j.GetBody1Rel().SetTargets([child])
        j.GetLocalPos0Attr().Set(Gf.Vec3f(*anchor))
        j.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
        j.GetLocalRot0Attr().Set(Gf.Quatf(1,0,0,0))
        j.GetLocalRot1Attr().Set(Gf.Quatf(1,0,0,0))
        if lo is not None: j.GetLowerLimitAttr().Set(lo)
        if hi is not None: j.GetUpperLimitAttr().Set(hi)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.GetStiffnessAttr().Set(stiffness)
        d.GetDampingAttr().Set(damping)
        d.GetMaxForceAttr().Set(max_force)
        d.GetTypeAttr().Set("force")
        if target is not None:
            d.GetTargetPositionAttr().Set(target)
        PhysxSchema.PhysxJointAPI.Apply(j.GetPrim())

    # Root
    robot = UsdGeom.Xform.Define(s, "/Robot")
    UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
    UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(0,0,0))

    # Chassis
    ch_hx = i2w(chassis_length_in) / 2
    ch_hy = i2w(chassis_width_in) / 2
    ch_hz = i2w(CHASSIS_HEIGHT) / 2
    make_box("/Robot/Chassis", ch_hx, ch_hy, ch_hz, chassis_mass_kg, (0.2,0.35,0.6))

    # Wheels -- positions based on wheelbase and track
    wr = i2w(wheel_radius_in)
    wh = i2w(wheel_width_in) / 2
    wz = -ch_hz  # axle at bottom of chassis
    wx = i2w(wheelbase_in) / 2   # half wheelbase
    wy = i2w(track_width_in) / 2  # half track width

    wheels = [
        ("FL", +wx, +wy, wz),
        ("FR", +wx, -wy, wz),
        ("RL", -wx, +wy, wz),
        ("RR", -wx, -wy, wz),
    ]

    for name, x, y, z in wheels:
        w = make_cyl(f"/Robot/{name}", wr, wh, wheel_mass_kg, "Y", (0.15,0.15,0.15))
        UsdGeom.XformCommonAPI(w).SetTranslate(Gf.Vec3d(x, y, z))

    # Arm base (minimal, collision off, just for mass/CG)
    ab_r = i2w(1.5)
    ab_hh = i2w(1.0)
    arm_z = ch_hz + ab_hh
    make_cyl("/Robot/ArmBase", ab_r, ab_hh, 1.0, "Z", (0.6,0.3,0.1), collision=False)
    arm_base_prim = s.GetPrimAtPath("/Robot/ArmBase")
    UsdGeom.XformCommonAPI(UsdGeom.Xform(arm_base_prim)).SetTranslate(Gf.Vec3d(0,0,arm_z))

    # Joints
    UsdGeom.Scope.Define(s, "/Robot/Joints")
    for name, x, y, z in wheels:
        jname = name[0:2] + "_Drive"
        make_joint(f"/Robot/Joints/{jname}", "/Robot/Chassis", f"/Robot/{name}",
                   "Y", (x, y, z), stiffness=0, damping=1000, max_force=motor_torque)

    # Arm yaw joint
    make_joint("/Robot/Joints/J1_Yaw", "/Robot/Chassis", "/Robot/ArmBase",
               "Z", (0, 0, ch_hz), stiffness=500, damping=100, max_force=20,
               lo=-180, hi=180, target=0.0)

    # Rubber material for wheels
    rmat = UsdShade.Material.Define(s, "/Robot/RubberMat")
    rp = UsdPhysics.MaterialAPI.Apply(rmat.GetPrim())
    rp.GetStaticFrictionAttr().Set(0.8)
    rp.GetDynamicFrictionAttr().Set(0.6)
    rp.GetRestitutionAttr().Set(0.2)

    for name, _, _, _ in wheels:
        col = s.GetPrimAtPath(f"/Robot/{name}/Col")
        if col and col.IsValid():
            UsdShade.MaterialBindingAPI.Apply(col).Bind(
                rmat, UsdShade.Tokens.weakerThanDescendants, "physics")

    s.GetRootLayer().Export(output_path)
    return True, "OK"


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------
from isaacsim.core.prims import XFormPrim, Articulation

timeline = omni.timeline.get_timeline_interface()

# Use alternating USD files to avoid layer cache conflicts
tmp_usd_a = os.path.join(tempfile.gettempdir(), "fortis_sweep_robot_A.usd")
tmp_usd_b = os.path.join(tempfile.gettempdir(), "fortis_sweep_robot_B.usd")

def run_straddle_turn_test(robot_usd_path, wheel_radius_in, drive_vel=15.0):
    """
    Test 90-degree point turn while straddling the step.
    Returns dict with turn performance metrics.
    """
    # Remove old robot
    old = stage.GetPrimAtPath("/Robot")
    if old and old.IsValid():
        stage.RemovePrim("/Robot")
    for _ in range(5):
        app.update()

    # Add new robot
    robot_prim = stage.DefinePrim("/Robot", "Xform")
    robot_prim.GetReferences().AddReference(robot_usd_path)
    for _ in range(5):
        app.update()

    # Position robot straddling the step
    z_floor = i2w(Z_OUTER + FLOOR_OFFSET)
    wr = i2w(wheel_radius_in)
    ch_hz = i2w(CHASSIS_HEIGHT) / 2
    robot_z = z_floor + wr + ch_hz + 0.02
    robot_x = i2w(STEP_R)

    xform = UsdGeom.Xformable(robot_prim)
    xform.ClearXformOpOrder()
    t_op = xform.AddTranslateOp()
    t_op.Set(Gf.Vec3d(robot_x, 0, robot_z))

    for _ in range(5):
        app.update()

    # Start physics
    timeline.play()
    for _ in range(10):
        app.update()

    try:
        robot_art = Articulation("/Robot")
        robot_art.initialize()
        for _ in range(3):
            app.update()

        dof_names = robot_art.dof_names
        n_dofs = robot_art.num_dof

        # Find wheel indices
        widx = {}
        for i, name in enumerate(dof_names):
            if "FL" in name and "Drive" in name: widx["FL"] = i
            elif "FR" in name and "Drive" in name: widx["FR"] = i
            elif "RL" in name and "Drive" in name: widx["RL"] = i
            elif "RR" in name and "Drive" in name: widx["RR"] = i

        if len(widx) < 4:
            raise RuntimeError(f"Only found {len(widx)} wheels: {widx}")

        # Switch wheels to velocity control
        wji = list(widx.values())
        robot_art.switch_control_mode("velocity", joint_indices=np.array(wji))

        chassis_xfp = XFormPrim("/Robot/Chassis")

        # Settle
        for _ in range(int(SETTLE_TIME * 60)):
            app.update()

        # Get settled pose
        p0, r0 = chassis_xfp.get_world_poses()
        settled_pos = p0[0].copy()
        settled_rot = r0[0].copy()

        w, x, y, z = float(settled_rot[0]), float(settled_rot[1]), float(settled_rot[2]), float(settled_rot[3])
        yaw0 = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
        settled_tilt = math.degrees(math.acos(min(1.0, max(-1.0, 1 - 2*(x*x + y*y)))))

        # Check if robot fell during settle
        if float(settled_pos[2]) < i2w(Z_INNER) - i2w(5):
            raise RuntimeError("Robot fell through floor during settle")

        # Skid-steer point turn: left wheels backward, right wheels forward
        vel_targets = np.zeros(n_dofs)
        for k in ["FL", "RL"]:
            if k in widx:
                vel_targets[widx[k]] = -drive_vel
        for k in ["FR", "RR"]:
            if k in widx:
                vel_targets[widx[k]] = drive_vel

        # Track metrics
        max_tilt = settled_tilt
        cumulative_yaw = 0.0
        prev_yaw = yaw0
        peak_yaw_rate = 0.0
        completed_90 = False
        time_to_90 = None
        fell_through = False
        min_z = float(settled_pos[2])

        frames = int(TEST_DURATION * 60)
        for f in range(frames):
            robot_art.set_joint_velocity_targets(vel_targets.reshape(1, -1))
            app.update()

            if f % 3 == 0:
                p, r = chassis_xfp.get_world_poses()
                pos = p[0]
                rot = r[0]

                w, x, y, z = float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])
                yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
                up_z = 1 - 2*(x*x + y*y)
                tilt = math.degrees(math.acos(min(1.0, max(-1.0, up_z))))

                dy = yaw - prev_yaw
                if dy > 180: dy -= 360
                elif dy < -180: dy += 360
                cumulative_yaw += dy
                prev_yaw = yaw

                rate = abs(dy) / (3.0 / 60.0)
                if rate > peak_yaw_rate:
                    peak_yaw_rate = rate
                if tilt > max_tilt:
                    max_tilt = tilt

                cur_z = float(pos[2])
                if cur_z < min_z:
                    min_z = cur_z

                if cur_z < i2w(Z_INNER) - i2w(10):
                    fell_through = True
                    break

                if abs(cumulative_yaw) >= 90.0 and not completed_90:
                    completed_90 = True
                    time_to_90 = f / 60.0
                    break

        # Final state
        pf, rf = chassis_xfp.get_world_poses()
        final_pos = pf[0]
        final_r = math.sqrt(float(final_pos[0])**2 + float(final_pos[1])**2) / ES
        drift_r = abs(final_r - STEP_R)

    except Exception as e:
        print(f"    ERROR: {e}")
        completed_90 = False
        time_to_90 = None
        max_tilt = 999
        cumulative_yaw = 0
        peak_yaw_rate = 0
        fell_through = True
        settled_tilt = 999
        drift_r = 999

    timeline.stop()
    for _ in range(5):
        app.update()

    return {
        "completed_90": completed_90,
        "time_to_90_s": round(time_to_90, 2) if time_to_90 else None,
        "total_yaw_deg": round(abs(cumulative_yaw), 1),
        "peak_yaw_rate_dps": round(peak_yaw_rate, 1),
        "settled_tilt_deg": round(settled_tilt, 1),
        "max_tilt_deg": round(max_tilt, 1),
        "radial_drift_in": round(drift_r, 1),
        "fell_through": fell_through,
    }


# ---------------------------------------------------------------------------
# Main sweep
# ---------------------------------------------------------------------------
results = []

for idx, cfg in enumerate(configs_to_test):
    wr = cfg["wheel_radius_in"]
    ww = cfg["wheel_width_in"]
    wb = cfg["wheelbase_in"]
    tw = cfg["track_width_in"]
    trq = cfg["motor_torque_nm"]
    dvel = cfg["drive_vel"]
    mass = cfg["chassis_mass_lbs"]

    print(f"[{idx+1}/{len(configs_to_test)}] R={wr}\" W={ww}\" WB={wb}\" TW={tw}\" "
          f"T={trq}Nm v={dvel} cross={cfg['crossing_angle_deg']}°", end="")

    # Alternate USD files to avoid layer cache issues
    tmp_usd = tmp_usd_a if idx % 2 == 0 else tmp_usd_b

    ok, msg = build_parametric_robot(tmp_usd, wr, ww, trq, mass,
                                      wheelbase_in=wb, track_width_in=tw)
    if not ok:
        print(f" SKIP: {msg}")
        continue

    res = run_straddle_turn_test(tmp_usd, wr, drive_vel=dvel)

    result = {**cfg, **res}
    results.append(result)

    ok_str = "YES" if res["completed_90"] else "NO"
    t_str = f"{res['time_to_90_s']:.1f}s" if res['time_to_90_s'] else "N/A"
    print(f"  90°={ok_str} t={t_str} yaw={res['total_yaw_deg']:.0f}° "
          f"tilt={res['max_tilt_deg']:.0f}° drift={res['radial_drift_in']:.1f}\"")

    # If we found a success, print it prominently
    if res["completed_90"]:
        print(f"\n  *** SUCCESS! Config completed 90-degree turn! ***\n")

# ---------------------------------------------------------------------------
# Save results
# ---------------------------------------------------------------------------
os.makedirs(RESULTS_DIR, exist_ok=True)
output_path = os.path.join(RESULTS_DIR, "step_sweep.json")
output = {
    "test": "straddle_turn_sweep_v2",
    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    "step_height_in": STEP_HEIGHT,
    "chamfer_in": CHAMFER_SIZE,
    "effective_step_in": STEP_HEIGHT - CHAMFER_SIZE,
    "tunnel_limit_in": TUNNEL_SIZE,
    "chassis_height_in": CHASSIS_HEIGHT,
    "total_configs_generated": len(all_configs),
    "total_configs_tested": len(results),
    "results": results,
}

with open(output_path, "w") as f:
    json.dump(output, f, indent=2)
print(f"\nResults saved to: {output_path}")

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print(f"\n{'='*70}")
print("RESULTS SUMMARY")
print(f"{'='*70}")

passed = [r for r in results if r["completed_90"]]
if passed:
    passed.sort(key=lambda r: r["time_to_90_s"])
    print(f"\nCONFIGS THAT COMPLETED 90-DEGREE TURN ({len(passed)}/{len(results)}):")
    print(f"{'R':>4} {'W':>4} {'WB':>4} {'TW':>4} {'Trq':>5} {'Vel':>4} "
          f"{'Time':>6} {'Rate':>8} {'Tilt':>6} {'Drift':>6}")
    for r in passed:
        print(f"{r['wheel_radius_in']:4.1f} {r['wheel_width_in']:4.1f} "
              f"{r['wheelbase_in']:4d} {r['track_width_in']:4d} "
              f"{r['motor_torque_nm']:5d} {r['drive_vel']:4d} "
              f"{r['time_to_90_s']:5.1f}s {r['peak_yaw_rate_dps']:7.0f}°/s "
              f"{r['max_tilt_deg']:5.1f}° {r['radial_drift_in']:5.1f}\"")
    best = min(passed, key=lambda r: r["time_to_90_s"] + r["radial_drift_in"])
    print(f"\nBEST: R={best['wheel_radius_in']}\" W={best['wheel_width_in']}\" "
          f"WB={best['wheelbase_in']}\" TW={best['track_width_in']}\" "
          f"T={best['motor_torque_nm']}Nm v={best['drive_vel']} "
          f"-> 90° in {best['time_to_90_s']:.1f}s")
else:
    print("\nNO configuration completed the 90-degree turn.")
    print("\nTop 15 attempts (by total yaw achieved):")
    by_yaw = sorted(results, key=lambda r: r["total_yaw_deg"], reverse=True)
    print(f"{'R':>4} {'W':>4} {'WB':>4} {'TW':>4} {'Trq':>5} {'Vel':>4} "
          f"{'Yaw':>6} {'Tilt':>6} {'Cross':>6} {'Climb':>6}")
    for r in by_yaw[:15]:
        print(f"{r['wheel_radius_in']:4.1f} {r['wheel_width_in']:4.1f} "
              f"{r['wheelbase_in']:4d} {r['track_width_in']:4d} "
              f"{r['motor_torque_nm']:5d} {r['drive_vel']:4d} "
              f"{r['total_yaw_deg']:5.1f}° {r['max_tilt_deg']:5.1f}° "
              f"{r['crossing_angle_deg']:5.1f}° {'Y' if r['can_climb_step'] else 'N':>5}")

# Clean up
for f in [tmp_usd_a, tmp_usd_b]:
    if os.path.exists(f):
        os.remove(f)

timeline.stop()
app.close()
