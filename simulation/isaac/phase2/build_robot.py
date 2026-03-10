#!/usr/bin/env python3
"""
build_robot.py  --  Build the FORTIS robot as a proper USD articulation.

Run with Isaac Sim's Python:
    E:\FORTIS\IsaacSim\python.bat build_robot.py

Creates: fortis_robot.usd  (standalone robot file, no reactor dependency)

Robot structure:
    /Robot                          (Xform, ArticulationRootAPI)
      /Robot/Chassis                (RigidBody, box collision)
      /Robot/FL_Wheel               (RigidBody, cylinder collision)
      /Robot/FR_Wheel               (RigidBody, cylinder collision)
      /Robot/RL_Wheel               (RigidBody, cylinder collision)
      /Robot/RR_Wheel               (RigidBody, cylinder collision)
      /Robot/Arm_Base               (RigidBody, cylinder collision)  -- yaw turret
      /Robot/Arm_Upper              (RigidBody, box collision)       -- shoulder to elbow
      /Robot/Arm_Lower              (RigidBody, box collision)       -- elbow to wrist
      /Robot/Arm_Wrist              (RigidBody, box collision)       -- wrist + end effector
      /Robot/Joints/FL_Drive        (RevoluteJoint, Y axis)
      /Robot/Joints/FR_Drive        (RevoluteJoint, Y axis)
      /Robot/Joints/RL_Drive        (RevoluteJoint, Y axis)
      /Robot/Joints/RR_Drive        (RevoluteJoint, Y axis)
      /Robot/Joints/J1_Yaw          (RevoluteJoint, Z axis)
      /Robot/Joints/J2_Shoulder     (RevoluteJoint, Y axis)
      /Robot/Joints/J3_Elbow        (RevoluteJoint, Y axis)
      /Robot/Joints/J4_Wrist        (RevoluteJoint, Y axis)

All dimensions are PARAMETRIC -- change the constants below and re-run.
"""

import os
import sys
import math
import argparse

# Bootstrap Isaac Sim (required for pxr imports)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

# ---------------------------------------------------------------------------
# PARAMETRIC ROBOT DIMENSIONS (all in inches, converted to world units below)
# ---------------------------------------------------------------------------

# Effective scale: reactor.usd mesh uses 0.01 * 2.4 = 0.024 per inch
ES = 0.024  # world units per inch

def i2w(inches):
    """Convert inches to world units."""
    return inches * ES

# --- Chassis ---
CHASSIS_LENGTH  = 14.0   # inches, X direction (front-back)
CHASSIS_WIDTH   = 13.0   # inches, Y direction (track width, center-to-center of wheels)
CHASSIS_HEIGHT  =  5.0   # inches, Z direction (box height, not full stowed height)
CHASSIS_MASS    = 22.68  # kg (50 lbs)

# --- Wheels ---
WHEEL_RADIUS    =  3.0   # inches
WHEEL_WIDTH     =  2.0   # inches
WHEEL_MASS      =  1.13  # kg (~2.5 lbs each)

# --- Arm ---
ARM_BASE_HEIGHT =  2.0   # inches (yaw turret height)
ARM_BASE_RADIUS =  1.5   # inches
ARM_BASE_MASS   =  1.0   # kg

ARM_UPPER_LEN   = 16.0   # inches (L1: shoulder to elbow)
ARM_UPPER_CROSS =  2.0   # inches (square cross section)
ARM_UPPER_MASS  =  2.0   # kg

ARM_LOWER_LEN   = 22.0   # inches (L2: elbow to wrist)
ARM_LOWER_CROSS =  1.5   # inches
ARM_LOWER_MASS  =  1.5   # kg

ARM_WRIST_LEN   =  8.0   # inches (L3: wrist to end effector)
ARM_WRIST_CROSS =  1.2   # inches
ARM_WRIST_MASS  =  0.8   # kg

# --- Drive parameters ---
WHEEL_DRIVE_STIFFNESS = 0.0     # zero for velocity control
WHEEL_DRIVE_DAMPING   = 1000.0  # tracks target velocity
WHEEL_DRIVE_MAX_FORCE = 5.0     # Nm torque cap

ARM_DRIVE_STIFFNESS   = 500.0   # position control for arm
ARM_DRIVE_DAMPING     = 100.0
ARM_DRIVE_MAX_FORCE   = 20.0    # Nm


# ---------------------------------------------------------------------------
# USD BUILDER
# ---------------------------------------------------------------------------

def build_robot(output_path):
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema

    stage = Usd.Stage.CreateNew(output_path)
    stage.SetMetadata("metersPerUnit", 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.SetDefaultPrim(stage.DefinePrim("/Robot"))

    # -----------------------------------------------------------------------
    # Helper: compute box inertia tensor (diagonal) for a solid box
    # -----------------------------------------------------------------------
    def box_inertia(mass, lx, ly, lz):
        """Returns (Ixx, Iyy, Izz) for a solid box."""
        return (
            mass / 12.0 * (ly**2 + lz**2),
            mass / 12.0 * (lx**2 + lz**2),
            mass / 12.0 * (lx**2 + ly**2),
        )

    def cylinder_inertia(mass, radius, height):
        """Returns (Ixx, Iyy, Izz) for a solid cylinder aligned along Y."""
        Iyy = 0.5 * mass * radius**2
        Ixx = mass / 12.0 * (3 * radius**2 + height**2)
        Izz = Ixx
        return (Ixx, Iyy, Izz)

    # -----------------------------------------------------------------------
    # Helper: create a rigid body link with collision and visual geometry
    # -----------------------------------------------------------------------
    def make_box_link(path, half_extents, mass, color=(0.5, 0.5, 0.5), collision=True):
        """Create a box-shaped rigid body. collision=False for arm links that
        need to pass through reactor walls without generating forces."""
        prim = UsdGeom.Xform.Define(stage, path)

        rb = UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())

        mass_api = UsdPhysics.MassAPI.Apply(prim.GetPrim())
        mass_api.GetMassAttr().Set(mass)
        hx, hy, hz = half_extents
        ixx, iyy, izz = box_inertia(mass, hx*2, hy*2, hz*2)
        mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(ixx, iyy, izz))

        if collision:
            col_path = path + "/Collision"
            col = UsdGeom.Cube.Define(stage, col_path)
            col.GetSizeAttr().Set(1.0)
            col.GetPurposeAttr().Set("guide")
            UsdGeom.XformCommonAPI(col).SetScale(Gf.Vec3f(hx*2, hy*2, hz*2))
            UsdPhysics.CollisionAPI.Apply(col.GetPrim())

        # Visual cube (always present so you can see the link)
        vis_path = path + "/Visual"
        vis = UsdGeom.Cube.Define(stage, vis_path)
        vis.GetSizeAttr().Set(1.0)
        UsdGeom.XformCommonAPI(vis).SetScale(Gf.Vec3f(hx*2, hy*2, hz*2))
        vis.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

        return prim

    def make_cylinder_link(path, radius, half_height, mass, axis="Y",
                           color=(0.3, 0.3, 0.3), collision=True):
        """Create a cylinder-shaped rigid body. collision=False for arm links."""
        prim = UsdGeom.Xform.Define(stage, path)

        rb = UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())

        mass_api = UsdPhysics.MassAPI.Apply(prim.GetPrim())
        mass_api.GetMassAttr().Set(mass)
        ixx, iyy, izz = cylinder_inertia(mass, radius, half_height*2)
        mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(ixx, iyy, izz))

        if collision:
            col_path = path + "/Collision"
            col = UsdGeom.Cylinder.Define(stage, col_path)
            col.GetRadiusAttr().Set(radius)
            col.GetHeightAttr().Set(half_height * 2)
            col.GetAxisAttr().Set(axis)
            col.GetPurposeAttr().Set("guide")
            UsdPhysics.CollisionAPI.Apply(col.GetPrim())

        # Visual cylinder (always present)
        vis_path = path + "/Visual"
        vis = UsdGeom.Cylinder.Define(stage, vis_path)
        vis.GetRadiusAttr().Set(radius)
        vis.GetHeightAttr().Set(half_height * 2)
        vis.GetAxisAttr().Set(axis)
        vis.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

        return prim

    def make_revolute_joint(path, parent_path, child_path, axis,
                            anchor_in_parent, anchor_in_child=(0,0,0),
                            lower_limit=None, upper_limit=None,
                            stiffness=0, damping=1000, max_force=5,
                            drive_type="force", target_position=None):
        """Create a revolute joint with angular drive."""
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        joint.GetAxisAttr().Set(axis)

        joint.GetBody0Rel().SetTargets([parent_path])
        joint.GetBody1Rel().SetTargets([child_path])

        joint.GetLocalPos0Attr().Set(Gf.Vec3f(*anchor_in_parent))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.GetLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.GetLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

        if lower_limit is not None:
            joint.GetLowerLimitAttr().Set(lower_limit)
        if upper_limit is not None:
            joint.GetUpperLimitAttr().Set(upper_limit)

        # Angular drive
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.GetStiffnessAttr().Set(stiffness)
        drive.GetDampingAttr().Set(damping)
        drive.GetMaxForceAttr().Set(max_force)
        drive.GetTypeAttr().Set(drive_type)
        if target_position is not None:
            drive.GetTargetPositionAttr().Set(target_position)

        # PhysX joint settings
        pxjoint = PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())

        return joint

    # -----------------------------------------------------------------------
    # ROOT: /Robot with ArticulationRootAPI
    # -----------------------------------------------------------------------
    robot = UsdGeom.Xform.Define(stage, "/Robot")
    UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
    # Set initial transform (will be repositioned when loaded into scene)
    UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(0, 0, 0))

    # -----------------------------------------------------------------------
    # CHASSIS: the main body, root of the articulation
    # -----------------------------------------------------------------------
    ch_hx = i2w(CHASSIS_LENGTH) / 2
    ch_hy = i2w(CHASSIS_WIDTH) / 2
    ch_hz = i2w(CHASSIS_HEIGHT) / 2

    chassis = make_box_link("/Robot/Chassis",
                            half_extents=(ch_hx, ch_hy, ch_hz),
                            mass=CHASSIS_MASS,
                            color=(0.2, 0.35, 0.6))

    # -----------------------------------------------------------------------
    # WHEELS: 4 wheels at corners of chassis
    # -----------------------------------------------------------------------
    wr = i2w(WHEEL_RADIUS)
    wh = i2w(WHEEL_WIDTH) / 2  # half-height

    # Wheel positions relative to chassis center:
    #   X: +/- half chassis length (front/rear)
    #   Y: +/- half chassis width (left/right) + wheel half-width for clearance
    #   Z: bottom of chassis minus tire radius contact
    #     Chassis bottom is at -ch_hz. Wheel center is at -ch_hz (axle at chassis bottom)
    wheel_z = -ch_hz  # wheel axle at bottom of chassis box
    wheel_x = ch_hx * 0.85  # wheels slightly inboard from edges
    wheel_y_offset = ch_hy + wh + i2w(0.25)  # small gap between chassis and wheel

    wheel_defs = [
        ("FL_Wheel", +wheel_x, +wheel_y_offset, wheel_z),
        ("FR_Wheel", +wheel_x, -wheel_y_offset, wheel_z),
        ("RL_Wheel", -wheel_x, +wheel_y_offset, wheel_z),
        ("RR_Wheel", -wheel_x, -wheel_y_offset, wheel_z),
    ]

    for name, wx, wy, wz in wheel_defs:
        w = make_cylinder_link(f"/Robot/{name}", wr, wh, WHEEL_MASS,
                               axis="Y", color=(0.15, 0.15, 0.15))
        # Position wheel at its location
        UsdGeom.XformCommonAPI(w).SetTranslate(Gf.Vec3d(wx, wy, wz))

    # -----------------------------------------------------------------------
    # ARM: 4-DOF chain mounted on top of chassis
    # No collision on arm links -- the arm needs to reach into/touch reactor
    # walls during normal operation. Collision forces would destabilize the
    # sim. The arm's mass/inertia still affect chassis stability via joints.
    # -----------------------------------------------------------------------
    arm_mount_z = ch_hz  # top of chassis

    ab_r = i2w(ARM_BASE_RADIUS)
    ab_hh = i2w(ARM_BASE_HEIGHT) / 2
    au_hx = i2w(ARM_UPPER_CROSS) / 2
    au_hz = i2w(ARM_UPPER_LEN) / 2
    al_hx = i2w(ARM_LOWER_CROSS) / 2
    al_hz = i2w(ARM_LOWER_LEN) / 2
    aw_hx = i2w(ARM_WRIST_CROSS) / 2
    aw_hz = i2w(ARM_WRIST_LEN) / 2

    arm_base_z = arm_mount_z + ab_hh

    # Arm collision OFF for now -- arm mass still affects CG/stability.
    # Arm collision will be handled properly during arm stability phase.
    arm_base = make_cylinder_link("/Robot/Arm_Base", ab_r, ab_hh, ARM_BASE_MASS,
                                   axis="Z", color=(0.6, 0.3, 0.1), collision=False)
    UsdGeom.XformCommonAPI(arm_base).SetTranslate(Gf.Vec3d(0, 0, arm_base_z))

    arm_upper = make_box_link("/Robot/Arm_Upper",
                              half_extents=(au_hx, au_hx, au_hz),
                              mass=ARM_UPPER_MASS,
                              color=(0.5, 0.5, 0.2), collision=False)
    arm_upper_z = arm_base_z + ab_hh + au_hz
    UsdGeom.XformCommonAPI(arm_upper).SetTranslate(Gf.Vec3d(0, 0, arm_upper_z))

    arm_lower = make_box_link("/Robot/Arm_Lower",
                              half_extents=(al_hx, al_hx, al_hz),
                              mass=ARM_LOWER_MASS,
                              color=(0.4, 0.5, 0.3), collision=False)
    arm_lower_z = arm_upper_z + au_hz + al_hz
    UsdGeom.XformCommonAPI(arm_lower).SetTranslate(Gf.Vec3d(0, 0, arm_lower_z))

    arm_wrist = make_box_link("/Robot/Arm_Wrist",
                              half_extents=(aw_hx, aw_hx, aw_hz),
                              mass=ARM_WRIST_MASS,
                              color=(0.6, 0.2, 0.2), collision=False)
    arm_wrist_z = arm_lower_z + al_hz + aw_hz
    UsdGeom.XformCommonAPI(arm_wrist).SetTranslate(Gf.Vec3d(0, 0, arm_wrist_z))

    # -----------------------------------------------------------------------
    # JOINTS: connect everything
    # -----------------------------------------------------------------------
    joints_scope = UsdGeom.Scope.Define(stage, "/Robot/Joints")

    # Wheel joints: revolute around Y, velocity-controlled
    for name, wx, wy, wz in wheel_defs:
        drive_name = name.replace("Wheel", "Drive")
        make_revolute_joint(
            path=f"/Robot/Joints/{drive_name}",
            parent_path="/Robot/Chassis",
            child_path=f"/Robot/{name}",
            axis="Y",
            anchor_in_parent=(wx, wy, wz),
            stiffness=WHEEL_DRIVE_STIFFNESS,
            damping=WHEEL_DRIVE_DAMPING,
            max_force=WHEEL_DRIVE_MAX_FORCE,
            drive_type="force",
        )

    # J1: Yaw -- chassis to arm base, around Z
    make_revolute_joint(
        path="/Robot/Joints/J1_Yaw",
        parent_path="/Robot/Chassis",
        child_path="/Robot/Arm_Base",
        axis="Z",
        anchor_in_parent=(0, 0, arm_mount_z),
        lower_limit=-180, upper_limit=180,
        stiffness=ARM_DRIVE_STIFFNESS,
        damping=ARM_DRIVE_DAMPING,
        max_force=ARM_DRIVE_MAX_FORCE,
        drive_type="force",
        target_position=0.0,
    )

    # J2: Shoulder -- arm base to upper arm, around Y
    make_revolute_joint(
        path="/Robot/Joints/J2_Shoulder",
        parent_path="/Robot/Arm_Base",
        child_path="/Robot/Arm_Upper",
        axis="Y",
        anchor_in_parent=(0, 0, ab_hh),  # top of base turret
        lower_limit=-90, upper_limit=135,
        stiffness=ARM_DRIVE_STIFFNESS,
        damping=ARM_DRIVE_DAMPING,
        max_force=ARM_DRIVE_MAX_FORCE,
        drive_type="force",
        target_position=0.0,
    )

    # J3: Elbow -- upper arm to lower arm, around Y
    make_revolute_joint(
        path="/Robot/Joints/J3_Elbow",
        parent_path="/Robot/Arm_Upper",
        child_path="/Robot/Arm_Lower",
        axis="Y",
        anchor_in_parent=(0, 0, au_hz),  # top of upper arm
        lower_limit=-150, upper_limit=150,
        stiffness=ARM_DRIVE_STIFFNESS,
        damping=ARM_DRIVE_DAMPING,
        max_force=15.0,
        drive_type="force",
        target_position=0.0,
    )

    # J4: Wrist -- lower arm to wrist, around Y
    make_revolute_joint(
        path="/Robot/Joints/J4_Wrist",
        parent_path="/Robot/Arm_Lower",
        child_path="/Robot/Arm_Wrist",
        axis="Y",
        anchor_in_parent=(0, 0, al_hz),  # top of lower arm
        lower_limit=-120, upper_limit=120,
        stiffness=ARM_DRIVE_STIFFNESS,
        damping=ARM_DRIVE_DAMPING,
        max_force=10.0,
        drive_type="force",
        target_position=0.0,
    )

    # -----------------------------------------------------------------------
    # RUBBER PHYSICS MATERIAL for wheels
    # -----------------------------------------------------------------------
    rubber_mat = UsdShade.Material.Define(stage, "/Robot/RubberMaterial")
    rubber_phys = UsdPhysics.MaterialAPI.Apply(rubber_mat.GetPrim())
    rubber_phys.GetStaticFrictionAttr().Set(0.8)
    rubber_phys.GetDynamicFrictionAttr().Set(0.6)
    rubber_phys.GetRestitutionAttr().Set(0.2)

    # Bind rubber material to wheel collisions
    for name, _, _, _ in wheel_defs:
        col_prim = stage.GetPrimAtPath(f"/Robot/{name}/Collision")
        UsdShade.MaterialBindingAPI.Apply(col_prim).Bind(
            rubber_mat,
            UsdShade.Tokens.weakerThanDescendants,
            "physics"
        )

    # -----------------------------------------------------------------------
    # SOFT PHYSICS MATERIAL for arm links
    # Zero restitution (no bounce), low friction (slides on contact).
    # Light touch is OK; violent bounce is not.
    # -----------------------------------------------------------------------
    arm_mat = UsdShade.Material.Define(stage, "/Robot/ArmMaterial")
    arm_phys = UsdPhysics.MaterialAPI.Apply(arm_mat.GetPrim())
    arm_phys.GetStaticFrictionAttr().Set(0.2)
    arm_phys.GetDynamicFrictionAttr().Set(0.15)
    arm_phys.GetRestitutionAttr().Set(0.0)  # zero bounce

    # Bind to all arm link collisions
    for arm_name in ["Arm_Base", "Arm_Upper", "Arm_Lower", "Arm_Wrist"]:
        col_prim = stage.GetPrimAtPath(f"/Robot/{arm_name}/Collision")
        if col_prim and col_prim.IsValid():
            UsdShade.MaterialBindingAPI.Apply(col_prim).Bind(
                arm_mat,
                UsdShade.Tokens.weakerThanDescendants,
                "physics"
            )

    # Also soft material on chassis collision (it shouldn't bounce off walls either)
    chassis_col = stage.GetPrimAtPath("/Robot/Chassis/Collision")
    UsdShade.MaterialBindingAPI.Apply(chassis_col).Bind(
        arm_mat,
        UsdShade.Tokens.weakerThanDescendants,
        "physics"
    )

    # -----------------------------------------------------------------------
    # SAVE
    # -----------------------------------------------------------------------
    stage.GetRootLayer().Save()
    print(f"Robot USD saved to: {output_path}")
    print()
    print("=== Robot Summary ===")
    print(f"  Chassis: {CHASSIS_LENGTH}\" x {CHASSIS_WIDTH}\" x {CHASSIS_HEIGHT}\" ({CHASSIS_MASS} kg)")
    print(f"  Wheels:  R={WHEEL_RADIUS}\", W={WHEEL_WIDTH}\" ({WHEEL_MASS} kg each)")
    print(f"  Arm:     L1={ARM_UPPER_LEN}\", L2={ARM_LOWER_LEN}\", L3={ARM_WRIST_LEN}\" (total {ARM_UPPER_LEN+ARM_LOWER_LEN+ARM_WRIST_LEN}\")")
    print(f"  Effective scale: {ES} (world units per inch)")
    print(f"  Chassis in world: {i2w(CHASSIS_LENGTH):.4f} x {i2w(CHASSIS_WIDTH):.4f} x {i2w(CHASSIS_HEIGHT):.4f}")
    print(f"  Wheel radius in world: {wr:.4f}")
    print()

    # Print joint hierarchy for verification
    print("=== Prim Hierarchy ===")
    for prim in stage.Traverse():
        depth = len(str(prim.GetPath()).split("/")) - 2
        apis = []
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            apis.append("ArticulationRoot")
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            apis.append("RigidBody")
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            apis.append("Collision")
        if prim.IsA(UsdPhysics.RevoluteJoint):
            apis.append("RevoluteJoint")
        api_str = f"  [{', '.join(apis)}]" if apis else ""
        print(f"{'  ' * depth}{prim.GetPath()}{api_str}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build FORTIS robot USD")
    parser.add_argument("--output", default=None, help="Output USD path")
    args = parser.parse_args()

    out = args.output or os.path.join(os.path.dirname(os.path.abspath(__file__)), "fortis_robot.usd")
    # Delete old file if it exists (Usd.Stage.CreateNew fails on existing files)
    if os.path.exists(out):
        os.remove(out)
    build_robot(out)
    app.close()
