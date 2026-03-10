"""
Phase 2 shared configuration.

After running setup_phase2_scene.py, update ROBOT_PRIM_PATH and joint paths
below if the URDF importer used different paths than expected.
"""
import os

ES = 0.024
FLOOR_OFFSET = 0.3

def i2w(inches):
    return inches * ES

PHASE2_DIR = os.path.dirname(os.path.abspath(__file__))
PHASE2_USD = os.path.join(PHASE2_DIR, "reactor_phase2.usd")
DATA_DIR = os.path.join(PHASE2_DIR, "data")
URDF_PATH = os.path.join(PHASE2_DIR, "fortis_robot.urdf")

# These will be set by discover_paths() at runtime
ROBOT_PRIM_PATH = None   # articulation root
WHEEL_JOINT_PATHS = {}   # {"fl_wheel": "/path/...", ...}
ARM_JOINT_PATHS = {}     # {"j1_yaw": "/path/...", ...}

WHEEL_JOINT_NAMES = ["fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"]
ARM_JOINT_NAMES = ["j1_yaw", "j2_shoulder", "j3_elbow", "j4_wrist"]
ALL_JOINT_NAMES = WHEEL_JOINT_NAMES + ARM_JOINT_NAMES

WHEEL_RADIUS = 0.072  # world units (3" * 0.024)
TIRE_R_INCHES = 3.0


def discover_paths(stage):
    """
    Discover robot prim paths from the imported URDF on the stage.
    Call this after loading the Phase 2 USD scene.
    Returns (robot_root_path, wheel_joint_paths_dict, arm_joint_paths_dict).
    """
    from pxr import UsdPhysics

    robot_root = None
    joint_map = {}

    for prim in stage.Traverse():
        name = prim.GetName()
        path = str(prim.GetPath())

        # Find articulation root
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            robot_root = path

        # Find joints by name
        if name in WHEEL_JOINT_NAMES or name in ARM_JOINT_NAMES:
            # Check if it's a joint (has joint API)
            if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.Joint):
                joint_map[name] = path
            elif UsdPhysics.DriveAPI.Get(prim, "angular"):
                joint_map[name] = path

    # If we didn't find joints by USD type, try by name alone
    if len(joint_map) < len(ALL_JOINT_NAMES):
        for prim in stage.Traverse():
            name = prim.GetName()
            if name in ALL_JOINT_NAMES and name not in joint_map:
                joint_map[name] = str(prim.GetPath())

    wheel_paths = {n: joint_map.get(n) for n in WHEEL_JOINT_NAMES}
    arm_paths = {n: joint_map.get(n) for n in ARM_JOINT_NAMES}

    # Report
    print(f"  Articulation root: {robot_root}")
    for n, p in wheel_paths.items():
        print(f"  Wheel joint {n}: {p}")
    for n, p in arm_paths.items():
        print(f"  Arm joint {n}: {p}")

    missing = [n for n, p in {**wheel_paths, **arm_paths}.items() if p is None]
    if missing:
        print(f"  WARNING: Missing joints: {missing}")

    return robot_root, wheel_paths, arm_paths
