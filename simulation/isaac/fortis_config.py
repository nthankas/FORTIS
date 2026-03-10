"""
Shared configuration for FORTIS Isaac Sim scripts.
All measurements stored in inches (original STL units).
Conversion to meters happens at scene-build time.
"""
import os

# Unit conversion
INCHES_TO_METERS = 0.0254

# Paths (relative to this file's location: simulation/isaac/)
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", ".."))
PROJECT_ROOT = os.path.abspath(os.path.join(_REPO_ROOT, "..", ".."))

ISAAC_DIR = _THIS_DIR
SCENE_DIR = os.path.join(_THIS_DIR, "scenes")
RESULTS_DIR = os.path.join(_REPO_ROOT, "optimizer", "results", "phase1")
MESH_DIR = os.path.join(PROJECT_ROOT, "Optimizer", "mesh")

STL_FLOOR_DECIMATED = os.path.join(MESH_DIR, "floor_decimated.stl")
STL_HALF_REACTOR = os.path.join(PROJECT_ROOT, "Optimizer", "halfReactor.stl")

SCENE_REACTOR_FLOOR = os.path.join(SCENE_DIR, "reactor_floor.usd")
SCENE_REACTOR = os.path.join(SCENE_DIR, "reactor.usd")

# The user-imported reactor.usd has mesh vertices in inches with scale=2.4.
# We fix this to INCHES_TO_METERS (0.0254) at load time for correct PhysX.
REACTOR_MESH_PRIM_PATH = "/World/halfReactor/mesh"

ISAAC_SIM_DIR = os.path.join(PROJECT_ROOT, "IsaacSim")

# Floor geometry from Phase 1a STL analysis (inches)
Z_INNER = -53.8          # Inner floor Z
Z_OUTER = -49.3          # Outer floor Z
STEP_HEIGHT = Z_OUTER - Z_INNER  # 4.5"
STEP_R = 55.5            # Approximate radial position of step
R_INNER_FLOOR_MIN = 45.8
R_INNER_FLOOR_MAX = 54.8
R_OUTER_FLOOR_MIN = 56.2
R_OUTER_FLOOR_MAX = 70.3
R_CENTER_POST = 38.0
R_OUTER_WALL = 70.3
STEP_GAP = R_OUTER_FLOOR_MIN - R_INNER_FLOOR_MAX  # ~1.4"

# Physics materials
GRAPHITE_MU_LOW = 0.3
GRAPHITE_MU_HIGH = 0.5
RUBBER_RESTITUTION = 0.2
GRAPHITE_RESTITUTION = 0.05

# Phase 1 recommended config from analytical sweep (reference)
RECOMMENDED_CHASSIS = {
    "length": 14.0,   # inches
    "width": 13.0,    # track width, inches
    "height": 9.0,    # inches
    "weight": 50.0,   # lbs
}
RECOMMENDED_ARM = {
    "L1": 16.0,  # shoulder to elbow, inches
    "L2": 22.0,  # elbow to wrist, inches
    "L3": 8.0,   # wrist to end effector, inches
    "mount": "top",
}
