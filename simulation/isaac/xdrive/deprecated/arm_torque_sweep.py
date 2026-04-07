"""
FORTIS arm torque / power / stability sweep.

All-NEMA-17 + Cricket MK II 25:1 on 4 joints, 0.25" 6061-T6 aluminum links,
Gemini 2 at mid-L3. Chassis on a simple step (no full reactor geometry).
Chassis and wheel code copied from canonical/xdrive_reactor_arm.py (STATE.md).

Usage:
  IsaacSim\\python.bat tools/arm_torque_sweep.py --gui     # manual test
  IsaacSim\\python.bat tools/arm_torque_sweep.py --sweep   # analytical sweep
"""
import os, sys, math, argparse, json, time
import numpy as np

os.environ["PYTHONIOENCODING"] = "utf-8"

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--sweep", action="store_true",
                    help="Run analytical torque/power/stability sweep (headless)")
parser.add_argument("--belly", type=float, default=2.5)
parser.add_argument("--hz", type=int, default=360)
args, _ = parser.parse_known_args()

if args.sweep:
    # Sweep mode is pure analytical — no Isaac Sim needed
    pass
else:
    if not args.gui:
        print("Specify --gui or --sweep"); sys.exit(1)

# ============================================================================
# Constants (copied from canonical/xdrive_reactor_arm.py — DO NOT diverge)
# ============================================================================
IN = 0.0254
MM = 0.001

XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ASSETS_DIR  = os.path.join(XDRIVE_ROOT, "assets")
RESULTS_DIR = os.path.join(XDRIVE_ROOT, "results", "arm_torque_sweep")
sys.path.insert(0, os.path.join(XDRIVE_ROOT, "lib"))
OMNIWHEEL_USD = os.path.join(ASSETS_DIR, "omniwheels.usd")

# Chassis
CHASSIS_L = 15.354 * IN
CHASSIS_W = 9.353 * IN
CHASSIS_H = 7.1 * IN
CHAMFER_FACE = 3.0 * IN
CHAMFER_CUT = CHAMFER_FACE / math.sqrt(2)
MOTOR_MOUNT_LEN = 2.5 * IN
BELLY_HEIGHT = args.belly * IN
ARCH_FLAT_WIDTH = 3.0 * IN

# Wheels
TARGET_DIA_MM = 203.0
TARGET_WIDTH_MM = 51.8
WHEEL_RADIUS = TARGET_DIA_MM / 2.0 * MM
WHEEL_WIDTH = TARGET_WIDTH_MM * MM
SRC_DIA_MM = 82.5
SRC_WIDTH_MM = 44.87
SCALE_XZ = TARGET_DIA_MM / SRC_DIA_MM
SCALE_Y = TARGET_WIDTH_MM / SRC_WIDTH_MM
SCALE_UNIFORM = SCALE_XZ

CHASSIS_MASS = 20.4
WHEEL_MASS = 1.0
ROLLER_MASS_FRAC = 0.3
NUM_ROLLERS = 10
HUB_MASS = WHEEL_MASS * (1.0 - ROLLER_MASS_FRAC)
ROLLER_MASS = WHEEL_MASS * ROLLER_MASS_FRAC / NUM_ROLLERS

PHYSICS_HZ = max(360, min(480, args.hz))
FRICTION_MU = 0.5
DRIVE_DAMPING = 100.0
DRIVE_MAX_FORCE = 200.0
DRIVE_SPEED = 0.2
ROTATE_SPEED = 0.5

# Step geometry
STEP_HEIGHT = 4.5 * IN  # 0.1143 m

# ============================================================================
# Arm constants — ALL NEMA 17 + Cricket MK II, aluminum links, cam at mid-L3
# ============================================================================
L_J1 = 0.1270   # 5.00"  J1 vertical stack
L_L2 = 0.5761   # 22.68" shoulder link
L_L3 = 0.5000   # 19.69" elbow link
L_L4 = 0.1500   # 5.91"  wrist link

ARM_MOUNT_X = -CHASSIS_L / 2.0
ARM_MOUNT_Y = 0.0
ARM_MOUNT_Z = CHASSIS_H / 2.0

# Lumped masses (see docs/arm_sweep_masses.md)
M_J1_BASE     = 0.630    # NEMA 17 + Cricket + misc
M_J2_SHOULDER = 0.9425   # NEMA 17 + Cricket + L2 aluminum (312.5g) + misc
M_J3_ELBOW    = 1.3231   # NEMA 17 + Cricket + L3 (248.1g) + Gemini 2 (445g) + misc
M_J4_WRIST    = 0.7312   # NEMA 17 + Cricket + L4 aluminum (101.2g) + misc

# Sub-masses for J3 COM calculation (camera at mid-L3)
M_J3_MOTOR    = 0.630    # motor + gearbox + misc at J3 pivot
M_J3_LINK     = 0.2481   # L3 aluminum distributed along link
M_J3_CAMERA   = 0.445    # Gemini 2 at L_L3/2

# Motor / gearbox constants
GEAR_RATIO     = 25.0
GEAR_EFF       = 0.855
KT             = 0.3095   # Nm/A (= 0.65 Nm / 2.1 A)
R_PHASE        = 1.6      # ohms
GEARBOX_RATED  = 12.0     # Nm output
MOTOR_STALL_OUT = 0.65 * GEAR_RATIO * GEAR_EFF  # 13.9 Nm

ARM_STIFFNESS = 1000.0
ARM_DAMPING   = 100.0
ARM_MAX_FORCE = 12.0

ARM_LIMITS_DEG = {
    "ArmJ1": (-180.0, 180.0),
    "ArmJ2": ( -20.0, 110.0),
    "ArmJ3": (-170.0, 170.0),
    "ArmJ4": (-180.0, 180.0),
}

LINK_BOX_W = 0.040
LINK_BOX_T = 0.025
VIS_Z_STACK = 0.045

# Chassis geometry helpers
SL = CHASSIS_L / 2.0
SW = CHASSIS_W / 2.0
C = CHAMFER_CUT

OCT_XY = [
    ( SL,       -(SW - C)), ( SL,        (SW - C)),
    ( (SL - C),  SW      ), (-(SL - C),  SW      ),
    (-SL,        (SW - C)), (-SL,       -(SW - C)),
    (-(SL - C), -SW      ), ( (SL - C), -SW      ),
]

def mid(a, b): return ((a[0]+b[0])/2, (a[1]+b[1])/2)

WHEEL_XY = {
    "FR": mid(OCT_XY[0], OCT_XY[7]), "FL": mid(OCT_XY[1], OCT_XY[2]),
    "BL": mid(OCT_XY[3], OCT_XY[4]), "BR": mid(OCT_XY[5], OCT_XY[6]),
}
AXLE_ANGLES = {"FR": 45.0, "FL": 135.0, "BL": 45.0, "BR": 135.0}
WHEEL_ORDER = ["FR", "FL", "BL", "BR"]


# ============================================================================
# Analytical sweep (no Isaac Sim needed)
# ============================================================================

def analytical_gravity_torques(j2_deg, j3_deg, j4_deg):
    """Compute static gravity torque at each joint for a given pose.

    All angles are pitch from the stowed (horizontal) reference:
    - J2=0 means L2 horizontal, J2=90 means L2 pointing straight down
    - J3=0 means L3 parallel to L2 (folded), etc.

    Convention: positive angle = pitch DOWN from the link's parent.
    J1 is yaw (vertical axis) and sees zero gravity torque.

    Returns dict with torques in Nm at J2, J3, J4 (positive = resisting gravity).
    """
    g = 9.81
    j2 = math.radians(j2_deg)
    j3 = math.radians(j3_deg)
    j4 = math.radians(j4_deg)

    # Absolute angles of each link from horizontal
    # J2 angle is measured from horizontal (0=horizontal, 90=down)
    theta2 = j2              # L2 angle from horizontal
    theta3 = j2 + j3         # L3 absolute angle
    theta4 = j2 + j3 + j4   # L4 absolute angle

    # Horizontal distances from each joint to mass centers (projected)
    # For gravity torque, we need horizontal projection of each mass point
    # relative to the joint we're computing torque for.

    # === J4 torque (only L4 link distal) ===
    # L4 COM at L_L4/2 along L4 direction
    d_l4_com = (L_L4 / 2.0) * math.cos(theta4)
    tau_j4 = M_J4_WRIST * g * d_l4_com

    # === J3 torque (L3 body + J4 body + L4 distal) ===
    # J3 motor/gearbox at J3 pivot = 0 distance -> no torque contribution
    # L3 link COM at L_L3/2 along L3
    d_l3_link_com = (L_L3 / 2.0) * math.cos(theta3)
    # Camera at L_L3/2 along L3 (same as link COM)
    d_camera = (L_L3 / 2.0) * math.cos(theta3)
    # J4 pivot at L_L3 along L3
    d_j4_from_j3 = L_L3 * math.cos(theta3)
    # L4 COM at J4 + L_L4/2 along L4
    d_l4_from_j3 = d_j4_from_j3 + (L_L4 / 2.0) * math.cos(theta4)

    tau_j3 = (M_J3_LINK * g * d_l3_link_com +
              M_J3_CAMERA * g * d_camera +
              M_J4_WRIST * g * d_l4_from_j3)
    # Add motor mass at J3 (zero distance) — no contribution

    # === J2 torque (everything distal: J2 body link + J3 stuff + J4 stuff) ===
    # L2 link COM at L_L2/2 along L2. J2 motor/gearbox at J2 = 0 distance.
    # M_J2_SHOULDER includes motor+gearbox+L2 link. The motor is at 0, link COM at L_L2/2.
    # Split: motor part (630g) at 0, L2 link part (312.5g) at L_L2/2
    m_j2_motor = 0.630   # motor+gearbox+misc at J2 pivot
    m_j2_link  = 0.3125  # L2 aluminum

    d_l2_link_com = (L_L2 / 2.0) * math.cos(theta2)
    d_j3_from_j2 = L_L2 * math.cos(theta2)

    # J3 motor mass at J3 pivot
    d_j3_motor_from_j2 = d_j3_from_j2  # motor is at J3 pivot
    # L3 link COM at J3 + L_L3/2 along L3
    d_l3_from_j2 = d_j3_from_j2 + (L_L3 / 2.0) * math.cos(theta3)
    # Camera at J3 + L_L3/2 along L3
    d_cam_from_j2 = d_j3_from_j2 + (L_L3 / 2.0) * math.cos(theta3)
    # J4 pivot at J3 + L_L3 along L3
    d_j4_from_j2 = d_j3_from_j2 + L_L3 * math.cos(theta3)
    # L4 COM at J4 + L_L4/2 along L4
    d_l4_from_j2 = d_j4_from_j2 + (L_L4 / 2.0) * math.cos(theta4)

    tau_j2 = (m_j2_link * g * d_l2_link_com +
              M_J3_MOTOR * g * d_j3_motor_from_j2 +
              M_J3_LINK * g * d_l3_from_j2 +
              M_J3_CAMERA * g * d_cam_from_j2 +
              M_J4_WRIST * g * d_l4_from_j2)

    return {"J2": tau_j2, "J3": tau_j3, "J4": tau_j4, "J1": 0.0}


def motor_current(torque_nm):
    """Motor phase current for a given joint output torque."""
    motor_torque = abs(torque_nm) / (GEAR_RATIO * GEAR_EFF)
    return motor_torque / KT


def electrical_power(torque_nm):
    """Holding power in watts (both phases energized)."""
    I = motor_current(torque_nm)
    return 2.0 * I * I * R_PHASE


def tipping_moment(j2_deg, j3_deg, j4_deg):
    """Tipping moment about chassis front wheel edge from the arm.

    Positive = stable (restoring), negative = tipping forward.
    """
    g = 9.81
    j2 = math.radians(j2_deg)
    j3 = math.radians(j3_deg)
    j4 = math.radians(j4_deg)
    theta2 = j2
    theta3 = j2 + j3
    theta4 = j2 + j3 + j4

    # Front wheel edge distance from chassis center (tipping axis)
    # Wheels at 45 deg chamfers, front wheels at ~SL-C/2 in X
    front_wheel_x = (WHEEL_XY["FR"][0] + WHEEL_XY["FL"][0]) / 2.0

    # Arm mount is at ARM_MOUNT_X (back edge). Compute horizontal position of each mass
    # in chassis frame relative to the front wheel tipping axis.
    def horiz_from_mount(link_dist_along_chain):
        """Unused — compute per-mass below."""
        pass

    # Position of each mass in chassis X frame
    # J1 base at ARM_MOUNT_X (motor at base, COM at midpoint of J1 stack)
    x_j1 = ARM_MOUNT_X  # motor at base
    x_j2_pivot = ARM_MOUNT_X  # J2 is at top of J1 stack (same X for vertical stack)

    # L2 extends from J2 along absolute angle theta2
    x_l2_com = x_j2_pivot + (L_L2 / 2.0) * math.cos(theta2)
    x_j3 = x_j2_pivot + L_L2 * math.cos(theta2)
    x_l3_com = x_j3 + (L_L3 / 2.0) * math.cos(theta3)
    x_cam = x_j3 + (L_L3 / 2.0) * math.cos(theta3)
    x_j4 = x_j3 + L_L3 * math.cos(theta3)
    x_l4_com = x_j4 + (L_L4 / 2.0) * math.cos(theta4)

    # Moment about front wheel axis (positive = behind axis = stable)
    # moment = mass * g * (front_wheel_x - mass_x)  [positive when mass is behind]
    masses_and_x = [
        (M_J1_BASE, x_j1),
        (0.630, x_j2_pivot),       # J2 motor at pivot
        (0.3125, x_l2_com),        # L2 link
        (M_J3_MOTOR, x_j3),        # J3 motor at J3
        (M_J3_LINK, x_l3_com),     # L3 link
        (M_J3_CAMERA, x_cam),      # camera
        (M_J4_WRIST, x_l4_com),    # J4 body (motor+link)
    ]

    arm_moment = sum(m * g * (front_wheel_x - x) for m, x in masses_and_x)

    # Chassis restoring moment (chassis CG at center, mass without arm)
    chassis_no_arm = CHASSIS_MASS  # wheels included in chassis mass here
    chassis_moment = chassis_no_arm * g * (front_wheel_x - 0.0)  # CG at x=0

    return chassis_moment + arm_moment


def run_analytical_sweep():
    """Run the full analytical sweep and generate plots + JSON."""
    os.makedirs(RESULTS_DIR, exist_ok=True)

    j2_range = np.arange(-20, 111, 5)    # -20 to 110 deg
    j3_range = np.arange(-170, 171, 10)  # -170 to 170 deg
    j4_vals  = [0, -45, 45]              # sample J4 angles

    results = []
    print(f"Analytical sweep: {len(j2_range)} x {len(j3_range)} x {len(j4_vals)} "
          f"= {len(j2_range)*len(j3_range)*len(j4_vals)} poses", flush=True)

    for j4 in j4_vals:
        for j2 in j2_range:
            for j3 in j3_range:
                torques = analytical_gravity_torques(float(j2), float(j3), float(j4))
                powers = {k: electrical_power(v) for k, v in torques.items()}
                tip = tipping_moment(float(j2), float(j3), float(j4))
                results.append({
                    "j2": float(j2), "j3": float(j3), "j4": float(j4),
                    "tau_J2": torques["J2"], "tau_J3": torques["J3"], "tau_J4": torques["J4"],
                    "power_J2": powers["J2"], "power_J3": powers["J3"], "power_J4": powers["J4"],
                    "power_total": powers["J2"] + powers["J3"] + powers["J4"],
                    "heat_J2": powers["J2"], "heat_J3": powers["J3"], "heat_J4": powers["J4"],
                    "tipping_moment": tip,
                    "j2_over_limit": abs(torques["J2"]) > GEARBOX_RATED,
                    "j3_over_limit": abs(torques["J3"]) > GEARBOX_RATED,
                    "j4_over_limit": abs(torques["J4"]) > GEARBOX_RATED,
                })

    # Save JSON
    json_path = os.path.join(RESULTS_DIR, "sweep_static.json")
    with open(json_path, "w") as f:
        json.dump(results, f, indent=1)
    print(f"Saved {len(results)} results to {json_path}", flush=True)

    # ---- Summary ----
    max_tau_j2 = max(abs(r["tau_J2"]) for r in results)
    max_tau_j3 = max(abs(r["tau_J3"]) for r in results)
    max_tau_j4 = max(abs(r["tau_J4"]) for r in results)
    n_j2_over = sum(1 for r in results if r["j2_over_limit"])
    n_j3_over = sum(1 for r in results if r["j3_over_limit"])
    n_total = len(results)
    min_tip = min(r["tipping_moment"] for r in results)

    print("\n" + "=" * 60)
    print("SWEEP SUMMARY")
    print("=" * 60)
    print(f"Motor+gearbox: NEMA 17 + Cricket MK II 25:1")
    print(f"Gearbox rated torque: {GEARBOX_RATED} Nm")
    print(f"Motor stall output:   {MOTOR_STALL_OUT:.1f} Nm")
    print(f"")
    print(f"J2 (shoulder) max torque: {max_tau_j2:.1f} Nm  "
          f"{'** EXCEEDS LIMIT **' if max_tau_j2 > GEARBOX_RATED else 'OK'}")
    print(f"J3 (elbow)    max torque: {max_tau_j3:.1f} Nm  "
          f"{'** EXCEEDS LIMIT **' if max_tau_j3 > GEARBOX_RATED else 'OK'}")
    print(f"J4 (wrist)    max torque: {max_tau_j4:.1f} Nm  "
          f"{'** EXCEEDS LIMIT **' if max_tau_j4 > GEARBOX_RATED else 'OK'}")
    print(f"")
    print(f"Poses where J2 exceeds {GEARBOX_RATED} Nm: {n_j2_over}/{n_total} "
          f"({100*n_j2_over/n_total:.0f}%)")
    print(f"Poses where J3 exceeds {GEARBOX_RATED} Nm: {n_j3_over}/{n_total} "
          f"({100*n_j3_over/n_total:.0f}%)")
    print(f"")
    print(f"Worst tipping moment: {min_tip:.1f} Nm  "
          f"{'** TIPS **' if min_tip < 0 else 'Stable'}")

    # J2 breakeven angle (where |tau_J2| = GEARBOX_RATED)
    for j2_test in np.arange(0, 110, 0.5):
        t = analytical_gravity_torques(j2_test, 0.0, 0.0)
        if abs(t["J2"]) <= GEARBOX_RATED:
            print(f"\nJ2 breakeven angle (J3=0, J4=0): {j2_test:.1f} deg below horizontal")
            print(f"  Arm can only operate below ~{j2_test:.0f} deg from horizontal"
                  f" at gearbox rated torque")
            break

    # ---- Plots ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        _generate_plots(results, j2_range, j3_range, j4_vals, plt)
    except ImportError:
        print("\nmatplotlib not available — skipping plots. JSON data saved.", flush=True)


def _generate_plots(results, j2_range, j3_range, j4_vals, plt):
    """Generate all sweep plots."""
    # Filter for J4=0 slice (primary view)
    r0 = [r for r in results if r["j4"] == 0]
    j2s = sorted(set(r["j2"] for r in r0))
    j3s = sorted(set(r["j3"] for r in r0))

    def make_grid(key):
        grid = np.full((len(j2s), len(j3s)), np.nan)
        lookup = {(r["j2"], r["j3"]): r[key] for r in r0}
        for i, j2 in enumerate(j2s):
            for j, j3 in enumerate(j3s):
                if (j2, j3) in lookup:
                    grid[i, j] = lookup[(j2, j3)]
        return grid

    J3, J2 = np.meshgrid(j3s, j2s)

    # --- Plot 1: J2 torque heatmap ---
    fig, ax = plt.subplots(figsize=(10, 6))
    tau_j2_grid = make_grid("tau_J2")
    im = ax.pcolormesh(J3, J2, np.abs(tau_j2_grid), cmap="hot", shading="auto")
    ax.contour(J3, J2, np.abs(tau_j2_grid), levels=[GEARBOX_RATED], colors=["cyan"],
               linewidths=2, linestyles="--")
    ax.contour(J3, J2, np.abs(tau_j2_grid), levels=[MOTOR_STALL_OUT], colors=["lime"],
               linewidths=1.5, linestyles=":")
    plt.colorbar(im, label="|Torque| (Nm)")
    ax.set_xlabel("J3 angle (deg)")
    ax.set_ylabel("J2 angle (deg)")
    ax.set_title(f"J2 Shoulder Torque (J4=0)\nCyan={GEARBOX_RATED}Nm rated, "
                 f"Green={MOTOR_STALL_OUT:.1f}Nm stall")
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS_DIR, "01_j2_torque.png"), dpi=150)
    plt.close(fig)
    print("  Saved 01_j2_torque.png", flush=True)

    # --- Plot 2: J3 torque heatmap ---
    fig, ax = plt.subplots(figsize=(10, 6))
    tau_j3_grid = make_grid("tau_J3")
    im = ax.pcolormesh(J3, J2, np.abs(tau_j3_grid), cmap="hot", shading="auto")
    ax.contour(J3, J2, np.abs(tau_j3_grid), levels=[GEARBOX_RATED], colors=["cyan"],
               linewidths=2, linestyles="--")
    plt.colorbar(im, label="|Torque| (Nm)")
    ax.set_xlabel("J3 angle (deg)")
    ax.set_ylabel("J2 angle (deg)")
    ax.set_title(f"J3 Elbow Torque (J4=0)\nCyan={GEARBOX_RATED}Nm rated")
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS_DIR, "02_j3_torque.png"), dpi=150)
    plt.close(fig)
    print("  Saved 02_j3_torque.png", flush=True)

    # --- Plot 3: Per-joint power (4 subplots) ---
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    for ax, key, title in [
        (axes[0, 0], "power_J2", "J2 Power (W)"),
        (axes[0, 1], "power_J3", "J3 Power (W)"),
        (axes[1, 0], "power_J4", "J4 Power (W)"),
        (axes[1, 1], "power_total", "Total Power (W)"),
    ]:
        grid = make_grid(key)
        im = ax.pcolormesh(J3, J2, grid, cmap="inferno", shading="auto")
        plt.colorbar(im, ax=ax, label="Watts")
        ax.set_xlabel("J3 (deg)")
        ax.set_ylabel("J2 (deg)")
        ax.set_title(title)
    fig.suptitle("Electrical Power (holding, J4=0)", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS_DIR, "03_power.png"), dpi=150)
    plt.close(fig)
    print("  Saved 03_power.png", flush=True)

    # --- Plot 4: Heat generation (same as power for holding) ---
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for ax, key, title in [
        (axes[0], "heat_J2", "J2 Heat (W)"),
        (axes[1], "heat_J3", "J3 Heat (W)"),
        (axes[2], "heat_J4", "J4 Heat (W)"),
    ]:
        grid = make_grid(key)
        im = ax.pcolormesh(J3, J2, grid, cmap="YlOrRd", shading="auto")
        plt.colorbar(im, ax=ax, label="Watts")
        ax.set_xlabel("J3 (deg)")
        ax.set_ylabel("J2 (deg)")
        ax.set_title(title)
    fig.suptitle("Heat Generation per Joint (J4=0)", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS_DIR, "04_heat.png"), dpi=150)
    plt.close(fig)
    print("  Saved 04_heat.png", flush=True)

    # --- Plot 5: Tipping moment ---
    fig, ax = plt.subplots(figsize=(10, 6))
    tip_grid = make_grid("tipping_moment")
    im = ax.pcolormesh(J3, J2, tip_grid, cmap="RdYlGn", shading="auto")
    ax.contour(J3, J2, tip_grid, levels=[0], colors=["black"], linewidths=2)
    plt.colorbar(im, label="Tipping moment (Nm)\n+stable / -tips")
    ax.set_xlabel("J3 angle (deg)")
    ax.set_ylabel("J2 angle (deg)")
    ax.set_title("Chassis Tipping Moment (J4=0)\nBlack line = tipping threshold")
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS_DIR, "05_tipping.png"), dpi=150)
    plt.close(fig)
    print("  Saved 05_tipping.png", flush=True)

    # --- Plot 6: Feasibility overlay ---
    fig, ax = plt.subplots(figsize=(10, 6))
    # Green = feasible (torque OK + stable), Red = infeasible
    feasible = np.ones_like(tau_j2_grid) * 0.0  # 0 = infeasible
    mask_torque = (np.abs(tau_j2_grid) <= GEARBOX_RATED) & (np.abs(tau_j3_grid) <= GEARBOX_RATED)
    mask_stable = tip_grid > 0
    feasible[mask_torque & mask_stable] = 2.0   # fully feasible
    feasible[mask_torque & ~mask_stable] = 1.0  # torque OK but tips
    feasible[~mask_torque & mask_stable] = 0.5  # stable but over-torque

    from matplotlib.colors import ListedColormap
    cmap = ListedColormap(["#d32f2f", "#ff9800", "#ffeb3b", "#4caf50"])
    im = ax.pcolormesh(J3, J2, feasible, cmap=cmap, vmin=0, vmax=2, shading="auto")
    cbar = plt.colorbar(im, ticks=[0.25, 0.75, 1.25, 1.75])
    cbar.ax.set_yticklabels(["Over-torque\n& tips", "Stable but\nover-torque",
                              "Torque OK\nbut tips", "Feasible"])
    ax.set_xlabel("J3 angle (deg)")
    ax.set_ylabel("J2 angle (deg)")
    ax.set_title("Feasibility Envelope (J4=0)\nNEMA 17 + Cricket MK II 25:1, Al links")
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS_DIR, "06_feasibility.png"), dpi=150)
    plt.close(fig)
    print("  Saved 06_feasibility.png", flush=True)

    print(f"\nAll plots saved to {RESULTS_DIR}/", flush=True)


# ============================================================================
# Run sweep if --sweep (no Isaac Sim)
# ============================================================================
if args.sweep:
    print("=" * 60)
    print("FORTIS Arm Torque Sweep — Analytical Mode")
    print("=" * 60)
    run_analytical_sweep()
    sys.exit(0)


# ============================================================================
# Isaac Sim GUI mode below
# ============================================================================
from isaacsim import SimulationApp
app = SimulationApp({"headless": False, "width": 1920, "height": 1080})

import omni.usd
import omni.timeline
import carb.input
import omni.appwindow
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema, Vt
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation


# ============================================================================
# Source wheel reader (copied from canonical)
# ============================================================================

def classify_part(name):
    if "Kaya_Wheel_Hub" in name: return "hub"
    if "_17_4775_001_" in name or "_17_4775_003_" in name: return "sideplate"
    if "_17_4775_004_" in name: return "roller_barrel"
    if "_17_4775_005_" in name: return "roller_cap"
    if "_17_4775_007_" in name: return "roller_pin"
    if "_17_2584_007_" in name: return "screw_large"
    if "_17_2584_009_" in name: return "screw_small"
    return "unknown"


def read_source_wheel():
    print(f"Reading wheel source: {OMNIWHEEL_USD}", flush=True)
    src = Usd.Stage.Open(OMNIWHEEL_USD)
    xfc = UsdGeom.XformCache()
    parts = {"hub": [], "sideplate": [], "roller_barrel": [], "roller_cap": [],
             "roller_pin": [], "screw_large": [], "screw_small": []}
    for prim in src.Traverse():
        if prim.GetTypeName() != "Mesh": continue
        mesh = UsdGeom.Mesh(prim)
        raw_pts = mesh.GetPointsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        if not raw_pts or len(raw_pts) == 0: continue
        wxf = xfc.GetLocalToWorldTransform(prim)
        world_pts = []
        for p in raw_pts:
            wp = wxf.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
            world_pts.append([wp[0], wp[1], wp[2]])
        cat = classify_part(prim.GetName())
        if cat == "unknown": continue
        parts[cat].append({
            "name": prim.GetName(), "points": np.array(world_pts),
            "indices": list(indices) if indices else [],
            "counts": list(counts) if counts else [],
        })
    barrel_centers = [p["points"].mean(axis=0) for p in parts["roller_barrel"]]
    wheel_center = np.mean(barrel_centers, axis=0)
    return parts, wheel_center


def create_mesh_prim(stage, path, part, src_center, sx, sy, sz,
                     body_offset=None, color=None):
    scaled = []
    for p in part["points"]:
        x = float((p[0] - src_center[0]) * sx)
        y = float((p[1] - src_center[1]) * sy)
        z = float((p[2] - src_center[2]) * sz)
        if body_offset is not None:
            x -= body_offset[0]; y -= body_offset[1]; z -= body_offset[2]
        scaled.append(Gf.Vec3f(x, y, z))
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(scaled))
    if part["indices"]: mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(part["indices"]))
    if part["counts"]: mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(part["counts"]))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    if color: mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    return mesh


# ============================================================================
# Scene builders
# ============================================================================

def build_physics_scene(stage):
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(True)
    px.CreateSolverTypeAttr("TGS")
    px.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
    print(f"Physics: {PHYSICS_HZ}Hz, CPU, TGS + CCD", flush=True)


def build_step_environment(stage):
    """Two flat platforms with a 4.5\" step between them.

    Outer platform (Z=0) at X<0 (rear wheels).
    Inner platform (Z=-STEP_HEIGHT) at X>0 (front wheels).
    """
    gs = 3.0  # half-size of each platform

    # Outer platform (higher)
    op = UsdGeom.Mesh.Define(stage, "/World/OuterFloor")
    op.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-2*gs, -gs, 0), Gf.Vec3f(0, -gs, 0),
        Gf.Vec3f(0, gs, 0), Gf.Vec3f(-2*gs, gs, 0)]))
    op.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    op.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    op.GetSubdivisionSchemeAttr().Set("none")
    op.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.6, 0.6, 0.65)]))
    UsdPhysics.CollisionAPI.Apply(op.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(op.GetPrim()).GetApproximationAttr().Set("none")

    # Inner platform (lower)
    ip = UsdGeom.Mesh.Define(stage, "/World/InnerFloor")
    ip.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(0, -gs, -STEP_HEIGHT), Gf.Vec3f(2*gs, -gs, -STEP_HEIGHT),
        Gf.Vec3f(2*gs, gs, -STEP_HEIGHT), Gf.Vec3f(0, gs, -STEP_HEIGHT)]))
    ip.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    ip.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    ip.GetSubdivisionSchemeAttr().Set("none")
    ip.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.55, 0.55, 0.6)]))
    UsdPhysics.CollisionAPI.Apply(ip.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(ip.GetPrim()).GetApproximationAttr().Set("none")

    # Step face (vertical wall)
    sf = UsdGeom.Mesh.Define(stage, "/World/StepFace")
    sf.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(0, -gs, 0), Gf.Vec3f(0, gs, 0),
        Gf.Vec3f(0, gs, -STEP_HEIGHT), Gf.Vec3f(0, -gs, -STEP_HEIGHT)]))
    sf.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    sf.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    sf.GetSubdivisionSchemeAttr().Set("none")
    sf.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.45, 0.45, 0.5)]))
    UsdPhysics.CollisionAPI.Apply(sf.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(sf.GetPrim()).GetApproximationAttr().Set("none")

    # Friction material
    gmat = UsdShade.Material.Define(stage, "/World/FloorMat")
    gpm = UsdPhysics.MaterialAPI.Apply(gmat.GetPrim())
    gpm.CreateStaticFrictionAttr(FRICTION_MU)
    gpm.CreateDynamicFrictionAttr(FRICTION_MU)
    gpm.CreateRestitutionAttr(0.05)
    for p in ["/World/OuterFloor", "/World/InnerFloor", "/World/StepFace"]:
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(p)).Bind(
            gmat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # Lighting
    dome = stage.DefinePrim("/World/DomeLight", "DomeLight")
    dome.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(800.0)
    dl = stage.DefinePrim("/World/Light", "DistantLight")
    dl.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)

    print(f"Step environment: outer Z=0, inner Z={-STEP_HEIGHT/IN:.1f}\" "
          f"({-STEP_HEIGHT*100:.1f}cm)", flush=True)


# ============================================================================
# Chassis builder (copied from canonical)
# ============================================================================

def build_arched_chassis(stage, path, half_h, color):
    arch_flat_half = ARCH_FLAT_WIDTH / 2.0
    ramp_start = arch_flat_half
    ramp_end = SL - MOTOR_MOUNT_LEN
    z_low = -half_h
    z_high = -half_h + BELLY_HEIGHT

    x_stations = sorted(set([
        -SL, -(SL - C), -ramp_end, -ramp_start, 0.0,
        ramp_start, ramp_end, (SL - C), SL,
    ]))

    def bottom_z(x):
        ax = abs(x)
        if ax <= ramp_start: return z_high
        elif ax <= ramp_end:
            t = (ax - ramp_start) / (ramp_end - ramp_start)
            return z_high + t * (z_low - z_high)
        else: return z_low

    def half_width_at(x):
        ax = abs(x)
        if ax <= (SL - C): return SW
        else:
            t = (ax - (SL - C)) / C
            return SW - t * C

    verts, face_indices, face_counts, sections = [], [], [], []
    for x in x_stations:
        hw = half_width_at(x); bz = bottom_z(x)
        idx_start = len(verts)
        verts.extend([Gf.Vec3f(x, hw, half_h), Gf.Vec3f(x, -hw, half_h),
                      Gf.Vec3f(x, -hw, bz), Gf.Vec3f(x, hw, bz)])
        sections.append((idx_start, idx_start+1, idx_start+2, idx_start+3))

    for i in range(len(sections) - 1):
        tl0,tr0,br0,bl0 = sections[i]; tl1,tr1,br1,bl1 = sections[i+1]
        face_indices.extend([tl0,tl1,tr1,tr0]); face_counts.append(4)
        face_indices.extend([bl0,br0,br1,bl1]); face_counts.append(4)
        face_indices.extend([tl0,bl0,bl1,tl1]); face_counts.append(4)
        face_indices.extend([tr0,tr1,br1,br0]); face_counts.append(4)
    tl,tr,br,bl = sections[-1]; face_indices.extend([tl,tr,br,bl]); face_counts.append(4)
    tl,tr,br,bl = sections[0]; face_indices.extend([tr,tl,bl,br]); face_counts.append(4)

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    return mesh


def build_chassis(stage, chassis_path):
    half_h = CHASSIS_H / 2.0
    chassis_mesh = build_arched_chassis(stage, chassis_path + "/body", half_h, (0.25, 0.25, 0.35))
    UsdPhysics.CollisionAPI.Apply(chassis_mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(chassis_mesh.GetPrim()).CreateApproximationAttr("convexDecomposition")
    arrow = UsdGeom.Cube.Define(stage, chassis_path + "/fwd")
    arrow.GetSizeAttr().Set(1.0)
    axf = UsdGeom.Xformable(arrow.GetPrim()); axf.ClearXformOpOrder()
    axf.AddTranslateOp().Set(Gf.Vec3d(SL * 0.6, 0, half_h + 0.003))
    axf.AddScaleOp().Set(Gf.Vec3d(SL * 0.4, SW * 0.15, 0.005))
    arrow.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.15, 0.15)]))


# ============================================================================
# Omniwheel builder (copied from canonical)
# ============================================================================

def build_omniwheel(stage, wheel_path, src_parts, src_center, wheel_mat, wname):
    hub_path = wheel_path + "/Hub"
    hub_xf = UsdGeom.Xform.Define(stage, hub_path)
    UsdPhysics.RigidBodyAPI.Apply(hub_xf.GetPrim())
    UsdPhysics.MassAPI.Apply(hub_xf.GetPrim()).CreateMassAttr(HUB_MASS)

    hub_color = (0.3, 0.3, 0.35)
    mi = 0
    for cat in ["hub", "sideplate", "screw_large", "screw_small"]:
        for part in src_parts[cat]:
            create_mesh_prim(stage, f"{hub_path}/vis_{mi}", part, src_center,
                             SCALE_XZ, SCALE_Y, SCALE_XZ, color=hub_color)
            mi += 1

    cap_data = [(p, p["points"].mean(axis=0)) for p in src_parts["roller_cap"]]
    pin_data = [(p, p["points"].mean(axis=0)) for p in src_parts["roller_pin"]]
    used_caps, used_pins = set(), set()
    roller_color = (0.15, 0.15, 0.15)

    all_scaled_centers = []
    for barrel in src_parts["roller_barrel"]:
        bc = barrel["points"].mean(axis=0)
        sc = np.array([float((bc[0]-src_center[0])*SCALE_XZ),
                       float((bc[1]-src_center[1])*SCALE_Y),
                       float((bc[2]-src_center[2])*SCALE_XZ)])
        all_scaled_centers.append(sc)

    min_pair_dist = float('inf')
    for i in range(len(all_scaled_centers)):
        for j in range(i+1, len(all_scaled_centers)):
            d = float(np.linalg.norm(all_scaled_centers[i] - all_scaled_centers[j]))
            if d < min_pair_dist: min_pair_dist = d
    safe_roller_r = min_pair_dist / 2.0 - 0.003
    max_surface_r = float(WHEEL_RADIUS - np.sqrt(
        all_scaled_centers[0][0]**2 + all_scaled_centers[0][2]**2))
    roller_coll_r_global = max(min(safe_roller_r, max_surface_r), 0.005)

    for ri, barrel in enumerate(src_parts["roller_barrel"]):
        bc = barrel["points"].mean(axis=0)
        scaled_center = all_scaled_centers[ri]
        rp = wheel_path + f"/Roller_{ri}"
        rxf = UsdGeom.Xform.Define(stage, rp)
        rprim = rxf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(rprim)
        UsdPhysics.MassAPI.Apply(rprim).CreateMassAttr(ROLLER_MASS)
        prb = PhysxSchema.PhysxRigidBodyAPI.Apply(rprim)
        prb.CreateSolverPositionIterationCountAttr(16)
        prb.CreateMaxDepenetrationVelocityAttr(5.0)
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(
            float(scaled_center[0]), float(scaled_center[1]), float(scaled_center[2])))

        bmp = rp + "/collider"
        sphere = UsdGeom.Sphere.Define(stage, bmp)
        sphere.GetRadiusAttr().Set(roller_coll_r_global)
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(
            wheel_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        UsdGeom.Imageable(sphere.GetPrim()).CreatePurposeAttr("guide")
        pcoll = PhysxSchema.PhysxCollisionAPI.Apply(sphere.GetPrim())
        pcoll.CreateContactOffsetAttr(0.001)
        pcoll.CreateRestOffsetAttr(0.0005)

        bvp = rp + "/barrel_vis"
        create_mesh_prim(stage, bvp, barrel, src_center,
                         SCALE_UNIFORM, SCALE_UNIFORM, SCALE_UNIFORM,
                         body_offset=scaled_center, color=roller_color)

        for label, data_list, used_set in [("cap", cap_data, used_caps),
                                            ("pin", pin_data, used_pins)]:
            best_idx, best_dist = -1, float('inf')
            for idx, (part, center) in enumerate(data_list):
                if idx in used_set: continue
                d = np.linalg.norm(bc - center)
                if d < best_dist: best_dist = d; best_idx = idx
            if best_idx >= 0:
                used_set.add(best_idx)
                part = data_list[best_idx][0]
                create_mesh_prim(stage, rp + f"/{label}_vis", part, src_center,
                                 SCALE_UNIFORM, SCALE_UNIFORM, SCALE_UNIFORM,
                                 body_offset=scaled_center, color=hub_color)

        dx = bc[0] - src_center[0]; dz = bc[2] - src_center[2]
        theta = math.atan2(dz, dx)
        jp = rp + "/joint"
        joint = UsdPhysics.RevoluteJoint.Define(stage, jp)
        joint.CreateBody0Rel().SetTargets([hub_path])
        joint.CreateBody1Rel().SetTargets([rp])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(
            float(scaled_center[0]), float(scaled_center[1]), float(scaled_center[2])))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        half_a = float(-theta / 2.0)
        qy = Gf.Quatf(float(math.cos(half_a)), 0.0, float(math.sin(half_a)), 0.0)
        joint.CreateLocalRot0Attr().Set(qy)
        joint.CreateLocalRot1Attr().Set(qy)
        joint.CreateAxisAttr("Z")
        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jp), "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.001)

    return hub_path


# ============================================================================
# Arm builder — aluminum links, camera at mid-L3
# ============================================================================

def _add_arm_link_visual(stage, path, center, size, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    cxf = UsdGeom.Xformable(cube.GetPrim()); cxf.ClearXformOpOrder()
    cxf.AddTranslateOp().Set(Gf.Vec3d(*center))
    cxf.AddScaleOp().Set(Gf.Vec3d(*size))
    cube.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))


def _apply_mass_box_vertical(prim, mass, com_z, length_z):
    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(mass))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, float(com_z)))
    Lx, Ly, Lz = LINK_BOX_T, LINK_BOX_W, float(length_z)
    Ixx = mass/12.0*(Ly*Ly + Lz*Lz)
    Iyy = mass/12.0*(Lx*Lx + Lz*Lz)
    Izz = mass/12.0*(Lx*Lx + Ly*Ly)
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(float(Ixx), float(Iyy), float(Izz)))


def _apply_mass_box_horizontal_x(prim, mass, length_x, dir_sign):
    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(mass))
    com_x = dir_sign * length_x / 2.0
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(float(com_x), 0, 0))
    Lx, Ly, Lz = float(length_x), LINK_BOX_W, LINK_BOX_T
    Ixx = mass/12.0*(Ly*Ly + Lz*Lz)
    Iyy = mass/12.0*(Lx*Lx + Lz*Lz)
    Izz = mass/12.0*(Lx*Lx + Ly*Ly)
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(float(Ixx), float(Iyy), float(Izz)))


def _apply_mass_composite_x(prim, total_mass, com_x, Ixx, Iyy, Izz):
    """Set mass, COM, and inertia directly for a composite body."""
    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(total_mass))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(float(com_x), 0, 0))
    mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(float(Ixx), float(Iyy), float(Izz)))


def _compute_j3_mass_properties(dir_sign):
    """Compute COM and inertia for J3 body: motor at pivot + distributed L3 + camera at mid-L3.

    dir_sign: +1 if link extends +X, -1 if -X.
    Returns (total_mass, com_x, Ixx, Iyy, Izz).
    """
    # Positions along X axis (in body frame, 0 = J3 pivot)
    x_motor = 0.0                         # motor+gearbox at pivot
    x_link_com = dir_sign * L_L3 / 2.0    # link COM at midpoint
    x_camera = dir_sign * L_L3 / 2.0      # camera at mid-L3

    total = M_J3_ELBOW  # = motor + link + camera + misc
    com_x = (M_J3_MOTOR * x_motor + M_J3_LINK * x_link_com +
             M_J3_CAMERA * x_camera) / total

    # Inertia about composite COM using parallel axis theorem
    W, T = LINK_BOX_W, LINK_BOX_T
    # Link (uniform box along X)
    Ixx_link = M_J3_LINK/12.0 * (W*W + T*T)
    Iyy_link = M_J3_LINK/12.0 * (L_L3*L_L3 + T*T)
    Izz_link = M_J3_LINK/12.0 * (L_L3*L_L3 + W*W)
    d_link = x_link_com - com_x
    Iyy_link += M_J3_LINK * d_link * d_link
    Izz_link += M_J3_LINK * d_link * d_link

    # Motor (point mass at pivot)
    d_motor = x_motor - com_x
    Iyy_motor = M_J3_MOTOR * d_motor * d_motor
    Izz_motor = M_J3_MOTOR * d_motor * d_motor

    # Camera (point mass at mid-L3)
    d_cam = x_camera - com_x
    Iyy_cam = M_J3_CAMERA * d_cam * d_cam
    Izz_cam = M_J3_CAMERA * d_cam * d_cam

    Ixx = Ixx_link  # transverse — point masses on X axis don't contribute
    Iyy = Iyy_link + Iyy_motor + Iyy_cam
    Izz = Izz_link + Izz_motor + Izz_cam

    return total, com_x, Ixx, Iyy, Izz


def _add_arm_joint(stage, jpath, body0, body1, localPos0, axis, limit_deg):
    joint = UsdPhysics.RevoluteJoint.Define(stage, jpath)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(x) for x in localPos0]))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    joint.CreateAxisAttr(axis)
    lo, hi = limit_deg
    joint.CreateLowerLimitAttr(float(lo))
    joint.CreateUpperLimitAttr(float(hi))
    drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jpath), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(ARM_STIFFNESS)
    drive.CreateDampingAttr(ARM_DAMPING)
    drive.CreateMaxForceAttr(ARM_MAX_FORCE)
    drive.CreateTargetPositionAttr(0.0)


def build_arm(stage, robot_path, chassis_path):
    """Build 4-DOF arm: aluminum links, camera at mid-L3, NEMA 17 + Cricket MK II."""
    j1_base = robot_path + "/ArmJ1base"
    j2_shd  = robot_path + "/ArmJ2shoulder"
    j3_elb  = robot_path + "/ArmJ3elbow"
    j4_wrs  = robot_path + "/ArmJ4wrist"

    z_top_j1  = ARM_MOUNT_Z + L_J1
    z_l3_body = z_top_j1 + VIS_Z_STACK
    z_l4_body = z_top_j1 + 2.0 * VIS_Z_STACK

    # ---- J1 base (vertical 5" motor stack) ----
    UsdGeom.Xform.Define(stage, j1_base)
    p1 = stage.GetPrimAtPath(j1_base)
    xf1 = UsdGeom.Xformable(p1); xf1.ClearXformOpOrder()
    xf1.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X, ARM_MOUNT_Y, ARM_MOUNT_Z))
    UsdPhysics.RigidBodyAPI.Apply(p1)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p1).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_mass_box_vertical(p1, M_J1_BASE, L_J1/2.0, L_J1)
    _add_arm_link_visual(stage, j1_base + "/vis",
                         (0, 0, L_J1/2.0), (0.050, 0.050, L_J1), (0.55, 0.55, 0.60))

    # ---- J2 shoulder (L2 22.68", horizontal +X) ----
    UsdGeom.Xform.Define(stage, j2_shd)
    p2 = stage.GetPrimAtPath(j2_shd)
    xf2 = UsdGeom.Xformable(p2); xf2.ClearXformOpOrder()
    xf2.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X, ARM_MOUNT_Y, z_top_j1))
    UsdPhysics.RigidBodyAPI.Apply(p2)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p2).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_mass_box_horizontal_x(p2, M_J2_SHOULDER, L_L2, +1)
    _add_arm_link_visual(stage, j2_shd + "/vis",
                         (+L_L2/2.0, 0, 0), (L_L2, LINK_BOX_W, LINK_BOX_T), (0.85, 0.30, 0.25))

    # ---- J3 elbow (L3 19.69", horizontal -X, camera at mid-L3) ----
    UsdGeom.Xform.Define(stage, j3_elb)
    p3 = stage.GetPrimAtPath(j3_elb)
    xf3 = UsdGeom.Xformable(p3); xf3.ClearXformOpOrder()
    xf3.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X + L_L2, ARM_MOUNT_Y, z_l3_body))
    UsdPhysics.RigidBodyAPI.Apply(p3)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p3).CreateMaxDepenetrationVelocityAttr(1.0)
    # Composite mass: motor at pivot + distributed link + camera at midpoint
    total, com_x, Ixx, Iyy, Izz = _compute_j3_mass_properties(dir_sign=-1)
    _apply_mass_composite_x(p3, total, com_x, Ixx, Iyy, Izz)
    _add_arm_link_visual(stage, j3_elb + "/vis",
                         (-L_L3/2.0, 0, 0), (L_L3, LINK_BOX_W, LINK_BOX_T), (0.25, 0.75, 0.30))
    # Camera visual at mid-L3
    cam = UsdGeom.Sphere.Define(stage, j3_elb + "/camera")
    cam.GetRadiusAttr().Set(0.035)
    cxf = UsdGeom.Xformable(cam.GetPrim()); cxf.ClearXformOpOrder()
    cxf.AddTranslateOp().Set(Gf.Vec3d(-L_L3/2.0, 0, 0))
    cam.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.05, 0.05, 0.05)]))

    # ---- J4 wrist (L4 5.91", horizontal +X) ----
    UsdGeom.Xform.Define(stage, j4_wrs)
    p4 = stage.GetPrimAtPath(j4_wrs)
    xf4 = UsdGeom.Xformable(p4); xf4.ClearXformOpOrder()
    xf4.AddTranslateOp().Set(Gf.Vec3d(ARM_MOUNT_X + L_L2 - L_L3, ARM_MOUNT_Y, z_l4_body))
    UsdPhysics.RigidBodyAPI.Apply(p4)
    PhysxSchema.PhysxRigidBodyAPI.Apply(p4).CreateMaxDepenetrationVelocityAttr(1.0)
    _apply_mass_box_horizontal_x(p4, M_J4_WRIST, L_L4, +1)
    _add_arm_link_visual(stage, j4_wrs + "/vis",
                         (+L_L4/2.0, 0, 0), (L_L4, LINK_BOX_W, LINK_BOX_T), (0.25, 0.30, 0.85))

    # ---- Joints ----
    _add_arm_joint(stage, robot_path + "/ArmJ1", chassis_path, j1_base,
                   (ARM_MOUNT_X, ARM_MOUNT_Y, ARM_MOUNT_Z), "Z", ARM_LIMITS_DEG["ArmJ1"])
    _add_arm_joint(stage, robot_path + "/ArmJ2", j1_base, j2_shd,
                   (0, 0, L_J1), "Y", ARM_LIMITS_DEG["ArmJ2"])
    _add_arm_joint(stage, robot_path + "/ArmJ3", j2_shd, j3_elb,
                   (L_L2, 0, VIS_Z_STACK), "Y", ARM_LIMITS_DEG["ArmJ3"])
    _add_arm_joint(stage, robot_path + "/ArmJ4", j3_elb, j4_wrs,
                   (-L_L3, 0, VIS_Z_STACK), "Y", ARM_LIMITS_DEG["ArmJ4"])

    return ["ArmJ1", "ArmJ2", "ArmJ3", "ArmJ4"]


# ============================================================================
# Robot builder (chassis + wheels + arm, spawn on step)
# ============================================================================

def build_robot(stage, src_parts, src_center):
    robot_path = "/World/Robot"
    chassis_path = robot_path + "/Chassis"

    UsdGeom.Xform.Define(stage, robot_path)
    UsdGeom.Xform.Define(stage, chassis_path)

    cp = stage.GetPrimAtPath(chassis_path)
    UsdPhysics.RigidBodyAPI.Apply(cp)
    UsdPhysics.MassAPI.Apply(cp).CreateMassAttr(CHASSIS_MASS)
    UsdPhysics.MassAPI(cp).CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    UsdPhysics.ArticulationRootAPI.Apply(cp)
    physx_art = PhysxSchema.PhysxArticulationAPI.Apply(cp)
    physx_art.CreateEnabledSelfCollisionsAttr(False)

    build_chassis(stage, chassis_path)

    wmat = UsdShade.Material.Define(stage, "/World/WheelMat")
    wpm = UsdPhysics.MaterialAPI.Apply(wmat.GetPrim())
    wpm.CreateStaticFrictionAttr(FRICTION_MU)
    wpm.CreateDynamicFrictionAttr(FRICTION_MU)
    wpm.CreateRestitutionAttr(0.1)
    wheel_mat = UsdShade.Material(stage.GetPrimAtPath("/World/WheelMat"))

    wheel_z = WHEEL_RADIUS - (BELLY_HEIGHT + CHASSIS_H / 2.0)
    wheel_offset = WHEEL_WIDTH / 2.0 + 0.04

    drive_joint_paths = []
    for wname in WHEEL_ORDER:
        cx, cy = WHEEL_XY[wname]
        angle_deg = AXLE_ANGLES[wname]
        angle_rad = math.radians(angle_deg)
        norm_len = math.sqrt(cx*cx + cy*cy)
        nx, ny = cx/norm_len, cy/norm_len
        wx = cx + nx * wheel_offset
        wy = cy + ny * wheel_offset

        wp = robot_path + f"/Wheel_{wname}"
        wxf = UsdGeom.Xform.Define(stage, wp)
        wxf.ClearXformOpOrder()
        wxf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wheel_z))
        wxf.AddRotateZOp().Set(float(angle_deg))

        hub_path = build_omniwheel(stage, wp, src_parts, src_center, wheel_mat, wname)

        djp = robot_path + f"/DriveJoint_{wname}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, djp)
        joint.CreateBody0Rel().SetTargets([chassis_path])
        joint.CreateBody1Rel().SetTargets([hub_path])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(float(wx), float(wy), float(wheel_z)))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        half_a = angle_rad / 2.0
        rot_q = Gf.Quatf(float(math.cos(half_a)), 0.0, 0.0, float(math.sin(half_a)))
        joint.CreateLocalRot0Attr().Set(rot_q)
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateAxisAttr("Y")
        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(djp), "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(DRIVE_DAMPING)
        drive.CreateMaxForceAttr(DRIVE_MAX_FORCE)
        drive.CreateTargetVelocityAttr(0.0)
        jprim = stage.GetPrimAtPath(djp)
        jprim.CreateAttribute("isaacmecanumwheel:radius",
                              Sdf.ValueTypeNames.Float).Set(float(WHEEL_RADIUS))
        jprim.CreateAttribute("isaacmecanumwheel:angle",
                              Sdf.ValueTypeNames.Float).Set(90.0)
        drive_joint_paths.append(djp)

    arm_joint_names = build_arm(stage, robot_path, chassis_path)

    # Spawn: chassis center at step edge (X=0), elevated to let it settle.
    # Front wheels (+X) go onto inner floor (lower), rear wheels (-X) on outer (Z=0).
    spawn_z = WHEEL_RADIUS + CHASSIS_H / 2.0 + 0.10  # above outer floor + buffer
    rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
    rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, spawn_z))

    total_arm = M_J1_BASE + M_J2_SHOULDER + M_J3_ELBOW + M_J4_WRIST
    print(f"\nSpawn: (0, 0, {spawn_z:.4f}m) on step", flush=True)
    print(f"Chassis: {CHASSIS_L/IN:.1f}x{CHASSIS_W/IN:.1f}x{CHASSIS_H/IN:.1f}\", "
          f"{CHASSIS_MASS}kg", flush=True)
    print(f"Arm: {total_arm:.3f} kg ({total_arm*2.205:.2f} lbs) — "
          f"all NEMA 17 + Cricket MK II, Al links, cam@mid-L3", flush=True)

    return robot_path, chassis_path, drive_joint_paths, arm_joint_names, spawn_z


# ============================================================================
# X-drive IK + keyboard
# ============================================================================

def xdrive_ik(vx, vy, omega):
    r = WHEEL_RADIUS
    vels = []
    for wn in WHEEL_ORDER:
        wx, wy = WHEEL_XY[wn]
        vwx = vx - omega * wy
        vwy = vy + omega * wx
        angle_rad = math.radians(AXLE_ANGLES[wn])
        v_drive = vwx * math.cos(angle_rad) + vwy * math.sin(angle_rad)
        vels.append(v_drive / r)
    return np.array(vels)


class KB:
    def __init__(self):
        self.speed = DRIVE_SPEED
        self.rspeed = ROTATE_SPEED
        self.arm_speed = 0.5
        self.arm_home = False
        self.reset = self.pstate = self.stop = False
        self._keys = set()
        self._inp = carb.input.acquire_input_interface()
        self._kb = omni.appwindow.get_default_app_window().get_keyboard()
        self._sub = self._inp.subscribe_to_keyboard_events(self._kb, self._on)

    def _on(self, ev, *a, **kw):
        k = ev.input; K = carb.input.KeyboardInput
        if ev.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._keys.add(k)
            if k == K.R: self.reset = True
            elif k == K.P: self.pstate = True
            elif k == K.SPACE: self.stop = True
            elif k == K.H: self.arm_home = True
            elif k == K.EQUAL: self.speed = min(self.speed + 0.05, 1.0)
            elif k == K.MINUS: self.speed = max(self.speed - 0.05, 0.05)
            elif k == K.RIGHT_BRACKET: self.arm_speed = min(self.arm_speed + 0.1, 2.0)
            elif k == K.LEFT_BRACKET: self.arm_speed = max(self.arm_speed - 0.1, 0.1)
        elif ev.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._keys.discard(k)
        return True

    def cmd(self):
        K = carb.input.KeyboardInput
        vx = self.speed * (int(K.UP in self._keys) - int(K.DOWN in self._keys))
        vy = self.speed * (int(K.LEFT in self._keys) - int(K.RIGHT in self._keys))
        w = self.rspeed * (int(K.Q in self._keys) - int(K.E in self._keys))
        return vx, vy, w

    def arm_delta(self, dt):
        K = carb.input.KeyboardInput
        d = np.zeros(4, dtype=np.float64)
        d[0] = int(K.KEY_2 in self._keys) - int(K.KEY_1 in self._keys)
        d[1] = int(K.KEY_4 in self._keys) - int(K.KEY_3 in self._keys)
        d[2] = int(K.KEY_6 in self._keys) - int(K.KEY_5 in self._keys)
        d[3] = int(K.KEY_8 in self._keys) - int(K.KEY_7 in self._keys)
        return d * self.arm_speed * dt


# ============================================================================
# MAIN — GUI mode
# ============================================================================

print("=" * 60)
print("FORTIS Arm Torque Sweep — GUI Mode")
print("All NEMA 17 + Cricket MK II 25:1, 0.25\" Al links, cam@mid-L3")
print("=" * 60)

src_parts, src_center = read_source_wheel()

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(10): app.update()
stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

build_physics_scene(stage)
build_step_environment(stage)
robot_path, chassis_path, drive_joint_paths, arm_joint_names, spawn_z = build_robot(
    stage, src_parts, src_center)

for _ in range(20): app.update()

world = World(stage_units_in_meters=1.0)
omni.timeline.get_timeline_interface().play()
for _ in range(10): app.update()

art = Articulation(chassis_path)
art.initialize()
for _ in range(10): world.step(render=True)

if not art.is_physics_handle_valid():
    print("FATAL: Articulation physics handle invalid", flush=True)
    app.close(); sys.exit(1)

ndof = art.num_dof
dof_names = art.dof_names
print(f"\nArticulation: {ndof} DOFs", flush=True)

# Find drive + arm DOF indices
drive_dof_indices = []
for djp in drive_joint_paths:
    jn = djp.split("/")[-1]
    for i, dn in enumerate(dof_names):
        if jn in dn or dn == jn:
            drive_dof_indices.append(i); break
if len(drive_dof_indices) != 4:
    drive_dof_indices = []
    for i, dn in enumerate(dof_names):
        if "Drive" in dn: drive_dof_indices.append(i)
    if len(drive_dof_indices) != 4: drive_dof_indices = list(range(4))

arm_dof_indices = []
for name in arm_joint_names:
    for i, dn in enumerate(dof_names):
        if dn == name or name in dn:
            arm_dof_indices.append(i); break
    else:
        print(f"FATAL: arm joint {name} not in DOF list", flush=True)
        app.close(); sys.exit(1)
arm_idx_np = np.array(arm_dof_indices, dtype=np.int64)
print(f"Drive DOFs: {drive_dof_indices}", flush=True)
print(f"Arm DOFs: {arm_dof_indices} -> {[dof_names[i] for i in arm_dof_indices]}", flush=True)

art.switch_control_mode("velocity", joint_indices=np.arange(ndof))
art.switch_control_mode("position", joint_indices=arm_idx_np)

arm_targets = np.zeros(4, dtype=np.float64)

def _apply_arm_targets():
    if art.is_physics_handle_valid():
        art.set_joint_position_targets(arm_targets.reshape(1, -1), joint_indices=arm_idx_np)

_apply_arm_targets()

# Settle
print("Settling 2s...", flush=True)
for i in range(2 * PHYSICS_HZ):
    _apply_arm_targets()
    world.step(render=True)

arm_lim_rad = np.array([
    [math.radians(ARM_LIMITS_DEG[n][0]), math.radians(ARM_LIMITS_DEG[n][1])]
    for n in arm_joint_names
])

print("\nControls:", flush=True)
print("  Drive:  Arrows=translate  Q/E=rotate  +/-=speed", flush=True)
print("  Arm:    1/2=J1  3/4=J2  5/6=J3  7/8=J4  H=home  [/]=arm speed", flush=True)
print("  P=print state  R=reset  Space=stop", flush=True)
print("  Live torque readout every 2s", flush=True)

kb = KB()
spawn_pos = [0.0, 0.0, spawn_z]
dt = 1.0 / PHYSICS_HZ
frame = 0

while app.is_running():
    vx, vy, w = kb.cmd()

    if kb.stop:
        kb.stop = False
        art.set_joint_velocity_targets(np.zeros((1, ndof)))
        vx = vy = w = 0.0

    if kb.reset:
        kb.reset = False
        omni.timeline.get_timeline_interface().stop()
        for _ in range(5): app.update()
        rxf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path))
        rxf.ClearXformOpOrder()
        rxf.AddTranslateOp().Set(Gf.Vec3d(*spawn_pos))
        omni.timeline.get_timeline_interface().play()
        for _ in range(10): app.update()
        art = Articulation(chassis_path)
        art.initialize()
        for _ in range(10): world.step(render=True)
        art.switch_control_mode("velocity", joint_indices=np.arange(ndof))
        art.switch_control_mode("position", joint_indices=arm_idx_np)
        arm_targets[:] = 0.0
        _apply_arm_targets()
        print("RESET", flush=True)
        continue

    if kb.arm_home:
        kb.arm_home = False
        arm_targets[:] = 0.0
        print("Arm HOME -> stowed", flush=True)

    arm_targets += kb.arm_delta(dt)
    arm_targets = np.clip(arm_targets, arm_lim_rad[:, 0], arm_lim_rad[:, 1])

    tv = xdrive_ik(vx, vy, w)
    va = np.zeros(ndof)
    for ii, di in enumerate(drive_dof_indices):
        va[di] = tv[ii]
    if art.is_physics_handle_valid():
        art.set_joint_velocity_targets(va.reshape(1, -1))
        _apply_arm_targets()

    world.step(render=True)
    frame += 1

    if kb.pstate:
        kb.pstate = False
        try:
            jp = art.get_joint_positions().flatten()
            arm_q = [math.degrees(jp[i]) for i in arm_dof_indices]
            print(f"\nArm angles (deg): J1={arm_q[0]:+.1f} J2={arm_q[1]:+.1f} "
                  f"J3={arm_q[2]:+.1f} J4={arm_q[3]:+.1f}", flush=True)
        except: pass

    # Live torque readout every 2 seconds
    if frame % (2 * PHYSICS_HZ) == 0:
        try:
            jp = art.get_joint_positions().flatten()
            arm_q_deg = [math.degrees(jp[i]) for i in arm_dof_indices]
            torques = analytical_gravity_torques(arm_q_deg[1], arm_q_deg[2], arm_q_deg[3])
            powers = {k: electrical_power(v) for k, v in torques.items()}
            tip = tipping_moment(arm_q_deg[1], arm_q_deg[2], arm_q_deg[3])

            j2_ok = "OK" if abs(torques["J2"]) <= GEARBOX_RATED else "OVER"
            j3_ok = "OK" if abs(torques["J3"]) <= GEARBOX_RATED else "OVER"
            tip_ok = "stable" if tip > 0 else "TIPS"

            print(f"[{frame/PHYSICS_HZ:.0f}s] "
                  f"q=[{arm_q_deg[0]:+.0f},{arm_q_deg[1]:+.0f},"
                  f"{arm_q_deg[2]:+.0f},{arm_q_deg[3]:+.0f}] "
                  f"tau: J2={torques['J2']:.1f}Nm({j2_ok}) "
                  f"J3={torques['J3']:.1f}Nm({j3_ok}) "
                  f"J4={torques['J4']:.2f}Nm "
                  f"P={sum(powers.values()):.1f}W "
                  f"tip={tip:.1f}Nm({tip_ok})", flush=True)
        except Exception as e:
            print(f"[{frame/PHYSICS_HZ:.0f}s] readout error: {e}", flush=True)

app.close()
