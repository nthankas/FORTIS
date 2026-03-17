"""
Analytical clearance sweep -- no simulation needed.

Checks which chassis configs clear the 4.5" step, the R0 lip, and fit
the 15.75" access tunnel. Pure geometry, runs in regular Python.

Usage: python clearance_sweep.py
"""
import math

IN = 0.0254  # only used for metric conversion, all math in inches

# Fixed constraints
TUNNEL_SIZE = 14.75      # usable tunnel opening (15.75 - 0.5 buffer per side)
STEP_HEIGHT = 4.5        # step between inner and outer floor
CHASSIS_W = 9.0          # fixed width
CHAMFER = 3.0            # chamfer face length
CHAMFER_CUT = CHAMFER / math.sqrt(2)  # 2.121" cut depth

# R0 port entry geometry (approximate)
# The robot drives over a curved lip at the R0 port.
# The lip radius of curvature and height determine the clearance needed.
# Conservative estimate: treat as a step of ~3" height with a ~6" radius curve.
R0_LIP_HEIGHT = 3.0      # effective height of the R0 entry lip
R0_LIP_RADIUS = 6.0      # radius of curvature at the lip

# Sweep ranges
CHASSIS_LENGTHS = [12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0]
CHASSIS_HEIGHTS = [3.0, 4.0, 5.0, 6.0]
WHEEL_RADII = [3.0, 3.54, 4.0, 4.5]  # inches (3.54 = 90mm Rotacaster)
WHEEL_DROPS = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]  # inches below chassis center
WHEEL_WIDTH = 2.5        # inches


def compute_clearances(chassis_l, chassis_h, wheel_r, wheel_drop):
    half_h = chassis_h / 2.0

    # Wheelbase: distance between front and rear wheel centers
    sl = chassis_l / 2.0
    # Wheel X position = SL - CHAMFER_CUT/2 (midpoint of chamfer in X)
    wheel_x = sl - CHAMFER_CUT / 2.0
    wheelbase = 2.0 * wheel_x

    # Track width: distance between left and right wheel centers
    sw = CHASSIS_W / 2.0
    wheel_y = sw - CHAMFER_CUT / 2.0
    track = 2.0 * wheel_y

    # Flat ground clearance
    # Chassis center height = wheel_drop + wheel_r
    # Chassis bottom = chassis_center - half_h
    flat_clearance = wheel_drop + wheel_r - half_h

    # Step straddling clearance
    # Front wheels on lower floor, rear on upper (4.5" higher)
    tilt_rad = math.atan2(STEP_HEIGHT, wheelbase)
    tilt_deg = math.degrees(tilt_rad)
    # Chassis center Z at step midpoint (relative to lower floor)
    mid_z = wheel_r + wheel_drop + STEP_HEIGHT / 2.0
    # Chassis bottom at step edge
    chassis_bottom_at_step = mid_z - half_h * math.cos(tilt_rad)
    step_clearance = chassis_bottom_at_step - STEP_HEIGHT

    # R0 entry clearance
    # When driving over the R0 lip, the chassis pitches.
    # Approximate: same geometry as step but with R0_LIP_HEIGHT
    r0_tilt_rad = math.atan2(R0_LIP_HEIGHT, wheelbase)
    r0_mid_z = wheel_r + wheel_drop + R0_LIP_HEIGHT / 2.0
    r0_chassis_bottom = r0_mid_z - half_h * math.cos(r0_tilt_rad)
    r0_clearance = r0_chassis_bottom - R0_LIP_HEIGHT

    # Stowed dimensions (for tunnel fit)
    # Total height when stowed: chassis_h + wheel_drop (wheels fold up?) or
    # chassis_h + 2*wheel_drop if wheels stay extended
    # Width: CHASSIS_W + 2 * wheel protrusion
    # The wheels protrude outward from the chamfer face by wheel_width/2
    stowed_h = chassis_h + wheel_drop + wheel_r  # from wheel bottom to chassis top
    # Actually: stowed height depends on orientation. Robot enters on its side or flat.
    # If flat: height = chassis_h/2 + wheel_drop + wheel_r (bottom of wheel to top of chassis)
    # Actually just: total height from ground to chassis top = wheel_r + wheel_drop + half_h
    stowed_height = wheel_r + wheel_drop + half_h  # if entering flat
    # If wheels retract or fold: stowed_height = chassis_h
    # Width with wheels: CHASSIS_W + 2*(wheel_width/2 + small_gap) ≈ CHASSIS_W + WHEEL_WIDTH + 0.25
    stowed_width = CHASSIS_W + WHEEL_WIDTH + 0.25

    # For tunnel: need max(stowed_height, stowed_width) < TUNNEL_SIZE
    # But robot might enter on its side. Check both orientations.
    fits_tunnel_flat = (stowed_height <= TUNNEL_SIZE) and (stowed_width <= TUNNEL_SIZE)
    fits_tunnel_side = (stowed_width <= TUNNEL_SIZE) and (chassis_l <= TUNNEL_SIZE)

    return {
        "chassis_l": chassis_l,
        "chassis_h": chassis_h,
        "wheel_r": wheel_r,
        "wheel_drop": wheel_drop,
        "wheelbase": wheelbase,
        "track": track,
        "flat_clearance": flat_clearance,
        "step_clearance": step_clearance,
        "step_tilt_deg": tilt_deg,
        "r0_clearance": r0_clearance,
        "r0_tilt_deg": math.degrees(r0_tilt_rad),
        "stowed_height": stowed_height,
        "stowed_width": stowed_width,
        "fits_tunnel": fits_tunnel_flat or fits_tunnel_side,
    }


print("=" * 100)
print("FORTIS X-Drive Clearance Sweep")
print("=" * 100)
print(f"Fixed: width={CHASSIS_W}\", chamfer={CHAMFER}\", step={STEP_HEIGHT}\", "
      f"R0 lip={R0_LIP_HEIGHT}\", tunnel={TUNNEL_SIZE}\"")
print()

# Find all viable configurations
viable = []
all_results = []

for cl in CHASSIS_LENGTHS:
    for ch in CHASSIS_HEIGHTS:
        for wr in WHEEL_RADII:
            for wd in WHEEL_DROPS:
                r = compute_clearances(cl, ch, wr, wd)
                all_results.append(r)

                if (r["flat_clearance"] >= 0.25 and
                    r["step_clearance"] >= 0.25 and
                    r["r0_clearance"] >= 0.25 and
                    r["fits_tunnel"]):
                    viable.append(r)

print(f"Total configurations tested: {len(all_results)}")
print(f"Viable configurations: {len(viable)}")
print()

if viable:
    # Sort by: most internal volume (L * W * H), then by least wheel drop
    viable.sort(key=lambda r: (-r["chassis_l"] * CHASSIS_W * r["chassis_h"], r["wheel_drop"]))

    print(f"{'L':>4} {'H':>4} {'Wr':>5} {'Drop':>5} | {'Flat':>5} {'Step':>5} {'R0':>5} | "
          f"{'Tilt':>5} {'WBase':>5} | {'StowH':>5} {'StowW':>5} {'Fit':>3}")
    print("-" * 90)

    shown = set()
    for r in viable[:40]:
        key = (r["chassis_l"], r["chassis_h"], r["wheel_r"])
        if key in shown:
            continue
        shown.add(key)

        print(f"{r['chassis_l']:4.0f} {r['chassis_h']:4.0f} {r['wheel_r']:5.2f} {r['wheel_drop']:5.1f} | "
              f"{r['flat_clearance']:5.2f} {r['step_clearance']:5.2f} {r['r0_clearance']:5.2f} | "
              f"{r['step_tilt_deg']:5.1f} {r['wheelbase']:5.1f} | "
              f"{r['stowed_height']:5.1f} {r['stowed_width']:5.1f} {'Y' if r['fits_tunnel'] else 'N':>3}")

    # Best configs for the 15" chassis specifically
    print(f"\n--- Best configs for L=15\" specifically ---")
    best_15 = [r for r in viable if r["chassis_l"] == 15.0]
    best_15.sort(key=lambda r: (r["wheel_drop"], -r["chassis_h"]))

    for r in best_15[:10]:
        print(f"  H={r['chassis_h']:.0f}\" Wr={r['wheel_r']:.2f}\" Drop={r['wheel_drop']:.1f}\" | "
              f"Flat={r['flat_clearance']:.2f}\" Step={r['step_clearance']:.2f}\" R0={r['r0_clearance']:.2f}\" | "
              f"Tilt={r['step_tilt_deg']:.1f}° WBase={r['wheelbase']:.1f}\" | "
              f"Stowed={r['stowed_height']:.1f}\"×{r['stowed_width']:.1f}\"")

else:
    print("No viable configurations found! Relaxing constraints...")
    # Show closest misses
    all_results.sort(key=lambda r: min(r["flat_clearance"], r["step_clearance"], r["r0_clearance"]),
                     reverse=True)
    print(f"\nTop 20 closest (sorted by worst clearance):")
    print(f"{'L':>4} {'H':>4} {'Wr':>5} {'Drop':>5} | {'Flat':>5} {'Step':>5} {'R0':>5} | {'Fit':>3}")
    print("-" * 70)
    for r in all_results[:20]:
        print(f"{r['chassis_l']:4.0f} {r['chassis_h']:4.0f} {r['wheel_r']:5.2f} {r['wheel_drop']:5.1f} | "
              f"{r['flat_clearance']:5.2f} {r['step_clearance']:5.2f} {r['r0_clearance']:5.2f} | "
              f"{'Y' if r['fits_tunnel'] else 'N':>3}")

print()
print("Legend: L=length, H=height, Wr=wheel radius, Drop=wheel center below chassis center")
print("        Flat=flat ground clearance, Step=clearance at 4.5\" step edge")
print("        R0=clearance at R0 port lip, Tilt=chassis tilt when straddling")
print("        StowH/W=stowed height/width, Fit=fits 14.75\" tunnel")
