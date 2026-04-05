"""
FORTIS Step Arch Optimizer — symmetric trapezoid
=================================================
Finds the optimal arch_height and arch_flat_width for the symmetric
trapezoidal channel on the chassis underbelly.

Fixed constraints:
  - Motor mount flat = 1.4" at each end (gearbox width)
  - Wheel drop = 2.0" (set by motor shaft position)
  - 15x9x5.5" octagonal chassis, 3" chamfer, 8" wheels
  - 4.5" step, robot crosses perpendicular, front faces center

Usage: python step_arch_optimizer.py
"""
import math, os, json
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ── Fixed geometry (inches) ───────────────────────────────────────────────
CHASSIS_L   = 15.0
CHASSIS_W   = 9.0
CHASSIS_H   = 5.5
CHAMFER     = 3.0 / math.sqrt(2)   # 2.121" cut
SL          = CHASSIS_L / 2.0       # 7.5"
half_h      = CHASSIS_H / 2.0       # 2.75"
WHEEL_X     = SL - CHAMFER / 2.0    # 6.439" (wheel center X)
WHEELBASE   = 2.0 * WHEEL_X         # 12.879"
WHEEL_R     = 4.0                    # 8" Dualie / 2
WHEEL_DROP  = 2.0                    # fixed by shaft
STEP        = 4.5                    # step height
MML         = 1.4                    # motor mount flat at each end

# ── Tilt when straddling ──────────────────────────────────────────────────
TILT    = math.asin(STEP / WHEELBASE)
SIN_T   = math.sin(TILT)
COS_T   = math.cos(TILT)
Z_OFF   = WHEEL_R - WHEEL_X * SIN_T + WHEEL_DROP * COS_T  # back wheel on Z=0


def bottom_z(x, ah, afw):
    """Chassis bottom Z (local frame) for symmetric trapezoid.

    ah  = arch height (raise above motor mount level)
    afw = arch flat width (total width of raised flat center)
    """
    ax = abs(x)
    z_low  = -half_h
    z_high = -half_h + ah
    rs = afw / 2.0                    # ramp start (from center)
    re = SL - MML                     # ramp end = 6.1"

    if rs >= re:                      # flat section fills ramp zone
        return z_high if ax <= re else z_low
    if ax <= rs:
        return z_high                 # flat raised center
    elif ax <= re:
        t = (ax - rs) / (re - rs)
        return z_high + t * (z_low - z_high)   # linear ramp
    else:
        return z_low                  # motor mount flat


def world_z(x, ah, afw):
    """World Z of chassis bottom at local x, tilted over step. Z=0 = outer floor."""
    return -x * SIN_T + bottom_z(x, ah, afw) * COS_T + Z_OFF


# ── Analysis ──────────────────────────────────────────────────────────────
RAMP_END = SL - MML  # 6.1"

print("=" * 65)
print("FORTIS Step Arch Optimizer")
print("=" * 65)
print(f"Chassis:     {CHASSIS_L}x{CHASSIS_W}x{CHASSIS_H}\"")
print(f"Wheel:       {WHEEL_R*2}\" dia, drop={WHEEL_DROP}\"")
print(f"Step:        {STEP}\"")
print(f"Wheelbase:   {WHEELBASE:.3f}\"")
print(f"Tilt:        {math.degrees(TILT):.2f} deg")
print(f"Motor mount: {MML}\" flat at each end (ramp ends at X={RAMP_END:.1f}\")")
print(f"Wheel X:     {WHEEL_X:.3f}\"")
print()

# Clearance at motor mount (X = ramp_end, bottom = -half_h regardless of arch)
clr_mml = -RAMP_END * SIN_T + (-half_h) * COS_T + Z_OFF
print(f"Motor mount clearance at X={RAMP_END:.1f}\": {clr_mml:.3f}\"")
print(f"Motor mount clearance at X={WHEEL_X:.1f}\" (wheel): "
      f"{-WHEEL_X*SIN_T + (-half_h)*COS_T + Z_OFF:.3f}\"")
print()

# Sweep arch parameters
print("-" * 65)
print("Clearance profile for various arch configs")
print("-" * 65)
print(f"{'AH':>5} {'FlatW':>6} | {'X=0':>6} {'X=2':>6} {'X=4':>6} "
      f"{'X=5':>6} {'X=6':>6} {'X=6.1':>6} | {'VolLost':>7}")
print("-" * 65)

results = []
for ah in np.arange(0.0, half_h + 0.01, 0.25):
    for afw in [0.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]:
        if afw / 2.0 > RAMP_END:
            continue
        # Clearance at key X positions
        cs = {x: world_z(x, ah, afw) for x in [0, 2, 4, 5, 6, RAMP_END]}
        # Volume lost to arch (integral of raise * width)
        xs = np.linspace(-SL, SL, 500)
        dx = xs[1] - xs[0]
        raises = [bottom_z(x, ah, afw) - (-half_h) for x in xs]
        vol = CHASSIS_W * sum(r * dx for r in raises)

        results.append({'ah': ah, 'afw': afw, 'cs': cs, 'vol': vol})
        print(f"{ah:5.2f} {afw:6.1f} | {cs[0]:6.2f} {cs[2]:6.2f} {cs[4]:6.2f} "
              f"{cs[5]:6.2f} {cs[6]:6.2f} {cs[RAMP_END]:6.2f} | {vol:7.1f} in3")

# ── Crossing sweep: clearance vs X for a few configs ─────────────────────
print()
print("-" * 65)
print("Full crossing clearance (step sweeps X = +6.4 to -6.4)")
print("-" * 65)

xs_cross = np.linspace(-WHEEL_X, WHEEL_X, 300)
configs = [
    (0.0, 0.0,  "No arch"),
    (1.0, 0.0,  "AH=1.0 FW=0"),
    (1.0, 4.0,  "AH=1.0 FW=4"),
    (1.5, 4.0,  "AH=1.5 FW=4"),
    (2.0, 4.0,  "AH=2.0 FW=4"),
    (2.5, 5.0,  "AH=2.5 FW=5 (current)"),
]

for ah, afw, label in configs:
    clrs = [world_z(x, ah, afw) for x in xs_cross]
    mn = min(clrs)
    mn_x = xs_cross[np.argmin(clrs)]
    pos_start = None
    for i, c in enumerate(clrs):
        if c >= 0.5 and pos_start is None:
            pos_start = xs_cross[i]
    safe_pct = 100.0 * sum(1 for c in clrs if c >= 0.5) / len(clrs)
    print(f"  {label:25s}: min={mn:+.3f}\" at X={mn_x:.1f}\", "
          f"safe(>=0.5\")={safe_pct:.0f}%")

# ── Find where clearance = 0 and 0.5 for no-arch case ────────────────────
# clearance(x) = -x*sin + (-half_h)*cos + zoff   (no arch, flat bottom)
x_zero = (-half_h * COS_T + Z_OFF) / SIN_T
x_half = (-half_h * COS_T + Z_OFF - 0.5) / SIN_T
print(f"\n  No-arch zero-clearance line: X = {x_zero:.2f}\"")
print(f"  No-arch 0.5\" clearance line: X = {x_half:.2f}\"")
print(f"  (Anything in front of these X values is below that clearance)")

# ── Visualization ─────────────────────────────────────────────────────────
if not HAS_MPL:
    print("\nNo matplotlib -- skipping plots.")
    exit(0)

fig, axes = plt.subplots(2, 2, figsize=(16, 12))

# --- Panel 1: Side view (tilted chassis over step) ---
ax = axes[0, 0]
ax.set_title("Side View: Chassis Over Step (tilted)", fontweight='bold', fontsize=11)

# Floors
ax.axhline(0, color='#8B7355', lw=2, label='Outer floor (Z=0)')
ax.axhline(-STEP, color='#8B7355', lw=2, ls='--', label=f'Inner floor (-{STEP}")')
ax.fill_between([-9, 9], 0, -0.15, color='#D2B48C', alpha=0.3)
ax.fill_between([-9, -0.5], -STEP, -STEP-0.15, color='#D2B48C', alpha=0.3)

# Step wall (at X=0 in this diagram for reference)
ax.fill_between([-0.5, 0.5], -STEP, 0, color='#A0522D', alpha=0.5)
ax.text(0, -STEP/2, f'{STEP}"', ha='center', va='center', fontsize=9,
        fontweight='bold', color='white')

# Draw a few chassis profiles
plot_cfgs = [
    (0.0, 0.0,  'gray',      '--', 'No arch'),
    (1.5, 4.0,  'green',     '-',  'AH=1.5 FW=4'),
    (2.5, 5.0,  'blue',      '-',  'AH=2.5 FW=5 (current)'),
]

xs_p = np.linspace(-SL, SL, 300)
for ah, afw, color, ls, label in plot_cfgs:
    zbot = [world_z(x, ah, afw) for x in xs_p]
    ztop = [-x * SIN_T + half_h * COS_T + Z_OFF for x in xs_p]
    ax.plot(xs_p, zbot, color=color, ls=ls, lw=2, label=label)
    if ls == '-':
        ax.fill_between(xs_p, zbot, ztop, alpha=0.08, color=color)

# Wheels
for wx, lbl in [(WHEEL_X, 'Front (inner)'), (-WHEEL_X, 'Back (outer)')]:
    wz = -wx * SIN_T + (-WHEEL_DROP) * COS_T + Z_OFF
    ax.add_patch(plt.Circle((wx, wz), WHEEL_R, fill=False, color='red', lw=2))
    ax.plot(wx, wz, 'r+', ms=8)
    ax.text(wx, wz + WHEEL_R + 0.4, lbl, ha='center', fontsize=7)

# Motor mount zone
ax.axvspan(RAMP_END, SL, alpha=0.1, color='red', label=f'Motor mount ({MML}" flat)')
ax.axvspan(-SL, -RAMP_END, alpha=0.1, color='red')

ax.set_xlabel('X along chassis (+ = front/center)')
ax.set_ylabel('Z (inches, outer floor = 0)')
ax.set_xlim(-9, 9)
ax.set_ylim(-7, 11)
ax.set_aspect('equal')
ax.legend(fontsize=7, loc='upper left')
ax.grid(True, alpha=0.3)

# --- Panel 2: Clearance vs X during crossing ---
ax = axes[0, 1]
ax.set_title("Clearance During Step Crossing", fontweight='bold', fontsize=11)

for ah, afw, label in configs:
    clrs = [world_z(x, ah, afw) for x in xs_cross]
    ax.plot(xs_cross, clrs, lw=2, label=label)

ax.axhline(0.5, color='red', ls='--', lw=1, label='0.5" target')
ax.axhline(0, color='black', ls='-', lw=0.5)
ax.axvspan(RAMP_END, WHEEL_X, alpha=0.15, color='red', label='Motor mount zone')
ax.set_xlabel('Step position X in robot frame (+ = front)')
ax.set_ylabel('Clearance above step (inches)')
ax.set_xlim(-WHEEL_X - 0.5, WHEEL_X + 0.5)
ax.set_ylim(-2.5, 6)
ax.legend(fontsize=7, loc='upper right')
ax.grid(True, alpha=0.3)

# --- Panel 3: Arch height vs volume lost ---
ax = axes[1, 0]
ax.set_title("Interior Volume Lost to Arch", fontweight='bold', fontsize=11)

for afw in [0.0, 2.0, 4.0, 6.0, 8.0]:
    ahs = np.arange(0, half_h + 0.01, 0.1)
    vols = []
    for ah in ahs:
        if afw / 2.0 > RAMP_END:
            vols.append(None)
            continue
        xs_v = np.linspace(-SL, SL, 500)
        dx_v = xs_v[1] - xs_v[0]
        raises = [bottom_z(x, ah, afw) - (-half_h) for x in xs_v]
        vols.append(CHASSIS_W * sum(r * dx_v for r in raises))
    valid = [(a, v) for a, v in zip(ahs, vols) if v is not None]
    if valid:
        ax.plot([a for a, v in valid], [v for a, v in valid],
                lw=2, label=f'FlatW={afw}"')

ax.set_xlabel('Arch height (inches)')
ax.set_ylabel('Volume lost (cubic inches)')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

total_vol = CHASSIS_L * CHASSIS_W * CHASSIS_H
ax2r = ax.twinx()
ax2r.set_ylim(0, ax.get_ylim()[1] / total_vol * 100)
ax2r.set_ylabel('% of total chassis volume')

# --- Panel 4: Cross-section at key X positions ---
ax = axes[1, 1]
ax.set_title("Arch Cross-Section Profile (X-Z plane, center slice)", fontweight='bold',
             fontsize=11)

# Draw the arch profile for a few configs
for ah, afw, color, ls, label in plot_cfgs:
    xs_s = np.linspace(-SL, SL, 300)
    zbot = [bottom_z(x, ah, afw) for x in xs_s]
    ax.plot(xs_s, zbot, color=color, ls=ls, lw=2, label=label)

ax.axhline(-half_h, color='gray', ls=':', lw=1, label=f'Motor mount level (-{half_h}")')
ax.axhline(half_h, color='gray', ls=':', lw=1, alpha=0.3)
ax.axvspan(RAMP_END, SL, alpha=0.1, color='red')
ax.axvspan(-SL, -RAMP_END, alpha=0.1, color='red')
ax.fill_between(xs_s, -half_h, half_h, alpha=0.05, color='blue')

ax.set_xlabel('X along chassis (+ = front)')
ax.set_ylabel('Z local (inches)')
ax.set_xlim(-SL - 0.5, SL + 0.5)
ax.set_ylim(-half_h - 0.5, half_h + 0.5)
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)
ax.set_aspect('equal')

plt.tight_layout()
_XDRIVE_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
out_dir = os.path.join(_XDRIVE_ROOT, "results", "step_arch_optimizer")
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, "step_arch_analysis.png")
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"\nPlot saved: {out_path}")
