# FORTIS Project State — Canonical Model Registry

**READ THIS FIRST before touching any simulation script.** If something here
contradicts memory, git log, or , trust git log and update this file.
If a script isn't listed, it's not canonical — ask before using it.

Last updated: 2026-04-04

---

## Directory layout (`simulation/isaac/xdrive/`)

```
xdrive/
  canonical/    # THE scripts to copy from (chassis + arm)
  deprecated/   # Kept for history, physics model is wrong — do NOT copy
  tools/        # Analysis / measurement / standalone utilities
  lib/          # Shared Python modules (sim_config.py)
  assets/       # USD files (diiid_reactor.usd, omniwheels.usd)
  docs/         # Spec docs, summaries, plans
  results/      # Per-script output (results/<script_name>/...)
```

Rule: every script writes its output into `results/<script_name>/`. No more
flat dumps in `xdrive/`.

---

## Canonical chassis / wheel model (THE ONE TO COPY FROM)

**`simulation/isaac/xdrive/canonical/xdrive_realwheel.py`** — arched octagonal
chassis + real Kaya omni-wheel meshes (1 hub + 10 rollers per wheel, separate
rigid bodies with revolute joints). 44-DOF articulation on CPU physics @ 360 Hz
with TGS + CCD. Loads `assets/omniwheels.usd` for the wheel mesh source.

Key constants (always match these in new scripts):
- `CHASSIS_L = 15.354" `, `CHASSIS_W = 9.353"`, `CHASSIS_H = 7.1"`
- `CHASSIS_MASS = 20.4 kg`
- `WHEEL_RADIUS = 0.1015 m` (8" AndyMark Dualie omni), `NUM_ROLLERS = 10`
- `BELLY_HEIGHT = 2.5"` default (CLI `--belly`)
- `PHYSICS_HZ = 360`, CPU physics, TGS, CCD on
- Chassis collision: **`convexDecomposition`** (NOT convexHull — hull fills the belly recess)
- Reactor spawn: `SPAWN_X=0, SPAWN_Y=-1.59, spawn_yaw=90°`, z computed from `cfg.Z_OUTER_IN`
- Drive joints found by name prefix `"DriveJoint_"`

**Rule:** any new sim that needs the FORTIS chassis copies its build path from
`canonical/xdrive_realwheel.py`. Don't rebuild chassis/wheel geometry from
scratch and don't copy from anything in `deprecated/`.

## Canonical arm model

**`simulation/isaac/xdrive/canonical/xdrive_reactor_arm.py`** (Sim3 phase 1)
- 4-DOF arm on top of the realwheel chassis, reactor straddle pose
- Link lengths from Carlos's `inverse_kinetmatic_solver.py` DH_PARAMS (not the
  spec-doc round numbers): L_J1=127mm, L_L2=576.1mm, L_L3=500mm, L_L4=150mm
- **Flat-stowed pose**: arm lays horizontal on chassis top, folded zigzag.
  Mount at **chassis back edge** (`ARM_MOUNT_X = -CHASSIS_L/2`), L2 extends
  forward over the chassis top, L3 folds back, L4 folds forward. CG sits near
  chassis center at spawn.
- All-NEMA-17 + Cricket Drive MK II 25:1 on every joint, maxForce = 12 Nm
  (gearbox rated torque, not motor stall)
- Companion spec doc: `docs/arm_spec.md`

## Active simulations (canonical + tools)

| Script | Purpose | Status |
|---|---|---|
| `canonical/xdrive_realwheel.py` | Canonical chassis model, flat ground + reactor + R0 contact logging | WORKING |
| `canonical/xdrive_reactor_arm.py` | Sim3 phase 1 — flat-stowed arm on realwheel in reactor straddle | WORKING (phase 1 visual verification pending final sign-off) |
| `tools/test_drift.py` | Flat-ground drift verification | WORKING |
| `tools/clearance_sweep.py` | Analytical R0 clearance calc | WORKING |
| `tools/step_arch_optimizer.py` | Belly-arch geometry optimizer | WORKING |
| `tools/orbit_torque.py` | Torque profiling at orbit radii | WORKING |
| `tools/measure_r0_port.py` | R0 port geometry extraction from USD | WORKING |
| `lib/sim_config.py` | Shared reactor/geometry constants — **import this**, don't duplicate | WORKING |

## DEPRECATED — do not copy from these

These predate the realwheel chassis and use the old octagonal-prism body with
sphere-roller wheel approximations. They still run but the physics model is
wrong. **Do not use them as the base for new scripts.** Kept under
`deprecated/` for history.

| Script | Why deprecated |
|---|---|
| `deprecated/xdrive_o3dyn.py` | Old o3dyn-style chassis + sphere rollers, flat ground only |
| `deprecated/xdrive_reactor.py` | Old chassis + sphere rollers, reactor env. Superseded by `canonical/xdrive_realwheel.py --reactor` |
| `deprecated/xdrive_reactor_v2.py` | Intermediate v2 attempt, superseded by realwheel |
| `deprecated/xdrive_r0_entry_v2.py` | R0 entry test with wrong tether model — needs rework on top of realwheel |

## Hardware (locked)

**Drive:**
- Wheels: AndyMark 8" Dualie Plastic Omni (am-0463), 203 mm dia, 2.04" wide, 80A, 36 rollers
- Motors: REV NEO 2.0 brushless (3.75 Nm stall, 5676 RPM)
- Gearbox: REV MAXPlanetary 9:1 single cartridge (30.38 Nm at wheel @ 90% eff)
- Robot mass: 45 lb (20.4 kg)

**Arm (Sim3):**
- Motors (J1–J4, all identical): NEMA 17 StepperOnline 17HS24-2104S, 500 g, 0.65 Nm hold
- Gearbox (all four): Cricket Drive MK II 25:1 (85.5% eff, ±8.5 arcmin backlash, **11–12 Nm rated torque = the binding constraint**)
- Links: carbon fiber tube (82 g total across L2/L3/L4)
- Camera: Orbbec Gemini 2 (445 g) at J4 link tip

## Stability / arm rules

- Chassis stability analysis uses **arm + chassis only** — never the tether.
  The tether is for R0 entry and recovery, not load-bearing on the reactor
  floor. (See `memory/feedback_no_tether_in_stability.md`.)
- Spec docs like `docs/arm_spec.md` are authoritative. If a doc and my
  computation disagree, the doc wins. Flag inconsistencies, don't silently
  override. (See `memory/feedback_follow_spec_docs.md`.)

## Reactor environment

- USD: `simulation/isaac/xdrive/assets/diiid_reactor.usd` (OnShape import,
  triangle mesh collision applied manually in the GUI — **do not regenerate**)
- Ground truth mesh (not in git): `E:/Capstone/assets/halfReactor.stl` (56 MB)
- Key radii (`lib/sim_config.py`): inner floor 45.8–54.8", step at 55.5",
  outer floor 56.2–70.3", step height 4.5", **central column r ≈ 38"**
- Z heights: outer floor −49.3", inner floor −53.8"

## Update protocol

Update this file whenever:
1. A new sim script becomes canonical (mark old ones deprecated, don't delete)
2. Hardware selection changes (update the Hardware section + `docs/arm_spec.md`)
3. A physics/solver constant that affects model fidelity changes
4. A new phase starts or the current phase ends

If a new chassis/wheel model supersedes `canonical/xdrive_realwheel.py`, update
the "Canonical chassis / wheel model" section first, then move the old entry
to `deprecated/`. Never leave two scripts labeled canonical.
