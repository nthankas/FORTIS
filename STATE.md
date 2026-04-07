# FORTIS Project State — Canonical Model Registry

**READ THIS FIRST before touching any simulation script.** If something here
contradicts memory, git log, or CLAUDE.md, trust git log and update this file.
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

**`simulation/isaac/xdrive/canonical/xdrive_reactor_arm.py`** — 4-DOF
parallel-link arm on the realwheel chassis. Flat-stowed zigzag layout, mounted
at chassis back edge. Keyboard control of each joint + real-time torque readout.

Arm spec (2026-04-06):
- J1 (Z yaw) + J2-J4 (Y pitch), all joints 0.629 kg (NEMA 17 0.80Nm + Cricket MK II 25:1)
- L2=17", L3=15"(-X), L4=4"(+gripper at tip)
- CF square tube 0.79"x0.79", 0.0053 lb/in (STD)
- Depth camera: Orbbec Gemini 2, 98g, on J2 shoulder (2" from joint, looking down arm)
- Gripper: 500g at L4 tip
- Total arm mass: ~2.75 kg (6.06 lb)
- Total reach: 36" (914mm)
- J1 stack: 86mm (NEMA 17 60mm + Cricket 26mm)

**Status:** TESTING — user verification of geometry and masses before sweep.

## Active simulations (canonical + tools)

| Script | Purpose | Status |
|---|---|---|
| `canonical/xdrive_realwheel.py` | Canonical chassis model, flat ground + reactor + R0 contact logging | WORKING |
| `canonical/xdrive_reactor_arm.py` | 4-DOF parallel-link arm (36" reach) on realwheel chassis, torque readout | TESTING |
| `tools/test_drift.py` | Flat-ground drift verification | WORKING |
| `tools/clearance_sweep.py` | Analytical R0 clearance calc | WORKING |
| `tools/step_arch_optimizer.py` | Belly-arch geometry optimizer | WORKING |
| `tools/orbit_torque.py` | Torque profiling at orbit radii | WORKING |
| `tools/measure_r0_port.py` | R0 port geometry extraction from USD | WORKING |
| `tools/arm_pose_sweep.py` | Headless arm pose sweep (all configs, step/reactor), CSV + JSON output | NEW |
| `tools/parse_sweep_results.py` | Aggregates sweep results, comparison tables + charts | NEW |
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
| `deprecated/xdrive_reactor_arm.py` | All-NEMA-17 + Cricket MK II arm — J2 torque insufficient (~17.5 Nm needed vs 12 Nm rated) |
| `deprecated/arm_torque_sweep.py` | Torque/power/stability sweep for the above arm config |
| `deprecated/arm_sweep_masses.md` | Mass breakdown for the deprecated arm config |

## Hardware (locked)

**Drive:**
- Wheels: AndyMark 8" Dualie Plastic Omni (am-0463), 203 mm dia, 2.04" wide, 80A, 36 rollers
- Motors: REV NEO 2.0 brushless (3.75 Nm stall, 5676 RPM)
- Gearbox: REV MAXPlanetary 9:1 single cartridge (30.38 Nm at wheel @ 90% eff)
- Robot mass: 45 lb (20.4 kg)

**Arm:** 4-DOF parallel-link (2026-04-06)
- Motors: NEMA 17 stepper 0.80Nm x4 (Lin Engineering / Oriental Motor)
- Gearbox: Cricket Drive MK II 25:1 x4 (Sweep Dynamics)
- Drivers: TMC5160-TA x4
- Joint mass: 0.629 kg each (motor 0.58 kg + hardware 49g)
- Links: CF square tube 0.79"x0.79", 0.0053 lb/in (STD), RockWest Composites
- L2=17", L3=15", L4=4" (total reach 36")
- Depth camera: Orbbec Gemini 2 (98g) on J2 shoulder, 2" from joint
- Gripper: 500g at L4 tip
- J1 stack: 86mm (NEMA 17 + Cricket MK II)
- Total arm mass: ~2.75 kg (6.06 lb)

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
