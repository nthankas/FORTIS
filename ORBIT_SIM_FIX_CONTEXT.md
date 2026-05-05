# Orbit Torque Sim - P95 Fix Context (handoff)

> **Read this, then DELETE this file before committing anything.** It is a one-shot scratchpad for the next chat session, not a permanent doc.

## What you're picking up

The 5-sphere realwheel orbit torque sweep produced P95 wheel-torque values that the
user judged unrealistically high. The chart has been re-styled (matches the v1
single-sphere chart now: 2 panels Mean | P95, line+marker, only 40 A and 60 A
continuous-rating reference lines). The styling is good. **The numbers underneath
it are not.**

The user wants you to:
1. Investigate why P95 is so high.
2. Propose 1-2 fix options **before** kicking off any new long sim.
3. Once approved, fix and re-run the sweep.
4. Regenerate the chart (existing `plot_5sphere.py` is already in the desired clean style; do not touch it).

## Files involved

- **Sweep script (suspect):** `sim/isaac/xdrive/tools/sweep_orbit_realwheel.py`
  - Has NOT been read yet this session. Read it first.
- **Source data:** `sim/isaac/xdrive/results/orbit_realwheel_5sphere/sweep_orbit_realwheel.json`
- **Plot script (already clean, don't change style):** `sim/isaac/xdrive/results/orbit_realwheel_5sphere/plot_5sphere.py`
- **Generated chart:** `sim/isaac/xdrive/results/orbit_realwheel_5sphere/torque_sweep_5sphere_vs_baseline.png`

## What the suspect numbers look like

From the existing JSON (last run, 14.144 kg chassis suptitle but actual sim mass needs verification):

- FR P95 at 0.30 m/s: ~9 Nm
- FR peak at 0.30 m/s: ~31 Nm
- FL P95 at 0.10 m/s: 3.77 Nm
- BL P95: ~7.26 Nm, BR P95: ~6.97 Nm
- M8325s 60 A continuous rating: 4.98 Nm. Most P95 values are above 60 A cont., escalating with speed.
- Tracking %: 91% at v=0.10, drops to 60-70% at v=0.20+. High P95 with low tracking suggests scrubbing/binding rather than rolling.

## Likely root causes (investigate before fixing)

1. **Sample window includes startup transient.** Check whether the script discards the first N seconds of each trial before computing mean/P95/peak. Teleport-to-orbit-radius + PD ramp will spike torque hard.
2. **5-sphere collider contact-popping / stick-slip.** The 5-sphere chain may be hammering at contacts, especially at higher commanded speeds. Compare to the v1 single-sphere case for the same orbit speed.
3. **Roller pre-load / penetration.** 5 spheres may be overlapping the floor at rest, inflating steady-state torque. Print penetration depth.
4. **Velocity-locked drive transients.** PD on wheel velocity may oscillate when the chassis can't keep up (low tracking %). P95 captures those oscillations.

Investigate first, fix second. **Do not just slap a new sample window on it without confirming the cause.**

## CRITICAL - Mass setup that was fixed earlier in this session

**Do NOT re-introduce the bug we already fixed.** When the user looked at the
arm sim earlier, the chassis mass had been silently set to **3.629 kg**, which
was wrong. The user corrected it.

Correct mass budget (40 lb total cap):

| Part      | Mass     | Notes                                 |
|-----------|----------|---------------------------------------|
| Chassis   | 10.144 kg | Just the chassis structure           |
| Arm       | 6.515 kg  | Computed from arm sim                |
| Wheels    | 4 x 1 kg = 4 kg | Per-wheel rigid body         |
| **Total** | **20.66 kg** | ~45.5 lb (slightly over 40 lb cap, accepted) |

The orbit sweep chart suptitle currently reads `(40 lb total, 14.144 kg
chassis + 4x 1 kg wheels, R=1.59 m)`. **That 14.144 kg "chassis" number is a
chassis+arm combined mass for the orbit case (no arm articulated, so its mass
is absorbed into the chassis body)** -- 10.144 (chassis) + 4.0 (arm-as-payload
proxy is... actually verify this number, may need to be 10.144 + 6.515 =
16.659 kg if the arm is bolted on for the drive sweep). **Verify what mass is
actually in `sweep_orbit_realwheel.py` before re-running.** If it's 14.144 kg
hardcoded, that may itself be wrong - the arm should be ~6.515 kg, so a
chassis+arm rigid-body lump should be 10.144 + 6.515 = 16.659 kg, plus 4 x 1
kg wheels = 20.66 kg total.

**This is a hard requirement: the orbit sim must reflect the same total mass
budget as the arm sim (20.66 kg total = 16.659 kg chassis+arm lump + 4 x 1 kg
wheels), not 14.144 + 4 = 18.144 kg.** If you find the orbit sweep is
underweighted vs the arm sim, that's a separate bug to flag.

## Other context worth knowing

- **Don't ask for permissions** - just execute. (Per user's standing feedback memory.)
- **If a long sim fails, fix and re-run autonomously.** Don't just report failure.
- **Use the existing `plot_5sphere.py`** - it's already in the clean v1 style the user
  approved (2 panels, line+marker, only 40 A + 60 A reference lines, no tables, no
  "free air"/"forced air" wording). Do not redesign it.
- **Read `sim/README.md` first** before touching anything in `sim/`. It is the
  canonical registry of which scripts are active vs deprecated.
- **Project ** lives at `E:\Capstone\` - read it. Key rules:
  no AI-assistant attribution in commits,
  STL is ground truth, dimensions are parametric.

## Verbatim user request that triggered this handoff

> "the P95s seem way off they shoiuld not that be high. can we rerun that and
> get new results and fix stuff first."

## After you finish reading

**Delete this file (`ORBIT_SIM_FIX_CONTEXT.md`) before making any commit.** It
is not a permanent artifact.
