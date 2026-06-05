# ODrive velocity-loop tuning — FORTIS X-drive

**Why:** the four S1s run ODrive stock gains (`vel_gain=0.167`, `vel_integrator_gain=0.333`)
that were never matched to the loaded robot (~45 lb, omniwheels). At low commanded
speed the integrator winds up too slowly to break wheel stiction, so the four
wheels break away at random, staggered times. Tuning fixes the staggered start.

**Scope:** velocity-loop P/I only (`vel_gain`, `vel_integrator_gain`) plus the
protective limits below. We deliberately do **not** tune `current_control_bandwidth`
(ultra-low-inductance M8325s → raising it risks `CURRENT_LIMIT_VIOLATION` / FET
whine) and keep `inertia = 0` (an omni X-drive's reflected inertia swings between
translate and rotate, so feed-forward injects torque chatter). Control mode is
**velocity** (the official drive control-mode decision).

**Key fact (from the velocity-loop math):**
`current_integral += vel_error * vel_integrator_gain`; at low speed with a stalled
wheel, breakaway time ∝ `1 / vel_integrator_gain`. **The integrator gain is the
lever for crisp, simultaneous breakaway.** Keep `input_mode = PASSTHROUGH` (a ramp
mode would *slow* the start).

## Serial ↔ node_id (this chain)
| node_id | serial | wheel |
|---|---|---|
| 0 | 006274728A35 | FL |
| 1 | 0062747281A6 | FR |
| 2 | 006274729CA6 | RR |
| 3 | 006274729F3E | RL |

## SAFETY (read first)
- **Stop the ROS launch** so odrivetool owns the bus (leave `can0` up).
- Phase A (find `vel_gain`): **wheels OFF the ground** — safe, no runaway.
- Phase B (trim integrator under load): **on the ground but restrain the chassis**
  (against a wall / strapped / in a corner) so the driven wheel loads against
  ground friction *without the robot escaping*. Omniwheels + ~45 lb = it WILL move.
  Low speeds only. **Hand on disarm** (`idle_all()`), clear the area.

## Connect
```bash
# ROS launch stopped, can0 up:
odrivetool --no-usb --can can0     # connects odrv0..3 over the chain
```
If `can0` is down: `sudo ip link set can0 up type can bitrate 250000`, then
`candump can0` should show heartbeats from node_ids 0/1/2/3 before you continue.

## Paste these helpers into the odrivetool shell
```python
DRIVES = [odrv0, odrv1, odrv2, odrv3]   # node 0..3 = FL,FR,RR,RL

def show_gains():
    for i, o in enumerate(DRIVES):
        c = o.axis0.controller.config
        print(f"node{i}: vel_gain={c.vel_gain:.4f}  vel_int={c.vel_integrator_gain:.4f}")

def arm(o):  o.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
def idle(o): o.axis0.requested_state = AxisState.IDLE
def idle_all():
    for o in DRIVES: idle(o)          # <-- your e-stop

def spin(o, v):  o.axis0.controller.input_vel = v          # turn/s
def vg(o, x):    o.axis0.controller.config.vel_gain = x
def vi(o, x):    o.axis0.controller.config.vel_integrator_gain = x
def bump(o, f=1.3):
    o.axis0.controller.config.vel_gain *= f
    print(f"vel_gain -> {o.axis0.controller.config.vel_gain:.4f}")
```

## Procedure
**Phase A — find `vel_gain` (wheels off ground), on odrv0/FL:**
```python
idle_all()
o = odrv0
vi(o, 0)                  # integrator off while finding vel_gain
start_liveplotter(lambda: [o.axis0.vel_estimate, o.axis0.controller.input_vel])
arm(o); spin(o, 2)        # excite at ~2 turn/s
# raise vel_gain ~30%/step until it buzzes/whines/vibrates:
bump(o)                   # repeat: 0.22, 0.29, 0.38, 0.49, 0.64, ...
#   poke it between bumps: spin(o,0); spin(o,2); spin(o,-2)
# once it vibrates, back off to 50% of the vibrating value:
vg(o, <vibrating_value> * 0.5)
idle(o)
```
**Phase B — set `vel_integrator_gain` (on ground, chassis restrained):**
```python
vi(o, o.axis0.controller.config.vel_gain)   # start: integrator == vel_gain
arm(o)
# step the setpoint and watch the plot for overshoot/ringing:
spin(o, 0); spin(o, 3); spin(o, 0)
# raise vi() until setpoint steps overshoot, then back to ~50% of that:
vi(o, <overshoot_value> * 0.5)
idle(o)
stop_liveplotter()
```
Repeat A+B per wheel (or tune odrv0 and reuse if the four match closely), and
record each converged pair in the results block below.

## Protective limits (set by tools/odrive_apply_gains.py)
The apply-script re-asserts these on every node so one file fully defines the
operational envelope:
- `current_soft_max = 30 A` (raised from calibration's 20 A for breakaway torque)
- `current_hard_max = 30 A` (protection trip, unchanged)
- `vel_integrator_limit = 30 A` (so the integrator isn't clamped below soft max)
- `vel_limit = 5.0 rev/s` (HW ceiling; the ROS layer clamps wheel speed to ~1.57)
- `inertia = 0.0`

## Apply + persist (versioned)
Once converged, put the final per-node numbers into the `GAINS` table in
`tools/odrive_apply_gains.py`, then from the odrivetool CAN shell:
```python
exec(open('tools/odrive_apply_gains.py').read())
check()        # dry-run: confirm the planned writes
apply_all()    # validate -> write -> save (each ODrive reboots on fw 0.6.11)
# reconnect odrivetool, then:
exec(open('tools/odrive_apply_gains.py').read()); verify_all()
```
The script is the source of truth: re-running it after any board swap / re-flash
restores the tuned drive in one step. `apply_all()` refuses to run until the
`GAINS` table is filled (no `None`).

## Verify
Restart the ROS stack, command a **low-speed** strafe/rotate from Foxglove — all
four wheels should now break away crisply and together.

## Tuned values (measured)
_Filled after the live session of 2026-06-__:_
| node | wheel | vel_gain | vel_integrator_gain |
|---|---|---|---|
| 0 | FL | _TBD_ | _TBD_ |
| 1 | FR | _TBD_ | _TBD_ |
| 2 | RR | _TBD_ | _TBD_ |
| 3 | RL | _TBD_ | _TBD_ |

Before/after: _staggered start → ____ ._

## Optional polish (only if still soft)
- If you raised the integrator a lot, check `vel_integrator_limit` isn't clamping
  (the apply-script sets it to 30 A = the soft current limit).
- Do **not** add `inertia` feed-forward on this omni X-drive — see the scope note.
- Residual tiny spread after tuning = 250 kbit/s CAN jitter → bump bus to 1 Mbit/s
  (ODrive auto-detects baud ≥ fw 0.6.11) and/or trim cyclic telemetry rates.
