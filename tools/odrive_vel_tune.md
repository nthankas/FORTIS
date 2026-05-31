# ODrive velocity-loop tuning — FORTIS X-drive

**Why:** the four S1s run ODrive stock gains (`vel_gain=0.167`, `vel_integrator_gain=0.333`)
that were never matched to the loaded robot (40 lb, omniwheels). At low commanded
speed the integrator winds up too slowly to break wheel stiction, so the four
wheels break away at random, staggered times. Tuning fixes the staggered start.

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
- **Stop the ROS launch** so odrivetool owns the bus (leave `can1` up).
- Phase A (find `vel_gain`): **wheels OFF the ground** — safe, no runaway.
- Phase B (trim integrator under load): **on the ground but restrain the chassis**
  (against a wall / strapped / in a corner) so the driven wheel loads against
  ground friction *without the robot escaping*. Omniwheels + 40 lb = it WILL move.
  Low speeds only. **Hand on disarm** (`idle_all()`), clear the area.

## Connect
```bash
# ROS launch stopped, can1 up:
odrivetool --no-usb --can can1     # connects odrv0..3 over the chain
```

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

def apply_all(g, gi):
    for o in DRIVES:
        o.axis0.controller.config.vel_gain = g
        o.axis0.controller.config.vel_integrator_gain = gi
    print(f"applied vel_gain={g} vel_integrator_gain={gi} to all 4")

def save_all():
    for o in DRIVES:
        try: o.save_configuration()       # NOTE: reboots each ODrive
        except Exception as e: print("save (reboot expected):", e)
    print("saved all 4 — ODrives rebooted; reconnect odrivetool if needed")
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
**Apply to all four + persist:**
```python
apply_all(o.axis0.controller.config.vel_gain, o.axis0.controller.config.vel_integrator_gain)
save_all()                # ODrives reboot
```

## Verify
Restart the ROS stack, command a **low-speed** strafe/rotate from Foxglove — all
four wheels should now break away crisply and together.

## Optional polish (only if still soft)
- `axis0.controller.config.inertia` (feed-forward): set ≈ wheel+rotor rotational
  inertia → instant torque on a setpoint step, less reliance on integrator wind-up.
- If you raised the integrator a lot, check `vel_integrator_limit` isn't clamping.
- Residual tiny spread after tuning = 250 kbit/s CAN jitter → bump bus to 1 Mbit/s
  (ODrive auto-detects baud ≥ fw 0.6.11) and/or trim cyclic telemetry rates.
