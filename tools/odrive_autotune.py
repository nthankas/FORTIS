"""Velocity-loop autotuner for the FORTIS drive ODrive S1s.

For each candidate (vel_gain, vel_integrator_gain) it commands a velocity step,
samples the response, and scores rise time, overshoot, settling, steady-state
error, and oscillation. A coarse-to-fine grid search returns the lowest-cost
STABLE point. Tune on the GROUND, loaded -- off-ground a free wheel can't go
unstable, so its gains don't transfer to the loaded robot.

USE (Jetson, ROS stack stopped, can1 up, wheels ON the ground, clear space):
    odrivetool --no-usb --can can1
    >>> exec(open('tools/odrive_autotune.py').read())
    >>> autotune(0)            # one node: search, print winner + metrics
    >>> res = autotune_all()   # all four: returns {node: {gains}}
    >>> apply_to_flash(res)    # optional: write+save tuned gains, test live
    >>> codify(res)            # write winners into tools/odrive_apply_gains.py

This drives the motors. Steps alternate direction to stay roughly in place, but
the robot WILL twitch a wheel-revolution each way -- give it room, hand near the
e-stop. Every trial clears errors, arms, steps, idles; any fault aborts the
trial at max cost. Native ODrive velocity unit is rev/s.
"""

import math
import time

# --- knobs --------------------------------------------------------------------
STEP_VEL = 1.0          # rev/s step magnitude per trial
SAMPLE_DT = 0.01        # ~100 Hz response sampling (throttled to spare the CAN bus)
WINDOW_T = 1.0          # s captured per step
SETTLE_BAND = 0.05      # +/-5% of step counts as settled
PAUSE_T = 0.4           # s at zero between trials

VG_COARSE = [0.3, 0.6, 0.9, 1.2]
VI_COARSE = [0.5, 1.0, 2.0, 4.0]
FINE_FRAC = 0.5         # fine grid spans +/-50% of the coarse spacing

# lower cost = better; a candidate whose settled ripple exceeds RIPPLE_UNSTABLE
# is rejected outright (OSC_PENALTY) so the search never picks an oscillator.
W_RISE, W_OVERSHOOT, W_SETTLE, W_SSERR, W_RIPPLE = 1.0, 2.0, 1.0, 3.0, 4.0
RIPPLE_UNSTABLE = 0.08
OSC_PENALTY = 1e3

_WHEEL = {0: "FL", 1: "FR", 2: "RR", 3: "RL"}


def _ns():
    return globals()


def _drives(ns):
    try:
        return [ns[f"odrv{i}"] for i in range(4)]
    except KeyError as miss:
        raise RuntimeError(f"{miss} missing -- run inside odrivetool --no-usb --can can1")


def _read_vel(odrv):
    ax = odrv.axis0
    try:
        return ax.vel_estimate
    except AttributeError:
        return ax.pos_vel_mapper.vel


def _velmode(odrv, ns):
    c = odrv.axis0.controller.config
    c.control_mode = ns["ControlMode"].VELOCITY_CONTROL
    c.input_mode = ns["InputMode"].PASSTHROUGH


def _set_gains(odrv, vg, vi):
    c = odrv.axis0.controller.config
    c.vel_gain = vg
    c.vel_integrator_gain = vi


def _faulted(odrv):
    return odrv.axis0.active_errors != 0


def _capture_step(odrv, ns, step_vel):
    """Arm, step 0 -> step_vel, sample vel for WINDOW_T, then idle."""
    odrv.clear_errors()
    _velmode(odrv, ns)
    odrv.axis0.controller.input_vel = 0.0
    odrv.axis0.requested_state = ns["AxisState"].CLOSED_LOOP_CONTROL
    time.sleep(0.1)
    samples = []
    t0 = time.monotonic()
    odrv.axis0.controller.input_vel = step_vel
    while True:
        t = time.monotonic() - t0
        if t > WINDOW_T:
            break
        samples.append((t, _read_vel(odrv)))
        if _faulted(odrv):
            break
        time.sleep(SAMPLE_DT)
    odrv.axis0.controller.input_vel = 0.0
    time.sleep(PAUSE_T)
    faulted = _faulted(odrv)
    odrv.axis0.requested_state = ns["AxisState"].IDLE
    return samples, faulted


def _metrics(samples, step_vel):
    """Step-response metrics, sign-aligned to the (signed) step target."""
    s = abs(step_vel)
    sign = 1.0 if step_vel >= 0 else -1.0
    vs = [sign * v for (_, v) in samples]
    ts = [t for (t, _) in samples]
    if len(vs) < 5:
        return None
    rise = next((t for t, v in zip(ts, vs) if v >= 0.9 * s), WINDOW_T)
    peak = max(vs)
    overshoot = max(0.0, (peak - s) / s)
    tail = [v for t, v in zip(ts, vs) if t >= 0.6 * WINDOW_T]
    final = sum(tail) / len(tail) if tail else 0.0
    ss_err = abs(s - final) / s
    mean = final
    ripple = math.sqrt(sum((v - mean) ** 2 for v in tail) / len(tail)) if tail else 0.0
    settle = next((t for t, v in zip(ts, vs) if abs(v - s) <= SETTLE_BAND * s), WINDOW_T)
    return {"rise": rise, "overshoot": overshoot, "settle": settle,
            "ss_err": ss_err, "ripple": ripple, "peak": peak, "final": final}


def _cost(m):
    if m is None:
        return OSC_PENALTY * 10
    base = (W_RISE * m["rise"] + W_OVERSHOOT * m["overshoot"] + W_SETTLE * m["settle"]
            + W_SSERR * m["ss_err"] + W_RIPPLE * m["ripple"])
    return base + (OSC_PENALTY if m["ripple"] > RIPPLE_UNSTABLE else 0.0)


def _trial(odrv, ns, vg, vi, direction):
    _set_gains(odrv, vg, vi)
    samples, faulted = _capture_step(odrv, ns, direction * STEP_VEL)
    if faulted:
        print(f"  vg={vg:.3f} vi={vi:.3f} -> FAULT (aborted)")
        odrv.clear_errors()
        return OSC_PENALTY * 5, None
    m = _metrics(samples, direction * STEP_VEL)
    c = _cost(m)
    if m:
        print(f"  vg={vg:.3f} vi={vi:.3f} -> cost={c:7.2f} rise={m['rise']:.2f} "
              f"over={m['overshoot']*100:.0f}% sserr={m['ss_err']*100:.0f}% "
              f"ripple={m['ripple']:.3f}")
    return c, m


def _search(odrv, ns, vgs, vis, dir_state):
    best = None
    for vg in vgs:
        for vi in vis:
            dir_state[0] *= -1          # alternate direction so net motion ~cancels
            c, m = _trial(odrv, ns, vg, vi, dir_state[0])
            if best is None or c < best[0]:
                best = (c, vg, vi, m)
    return best


def autotune(node):
    """Coarse-to-fine search on one node; returns its tuned gains dict."""
    ns = _ns()
    odrv = _drives(ns)[node]
    dir_state = [1]
    print(f"=== node{node} {_WHEEL[node]} coarse ===")
    best = _search(odrv, ns, VG_COARSE, VI_COARSE, dir_state)
    _, vg0, vi0, _ = best
    dvg = (VG_COARSE[1] - VG_COARSE[0]) * FINE_FRAC
    dvi = (VI_COARSE[1] - VI_COARSE[0]) * FINE_FRAC
    vgs = [max(0.01, vg0 - dvg), vg0, vg0 + dvg]
    vis = [max(0.01, vi0 - dvi), vi0, vi0 + dvi]
    print(f"=== node{node} {_WHEEL[node]} fine (vg~{vg0:.2f} vi~{vi0:.2f}) ===")
    best = min([best, _search(odrv, ns, vgs, vis, dir_state)], key=lambda b: b[0])
    c, vg, vi, _ = best
    print(f">>> node{node} {_WHEEL[node]} BEST: vel_gain={vg:.4f} "
          f"vel_integrator_gain={vi:.4f} (cost={c:.2f})")
    return {"vel_gain": round(vg, 4), "vel_integrator_gain": round(vi, 4)}


def autotune_all():
    """Tune all four nodes; returns {node: {vel_gain, vel_integrator_gain}}."""
    res = {node: autotune(node) for node in range(4)}
    print("\nGAINS = {")
    for node in range(4):
        g = res[node]
        print(f"    {node}: {{'vel_gain': {g['vel_gain']}, "
              f"'vel_integrator_gain': {g['vel_integrator_gain']}}},  # {_WHEEL[node]}")
    print("}")
    return res


def apply_to_flash(results):
    """Write tuned gains to each board and save (for a live test before codify)."""
    drives = _drives(_ns())
    for node, g in results.items():
        _set_gains(drives[node], g["vel_gain"], g["vel_integrator_gain"])
        try:
            drives[node].save_configuration()
        except Exception as exc:  # save reboots the board, dropping CAN -- expected
            print(f"node{node}: save -> reboot/disconnect ({exc})")
    print("Saved. Reconnect odrivetool to verify.")


def codify(results, path="tools/odrive_apply_gains.py"):
    """Rewrite the GAINS table in apply_gains.py with the tuned values."""
    import re
    with open(path) as f:
        src = f.read()
    rows = [f"    {n}: {{\"vel_gain\": {results[n]['vel_gain']}, "
            f"\"vel_integrator_gain\": {results[n]['vel_integrator_gain']}}},  # {_WHEEL[n]}"
            for n in range(4)]
    block = "GAINS = {\n" + "\n".join(rows) + "\n}"
    new = re.sub(r"GAINS = \{.*?\n\}", block, src, count=1, flags=re.DOTALL)
    if new == src:
        print(f"GAINS block not found in {path} -- paste these by hand instead.")
        return
    with open(path, "w") as f:
        f.write(new)
    print(f"Updated GAINS in {path}. Review the diff, then apply_all() + commit.")
