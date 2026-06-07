"""Idempotent re-apply of the FORTIS drive ODrive S1 operational config.

WHAT THIS IS
------------
The four drive ODrive S1s keep their velocity-loop gains and protective limits
in on-board flash, set once by hand-tuning. Those values live nowhere else, so
a board swap or firmware re-flash silently reverts them to ODrive factory
defaults (vel_gain=0.167, vel_integrator_gain=0.333) -- which reproduce the
loaded staggered-wheel-start this config exists to fix. This script is the
version-controlled source of truth for the drive *operational envelope*. Run it
after any re-flash to restore the tuned drive in one step.

SCOPE BOUNDARY
--------------
Asserts the operational/limit envelope ONLY -- control mode, velocity-loop
gains, velocity/current limits. It does NOT touch motor or encoder *identity*
(pole_pairs, phase R/L, thermistor, calibration); that is owned by
tools/odrive_calibrate.md. Run this only against an already-calibrated board.

HOW TO RUN (on the Jetson, ROS stack stopped, can1 up)
------------------------------------------------------
    odrivetool --no-usb --can can1       # exposes odrv0..odrv3 over the chain
    # can1 = gs_usb USB-CAN adapter (the ODrive bus); can0 is onboard mttcan.
    >>> exec(open('tools/odrive_apply_gains.py').read())
    >>> check()                          # dry-run: prints the planned writes
    >>> apply_all()                      # validate -> write -> save -> (reboot)
    # reconnect odrivetool, then:
    >>> exec(open('tools/odrive_apply_gains.py').read())
    >>> verify_all()                     # read the envelope back

apply_all() refuses to run until the MEASURED gains below are filled in (None in
a fresh skeleton). Native ODrive units are rev/s (turn/s), NOT rad/s.
"""

# --- Source of truth: per-node operational config ----------------------------
# node_id -> wheel (CAN daisy-chain FL->FR->RR->RL = node 0->1->2->3).
WHEEL_BY_NODE = {0: "FL", 1: "FR", 2: "RR", 3: "RL"}

# vel_gain / vel_integrator_gain MEASURED in the live tuning session
# (tools/odrive_vel_tune.md). Tuned on FL 2026-06-05 and applied to all four:
# identical M8325s + omniwheels -> identical gains -> simultaneous breakaway.
GAINS = {
    0: {"vel_gain": 0.925, "vel_integrator_gain": 1.0},  # FL
    1: {"vel_gain": 0.925, "vel_integrator_gain": 1.0},  # FR
    2: {"vel_gain": 0.925, "vel_integrator_gain": 1.0},  # RR
    3: {"vel_gain": 0.925, "vel_integrator_gain": 1.0},  # RL
}

# Limits + modes are identical for all four wheels and re-asserted on every
# apply so this one file fully defines the operational envelope:
#   current_soft_max 30 A: raised from the calibration-time 20 A for breakaway
#                          torque (~2.5 Nm/wheel at Kt=0.083).
#   current_hard_max 40 A: protection trip. MUST stay above soft_max -- with no
#                          margin the current loop trips CURRENT_LIMIT_VIOLATION
#                          on any transient (datasheet: 40 A cont / 80 A 3 s peak).
#   vel_integrator_limit 30 A: let the integrator drive up to soft max, not clamp.
#   vel_limit 5.0 rev/s: HW ceiling; the ROS layer already clamps to ~1.57.
#   inertia 0.0: NO feed-forward (omni reflected inertia swings -> chatter).
LIMITS = {
    "current_soft_max": 30.0,
    "current_hard_max": 40.0,
    "vel_integrator_limit": 30.0,
    "vel_limit": 5.0,
    "inertia": 0.0,
}


def validate_table(gains, limits, require_measured):
    """Return a list of human-readable problems (empty list == OK).

    Pure function -- takes no ODrive handles -- so it is unit-testable
    off-hardware. require_measured=True (the apply path) rejects unfilled None
    gains; False (the dry-run path) tolerates them so a fresh skeleton inspects
    cleanly.
    """
    problems = []
    for node in (0, 1, 2, 3):
        if node not in gains:
            problems.append(f"node {node} missing from GAINS")
            continue
        vg = gains[node].get("vel_gain")
        vi = gains[node].get("vel_integrator_gain")
        if require_measured and (vg is None or vi is None):
            problems.append(
                f"node {node} ({WHEEL_BY_NODE.get(node, '?')}): "
                "vel_gain/vel_integrator_gain not measured yet"
            )
        for name, val in (("vel_gain", vg), ("vel_integrator_gain", vi)):
            if val is not None and (not isinstance(val, (int, float)) or val < 0):
                problems.append(f"node {node}: {name}={val!r} must be a number >= 0")
    if limits["current_soft_max"] >= limits["current_hard_max"]:
        problems.append(
            "current_hard_max must be strictly above current_soft_max -- with no "
            "margin the current loop trips CURRENT_LIMIT_VIOLATION on transients"
        )
    if limits["vel_integrator_limit"] > limits["current_soft_max"]:
        problems.append(
            "vel_integrator_limit exceeds current_soft_max "
            "(integrator output would be clamped by the soft current limit)"
        )
    if limits["vel_limit"] <= 0:
        problems.append("vel_limit must be > 0")
    return problems


def _shell_globals():
    """Return the interpreter namespace this script was exec()'d into.

    Inside `odrivetool --can can1` that namespace holds the per-node handles
    odrv0..odrv3 and the ControlMode / InputMode / AxisState enums. Reading them
    from here (rather than as bare names) keeps this file importable and
    lint-clean off-hardware.
    """
    return globals()


def _drives(ns):
    """Return [odrv0, odrv1, odrv2, odrv3] from the odrivetool namespace."""
    try:
        return [ns[f"odrv{i}"] for i in range(4)]
    except KeyError as missing:
        raise RuntimeError(
            f"{missing} not found -- run this inside "
            "`odrivetool --no-usb --can can1` so odrv0..odrv3 exist."
        )


def _apply_one(odrv, node, ns):
    """Write the full operational envelope to one axis (does not save)."""
    cfg = odrv.axis0.controller.config
    mcfg = odrv.axis0.config.motor
    odrv.axis0.requested_state = ns["AxisState"].IDLE      # config writes need IDLE
    cfg.control_mode = ns["ControlMode"].VELOCITY_CONTROL
    cfg.input_mode = ns["InputMode"].PASSTHROUGH
    cfg.vel_gain = GAINS[node]["vel_gain"]
    cfg.vel_integrator_gain = GAINS[node]["vel_integrator_gain"]
    cfg.vel_integrator_limit = LIMITS["vel_integrator_limit"]
    cfg.vel_limit = LIMITS["vel_limit"]
    cfg.inertia = LIMITS["inertia"]
    mcfg.current_hard_max = LIMITS["current_hard_max"]     # raise hard before soft
    mcfg.current_soft_max = LIMITS["current_soft_max"]
    print(
        f"node{node} {WHEEL_BY_NODE[node]}: vel_gain={cfg.vel_gain} "
        f"vel_int={cfg.vel_integrator_gain} vi_lim={cfg.vel_integrator_limit} "
        f"vlim={cfg.vel_limit} isoft={mcfg.current_soft_max} "
        f"ihard={mcfg.current_hard_max}"
    )


def apply_all():
    """Validate, then write + save the envelope to all four ODrives."""
    problems = validate_table(GAINS, LIMITS, require_measured=True)
    if problems:
        raise SystemExit(
            "Refusing to apply -- fix these first:\n  - " + "\n  - ".join(problems)
        )
    ns = _shell_globals()
    drives = _drives(ns)
    for node, odrv in enumerate(drives):
        _apply_one(odrv, node, ns)
    print("Saving per node (each ODrive reboots on fw 0.6.11)...")
    for node, odrv in enumerate(drives):
        try:
            odrv.save_configuration()
        except Exception as exc:  # the reboot drops the CAN link -- expected
            print(f"node{node}: save triggered reboot/disconnect ({exc})")
    print("Done. Reconnect odrivetool, re-exec this file, call verify_all().")


def verify_all():
    """Read the envelope back from all four ODrives (run after reconnect)."""
    ns = _shell_globals()
    for node, odrv in enumerate(_drives(ns)):
        cfg = odrv.axis0.controller.config
        mcfg = odrv.axis0.config.motor
        print(
            f"node{node} {WHEEL_BY_NODE[node]}: vel_gain={cfg.vel_gain:.4f} "
            f"vel_int={cfg.vel_integrator_gain:.4f} "
            f"vi_lim={cfg.vel_integrator_limit} vlim={cfg.vel_limit} "
            f"isoft={mcfg.current_soft_max} ihard={mcfg.current_hard_max} "
            f"inertia={cfg.inertia}"
        )


def check():
    """Dry-run: validate (allowing unmeasured gains) and print the planned writes."""
    problems = validate_table(GAINS, LIMITS, require_measured=False)
    status = "OK" if not problems else "PROBLEMS:\n  - " + "\n  - ".join(problems)
    print(f"validate (dry-run): {status}")
    for node in (0, 1, 2, 3):
        print(f"node{node} {WHEEL_BY_NODE[node]}: {GAINS[node]} + {LIMITS}")
