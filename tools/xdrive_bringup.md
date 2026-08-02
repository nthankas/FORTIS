# FORTIS X-drive hardware test — bench runbook

End-to-end test: operator drives from Foxglove (PC) → mission FSM + X-drive
kinematics on the Jetson → `odrive_ros2_control` → CAN → 4× ODrive S1 → wheels.

Everything runs **in the container on the Jetson**. From the PC:
`ssh fortis@<jetson-tailscale-ip>` → `cd /data/fortis_ws/src/FORTIS && ./stack exec`
→ inside: `source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash`.

---

## 0. Wiring (do before powering anything)

- [ ] CAN daisy-chain **FL → FR → RR → RL** (chain position = CAN node_id: FL=0, FR=1, RR=2, RL=3).
- [ ] On each S1, the two CAN ports **J16 / J17 are parallel and interchangeable** — use one for the incoming cable, the other for outgoing. Wire **CANH, CANL, and GND** through.
- [ ] Terminate **both physical ends**: USB-CAN adapter termination switch ON at the FL end; RL's on-board 120 Ω jumper closed at the far end. (Nominal bus resistance ≈ 60 Ω across CANH–CANL with power off.)
- [ ] CAN GND tied to DC− at a single system star point (no ground loops).
- [ ] Motors mounted on the frame, or robot **up on blocks / wheels off the ground** for first power-on (see §4 — rotation-sign check).

## 1. CAN interface up

The committed interface is the **ODrive USB-CAN adapter**. JetPack 6's stock
kernel lacked `gs_usb`, but it is now installed on the Jetson (slamcore-dkms)
and auto-loads on plug-in, so the adapter appears as `canX` (e.g. `can1`) when
plugged. (Onboard Tegra CAN — `can0`, driver `mttcan` — also works if a CAN
transceiver is wired to the Jetson CAN pins, but the USB-CAN adapter is the
committed path; use it.)

Identify the interface, then bring it up at the bus bitrate:

```bash
ip -br link                                   # find the interface (canX)
sudo ip link set canX up type can bitrate 250000
sudo ip link set canX txqueuelen 1000
```

- [ ] **Heartbeats from all four:** `candump canX` shows 10 Hz frames at IDs
      `0x001 0x021 0x041 0x061` (node 0–3 heartbeat = `node_id<<5 | 0x01`). All four present = bus good.
      Missing one → check that ODrive's power, node_id, and chain link.

> Bitrate must match the value set on the ODrives (FORTIS uses 250000). A mismatch
> shows as zero frames, not an error.

## 2. Launch the stack (one command)

```bash
ros2 launch fortis_bringup drive_test.launch.py can_interface:=canX
```

Brings up controller_manager + ODrive plugin (all 4 wheels), the mission FSM,
drive_node, health monitor, and the Foxglove bridge. **No motor arms on launch** —
the wheel controller loads INACTIVE on purpose.

- [ ] `ros2 control list_hardware_interfaces` → all 4 wheels show a `velocity`
      command interface + `position/velocity/effort` state, all `available`.
- [ ] `ros2 control list_controllers` → `joint_state_broadcaster [active]`,
      `wheel_velocity_controller [inactive]`.

## 3. Connect Foxglove (PC)

- [ ] Foxglove Studio → Open connection → Foxglove WebSocket →
      `ws://<jetson-tailscale-ip>:8765` (Tailscale) or the wired/LAN IP.
- [ ] Import the ready-made layout **`foxglove/fortis_xdrive_teleop.json`**
      (Studio → Layouts → Import from file…). It loads a **Translate** pad
      (fwd/back + strafe), a **Rotate** pad, **START ORBIT** / **STOP** buttons,
      a **plot** of `/fortis/drive/wheel_velocities`, the **mission_state**
      readout, and a **3D** view. Once imported it persists in Studio's layout
      list; re-import from the repo to restore it on any machine.

## 4. Arm and drive — wheels OFF the ground for the first run

All from the Foxglove layout — no CLI needed:

- [ ] Click **ENABLE DRIVE** (green) → `drive_enable_node` activates the wheel
      controller; the four ODrive S1s enter CLOSED_LOOP_CONTROL and
      `/fortis/drive/armed` goes true.
- [ ] Click **START ORBIT** → mission FSM IDLE → ORBIT; `drive_node` now
      accepts `/cmd_vel`.
- [ ] Drive with the **Translate** / **Rotate** pads.

CLI equivalents (fallback / debugging):
```bash
ros2 topic pub --once /fortis/commands/drive_enable std_msgs/msg/Bool "{data: true}"  # ARM
ros2 topic pub --once /fortis/events/start_orbit std_msgs/msg/Empty "{}"              # gate
```

Verify each axis (slowly):

- [ ] **Forward (Vx)** → all four wheels roll the robot forward.
- [ ] **Strafe (Vy)** → robot translates sideways (no rotation).
- [ ] **Rotate (ωz) — THE CRITICAL CHECK.** Command a slow pure rotation.
      The robot must spin the direction you intend. The X-drive H-matrix's
      ω-column signs were inherited from senior design and **never re-derived**
      against the URDF wheel yaws; the round-trip unit tests cannot catch a
      sign error (it cancels through the pseudo-inverse). If it spins the wrong
      way or tries to translate, that's the bug — a one-line fix in
      `src/fortis_comms/fortis_comms/xdrive_kinematics.py` (H-matrix ω column).
      **Do this with wheels off the ground.**

Expected wheel directions (verify against reality):
| Command | FL | FR | RL | RR |
|---|---|---|---|---|
| +Vx (fwd) | + | + | + | + |
| +Vy (left)| + | − | − | + |
| +ωz (CCW) | + | − | + | − |

## 5. Teardown

```bash
ros2 control switch_controllers --deactivate wheel_velocity_controller  # idles motors
# Ctrl-C the launch
sudo ip link set canX down
```

---

## Dry run without any hardware (optional, de-risks the software)

The Jetson has `vcan`, so you can prove the whole Foxglove→plugin→CAN transmit
path before the bus is wired:

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
ros2 launch fortis_bringup drive_test.launch.py can_interface:=vcan0
# in another shell: candump vcan0   (after activating the controller in §4)
```

You'll see real `Set_Input_Vel` frames (IDs `0x00D 0x02D 0x04D 0x06D`) whose
payloads track your Foxglove input — every software link validated, zero hardware.

## Troubleshooting

- **`candump` empty** → nothing transmitting: bus not powered, wrong interface,
  or bitrate mismatch.
- **`RTNETLINK answers: Device or resource busy`** → interface already up.
- **Plugin fails `on_configure`** → the CAN interface doesn't exist; check `ip -br link`.
- **One wheel doesn't move** → that node_id's chain link / power / calibration.
- **Foxglove won't connect** → confirm `ping` to the Jetson works and port 8765 is open.
