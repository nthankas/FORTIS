# Arm Torque Sweep — Mass Breakdown

Configuration: All NEMA 17 + Cricket Drive MK II 25:1, 0.25" 6061-T6 aluminum links,
Gemini 2 camera at mid-L3.

## Link Masses (bare aluminum, 0.25" thick 6061-T6)

| Link | Span | Mass (lbs) | Mass (g) |
|------|------|-----------|----------|
| L2 (J2 to J3) | 576.1 mm (22.68") | 0.689 | 312.5 |
| L3 (J3 to J4) | 500.0 mm (19.69") | 0.547 | 248.1 |
| L4 (J4 to EE) | 150.0 mm (5.91")  | 0.223 | 101.2 |
| **Total links** | | **1.459** | **661.8** |

## Motors (all identical)

| Param | Value |
|-------|-------|
| Model | StepperOnline 17HS24-2104S (NEMA 17) |
| Mass | 500 g |
| Holding torque | 0.65 Nm |
| Rated current | 2.1 A/phase |
| Phase resistance | 1.6 ohm |
| Inductance | 3 mH |
| Dimensions | 42 x 42 x 60 mm |
| Insulation | Class B (130 C) |

## Gearboxes (all identical)

| Param | Value |
|-------|-------|
| Model | Cricket Drive MK II (Sweep Dynamics) |
| Mass | 80 g |
| Ratio | 25:1 |
| Efficiency | 85.5% |
| Rated torque | 11-12 Nm |
| Backlash | +/- 8.5 arcmin |

## Camera

| Param | Value |
|-------|-------|
| Model | Orbbec Gemini 2 |
| Mass | 445 g |
| Location | Mid-L3 (250 mm from J3) |

## Lumped Mass Model (per body in articulation)

| Body | Contents | Mass (g) |
|------|----------|----------|
| J1_base | NEMA 17 + Cricket + misc | 630 |
| J2_shoulder | NEMA 17 + Cricket + L2 (312.5g) + misc | 942.5 |
| J3_elbow | NEMA 17 + Cricket + L3 (248.1g) + Gemini 2 (445g) + misc | 1323.1 |
| J4_wrist | NEMA 17 + Cricket + L4 (101.2g) + misc | 731.2 |
| **Total arm** | | **3626.8 (7.99 lbs)** |

Misc = 50 g per joint (brackets, couplers, fasteners, wiring share).

## Derived Motor Constants

| Param | Value |
|-------|-------|
| Kt (torque constant) | 0.3095 Nm/A (= 0.65 / 2.1) |
| Max output torque (gearbox rated) | 12 Nm |
| Max output torque (motor stall) | 13.9 Nm (0.65 x 25 x 0.855) |
| Current for 12 Nm output | 1.81 A (= 12 / 25 / 0.855 / 0.3095) |
| Holding power at 12 Nm | 10.5 W (= 2 x 1.81^2 x 1.6) |
