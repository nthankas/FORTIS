# FORTIS V3 Arm Sweep -- Torque Summary

30-inch CF arm, loaded with 3 lb payload. Valid pose count: **956**.

Physics-measured |tau| over valid poses (geometry + reactor envelope).
J1 omitted -- gravity torque on a vertical-axis joint is identically 0;
J1 is sized against arm slewing inertia (table at the bottom).

| Joint | Mean (Nm) | P95 (Nm) | Peak (Nm) | Cont. rating (Nm) | P95 util | Peak util |
|---|---:|---:|---:|---:|---:|---:|
| J2 (shoulder) | 5.526 | 12.260 | 14.309 | 30.0 | 40.9% | 47.7% |
| J3 (elbow) | 3.265 | 6.190 | 6.757 | 12.0 | 51.6% | 56.3% |
| J4 (wrist) | 1.206 | 1.897 | 1.907 | 4.9 | 38.7% | 38.9% |

## J1 inertial torque (analytical)

Worst-case arm inertia about J1 axis: **I_zz = 0.6679 kg*m^2**
(at FK pose J2=0.0, J3=60.0, J4=-155.0 deg)

J1 torque required to angularly accelerate the arm at various rates:

| Angular accel (deg/s^2) | T_J1 required (Nm) | % of NEMA17+Cricket 12 Nm |
|---:|---:|---:|
| 10 | 0.117 | 1.0% |
| 30 | 0.350 | 2.9% |
| 60 | 0.699 | 5.8% |
| 90 | 1.049 | 8.7% |
| 180 | 2.098 | 17.5% |
