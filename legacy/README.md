# legacy/

Reference code from earlier design iterations. Not imported by anything in the current build, kept so we can look back at how a decision was made.

## Contents

- `coppelia_ik_reference/inverse_kinetmatic_solver.py` - Carlos Vazquez's CoppeliaSim IK solver for a 4-DOF arm. Filename typo ("kinetmatic") preserved from original. Reference only - the current arm planning happens in `sim/isaac/xdrive/lib/arm_ik.py` and `canonical/xdrive_reactor_arm.py`. Targets a different sim, different DH params, and a different arm design (the older 4-DOF concept before the parallel-link rework).

Don't add new code here. If something is being deprecated, a `deprecated/` subdir alongside the active code (the way `sim/isaac/xdrive/deprecated/` is laid out) keeps it next to its replacement and easier to find.
