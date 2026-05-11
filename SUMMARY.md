# Lint → CI + pre-commit refactor

Branch: `chore/lint-to-ci-and-precommit` (cut from `main` at `1528b9d`).

## What changed

Lint (flake8, pep257, copyright check) and other style checks moved out of
the colcon test suite. They now run as:

- **Pre-commit hooks** locally on every commit (`.pre-commit-config.yaml`).
- **`pre-commit/action`** in GitHub Actions before colcon build, so a
  contributor who skipped `pre-commit install` still gets lint enforced
  in CI.

Functional tests (pytest, launch_testing) stay in `colcon test` and are
gated by the same CI job.

## Added

| Path | Purpose |
|---|---|
| `.github/workflows/ci.yml` | GitHub Actions workflow. Two jobs: `pre-commit` (lint) and `colcon` (build + test on ubuntu-22.04, Humble, `ROS_DOMAIN_ID=42`). |
| `.pre-commit-config.yaml` | Hook config. flake8 inherits workspace `.flake8`. pydocstyle mirrors ament_pep257's default `add-ignore` list (D203, D212, D404 plus D100-D107). Hygiene hooks: trailing-whitespace, end-of-file-fixer, check-merge-conflict, check-yaml. Python lint scoped to `^src/.*\.py$`; sim/, legacy/, analysis/, build/, install/, log/, venv/ excluded. |

## Removed per package

| Package | `package.xml` test_depends removed | CMake change | Test files deleted |
|---|---|---|---|
| `fortis_arm` | `ament_copyright`, `ament_flake8`, `ament_pep257` | – | `test/test_flake8.py`, `test/test_pep257.py` |
| `fortis_bringup` | `ament_copyright`, `ament_flake8`, `ament_pep257` | – | `test/test_flake8.py`, `test/test_pep257.py` |
| `fortis_comms` | `ament_copyright`, `ament_flake8`, `ament_pep257` | – | `test/test_flake8.py`, `test/test_pep257.py` |
| `fortis_description` | `ament_lint_auto`, `ament_lint_common` | Dropped `BUILD_TESTING` block that called `ament_lint_auto_find_test_dependencies()` | – (no test scaffolds were present) |
| `fortis_drive` | `ament_copyright`, `ament_flake8`, `ament_pep257` | – | `test/test_flake8.py`, `test/test_pep257.py` |
| `fortis_integration_tests` | `ament_copyright`, `ament_flake8`, `ament_pep257` | – | `test/test_flake8.py`, `test/test_pep257.py` |
| `fortis_msgs` | `ament_lint_common` | Dropped `BUILD_TESTING` block that called `ament_lint_auto_find_test_dependencies()` | – |
| `fortis_safety` | `ament_copyright`, `ament_flake8`, `ament_pep257` | – | `test/test_flake8.py`, `test/test_pep257.py` |

Total: 12 lint-only test files deleted, 23 `test_depend` lines removed
across 8 `package.xml` files, two `BUILD_TESTING`/`ament_lint_auto` blocks
stripped from `CMakeLists.txt`.

`python3-pytest` and all functional `test_depend`s
(`launch_testing`, `launch_testing_ros`, `rclpy`, `std_msgs`, `std_srvs`,
`geometry_msgs`, `fortis_*` interpackage deps) are preserved.

`test_copyright.py` files in `fortis_drive` and `fortis_safety` are kept:
they were scaffold tests already marked `@pytest.mark.skip`, so leaving
them in does not change behavior and avoids touching the copyright story.

## Documentation

- `README.md` Dev environment section grew two subsections: **Pre-commit
  hooks** (host install instructions) and **CI** (what runs on PRs).

## Verification

Three verification steps were requested. Results below.

### 1. `colcon build` + `colcon test`

**Not run in this worktree.** colcon and the ROS 2 Humble toolchain live
inside the `fortis-dev` container; this worktree is on the Windows host.
Run the verification yourself in the dev container:

```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Expected drop in test count: the README baseline on `main` was
**126 tests, 0 failures, 8 skipped**. Lint-only `test_flake8.py` /
`test_pep257.py` files were removed from 6 packages (12 files total).
Each registered one pytest test, plus `ament_lint_auto` in
`fortis_description` and `fortis_msgs` registered the standard
`ament_lint_common` battery (flake8, pep257, copyright, xmllint, cpplint
where applicable). Order-of-magnitude expectation: drop from 126 to
roughly 40-50, all functional, all passing. Record the exact number when
you run it.

Remaining functional tests by package:

| Package | Tests kept |
|---|---|
| `fortis_arm` | `test_bringup.py` (2), `test_state_gating.py` (4) |
| `fortis_bringup` | – (no tests; package is launch-only) |
| `fortis_comms` | `test_imports.py` (3), `test_xdrive_kinematics.py` (parametrized), `test_ekf.py` (empty no-op module) |
| `fortis_description` | – |
| `fortis_drive` | `test_drive_node.py` (5), `test_copyright.py` (1 skipped) |
| `fortis_integration_tests` | `test_bringup_launch.py` (2), `test_safety_arm_integration.py` (4), `test_safety_drive_integration.py` (4) |
| `fortis_msgs` | – |
| `fortis_safety` | `test_mission_state_machine.py` (20), `test_copyright.py` (1 skipped) |

### 2. `pre-commit run --all-files`

Run on the Windows host:

```
pre-commit 4.6.0
trim trailing whitespace.................................................Passed
fix end of files.........................................................Passed
check for merge conflicts................................................Passed
check yaml...............................................................Passed
flake8...................................................................Passed
pydocstyle...............................................................Passed
```

All six hooks pass on the current tree. No auto-fix changes to review.

Note: initial run of pydocstyle with `--convention=pep257` alone reported
~50 missing-docstring violations (D100-D107) on existing code. Fixed by
mirroring ament_pep257's default `ament_pep257.ini` ignore list, so
pydocstyle accepts exactly what ament_pep257 was accepting.

### 3. `ci.yml` YAML syntax

Validated with `python -c "import yaml; yaml.safe_load(open(...))"` for
both `.github/workflows/ci.yml` and `.pre-commit-config.yaml`. Both parse
cleanly. Workflow was not triggered against GitHub Actions from this
branch.

## Commit list

```
0b6b880 chore: mirror ament_pep257 ignore list in pydocstyle hook
1f812b3 ci: run pre-commit hooks before colcon build/test
4d78959 docs: document pre-commit and CI workflow in README
3ff8c96 test: delete ament_flake8 / ament_pep257 scaffold tests
1feb877 chore: drop ament_lint_auto from CMakeLists.txt
c427c55 chore: drop ament_lint test_depends from package.xml
e3503a5 chore: add pre-commit hook configuration
0715a1f ci: add GitHub Actions workflow for colcon build + test
```

## Files explicitly NOT touched

Reserved for in-progress work on `feat/bringup-wiring`:

- `src/fortis_bringup/launch/bringup.launch.py`
- `src/fortis_integration_tests/test/test_bringup_launch.py`

The package.xml in `fortis_integration_tests` was edited (lint
test_depend removal only); no functional or test code in that package
was changed. `fortis_bringup/launch/*.py` were untouched entirely.
