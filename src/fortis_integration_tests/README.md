# fortis_integration_tests

Cross-package integration tests for FORTIS. Each test launches one or more real ROS 2 nodes (via `launch_testing`) and exercises the contract between them. No runtime code lives here -- this package is test-only.

## Why a separate package

Three reasons:

1. **Process isolation.** colcon test runs each package's tests as one pytest process. A `launch_testing` test launches subprocesses that linger in DDS state until shutdown. If that test shared a pytest process with unit tests, the lingering publishers would cross-talk with the unit tests' own helper publishers on shared topic names (`/fortis/mission_state`, etc.) and produce intermittent flakes.
2. **Domain isolation.** A package-local `test/conftest.py` sets a per-PID `ROS_DOMAIN_ID` so this package's launched nodes do not collide with `fortis_arm` or `fortis_drive` unit-test publishers running in parallel under colcon.
3. **Different dependency profile.** Integration tests legitimately depend on every package they integrate (`fortis_safety`, `fortis_drive`, `fortis_comms`, `fortis_msgs`). Adding those as `<test_depend>` to one of the production packages would imply a coupling the production code does not actually have.

## Tests

| Test file | What it exercises |
|---|---|
| `test/test_safety_drive_integration.py` | Brings up `mission_state_node` (fortis_safety) and `drive_node` (fortis_drive) in one launch. Verifies (a) default-IDLE rejects `/cmd_vel` for >= 1 s producing only `zero_velocities`; (b) `START_ORBIT` event makes `/cmd_vel` produce `wheel_velocities` matching the X-drive IK; (c) transitioning out of `ORBIT` to `IDLE` makes the next `/cmd_vel` produce zeros within 200 ms; (d) the full `INSPECT -> DONE -> RETURN_HOME` path restores motion. |

## Running

```bash
cd /workspace
colcon build --symlink-install --packages-select \
  fortis_msgs fortis_comms fortis_safety fortis_drive fortis_integration_tests
source install/setup.bash
colcon test --packages-select fortis_integration_tests
colcon test-result --verbose
```

To run the launch test directly (more verbose, shows the inner `unittest` test names and individual outcomes rather than the single pytest wrapper):

```bash
launch_test src/fortis_integration_tests/test/test_safety_drive_integration.py
```

## Adding a new integration test

1. Add `test/test_<scenario>.py` following the same `@pytest.mark.launch_test` + `unittest.TestCase` pattern.
2. Import topic names, QoS profiles, and event / context identifiers from the production modules; do **not** hardcode strings -- the integration test's job is to catch divergence between producer and consumer, which only works if both sides import from the same source of truth.
3. Use `rclpy` clock + polling loops for timing assertions, not `time.sleep`. Bound every wait with an explicit timeout.
4. Add `<test_depend>` entries to `package.xml` for any new package the test launches or imports from.
