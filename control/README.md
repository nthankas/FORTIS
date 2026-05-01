# control/

Pure-Python libraries that don't (yet) live in a ROS 2 package. Things end up here when:

- They predate the ROS 2 migration and we haven't decided their fate (e.g. `fortis_comms/`).
- They're useful in scripts and notebooks outside a ROS context (analysis, prototyping).
- They will eventually be wrapped by a ROS node, but the wrapping is not the priority right now.

Anything that has a ROS 2 entry point or talks to a ROS node belongs in `src/` instead.

## Subdirectories

- `fortis_comms/` - motor abstractions, ODrive S1 wrapper, X-drive kinematics, EKF. See its README for the integration path.

## Running tests

```bash
cd control/fortis_comms
python3 -m pytest -v
```
