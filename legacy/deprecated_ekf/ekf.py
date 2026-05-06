"""
Six-state EKF (x, y, theta, vx, vy, omega) for X-drive odometry.

INTERIM module. Production localization will use the ``robot_localization``
ROS 2 package's ``ekf_node`` once IMU + wheel-encoder topics exist and
the EKF noise covariances have been re-tuned against real hardware
data. This file's math has been validated in isolation and stays here
as a reference / unit-test fixture; do not extend it. See
``docs/adr/0002-arm-and-drive-use-upstream-ros2-packages.md``.
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from importlib.resources import files
from pathlib import Path
from typing import Optional, Union

import numpy as np


# Required keys in the noise-covariance config. Validated up front so a
# malformed file produces a clear KeyError instead of an obscure NumPy
# error five lines into __init__.
REQUIRED_KEYS = (
    "initial_variance",
    "process_noise",
    "optical_flow_noise",
    "imu_noise",
)


def _load_default_params() -> dict:
    """Read the bundled default ekf_params.json via importlib.resources."""
    resource = files("fortis_comms.cfgs") / "ekf_params.json"
    return json.loads(resource.read_text())


class EKF:
    """
    6-state EKF with optical-flow and IMU updates.

    The default constructor loads the bundled noise covariances from
    ``fortis_comms/cfgs/ekf_params.json`` via importlib.resources. Pass a
    dict or a path to override (useful in tests and for retuning at
    runtime without rebuilding the package).
    """

    def __init__(
        self,
        config: Optional[Union[str, Path, Mapping]] = None,
    ) -> None:
        params = self._resolve_params(config)
        self._validate_params(params)

        self.P = np.diag(params["initial_variance"])
        self.Q = np.diag(params["process_noise"])
        self.R_of = np.diag(params["optical_flow_noise"])   # 2x2
        self.R_imu = np.diag(params["imu_noise"])           # 3x3

        self.state = np.zeros(6, dtype=float)

    # --- Config loading ----------------------------------------------------

    @staticmethod
    def _resolve_params(
        config: Optional[Union[str, Path, Mapping]],
    ) -> Mapping:
        if config is None:
            return _load_default_params()
        if isinstance(config, Mapping):
            return config
        path = Path(config)
        if not path.is_file():
            raise FileNotFoundError(
                f"EKF config not found: {path}. Pass a dict, a valid path, "
                "or omit to use the bundled default."
            )
        return json.loads(path.read_text())

    @staticmethod
    def _validate_params(params: Mapping) -> None:
        missing = [k for k in REQUIRED_KEYS if k not in params]
        if missing:
            raise KeyError(
                f"EKF config missing required keys: {missing}. "
                f"Required: {list(REQUIRED_KEYS)}."
            )

    # --- Filter ------------------------------------------------------------

    def predict(self, dt) -> None:
        F = np.array([[1, 0, 0, dt, 0, 0],
                     [0, 1, 0, 0, dt, 0],
                     [0, 0, 1, 0, 0, dt],
                     [0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 1, 0],
                     [0, 0, 0, 0, 0, 1]])
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q

    def update_optical_flow(self, dx, dy) -> None:
        H = np.array([[1, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0]])
        z = np.array([dx, dy])
        y = z - H @ self.state
        S = H @ self.P @ H.T + self.R_of
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def update_imu(self, dt, ax, ay, omega):
        z = np.array([self.state[3] + ax * dt, self.state[4] + ay * dt, omega])
        H = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
        y = z - H @ self.state
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
