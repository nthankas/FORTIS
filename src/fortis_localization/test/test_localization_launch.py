"""
Config/launch sanity for the fortis_localization EKF variants.

Light checks only: both yaml configs must parse, the VIO variant must
fuse /fortis/vo as velocity-only (vx, vy) without disturbing the wheel
and IMU channels, and the launch file must stay syntactically valid and
declare the vio arg. No live EKF is started.
"""

from __future__ import annotations

import ast
from pathlib import Path

import yaml

PKG_ROOT = Path(__file__).resolve().parents[1]
EKF_YAML = PKG_ROOT / "config" / "ekf.yaml"
EKF_VIO_YAML = PKG_ROOT / "config" / "ekf_vio.yaml"
LAUNCH_FILE = PKG_ROOT / "launch" / "localization.launch.py"


def _params(path):
    """Load a robot_localization yaml and return its ros__parameters dict."""
    data = yaml.safe_load(path.read_text())
    return data["ekf_filter_node"]["ros__parameters"]


def test_default_ekf_yaml_parses_and_stays_wheel_imu_only():
    """Parse the default config and keep it free of any VO input."""
    params = _params(EKF_YAML)
    assert params["odom0"] == "/odom"
    assert params["imu0"] == "/imu"
    assert "odom1" not in params


def test_ekf_vio_yaml_fuses_vo_velocities_only():
    """Fuse /fortis/vo as odom1 with vx, vy and nothing else."""
    params = _params(EKF_VIO_YAML)
    assert params["odom1"] == "/fortis/vo"
    config = params["odom1_config"]
    assert len(config) == 15
    assert config[6] is True and config[7] is True  # vx, vy
    assert sum(bool(flag) for flag in config) == 2  # no pose, no vyaw
    assert params["odom1_differential"] is False
    assert params["odom1_relative"] is False
    assert params["odom1_queue_size"] == 10


def test_vio_variant_preserves_wheel_and_imu_channels():
    """Keep odom0/imu0 and the noise matrices identical across variants."""
    base = _params(EKF_YAML)
    vio = _params(EKF_VIO_YAML)
    for key in (
        "odom0",
        "odom0_config",
        "imu0",
        "imu0_config",
        "two_d_mode",
        "publish_tf",
        "process_noise_covariance",
        "initial_estimate_covariance",
    ):
        assert vio[key] == base[key], f"{key} drifted between ekf.yaml and ekf_vio.yaml"


def test_launch_file_compiles_and_declares_vio():
    """Compile the launch file and check the vio arg selects ekf_vio.yaml."""
    source = LAUNCH_FILE.read_text()
    ast.parse(source)
    assert '"vio"' in source
    assert "ekf_vio.yaml" in source
    assert "ekf.yaml" in source
