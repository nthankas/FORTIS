import json, tempfile
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import pytest


@pytest.fixture(autouse=True)
def ekf_config(tmp_path):
    """Create config and patch EKF to use it."""
    config = {
        "initial_variance": [0.01, 0.01, 0.01, 0.1, 0.1, 0.1],
        "process_noise": [0.0001, 0.0001, 0.001, 0.01, 0.01, 0.01],
        "optical_flow_noise": [0.001, 0.001],
        "imu_noise": [0.01, 0.01, 0.01]
    }
    config_path = str(tmp_path / "ekf_params.json")
    with open(config_path, "w") as f:
        json.dump(config, f)

    from ekf import EKF
    EKF.CONFIG_FILE = config_path
    return config


class TestInit:
    def test_state_is_zero(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        np.testing.assert_array_equal(ekf.state, np.zeros(6))

    def test_P_shape(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        assert ekf.P.shape == (6, 6)

    def test_Q_shape(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        assert ekf.Q.shape == (6, 6)

    def test_R_of_shape(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        assert ekf.R_of.shape == (2, 2)

    def test_R_imu_shape(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        assert ekf.R_imu.shape == (3, 3)

    def test_P_diagonal_values(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        expected = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        np.testing.assert_array_almost_equal(ekf.P, expected)


class TestPredict:
    def test_zero_state_stays_zero(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        ekf.predict(dt=0.01)
        np.testing.assert_array_almost_equal(ekf.state, np.zeros(6))

    def test_constant_velocity_propagates(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        ekf.state = np.array([0.0, 0.0, 0.0, 1.0, 0.5, 0.1])
        ekf.predict(dt=0.1)
        # x += vx*dt = 0 + 1.0*0.1 = 0.1
        # y += vy*dt = 0 + 0.5*0.1 = 0.05
        # theta += omega*dt = 0 + 0.1*0.1 = 0.01
        # vx, vy, omega unchanged
        expected = np.array([0.1, 0.05, 0.01, 1.0, 0.5, 0.1])
        np.testing.assert_array_almost_equal(ekf.state, expected)

    def test_P_grows_after_predict(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        P_before = ekf.P.copy()
        ekf.predict(dt=0.01)
        # diagonal should grow because Q is added
        for i in range(6):
            assert ekf.P[i, i] >= P_before[i, i]

    def test_P_stays_symmetric(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        ekf.state = np.array([1.0, 2.0, 0.5, 0.3, -0.1, 0.05])
        for _ in range(50):
            ekf.predict(dt=0.01)
        np.testing.assert_array_almost_equal(ekf.P, ekf.P.T)


class TestUpdateOpticalFlow:
    def test_state_moves_toward_measurement(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        ekf.predict(dt=0.01)
        ekf.update_optical_flow(dx=0.5, dy=0.3)
        # state x should move toward 0.5, y toward 0.3
        assert ekf.state[0] > 0.0
        assert ekf.state[1] > 0.0

    def test_P_shrinks_after_update(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        ekf.predict(dt=0.01)
        P_before = ekf.P.copy()
        ekf.update_optical_flow(dx=0.1, dy=0.1)
        # x and y uncertainty should decrease
        assert ekf.P[0, 0] < P_before[0, 0]
        assert ekf.P[1, 1] < P_before[1, 1]

    def test_zero_measurement_no_change(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        # state is zero, measurement is zero, innovation is zero
        ekf.update_optical_flow(dx=0.0, dy=0.0)
        np.testing.assert_array_almost_equal(ekf.state[:2], [0.0, 0.0])

    def test_repeated_same_measurement_converges(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        for _ in range(100):
            ekf.predict(dt=0.01)
            ekf.update_optical_flow(dx=1.0, dy=2.0)
        # should converge close to measurement
        assert abs(ekf.state[0] - 1.0) < 0.1
        assert abs(ekf.state[1] - 2.0) < 0.1


class TestUpdateIMU:
    def test_omega_corrects_heading_rate(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        ekf.predict(dt=0.01)
        ekf.update_imu(dt=0.01, ax=0.0, ay=0.0, omega=0.5)
        # omega state should move toward 0.5
        assert ekf.state[5] > 0.0

    def test_acceleration_corrects_velocity(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        ekf.predict(dt=0.01)
        ekf.update_imu(dt=0.01, ax=1.0, ay=0.0, omega=0.0)
        # vx should increase (ax integrated into velocity measurement)
        assert ekf.state[3] > 0.0

    def test_P_shrinks_after_imu_update(self, ekf_config):
        from ekf import EKF
        ekf = EKF()
        ekf.predict(dt=0.01)
        P_before = ekf.P.copy()
        ekf.update_imu(dt=0.01, ax=0.0, ay=0.0, omega=0.1)
        # omega uncertainty should decrease
        assert ekf.P[5, 5] < P_before[5, 5]


class TestFullLoop:
    def test_predict_update_cycle_stable(self, ekf_config):
        """Run 1000 iterations, confirm no NaN or explosion."""
        from ekf import EKF
        ekf = EKF()
        for i in range(1000):
            ekf.predict(dt=0.01)
            ekf.update_optical_flow(dx=0.001 * i, dy=0.0005 * i)
            ekf.update_imu(dt=0.01, ax=0.01, ay=0.0, omega=0.001)
        assert not np.any(np.isnan(ekf.state))
        assert not np.any(np.isinf(ekf.state))
        assert np.all(np.abs(ekf.state) < 1e6)

    def test_P_stays_positive_definite(self, ekf_config):
        """P must stay positive definite for the filter to be valid."""
        from ekf import EKF
        ekf = EKF()
        for _ in range(200):
            ekf.predict(dt=0.01)
            ekf.update_optical_flow(dx=0.1, dy=0.05)
            ekf.update_imu(dt=0.01, ax=0.0, ay=0.0, omega=0.02)
        eigenvalues = np.linalg.eigvalsh(ekf.P)
        assert np.all(eigenvalues > 0)