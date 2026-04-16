import json, os
import numpy as np


class EKF:
    CONFIG_FILE = "/cfgs/ekf_params.json"
    def __init__(self):
        # initialize configs
        if os.path.exists(self.CONFIG_FILE):
            _ekf_params = json.load(open(self.CONFIG_FILE))
        self.P = np.diag(_ekf_params["initial_variance"])
        self.Q = np.diag(_ekf_params["process_noise"])
        self.R_of = np.diag(_ekf_params["optical_flow_noise"])   # 2x2
        self.R_imu = np.diag(_ekf_params["imu_noise"])
                             
        self.state = np.zeros(6, dtype=float)


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
        H = np.array([[1,0,0,0,0,0],
                     [0,1,0,0,0,0]])
        z = np.array([dx, dy])
        y = z - H @ self.state
        S = H @ self.P @ H.T + self.R_of
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def update_imu(self, dt, ax, ay, omega):
        z = np.array([self.state[3] + ax * dt, self.state[4]+ ay * dt, omega])
        H = np.array([[0,0,0,1,0,0],
                      [0,0,0,0,1,0],
                      [0,0,0,0,0,1]])
        y = z - H @ self.state
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
