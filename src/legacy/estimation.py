### KineticDefenseSim/src/legacy/estimation.py
import numpy as np

class KalmanFilter:
    def __init__(self, dt: float, pos_std: float, vel_std: float):
        self.dt = dt
        self.F = np.eye(6)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt
        
        self.H = np.eye(6)
        
        q_pos = 0.1
        q_vel = 0.1
        self.Q = np.diag([q_pos]*3 + [q_vel]*3)
        
        self.R = np.diag([pos_std**2]*3 + [vel_std**2]*3)
        
        self.x = np.zeros(6)
        self.P = np.eye(6) * 500.0

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement: np.ndarray):
        y = measurement - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(self.F.shape[0])
        self.P = (I - K @ self.H) @ self.P

    def get_state(self) -> np.ndarray:
        return self.x
