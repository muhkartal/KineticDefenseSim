### KineticDefenseSim/src/estimation/ekf.py
import numpy as np

class EKF6DOF:
    def __init__(self, dt: float, process_noise, measure_noise):
        self.dt = dt
        self.x = np.zeros(9)
        self.P = np.eye(9) * 100.0
        self.Q = np.eye(9) * process_noise
        self.R = np.eye(3) * measure_noise

    def f_jacobian(self, x):
        dt = self.dt
        F = np.eye(9)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        F[0, 6] = 0.5 * dt**2
        F[1, 7] = 0.5 * dt**2
        F[2, 8] = 0.5 * dt**2
        F[3, 6] = dt
        F[4, 7] = dt
        F[5, 8] = dt
        return F

    def h_jacobian(self, x):
        px, py, pz = x[0], x[1], x[2]
        r2 = px**2 + py**2 + pz**2
        r = np.sqrt(r2)
        rho2 = px**2 + py**2
        rho = np.sqrt(rho2)
        H = np.zeros((3, 9))
        if r > 1e-3:
            H[0, 0] = px/r
            H[0, 1] = py/r
            H[0, 2] = pz/r
            H[1, 0] = -py/rho2
            H[1, 1] = px/rho2
            H[2, 0] = (px * pz) / (rho * r2)
            H[2, 1] = (py * pz) / (rho * r2)
            H[2, 2] = -rho / r2
        return H

    def h_func(self, x):
        px, py, pz = x[0], x[1], x[2]
        r = np.sqrt(px**2 + py**2 + pz**2)
        az = np.arctan2(py, px)
        el = np.arctan2(-pz, np.sqrt(px**2 + py**2))
        return np.array([r, az, el])

    def predict(self):
        F = self.f_jacobian(self.x)
        pos = self.x[0:3]
        vel = self.x[3:6]
        acc = self.x[6:9]
        new_pos = pos + vel*self.dt + 0.5*acc*self.dt**2
        new_vel = vel + acc*self.dt
        new_acc = acc * 0.99
        self.x = np.concatenate((new_pos, new_vel, new_acc))
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement_polar):
        z = measurement_polar
        H = self.h_jacobian(self.x)
        y = z - self.h_func(self.x)
        y[1] = (y[1] + np.pi) % (2 * np.pi) - np.pi
        y[2] = (y[2] + np.pi) % (2 * np.pi) - np.pi
        S = H @ self.P @ H.T + self.R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = np.zeros((9, 3))
        self.x = self.x + K @ y
        I = np.eye(9)
        self.P = (I - K @ H) @ self.P