### KineticDefenseSim/src/gnc/control.py
import numpy as np

class Autopilot:
    def __init__(self, dt: float):
        self.dt = dt
        self.omega_n = 20.0
        self.zeta = 0.7
        self.fin_deflection = np.zeros(3)
        self.fin_rate = np.zeros(3)
        self.Kp = 2.0
        self.Kq = 0.5
        self.Ki = 0.1
        self.err_int = np.zeros(3)

    def update(self, cmd_acc_body: np.ndarray, current_acc_body: np.ndarray, rates: np.ndarray):
        err = cmd_acc_body - current_acc_body
        self.err_int += err * self.dt
        rate_cmd = self.Kp * err + self.Ki * self.err_int
        delta_cmd = self.Kq * (rate_cmd - rates)
        u = np.clip(delta_cmd, -0.5, 0.5)
        fin_acc = (self.omega_n**2 * u) - (2 * self.zeta * self.omega_n * self.fin_rate) - (self.omega_n**2 * self.fin_deflection)
        self.fin_rate += fin_acc * self.dt
        self.fin_deflection += self.fin_rate * self.dt
        return self.fin_deflection