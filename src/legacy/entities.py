### KineticDefenseSim/src/legacy/entities.py
import numpy as np
from src.legacy.physics import rk4_integration

class Projectile:
    def __init__(self, pos, vel, mass=40.0, cd=0.3, area=0.1):
        self.state = np.concatenate((np.array(pos, dtype=float), np.array(vel, dtype=float)))
        self.mass = mass
        self.cd = cd
        self.area = area
        self.active = True
        self.history = [self.state[:3]]

    def _no_thrust(self, t, v):
        return np.zeros(3)

    def update(self, dt):
        if not self.active: return
        self.state = rk4_integration(self.state, dt, self.mass, self.cd, self.area, self._no_thrust)
        self.history.append(self.state[:3])
        if self.state[2] < 0: 
            self.active = False

class ManeuveringDrone(Projectile):
    def __init__(self, pos, vel, seed=None):
        super().__init__(pos, vel, mass=200.0, cd=0.04, area=0.4)
        self.time = 0.0
        rng = np.random.default_rng(seed)
        self.omega_x = rng.uniform(0.3, 1.2)
        self.omega_y = rng.uniform(0.3, 1.2)
        self.phase_x = rng.uniform(0, 2*np.pi)
        self.phase_y = rng.uniform(0, 2*np.pi)
        self.g_load = rng.uniform(4.0, 9.0)

    def _evasive_pilot(self, t, vel):
        g_force = np.array([0, 0, 9.80665 * self.mass])
        v_mag = np.linalg.norm(vel)
        if v_mag > 0:
            drag_est = 0.5 * 1.225 * (v_mag**2) * self.cd * self.area
            thrust_fwd = (vel / v_mag) * drag_est
        else:
            thrust_fwd = np.zeros(3)
        fwd = vel / v_mag if v_mag > 0 else np.array([1,0,0])
        up_ref = np.array([0, 0, 1])
        right = np.cross(fwd, up_ref)
        if np.linalg.norm(right) < 0.1: right = np.array([0, 1, 0])
        right = right / np.linalg.norm(right)
        real_up = np.cross(right, fwd)
        amp = self.g_load * 9.80665 * self.mass
        force_right = right * amp * np.sin(self.omega_x * self.time + self.phase_x)
        force_up = real_up * amp * np.cos(self.omega_y * self.time + self.phase_y)
        return g_force + thrust_fwd + force_right + force_up

    def update(self, dt):
        if not self.active: return
        self.time += dt
        self.state = rk4_integration(self.state, dt, self.mass, self.cd, self.area, self._evasive_pilot)
        self.history.append(self.state[:3])
        if self.state[2] < 0: self.active = False

class Interceptor(Projectile):
    def __init__(self, pos, vel):
        super().__init__(pos, vel, mass=90.0, cd=0.25, area=0.02)
        self.dry_mass = 40.0
        self.fuel_mass = 50.0
        self.isp = 270.0 
        self.burn_time = 12.0 
        self.thrust_max = (self.fuel_mass / self.burn_time) * self.isp * 9.80665
        self.command_acc = np.zeros(3)
        self.realized_acc = np.zeros(3) 
        self.time_elapsed = 0.0
        self.tau = 0.05 

    def _thrust_control_law(self, t, vel):
        thrust_vec = np.zeros(3)
        if self.fuel_mass > 0:
            v_norm = np.linalg.norm(vel)
            if v_norm > 0:
                body_axis = vel / v_norm
                thrust_vec = body_axis * self.thrust_max
        control_force = self.realized_acc * self.mass
        return thrust_vec + control_force

    def update_guidance(self, dt, target_pos, target_vel, guidance_func):
        if not self.active: return
        if self.fuel_mass > 0:
            mdot = self.thrust_max / (self.isp * 9.80665)
            dm = mdot * dt
            if dm > self.fuel_mass: dm = self.fuel_mass
            self.fuel_mass -= dm
            self.mass = self.dry_mass + self.fuel_mass
            self.time_elapsed += dt
        p_int = self.state[:3]
        v_int = self.state[3:]
        raw_cmd = guidance_func(target_pos - p_int, target_vel - v_int)
        self.realized_acc += (raw_cmd - self.realized_acc) * (dt / self.tau)
        self.state = rk4_integration(self.state, dt, self.mass, self.cd, self.area, self._thrust_control_law)
        self.history.append(self.state[:3])
        if self.state[2] < 0:
            self.active = False