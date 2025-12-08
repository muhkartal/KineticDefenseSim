### KineticDefenseSim/src/models/missile.py
import numpy as np
from src.core.types import AeroCoefficients
from src.physics.environment import Atmosphere

class Missile6DOF:
    def __init__(self, pos, vel, mass_props, aero_props: AeroCoefficients):
        self.state = np.zeros(12)
        self.state[0:3] = pos
        self.state[3:6] = vel
        self.mass = mass_props['mass']
        self.inertia = mass_props['inertia']
        self.aero = aero_props
        self.fuel_mass = mass_props.get('fuel', 0.0)
        self.history = []

    def equations_of_motion(self, t, state, wind_func, thrust_func, fin_func):
        pos = state[0:3]
        vel_body = state[3:6]
        euler = state[6:9]
        rates = state[9:12]
        rho, press, temp, sos = Atmosphere.get_properties(-pos[2])
        wind_inertial = wind_func(-pos[2])
        phi, theta, psi = euler
        c_th, s_th = np.cos(theta), np.sin(theta)
        c_ph, s_ph = np.cos(phi), np.sin(phi)
        c_ps, s_ps = np.cos(psi), np.sin(psi)
        dcm = np.array([
            [c_th*c_ps, c_th*s_ps, -s_th],
            [s_ph*s_th*c_ps - c_ph*s_ps, s_ph*s_th*s_ps + c_ph*c_ps, s_ph*c_th],
            [c_ph*s_th*c_ps + s_ph*s_ps, c_ph*s_th*s_ps - s_ph*c_ps, c_ph*c_th]
        ])
        vel_inertial = dcm.T @ vel_body
        airspeed_inertial = vel_inertial - wind_inertial
        airspeed_body = dcm @ airspeed_inertial
        V_mag = np.linalg.norm(airspeed_body)
        mach = V_mag / sos
        alpha = np.arctan2(airspeed_body[2], airspeed_body[0])
        q_bar = 0.5 * rho * V_mag**2
        ref_area = 0.02
        cd = self.aero.get_drag(mach, alpha)
        drag = q_bar * ref_area * cd
        lift = q_bar * ref_area * self.aero.cla * alpha
        f_aero = np.array([-drag, 0, -lift])
        thrust = thrust_func(t)
        f_thrust = np.array([thrust, 0, 0])
        g_inertial = np.array([0, 0, 9.81])
        f_grav = dcm @ (g_inertial * self.mass)
        f_total = f_aero + f_thrust + f_grav
        m_aero = np.array([
            -0.1 * rates[0],
            q_bar * ref_area * 0.1 * self.aero.cma * alpha - 50.0 * rates[1],
            -50.0 * rates[2]
        ])
        pos_dot = dcm.T @ vel_body
        vel_dot = (f_total / self.mass) - np.cross(rates, vel_body)
        rates_dot = np.linalg.inv(self.inertia) @ (m_aero - np.cross(rates, self.inertia @ rates))
        p, q, r = rates
        phi_dot = p + np.tan(theta)*(q*s_ph + r*c_ph)
        theta_dot = q*c_ph - r*s_ph
        psi_dot = (q*s_ph + r*c_ph) / c_th
        return np.concatenate((pos_dot, vel_dot, np.array([phi_dot, theta_dot, psi_dot]), rates_dot))

    def rk4_step(self, t, dt, wind_func, thrust_func, fin_func):
        k1 = self.equations_of_motion(t, self.state, wind_func, thrust_func, fin_func)
        k2 = self.equations_of_motion(t + 0.5*dt, self.state + 0.5*dt*k1, wind_func, thrust_func, fin_func)
        k3 = self.equations_of_motion(t + 0.5*dt, self.state + 0.5*dt*k2, wind_func, thrust_func, fin_func)
        k4 = self.equations_of_motion(t + dt, self.state + dt*k3, wind_func, thrust_func, fin_func)
        self.state += (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
        self.history.append(self.state[0:3].copy())