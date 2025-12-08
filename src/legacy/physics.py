### KineticDefenseSim/src/legacy/physics.py
import numpy as np
from typing import Callable, Tuple

GRAVITY = 9.80665
R_EARTH = 6371000.0
GAMMA = 1.4
R_GAS = 287.05
T_0 = 288.15
L_RATE = 0.0065
P_0 = 101325.0

def get_atmosphere(altitude: float) -> Tuple[float, float]:
    h = max(0.0, altitude)
    if h > 11000:
        temp = 216.65
        pressure = 22632.1 * np.exp(-GRAVITY * 0.0289644 * (h - 11000) / (8.31447 * 216.65))
    else:
        temp = T_0 - L_RATE * h
        pressure = P_0 * (1 - L_RATE * h / T_0) ** (GRAVITY / (L_RATE * 287.05))
    
    rho = pressure / (R_GAS * temp)
    a = np.sqrt(GAMMA * R_GAS * temp)
    return rho, a

def get_drag_coeff(mach: float, base_cd: float) -> float:
    if mach < 0.8: return base_cd
    elif mach < 1.2: return base_cd * (1 + 2.5 * (mach - 0.8)) 
    else: return base_cd * (2.0) * (1.2 / mach)

def equations_of_motion(t: float, state: np.ndarray, mass: float, cd: float, area: float, thrust_func: Callable) -> np.ndarray:
    pos = state[:3]
    vel = state[3:]
    v_mag = np.linalg.norm(vel)
    
    rho, sound_speed = get_atmosphere(pos[2])
    
    drag_force = np.zeros(3)
    if v_mag > 0.1:
        mach = v_mag / sound_speed
        cd_dyn = get_drag_coeff(mach, cd)
        drag_mag = 0.5 * rho * (v_mag**2) * cd_dyn * area
        drag_force = -drag_mag * (vel / v_mag)

    thrust_vec = thrust_func(t, vel)
    total_acc = (drag_force + thrust_vec) / mass + np.array([0.0, 0.0, -GRAVITY])
    
    return np.concatenate((vel, total_acc))

def rk4_integration(state: np.ndarray, dt: float, mass: float, cd: float, area: float, thrust_func: Callable) -> np.ndarray:
    k1 = equations_of_motion(0, state, mass, cd, area, thrust_func)
    
    state_k2 = state + k1 * (0.5 * dt)
    k2 = equations_of_motion(0.5 * dt, state_k2, mass, cd, area, thrust_func)
    
    state_k3 = state + k2 * (0.5 * dt)
    k3 = equations_of_motion(0.5 * dt, state_k3, mass, cd, area, thrust_func)
    
    state_k4 = state + k3 * dt
    k4 = equations_of_motion(dt, state_k4, mass, cd, area, thrust_func)
    
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)