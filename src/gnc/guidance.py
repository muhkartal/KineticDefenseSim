### KineticDefenseSim/src/gnc/guidance.py
import numpy as np

def pro_nav_3d(pos_m, vel_m, pos_t, vel_t, N=4.0):
    r_tm = pos_t - pos_m
    v_tm = vel_t - vel_m
    range_mag = np.linalg.norm(r_tm)
    if range_mag < 0.1: return np.zeros(3)
    omega = np.cross(r_tm, v_tm) / (range_mag**2)
    acc_cmd = N * np.cross(v_tm, omega)
    return acc_cmd