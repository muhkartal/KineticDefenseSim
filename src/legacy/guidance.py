### KineticDefenseSim/src/legacy/guidance.py
import numpy as np

def augmented_proportional_navigation(r_tm: np.ndarray, v_tm: np.ndarray, n_gain: float = 4.0) -> np.ndarray:
    range_mag = np.linalg.norm(r_tm)
    if range_mag < 1.0: return np.zeros(3)

    omega_vec = np.cross(r_tm, v_tm) / (range_mag**2)
    acc_cmd = n_gain * np.cross(v_tm, omega_vec)

    return acc_cmd

def limit_g_load(acc_cmd: np.ndarray, max_g: float) -> np.ndarray:
    mag = np.linalg.norm(acc_cmd)
    limit = max_g * 9.80665
    if mag > limit:
        return acc_cmd / mag * limit
    return acc_cmd