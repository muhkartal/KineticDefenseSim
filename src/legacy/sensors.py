### KineticDefenseSim/src/legacy/sensors.py
import numpy as np
from src.legacy.config import RadarConfig

class Radar:
    def __init__(self, config: RadarConfig):
        self.config = config
        self.rng = np.random.default_rng()

    def measure(self, true_state: np.ndarray) -> np.ndarray:
        noise_pos = self.rng.normal(0, self.config.POS_NOISE_STD, 3)
        noise_vel = self.rng.normal(0, self.config.VEL_NOISE_STD, 3)
        noise = np.concatenate((noise_pos, noise_vel))
        return true_state + noise