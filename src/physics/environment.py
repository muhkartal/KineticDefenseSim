### KineticDefenseSim/src/physics/environment.py
import numpy as np

class Atmosphere:
    R_GAS = 287.05
    G = 9.80665
    L_RATE = 0.0065
    T0 = 288.15
    P0 = 101325.0
    
    @staticmethod
    def get_properties(alt: float):
        h = max(0.0, alt)
        if h < 11000:
            temp = Atmosphere.T0 - Atmosphere.L_RATE * h
            press = Atmosphere.P0 * (1 - Atmosphere.L_RATE * h / Atmosphere.T0) ** (Atmosphere.G / (Atmosphere.L_RATE * Atmosphere.R_GAS))
        else:
            t_strat = 216.65
            p_strat = 22632.1
            temp = t_strat
            press = p_strat * np.exp(-Atmosphere.G * (h - 11000) / (Atmosphere.R_GAS * t_strat))
        rho = press / (Atmosphere.R_GAS * temp)
        sos = np.sqrt(1.4 * Atmosphere.R_GAS * temp)
        return rho, press, temp, sos

class WindModel:
    def __init__(self, seed: int):
        self.rng = np.random.default_rng(seed)
        self.base_wind = self.rng.uniform(-5, 5, 3)
        
    def get_wind(self, alt: float) -> np.ndarray:
        shear_factor = np.log(max(alt, 1.0) / 0.1) / np.log(10/0.1)
        shear_wind = self.base_wind * min(shear_factor, 2.5)
        turb = self.rng.normal(0, 0.5, 3)
        return shear_wind + turb