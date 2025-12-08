### KineticDefenseSim/src/legacy/config.py
from dataclasses import dataclass

@dataclass(frozen=True)
class PhysicsConstants:
    GRAVITY: float = 9.80665
    R_EARTH: float = 6371000.0
    GAMMA: float = 1.4
    R_GAS: float = 287.05
    T_0: float = 288.15
    L_RATE: float = 0.0065
    P_0: float = 101325.0

@dataclass(frozen=True)
class RadarConfig:
    UPDATE_RATE: float = 20.0 
    POS_NOISE_STD: float = 5.0 
    VEL_NOISE_STD: float = 2.0 

@dataclass(frozen=True)
class SimulationConfig:
    DT: float = 0.01
    MAX_DURATION: float = 30.0
    INTERCEPT_THRESHOLD: float = 8.0 
