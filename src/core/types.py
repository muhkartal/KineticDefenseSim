### KineticDefenseSim/src/core/types.py
from dataclasses import dataclass
import numpy as np
from typing import NamedTuple

class State6DOF(NamedTuple):
    x: float
    y: float
    z: float
    u: float
    v: float
    w: float
    phi: float
    theta: float
    psi: float
    p: float
    q: float
    r: float

@dataclass
class AeroCoefficients:
    cd0: float
    cla: float
    cma: float
    
    def get_drag(self, mach: float, alpha: float) -> float:
        beta = np.sqrt(1 - mach**2) if mach < 1.0 else np.sqrt(mach**2 - 1)
        return self.cd0 / max(0.1, beta) + 0.1 * alpha**2