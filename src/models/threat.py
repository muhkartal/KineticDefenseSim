### src/models/threat.py
import numpy as np
from typing import Tuple

class Threat:
    """
    Represents an incoming threat with 3-DOF kinematics.
    Modeled as a point mass with constant velocity (can be extended to maneuvering).
    """

    def __init__(self, threat_id: int, position: np.ndarray, velocity: np.ndarray):
        """
        Args:
            threat_id: Unique identifier
            position: Initial position [x, y, z] (m)
            velocity: Initial velocity [vx, vy, vz] (m/s)
        """
        self.id = threat_id
        self.state = np.concatenate([position, velocity])  # [x, y, z, vx, vy, vz]
        self.active = True
        self.history = [self.state.copy()]

    @property
    def position(self) -> np.ndarray:
        return self.state[0:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.state[3:6]

    def equations_of_motion(self, t: float, state: np.ndarray) -> np.ndarray:
        """
        Simple ballistic/linear motion model.
        d/dt [pos, vel] = [vel, 0] (for constant velocity)
        """
        # dx/dt = v
        # dv/dt = 0 (gravity/drag can be added here)
        return np.concatenate([state[3:6], np.zeros(3)])

    def step(self, dt: float):
        """RK4 integration step"""
        if not self.active:
            return

        k1 = self.equations_of_motion(0, self.state)
        k2 = self.equations_of_motion(0, self.state + dt/2 * k1)
        k3 = self.equations_of_motion(0, self.state + dt/2 * k2)
        k4 = self.equations_of_motion(0, self.state + dt * k3)

        self.state = self.state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        self.history.append(self.state.copy())
