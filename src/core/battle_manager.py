### src/core/battle_manager.py
import numpy as np
from typing import List, Dict, Optional
from src.models.missile import Missile6DOF
from src.models.threat import Threat
from src.gnc.guidance import pro_nav_3d
from src.core.types import AeroCoefficients

class BattleManager:
    """
    Orchestrates the engagement: manages entities, sensors, and allocation.
    """
    
    def __init__(self):
        self.threats: List[Threat] = []
        self.interceptors: List[Missile6DOF] = []
        
        # Mapping: Interceptor ID -> Threat ID
        self.assignments: Dict[int, int] = {}
        
        self.next_threat_id = 0
        self.next_interceptor_id = 100

    def spawn_threat(self, position: np.ndarray, velocity: np.ndarray) -> int:
        t = Threat(self.next_threat_id, position, velocity)
        self.threats.append(t)
        self.next_threat_id += 1
        return t.id

    def spawn_interceptor(self, position: np.ndarray, velocity: np.ndarray, 
                          euler: np.ndarray, ang_vel: np.ndarray) -> int:
        """
        Spawns a standard Missile6DOF.
        """
        # Default properties for a generic interceptor
        mass_props = {
            'mass': 50.0,
            'inertia': np.eye(3) * 5.0,
            'fuel': 20.0
        }
        aero_props = AeroCoefficients(cd0=0.2, cla=3.0, cma=-1.5)

        missile = Missile6DOF(position, velocity, mass_props, aero_props)
        
        # Hack: attach an ID to the missile instance for tracking
        missile.id = self.next_interceptor_id
        self.interceptors.append(missile)
        self.next_interceptor_id += 1
        return missile.id

    def assign_targets(self):
        """
        Greedy Allocation: Assign closest free interceptor to closest threat.
        Refreshes assignments every frame (can be optimized).
        """
        active_threats = [t for t in self.threats if t.active]
        free_interceptors = [i for i in self.interceptors if i.id not in self.assignments]

        if not active_threats or not free_interceptors:
            return

        # Simple distance-based cost matrix could be used here
        # For now, just assign first free to first active (FIFO)
        # TODO: Implement true Priority Queue based on Time-to-Go
        
        for interceptor in free_interceptors:
            # Find closest threat
            best_threat = None
            min_dist = float('inf')
            
            for threat in active_threats:
                # Check if threat is already engaged by another (optional logic)
                # Allowing multi-interceptor to single threat for now? No, 1-1.
                is_engaged = False
                for i_id, t_id in self.assignments.items():
                    if t_id == threat.id:
                        is_engaged = True
                        break
                
                if not is_engaged:
                    dist = np.linalg.norm(interceptor.position - threat.position)
                    if dist < min_dist:
                        min_dist = dist
                        best_threat = threat
            
            if best_threat:
                self.assignments[interceptor.id] = best_threat.id

    def check_interceptions(self, kill_radius: float = 10.0):
        for i_id, t_id in list(self.assignments.items()):
            interceptor = next((m for m in self.interceptors if m.id == i_id), None)
            threat = next((t for t in self.threats if t.id == t_id), None)
            
            if interceptor and threat and threat.active:
                dist = np.linalg.norm(interceptor.position - threat.position)
                if dist < kill_radius:
                    print(f"!!! INTERCEPTION: Interceptor {i_id} hit Threat {t_id} at dist {dist:.2f}m")
                    threat.active = False
                    # Remove assignment
                    del self.assignments[i_id]

    def update(self, t: float, dt: float):
        # Define environment/control functions (Placeholders for now)
        wind_func = lambda h: np.zeros(3)
        thrust_func = lambda t_sim: 15000.0 if t_sim < 5.0 else 0.0 # Simple boost phase
        fin_func = lambda *args: None

        # 1. Update Assignments
        self.assign_targets()
        
        # 2. Step Threats
        for threat in self.threats:
            threat.step(dt)
            
        # 3. Step Interceptors
        for interceptor in self.interceptors:
            # Determine guidance command
            target_accel_cmd = np.zeros(3)
            
            if interceptor.id in self.assignments:
                threat_id = self.assignments[interceptor.id]
                threat = next((t for t in self.threats if t.id == threat_id), None)
                
                if threat and threat.active:
                    # Call Guidance Law
                    # pro_nav_3d expects (pos_m, vel_m, pos_t, vel_t, N)
                    target_accel_cmd = pro_nav_3d(
                        interceptor.position, 
                        interceptor.velocity, 
                        threat.position, 
                        threat.velocity, 
                        N=3.0
                    )
            
            # Apply Physics Step
            # Note: Guidance command is currently calculated but NOT passed to physics
            # Real implementation would map target_accel_cmd -> fin deflections via Control System
            interceptor.rk4_step(t, dt, wind_func, thrust_func, fin_func)
        
        # 4. Check End Conditions
        self.check_interceptions()
