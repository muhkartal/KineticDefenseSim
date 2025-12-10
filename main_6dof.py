### main_6dof.py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from src.core.battle_manager import BattleManager

def main():
    print("Initializing Multi-Target Kinetic Defense Simulation (6-DOF)...")
    
    manager = BattleManager()
    
    # --- SCENARIO SETUP ---
    
    # Threat 1: High altitude, coming straight in
    manager.spawn_threat(
        position=np.array([10000.0, 5000.0, 5000.0]),
        velocity=np.array([-300.0, 0.0, 0.0])
    )
    
    # Threat 2: Lower altitude, crossing
    manager.spawn_threat(
        position=np.array([8000.0, -2000.0, 3000.0]),
        velocity=np.array([-250.0, 100.0, -50.0])
    )

    # Interceptor 1
    manager.spawn_interceptor(
        position=np.array([0.0, 0.0, 0.0]),
        velocity=np.array([0.0, 0.0, 100.0]), # Vertical launch
        euler=np.zeros(3),
        ang_vel=np.zeros(3)
    )

    # Interceptor 2
    manager.spawn_interceptor(
        position=np.array([100.0, 100.0, 0.0]), # Slightly offset
        velocity=np.array([0.0, 0.0, 100.0]),
        euler=np.zeros(3),
        ang_vel=np.zeros(3)
    )

    # --- SIMULATION LOOP ---
    dt = 0.05
    max_time = 40.0
    steps = int(max_time / dt)
    
    print(f"Starting simulation: {steps} steps ({max_time}s)")
    
    for step in range(steps):
        t = step * dt
        manager.update(t, dt)
        
        # Stop if all threats neutralized
        if not any(t.active for t in manager.threats):
            print(f"All threats neutralized at step {step} ({step*dt:.2f}s)!")
            break

    # --- VISUALIZATION ---
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot Threats
    for t in manager.threats:
        hist = np.array(t.history)
        ax.plot(hist[:,0], hist[:,1], hist[:,2], label=f'Threat {t.id} {"(Kill)" if not t.active else ""}')
        ax.scatter(hist[-1,0], hist[-1,1], hist[-1,2], marker='x')

    # Plot Interceptors
    for m in manager.interceptors:
        hist = np.array(m.history)
        ax.plot(hist[:,0], hist[:,1], hist[:,2], linestyle='--', label=f'Int {m.id}')
        
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()
