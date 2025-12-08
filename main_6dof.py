### KineticDefenseSim/main_6dof.py
import argparse
import numpy as np
import matplotlib.pyplot as plt
from src.core.types import AeroCoefficients
from src.models.missile import Missile6DOF
from src.physics.environment import WindModel
from src.gnc.control import Autopilot
from src.gnc.guidance import pro_nav_3d
from src.estimation.ekf import EKF6DOF

def main():
    wind = WindModel(seed=42)
    mass_props = {'mass': 90.0, 'inertia': np.diag([2.0, 15.0, 15.0]), 'fuel': 40.0}
    aero_props = AeroCoefficients(cd0=0.3, cla=4.0, cma=-2.0)
    interceptor = Missile6DOF(
        pos=np.array([0.0, 0.0, -10.0]),
        vel=np.array([0.1, 0.1, -50.0]),
        mass_props=mass_props,
        aero_props=aero_props
    )
    tgt_pos = np.array([5000.0, 2000.0, -3000.0])
    tgt_vel = np.array([-200.0, -50.0, 0.0])
    ekf = EKF6DOF(dt=0.01, process_noise=0.1, measure_noise=0.05)
    t = 0
    dt = 0.01
    duration = 20.0
    hist_int = []
    hist_tgt = []
    while t < duration:
        r_true = tgt_pos - interceptor.state[0:3]
        rng = np.linalg.norm(r_true)
        az = np.arctan2(r_true[1], r_true[0])
        el = np.arctan2(-r_true[2], np.linalg.norm(r_true[:2]))
        meas = np.array([rng, az, el]) + np.random.normal(0, 0.01, 3)
        ekf.predict()
        ekf.update(meas)
        est_tgt_pos = ekf.x[0:3] + interceptor.state[0:3]
        est_tgt_vel = ekf.x[3:6] + interceptor.state[3:6]
        acc_cmd_inertial = pro_nav_3d(
            interceptor.state[0:3], interceptor.state[3:6],
            est_tgt_pos, est_tgt_vel
        )
        def thrust_profile(time):
            return 2000.0 if time < 5.0 else 0.0
        def fin_profile(time):
            return np.zeros(3)
        interceptor.rk4_step(t, dt, wind.get_wind, thrust_profile, fin_profile)
        interceptor.state[3:6] += (acc_cmd_inertial * dt)
        tgt_pos += tgt_vel * dt
        hist_int.append(interceptor.state[0:3].copy())
        hist_tgt.append(tgt_pos.copy())
        t += dt
        if np.linalg.norm(interceptor.state[0:3] - tgt_pos) < 5.0:
            print(f"INTERCEPT at T={t:.2f}")
            break
    hist_int = np.array(hist_int)
    hist_tgt = np.array(hist_tgt)
    plt.style.use('dark_background')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(hist_int[:,0], hist_int[:,1], -hist_int[:,2], color='#00FFDD', label='Interceptor')
    ax.plot(hist_tgt[:,0], hist_tgt[:,1], -hist_tgt[:,2], color='#FF2222', label='Target')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Alt')
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()