### KineticDefenseSim/main.py
import argparse
import logging
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import MaxNLocator
from src.legacy.entities import Projectile, Interceptor, ManeuveringDrone
from src.legacy.guidance import augmented_proportional_navigation, limit_g_load
from src.legacy.config import SimulationConfig, RadarConfig
from src.legacy.sensors import Radar
from src.legacy.estimation import KalmanFilter

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)8s | %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger("KineticDefenseSim")

def parse_args():
    parser = argparse.ArgumentParser(description="Kinetic Defense Simulation")
    parser.add_argument("--seed", type=int, help="Scenario Seed")
    parser.add_argument("--mode", choices=['random', 'dogfight', 'ballistic'], default='random')
    parser.add_argument("--intercept-g", type=float, default=55.0)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()

def generate_scenario(mode, seed):
    rng = np.random.default_rng(seed)
    if mode == 'random':
        mode = rng.choice(['dogfight', 'ballistic'])
    logger.info(f"GENERATING SCENARIO: {mode.upper()} | SEED: {seed}")
    if mode == 'ballistic':
        dist = rng.uniform(20000, 35000)
        angle = np.deg2rad(rng.uniform(40, 60))
        speed = np.sqrt(dist * 9.81 / np.sin(2*angle)) * rng.uniform(1.1, 1.3)
        azimuth = rng.uniform(0, 2*np.pi)
        spawn_pos = [dist * np.cos(azimuth), dist * np.sin(azimuth), 0]
        v_rad = -speed * np.cos(angle)
        v_z = speed * np.sin(angle)
        vel = [v_rad * np.cos(azimuth), v_rad * np.sin(azimuth), v_z]
        target = Projectile(spawn_pos, vel, mass=300.0, cd=0.2, area=0.1)
    else: 
        dist = rng.uniform(12000, 25000)
        alt = rng.uniform(2000, 8000)
        azimuth = rng.uniform(0, 2*np.pi)
        spawn_pos = [dist * np.cos(azimuth), dist * np.sin(azimuth), alt]
        speed = rng.uniform(270, 680)
        offset_x = rng.uniform(-3000, 3000)
        offset_y = rng.uniform(-3000, 3000)
        target_point = np.array([offset_x, offset_y, 0])
        curr_pos = np.array(spawn_pos)
        direction = target_point - curr_pos
        direction = direction / np.linalg.norm(direction)
        vel = direction * speed
        target = ManeuveringDrone(spawn_pos, vel, seed=seed)
    return target

def run_simulation(args):
    if args.seed is None:
        args.seed = np.random.randint(0, 100000)
    target = generate_scenario(args.mode, args.seed)
    interceptor = Interceptor(pos=[0, 0, 0], vel=[0.01, 0.01, 100])
    sim_cfg = SimulationConfig()
    radar_cfg = RadarConfig()
    radar = Radar(radar_cfg)
    kf = KalmanFilter(sim_cfg.DT, radar_cfg.POS_NOISE_STD, radar_cfg.VEL_NOISE_STD)
    kf.x = radar.measure(target.state)
    time = 0.0
    sim_running = True
    intercepted = False
    est_history = []
    while sim_running and time < 90.0:
        target.update(sim_cfg.DT)
        meas = radar.measure(target.state)
        kf.predict()
        kf.update(meas)
        est_state = kf.get_state()
        est_history.append(est_state[:3])
        if interceptor.active:
            def guidance(r, v):
                cmd = augmented_proportional_navigation(r, v, n_gain=5.0) 
                return limit_g_load(cmd, max_g=args.intercept_g)
            interceptor.update_guidance(sim_cfg.DT, est_state[:3], est_state[3:], guidance)
            dist = np.linalg.norm(interceptor.state[:3] - target.state[:3])
            if dist < 15.0:
                logger.info(f"SPLASH | T={time:.2f}s | Miss={dist:.2f}m")
                intercepted = True
                sim_running = False
        if target.state[2] < 0:
            logger.info("TARGET GROUND IMPACT")
            sim_running = False
        time += sim_cfg.DT
    if not args.headless:
        visualize_results(target, interceptor, est_history, intercepted, args.seed)

def visualize_results(target, interceptor, est_hist, success, seed):
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(16, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.set_major_locator(MaxNLocator(nbins=6))
    ax.yaxis.set_major_locator(MaxNLocator(nbins=6))
    ax.zaxis.set_major_locator(MaxNLocator(nbins=5))
    ax.grid(color='#707070', linestyle=':', linewidth=0.8, alpha=0.12)
    t_hist = np.array(target.history)
    i_hist = np.array(interceptor.history)
    ax.set_title(f'SIMULATION ID: {seed} | STATUS: {"NEUTRALIZED" if success else "FAILURE"}', color='white', pad=20)
    ax.set_xlabel('X [m]', color='gray')
    ax.set_ylabel('Y [m]', color='gray')
    ax.set_zlabel('ALT [m]', color='gray')
    all_points = np.vstack((t_hist, i_hist))
    max_ex = np.max(np.abs(all_points[:,:2])) 
    ax.set_xlim([-max_ex, max_ex])
    ax.set_ylim([-max_ex, max_ex])
    ax.set_zlim([0, np.max(all_points[:,2])*1.1])
    line_tgt, = ax.plot([], [], [], color='#FF2222', linewidth=2.0, alpha=0.9, label='THREAT')
    line_int, = ax.plot([], [], [], color='#00FFDD', linewidth=2.5, alpha=1.0, label='INTERCEPTOR')
    scat_hit = ax.scatter([], [], [], s=0, c='white', marker='o', edgecolors='#ffcc00', linewidth=1.5, zorder=100)
    ax.legend(frameon=False, labelcolor='linecolor')
    frames = len(t_hist)
    skip = max(1, frames // 400) 
    def update(frame):
        idx = frame * skip
        if idx >= frames: idx = frames - 1
        line_tgt.set_data(t_hist[:idx, 0], t_hist[:idx, 1])
        line_tgt.set_3d_properties(t_hist[:idx, 2])
        if idx < len(i_hist):
            line_int.set_data(i_hist[:idx, 0], i_hist[:idx, 1])
            line_int.set_3d_properties(i_hist[:idx, 2])
            head_t = t_hist[idx]
            head_i = i_hist[idx]
            mid = (head_t + head_i) / 2
            sep = np.linalg.norm(head_t - head_i)
            zoom = max(sep, 3000) 
            ax.set_xlim([mid[0]-zoom, mid[0]+zoom])
            ax.set_ylim([mid[1]-zoom, mid[1]+zoom])
            ax.set_zlim([max(0, mid[2]-zoom/2), mid[2]+zoom])
        if success and idx >= len(i_hist)-1:
            scat_hit._offsets3d = ([i_hist[-1,0]], [i_hist[-1,1]], [i_hist[-1,2]])
            scat_hit.set_sizes([150])
        return line_tgt, line_int, scat_hit
    ani = FuncAnimation(fig, update, frames=frames//skip + 30, interval=20, blit=False)
    plt.show()

if __name__ == "__main__":
    args = parse_args()
    run_simulation(args)