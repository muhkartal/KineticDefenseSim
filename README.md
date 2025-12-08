# Kinetic Defense Simulation

A dual-mode ballistic simulation engine modeling a high-performance Kinetic Interceptor system. This project provides both a high-fidelity engineering sandbox (6-DOF Rigid Body Dynamics) and a visually rich engagement visualizer (3-DOF).

![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Status](https://img.shields.io/badge/Status-Production-orange.svg)

## Architecture

The system is divided into two distinct computation kernels:

### 1. Standard Mode (Visual/Kinematic)
*   **Physics:** 3-DOF Point Mass ($F=ma$).
*   **Focus:** Procedural scenario generation (Dogfights, Ballistic Arcs), real-time Matplotlib visualization, and chaotic engagement logic.
*   **Use Case:** Visual demonstrations, parameter tweaking, scenario replay.

### 2. Research Mode (Engineering/Dynamics)
*   **Physics:** 6-DOF Rigid Body Dynamics (Euler Angles, Body Rates).
*   **Environment:** US Standard Atmosphere 1976, Dryden Wind Turbulence Model.
*   **GNC Stack:**
    *   **Guidance:** True Proportional Navigation (TPN).
    *   **Control:** Three-loop Autopilot topology (Acceleration -> Rate -> Fin Deflection).
    *   **Estimation:** Extended Kalman Filter (EKF) for Cartesian tracking from noisy Polar radar measurements.
*   **Use Case:** Algorithm validation, sensor fusion testing, aerodynamic stability analysis.

## Installation

The project utilizes a self-contained virtual environment.

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/muhkartal/KineticDefenseSim.git
    cd KineticDefenseSim
    ```

2.  **Initialize Environment:**
    Run the automated setup script to create the virtual environment and install dependencies.
    ```cmd
    setup.bat
    ```

## Usage

### Interactive Launcher
The recommended way to run the simulation is via the interactive menu:

```cmd
run_sim.bat
```

## Project Structure

```text
src/
├── core/           # Shared types and data structures
├── physics/        # Environmental models (USSA76, Wind)
├── models/         # 6-DOF Rigid Body equations of motion
├── gnc/            # Guidance, Navigation, and Control algorithms
├── estimation/     # Kalman Filters (EKF)
└── legacy/         # 3-DOF Point-Mass entities (Visual Mode)
```

## License

MIT License
