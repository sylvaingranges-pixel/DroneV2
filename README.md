# Drone Control System with Suspended Load

## Description

This project implements an optimal control system for a 40kg drone carrying a 24kg suspended load on a single axis (X). The system uses Model Predictive Control (MPC) with linear optimization to generate optimal trajectories that bring the load to target positions as quickly as possible while ensuring zero oscillation and zero velocity at arrival.

## System Specifications

- **Drone mass**: 40 kg
- **Load mass**: 24 kg  
- **Cable length**: 19 m
- **Load cross-section area**: 0.2 m²
- **Aerodynamic drag coefficient**: 1.0
- **Sample time**: 0.1 s

## Features

### Models Implemented

1. **Non-linear Model**: Full dynamics with air drag opposing load velocity
   - State variables: `[x_d, v_d, theta, omega]` (drone position, drone velocity, cable angle, angular velocity)
   - Input: `u = a_d` (drone acceleration)
   - Output: `x_l` (load position)

2. **Linearized Model**: Linearization around equilibrium point (load directly below drone, no motion, no aerodynamic effects)
   - Used for optimization to ensure fast computation

3. **Discrete-time Model**: Exact discretization with zero-order hold at Ts=0.1s

### Optimal Controller

- Uses CVXPY with OSQP solver for fast quadratic programming
- Optimization objectives:
  - Minimize control effort
  - Bring load to target position
  - Ensure zero velocity and zero oscillation at arrival
- Constraints:
  - Drone acceleration limits: ±5 m/s²
  - Terminal constraints: zero velocity, zero angle, zero angular velocity

### Validation

- Linear model simulation using discrete-time system
- Non-linear model simulation using RK45 ODE solver
- Comparison plots showing linear vs non-linear behavior

## Installation

```bash
pip install -r requirements.txt
```

## Usage

### Quick Start

For a simple example, run:

```bash
python quick_example.py
```

This demonstrates a basic trajectory from rest to 30m target position.

### Full Test Suite

Run the main script to execute all test scenarios:

```bash
python drone_control.py
```

This will:
1. Display system parameters and model matrices
2. Run multiple test scenarios with different initial conditions and targets
3. Generate comparison plots for each scenario
4. Display optimization timing statistics
5. Save all plots as PNG files

## Test Scenarios

The system is tested with:
- Initial conditions: At rest and moving (v=2 m/s, theta=0.05 rad)
- Target positions: 20m, 40m, and 80m

## Results

The controller successfully:
- Generates optimal trajectories in ~10-50ms (fast enough for closed-loop control)
- Brings the load to target with minimal oscillation
- Shows good agreement between linear and non-linear models for reasonable operating ranges
- Demonstrates that air drag effects are captured in non-linear simulation

## Files

- `drone_control.py`: Main implementation with all models, controller, and test scenarios
- `quick_example.py`: Simple example script for quick start
- `requirements.txt`: Python dependencies
- `view_results.html`: Interactive HTML viewer for results
- `result_plot_*.png`: Generated plots (not committed to git)
- `IMPLEMENTATION_SUMMARY.md`: Detailed results and performance analysis

## Next Steps

The fast optimization time (<100ms) makes this controller suitable for:
- Closed-loop Model Predictive Control (MPC)
- Real-time trajectory re-planning
- Disturbance rejection and robustness improvements

## Author

Developed for DroneV2 project
