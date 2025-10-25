# 1D Drone Control System with Suspended Load

This project implements a Model Predictive Control (MPC) system for a drone carrying a suspended load in one dimension (X-axis).

## System Description

- **Drone mass**: 40 kg
- **Load mass**: 24 kg  
- **Cable length**: 19 m
- **Load surface area**: 0.2 m²
- **Drag coefficient**: 1.0
- **Sampling time**: 0.1 s

## Features

1. **Non-linear dynamics model** with air resistance on the load
2. **Linearized model** around equilibrium (load directly below drone)
3. **Discrete-time model** with matrix exponential discretization
4. **MPC controller** using CVXPY optimization library
5. **Closed-loop MPC with receding horizon**:
   - Replanning every 3 seconds (30 time steps)
   - Warm-start from previous solution
   - Feedback from actual nonlinear state
   - Better handling of model uncertainties and nonlinearities
6. **Simulation and visualization** comparing:
   - Open-loop MPC (single optimization)
   - Closed-loop MPC (replanning with feedback)
   - Tracking performance and error analysis

## State Variables

The system state is defined as:
- `x_d`: Drone position (m)
- `v_d`: Drone velocity (m/s)
- `theta`: Cable angle from vertical (rad)
- `omega`: Angular velocity of cable (rad/s)

## Control Input

- `u`: Drone acceleration (m/s²)

## Installation

```bash
pip install -r requirements.txt
```

## Usage

Run the main simulation:

```bash
python drone_control_1d.py
```

This will execute comparison tests between open-loop and closed-loop MPC, generating plots showing:
- System trajectories (drone position, velocity, cable angle, angular velocity)
- Control inputs
- Load position tracking comparison
- Error analysis (final, mean, max, RMS errors)
- Performance statistics

## Control Strategies

### Open-Loop MPC
- Single optimization at the beginning
- Control sequence applied over the full horizon
- Does not account for model mismatch or disturbances during execution

### Closed-Loop MPC (Receding Horizon)
- Re-optimization every 3 seconds (30 time steps)
- Uses actual nonlinear state as feedback
- Warm-start from shifted previous solution
- Terminal constraints adapted for receding horizon
- **Significantly better performance**: 94-99% reduction in final tracking error

## Test Cases

The system includes comparison tests with:
- Starting from rest (20m, 40m targets)
- Starting with initial velocity and angle (20m target)
- Horizons of 10-15 seconds

## Performance Results

Typical improvements with closed-loop MPC:
- **Final tracking error**: 94-99% reduction vs open-loop
- **Mean error**: 15-31% reduction
- **Average solve time**: ~400-640ms per optimization
- **Total computation time**: Scales with number of replanning steps

Example results for 20m target from rest:
- Open-loop final error: 1.53m
- Closed-loop final error: 0.08m (94.8% improvement)

## Solver Optimization

The system uses **CLARABEL**, a modern Rust-based conic solver, which was identified as the fastest and most robust solver through comprehensive benchmarking:
- **Average solve time**: 400-1100ms (depending on horizon)
- **Success rate**: 100%
- **Closed-loop precision**: <0.1m final load position error
- **Open-loop precision**: 0.5-12m final load position error (depends on horizon and initial conditions)

See `RAPPORT_COMPARATIF_SOLVEURS.md` for detailed solver comparison (CLARABEL, SCS, ECOS, OSQP, CVXOPT).

## Generated Outputs

Plots are saved as PNG files in the working directory:
- `comparison_open_closed_*.png`: State and control trajectories comparison
- `load_comparison_*.png`: Load position tracking and error analysis with statistics

## Solver Benchmark

To run the solver benchmark and generate comparative analysis:
```bash
python solver_benchmark.py
```

This generates:
- `RAPPORT_COMPARATIF_SOLVEURS.md`: Detailed comparative report in French
- `solver_comparison.png`: Performance visualizations
- `solver_benchmark_results.csv`: Raw benchmark data
