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
5. **Closed-loop MPC** with continuity constraints to eliminate control jumps
6. **Simulation and visualization** comparing:
   - Optimal trajectory from MPC
   - Linear model response
   - Non-linear model response (using RK45 ODE solver)

## New: Closed-Loop MPC with Continuity Constraints

The system now supports closed-loop MPC with special constraints to ensure smooth control transitions:

- **Hard constraint**: First control input matches last applied control (eliminates jumps)
- **Soft penalty**: Penalizes large deviations from previous solution (smooth evolution)

**Results**: 36-41% reduction in control rate, 43% fewer large control jumps, while maintaining excellent tracking performance.

See [CLOSED_LOOP_MPC_DOCUMENTATION.md](CLOSED_LOOP_MPC_DOCUMENTATION.md) for detailed explanation.

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

### Interactive Parameter Tuning (Jupyter Notebook)

**NEW**: Launch the interactive Jupyter notebook to tune MPC parameters:

```bash
jupyter notebook interactive_drone_tuning.ipynb
```

This notebook provides:
- **All parameters in one place**: simulation step, horizon, initial conditions, cost weights, terminal weights
- **Interactive widgets**: adjust parameters with sliders and see results immediately
- **Predefined scenario**: Initial conditions (position=0m, velocity=10m/s, load angle=-30°, angular velocity=0) with target load position at 10m
- **Visual comparison**: side-by-side plots of linear and non-linear system responses

See [NOTEBOOK_GUIDE.md](NOTEBOOK_GUIDE.md) for detailed usage instructions and parameter tuning guide.

### Basic Open-Loop MPC

Run the main simulation:

```bash
python drone_control_1d.py
```

This executes closed-loop MPC tests with and without continuity constraints.

### Test Closed-Loop with Continuity Comparison

Run the comparison test:

```bash
python test_closed_loop_continuity.py
```

This compares three strategies:
1. Open-loop MPC (single optimization)
2. Closed-loop MPC without continuity constraints
3. Closed-loop MPC with continuity constraints

Generates detailed comparison plots showing control smoothness improvements.

## Test Cases

The system includes tests with:
- Starting from rest
- Starting with initial velocity
- Destinations at 20m, 40m, and 80m

## Solver Optimization

The system uses **CLARABEL**, a modern Rust-based conic solver, which was identified as the fastest and most robust solver through comprehensive benchmarking:
- **Average solve time**: 800-1700ms (depending on horizon)
- **Success rate**: 100%
- **Precision**: <0.001m load position error

See `RAPPORT_COMPARATIF_SOLVEURS.md` for detailed solver comparison (CLARABEL, SCS, ECOS, OSQP, CVXOPT).

## Performance

The optimization typically solves in 800-1700ms with CLARABEL, making it suitable for real-time closed-loop control applications.

## Results

Generated plots are saved as PNG files in the working directory:
- `results_*.png`: Full state trajectories and control inputs
- `load_position_*.png`: Load position tracking performance

## Solver Benchmark

To run the solver benchmark and generate comparative analysis:
```bash
python solver_benchmark.py
```

This generates:
- `RAPPORT_COMPARATIF_SOLVEURS.md`: Detailed comparative report in French
- `solver_comparison.png`: Performance visualizations
- `solver_benchmark_results.csv`: Raw benchmark data
