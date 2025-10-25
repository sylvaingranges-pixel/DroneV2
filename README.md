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
5. **Simulation and visualization** comparing:
   - Optimal trajectory from MPC
   - Linear model response
   - Non-linear model response (using RK45 ODE solver)

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

This will execute multiple test cases and generate plots showing:
- System trajectories (drone position, velocity, cable angle, angular velocity)
- Control inputs
- Comparison between optimal, linear model, and non-linear model responses
- Load position tracking

## Test Cases

The system includes tests with:
- Starting from rest
- Starting with initial velocity
- Destinations at 20m, 40m, and 80m

## Solver Optimization

The system uses **SCS (Splitting Conic Solver)** with optimized parameters for fast computation:
- **Solver**: SCS with relaxed tolerances (`eps=1e-2`)
- **Average solve time**: 330-495ms (2.0-2.5x faster than baseline)
- **Horizons**: Optimized 50-70 steps (vs 100-200 previously)
- **Precision**: Meets requirements (0.3m position, 0.15m/s velocity)
- **Success rate**: 100%

### Performance Optimization
The optimization strategy combines:
1. **Relaxed tolerances** from 1e-4 to 1e-2 (sufficient for required precision)
2. **Shorter horizons** adapted to target distance (50-70 vs 100-200 steps)
3. **SCS solver** more efficient than CLARABEL for this configuration

See `OPTIMISATION_PERFORMANCES.md` for detailed optimization analysis.

### Solver Comparison
Previous benchmarking identified CLARABEL as optimal for high precision:
- See `RAPPORT_COMPARATIF_SOLVEURS.md` for comparison (CLARABEL, SCS, ECOS, OSQP, CVXOPT)

Current configuration prioritizes speed while maintaining required precision:
- See `optimization_benchmark.py` for speed-optimized comparison

## Performance

The optimization now solves in **330-495ms**, making it highly suitable for real-time closed-loop control at 2-3 Hz update rate.

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
