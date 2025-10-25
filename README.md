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

## Performance

The optimization typically solves in 200-600ms, making it suitable for real-time closed-loop control applications.

## Results

Generated plots are saved as PNG files in the working directory:
- `results_*.png`: Full state trajectories and control inputs
- `load_position_*.png`: Load position tracking performance
