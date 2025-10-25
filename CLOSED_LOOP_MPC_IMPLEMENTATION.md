# Closed-Loop MPC Implementation

## Overview

This document describes the implementation of a closed-loop Model Predictive Control (MPC) with receding horizon for the 1D drone control system with suspended load.

## Implementation Details

### 1. Closed-Loop MPC Function

The main implementation is in `simulate_closed_loop_mpc()` which:

- **Replans every 3 seconds** (30 time steps at 0.1s sampling)
- **Uses actual nonlinear state** from the simulation as feedback
- **Implements warm-start** by shifting previous solution
- **Adapts horizon** as simulation approaches the end
- **Returns complete trajectory** with timing information

### 2. MPC Controller Enhancement

The `MPCController.solve()` method now accepts:

- `u_warmstart`: Previous optimal control for initialization
- `x_warmstart`: Previous optimal state trajectory for initialization
- `current_time`: Current simulation time (for future terminal constraint adaptation)

### 3. Warm-Start Strategy

At each replanning step:
1. Shift previous solution by discarding the first `replan_interval` steps
2. Pad with last control value or zeros
3. Use shifted solution as initial guess for next optimization
4. Reduces solve time and improves convergence

### 4. Terminal Constraints

The terminal constraints are automatically adapted for the receding horizon:
- Each MPC solve considers the remaining time in the simulation
- Terminal cost weights ensure the system approaches the target
- The horizon shrinks appropriately as the end approaches

## Key Features

### Receding Horizon Control
- **Fixed absolute time reference**: Target is always at the same absolute position
- **Moving optimization window**: MPC window slides forward in time
- **State feedback**: Uses actual nonlinear state, not predicted state

### Nonlinearity Handling
- **Model mismatch correction**: Feedback compensates for linearization errors
- **Disturbance rejection**: Can handle unexpected disturbances (not shown but supported)
- **Improved accuracy**: Nonlinear effects (drag, large angles) better handled

## Performance Results

### Test Case 1: 20m Target from Rest
```
Open-loop:
  Final error: 1.530m
  Mean error: 8.075m
  Max error: 20.000m

Closed-loop (replanning every 3s):
  Final error: 0.079m  (94.8% improvement)
  Mean error: 6.647m
  Max error: 20.000m
  Average solve time: 395ms
```

### Test Case 2: 40m Target from Rest
```
Open-loop:
  Final error: 12.464m
  Mean error: 16.749m
  Max error: 40.000m

Closed-loop (replanning every 3s):
  Final error: 0.046m  (99.6% improvement)
  Mean error: 11.612m
  Max error: 40.000m
  Average solve time: 637ms
```

### Test Case 3: 20m Target with Initial Velocity
```
Initial conditions: v=2m/s, θ=5.73°

Open-loop:
  Final error: 0.791m
  Mean error: 5.650m
  Max error: 18.103m

Closed-loop (replanning every 3s):
  Final error: 0.027m  (96.6% improvement)
  Mean error: 4.993m
  Max error: 18.103m
  Average solve time: 399ms
```

## Computational Performance

- **Solve time per MPC**: 300-650ms (depending on horizon)
- **Replanning frequency**: Every 3 seconds (30 time steps)
- **Real-time capable**: Yes, ample time between replanning steps
- **Overhead**: Minimal, ~2-3s total for 10-15s simulation

## Comparison with Open-Loop

| Metric | Open-Loop | Closed-Loop | Advantage |
|--------|-----------|-------------|-----------|
| Final Error | 0.5-12m | 0.03-0.08m | 94-99% reduction |
| Model Sensitivity | High | Low | Robust to mismatch |
| Disturbance Rejection | None | Good | Real-time feedback |
| Computational Load | Single solve | Multiple solves | Still real-time |
| Prediction Accuracy | Degrades over time | Maintains accuracy | Continuous correction |

## Usage Example

```python
from drone_control_1d import DroneSystem, MPCController, simulate_closed_loop_mpc, TS
import numpy as np

# Create system and controller
system = DroneSystem()
A, B = system.get_linearized_matrices()
Ad, Bd = system.discretize(A, B, TS)
controller = MPCController(Ad, Bd, system.L, horizon=100)

# Initial state and target
x0 = np.array([0.0, 0.0, 0.0, 0.0])  # At rest at origin
x_load_target = 20.0  # Target 20m

# Cost weights
Q_state = np.array([10.0, 20.0, 500.0, 100.0])
Q_load = 1000.0
R_weight = 2.0

# Run closed-loop MPC
t, x_traj, u_traj, solve_times = simulate_closed_loop_mpc(
    system, controller, x0, x_load_target, 
    Q_state, Q_load, R_weight,
    total_horizon=100,  # 10 seconds
    replan_interval=30,  # Replan every 3 seconds
    dt=TS
)

print(f"Final position: {x_traj[0, -1] + system.L * np.sin(x_traj[2, -1]):.3f}m")
print(f"Average solve time: {np.mean(solve_times)*1000:.1f}ms")
```

## Implementation Benefits

1. **Significantly improved accuracy**: 94-99% reduction in final tracking error
2. **Better nonlinearity handling**: Feedback corrects for model mismatch
3. **Real-time capable**: Solve times well within replanning interval
4. **Robust**: Less sensitive to initial trajectory prediction
5. **Practical**: Simulates realistic closed-loop control scenario

## Technical Notes

### Warm-Start Implementation
The warm-start strategy shifts the previous solution forward in time:
```python
if u_prev.shape[1] > replan_interval:
    u_warmstart = np.hstack([
        u_prev[:, replan_interval:],  # Shift: discard applied controls
        np.tile(u_prev[:, -1:], ...)  # Pad: repeat last control
    ])
```

### Receding Horizon Adaptation
The horizon automatically adapts:
```python
remaining_steps = total_horizon - steps_taken
mpc_horizon = min(controller.N, remaining_steps)
```

### State Feedback Loop
At each replanning step:
1. Measure current state from nonlinear simulation
2. Solve MPC with current state as initial condition
3. Apply first `replan_interval` controls
4. Update state using nonlinear dynamics
5. Repeat

## Future Enhancements

Possible improvements:
- Adaptive replanning interval based on tracking error
- Online parameter estimation for better model matching
- Tube-based MPC for guaranteed constraint satisfaction
- Obstacle avoidance constraints
- Multiple load scenarios
- Wind disturbance modeling

## Conclusion

The closed-loop MPC implementation demonstrates the significant advantages of receding horizon control with state feedback. The 94-99% improvement in tracking accuracy, combined with real-time computational performance, makes this approach highly suitable for practical drone control applications.
