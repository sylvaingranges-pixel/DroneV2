#!/usr/bin/env python3
"""
Quick Start Example for Drone Control System

This script demonstrates how to use the drone control system
for a simple trajectory optimization task.
"""

from drone_control import DroneLoadSystem, OptimalController
import numpy as np
import matplotlib.pyplot as plt

# Initialize the system
print("Initializing drone control system...")
system = DroneLoadSystem()

# Create the controller
controller = OptimalController(system, N_horizon=100)

# Define initial state: [x_d, v_d, theta, omega]
# Starting at origin, at rest
x0 = np.array([0.0, 0.0, 0.0, 0.0])

# Target load position
target_position = 30.0  # meters

print(f"\nOptimizing trajectory from rest to {target_position}m...")

# Solve for optimal trajectory
u_opt, x_opt, solve_time = controller.solve_trajectory(x0, target_position)

if u_opt is not None:
    print(f"✅ Optimization successful!")
    print(f"   Solve time: {solve_time*1000:.1f} ms")
    print(f"   Horizon: {x_opt.shape[1]-1} steps")
    
    # Extract results
    N = x_opt.shape[1] - 1
    t = np.arange(N + 1) * system.Ts
    x_load = (controller.Cd @ x_opt)[0, :]
    
    print(f"\nFinal state:")
    print(f"   Load position: {x_load[-1]:.4f} m (target: {target_position:.2f} m)")
    print(f"   Position error: {abs(x_load[-1] - target_position):.6f} m")
    print(f"   Drone velocity: {x_opt[1, -1]:.6f} m/s")
    print(f"   Cable angle: {np.rad2deg(x_opt[2, -1]):.6f} deg")
    
    # Create a simple plot
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f'Drone Trajectory: 0m → {target_position}m', fontsize=14, fontweight='bold')
    
    # Load position
    axes[0, 0].plot(t, x_load, 'r-', linewidth=2)
    axes[0, 0].axhline(y=target_position, color='k', linestyle='--', label='Target')
    axes[0, 0].set_ylabel('Load Position (m)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Drone velocity
    axes[0, 1].plot(t, x_opt[1, :], 'g-', linewidth=2)
    axes[0, 1].set_ylabel('Drone Velocity (m/s)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Cable angle
    axes[1, 0].plot(t, np.rad2deg(x_opt[2, :]), 'm-', linewidth=2)
    axes[1, 0].set_ylabel('Cable Angle (deg)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].grid(True, alpha=0.3)
    
    # Control input
    axes[1, 1].step(t[:-1], u_opt[0, :], 'orange', linewidth=2, where='post')
    axes[1, 1].set_ylabel('Drone Acceleration (m/s²)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('quick_example.png', dpi=150, bbox_inches='tight')
    print(f"\n📊 Plot saved as 'quick_example.png'")
    plt.show()
    
else:
    print("❌ Optimization failed!")
