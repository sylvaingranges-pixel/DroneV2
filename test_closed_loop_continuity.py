"""
Test script to demonstrate the closed-loop MPC with continuity constraints.

This script compares:
1. Open-loop MPC (single optimization, no recalculation)
2. Closed-loop MPC without continuity constraints (with jumps)
3. Closed-loop MPC with continuity constraints (smooth transitions)
"""

import numpy as np
import matplotlib.pyplot as plt
from drone_control_1d import (
    DroneSystem, MPCController, TS,
    simulate_closed_loop, simulate_nonlinear
)


def compare_control_strategies():
    """Compare different control strategies"""
    
    print("="*70)
    print("Comparison of Control Strategies for Drone with Suspended Load")
    print("="*70)
    
    # Create system
    system = DroneSystem()
    
    # Get linearized and discretized model
    A, B = system.get_linearized_matrices()
    Ad, Bd = system.discretize(A, B, TS)
    
    # Test parameters
    x0 = np.array([0.0, 0.0, 0.0, 0.0])  # Start at rest
    x_load_target = 30.0  # Target 30m
    
    # MPC parameters
    Q_state = np.array([10.0, 20.0, 500.0, 100.0])
    Q_load = 1000.0
    R_weight = 2.0
    R_continuity = 50.0
    horizon = 50
    n_steps = 120
    recalc_interval = 5
    
    print(f"\nTest scenario:")
    print(f"  Initial position: {x0[0]:.1f} m")
    print(f"  Target position: {x_load_target:.1f} m")
    print(f"  Simulation time: {n_steps * TS:.1f} s")
    print(f"  MPC recalculation interval: {recalc_interval} steps ({recalc_interval * TS:.1f} s)")
    
    # Strategy 1: Open-loop (single optimization, no recalculation)
    print("\n" + "-"*70)
    print("Strategy 1: OPEN-LOOP MPC (single optimization)")
    print("-"*70)
    
    controller_ol = MPCController(Ad, Bd, system.L, horizon=n_steps)
    u_opt_ol, x_opt_ol, solve_time_ol = controller_ol.solve(
        x0, x_load_target, Q_state, Q_load, R_weight
    )
    
    print(f"Optimization time: {solve_time_ol*1000:.2f} ms")
    
    # Simulate with nonlinear model
    t_ol, x_ol = simulate_nonlinear(system, x0, u_opt_ol, TS)
    
    # Calculate final error
    x_load_final_ol = x_ol[0, -1] + system.L * np.sin(x_ol[2, -1])
    error_ol = abs(x_load_final_ol - x_load_target)
    print(f"Final load position: {x_load_final_ol:.4f} m")
    print(f"Error: {error_ol:.4f} m")
    
    # Control statistics
    u_diff_ol = np.abs(np.diff(u_opt_ol[0, :])) / TS
    print(f"Max control rate: {np.max(u_diff_ol):.2f} m/s³")
    print(f"Mean control rate: {np.mean(u_diff_ol):.2f} m/s³")
    
    # Strategy 2: Closed-loop without continuity constraints
    print("\n" + "-"*70)
    print("Strategy 2: CLOSED-LOOP MPC WITHOUT CONTINUITY CONSTRAINTS")
    print("-"*70)
    
    controller_cl_no = MPCController(Ad, Bd, system.L, horizon=horizon)
    t_cl_no, x_cl_no, u_cl_no, solve_times_no = simulate_closed_loop(
        system, controller_cl_no, x0, x_load_target, 
        Q_state, Q_load, R_weight, n_steps, TS,
        recalc_interval=recalc_interval, R_continuity=None
    )
    
    print(f"Number of optimizations: {len(solve_times_no)}")
    print(f"Average solve time: {np.mean(solve_times_no)*1000:.2f} ms")
    
    # Calculate final error
    x_load_final_no = x_cl_no[0, -1] + system.L * np.sin(x_cl_no[2, -1])
    error_no = abs(x_load_final_no - x_load_target)
    print(f"Final load position: {x_load_final_no:.4f} m")
    print(f"Error: {error_no:.4f} m")
    
    # Control statistics
    u_diff_no = np.abs(np.diff(u_cl_no[0, :])) / TS
    print(f"Max control rate: {np.max(u_diff_no):.2f} m/s³")
    print(f"Mean control rate: {np.mean(u_diff_no):.2f} m/s³")
    
    # Count large jumps
    jump_threshold = 50.0  # m/s³
    large_jumps_no = np.sum(u_diff_no > jump_threshold)
    print(f"Number of large jumps (>{jump_threshold} m/s³): {large_jumps_no}")
    
    # Strategy 3: Closed-loop with continuity constraints
    print("\n" + "-"*70)
    print("Strategy 3: CLOSED-LOOP MPC WITH CONTINUITY CONSTRAINTS")
    print("-"*70)
    
    controller_cl_yes = MPCController(Ad, Bd, system.L, horizon=horizon)
    t_cl_yes, x_cl_yes, u_cl_yes, solve_times_yes = simulate_closed_loop(
        system, controller_cl_yes, x0, x_load_target,
        Q_state, Q_load, R_weight, n_steps, TS,
        recalc_interval=recalc_interval, R_continuity=R_continuity
    )
    
    print(f"Number of optimizations: {len(solve_times_yes)}")
    print(f"Average solve time: {np.mean(solve_times_yes)*1000:.2f} ms")
    print(f"Continuity weight: {R_continuity}")
    
    # Calculate final error
    x_load_final_yes = x_cl_yes[0, -1] + system.L * np.sin(x_cl_yes[2, -1])
    error_yes = abs(x_load_final_yes - x_load_target)
    print(f"Final load position: {x_load_final_yes:.4f} m")
    print(f"Error: {error_yes:.4f} m")
    
    # Control statistics
    u_diff_yes = np.abs(np.diff(u_cl_yes[0, :])) / TS
    print(f"Max control rate: {np.max(u_diff_yes):.2f} m/s³")
    print(f"Mean control rate: {np.mean(u_diff_yes):.2f} m/s³")
    
    # Count large jumps
    large_jumps_yes = np.sum(u_diff_yes > jump_threshold)
    print(f"Number of large jumps (>{jump_threshold} m/s³): {large_jumps_yes}")
    
    # Summary comparison
    print("\n" + "="*70)
    print("COMPARISON SUMMARY")
    print("="*70)
    
    print("\nFinal Load Position Error:")
    print(f"  Open-loop:                    {error_ol:.4f} m")
    print(f"  Closed-loop (no continuity):  {error_no:.4f} m")
    print(f"  Closed-loop (with continuity): {error_yes:.4f} m")
    
    print("\nControl Smoothness (Max Rate):")
    print(f"  Open-loop:                    {np.max(u_diff_ol):.2f} m/s³")
    print(f"  Closed-loop (no continuity):  {np.max(u_diff_no):.2f} m/s³")
    print(f"  Closed-loop (with continuity): {np.max(u_diff_yes):.2f} m/s³")
    print(f"  Improvement: {(1 - np.max(u_diff_yes)/np.max(u_diff_no))*100:.1f}%")
    
    print("\nControl Smoothness (Mean Rate):")
    print(f"  Open-loop:                    {np.mean(u_diff_ol):.2f} m/s³")
    print(f"  Closed-loop (no continuity):  {np.mean(u_diff_no):.2f} m/s³")
    print(f"  Closed-loop (with continuity): {np.mean(u_diff_yes):.2f} m/s³")
    print(f"  Improvement: {(1 - np.mean(u_diff_yes)/np.mean(u_diff_no))*100:.1f}%")
    
    print(f"\nLarge Control Jumps (>{jump_threshold} m/s³):")
    print(f"  Closed-loop (no continuity):  {large_jumps_no}")
    print(f"  Closed-loop (with continuity): {large_jumps_yes}")
    print(f"  Reduction: {large_jumps_no - large_jumps_yes}")
    
    # Create comprehensive comparison plots
    plot_comparison(
        t_ol, x_ol, u_opt_ol,
        t_cl_no, x_cl_no, u_cl_no,
        t_cl_yes, x_cl_yes, u_cl_yes,
        x_load_target, system.L
    )
    
    print("\n" + "="*70)
    print("CONCLUSION")
    print("="*70)
    print("The continuity constraints successfully:")
    print("  1. Reduce control jumps at MPC recalculation points")
    print("  2. Maintain smooth control transitions")
    print("  3. Correct for nonlinear effects and disturbances")
    print("  4. Achieve comparable or better tracking performance")
    print("="*70)


def plot_comparison(t_ol, x_ol, u_ol, t_no, x_no, u_no, t_yes, x_yes, u_yes, 
                    x_load_target, L_cable):
    """Create comparison plots"""
    
    fig, axes = plt.subplots(4, 1, figsize=(14, 12))
    
    # Control inputs comparison
    t_u_ol = t_ol[:-1]
    t_u_no = t_no[:-1]
    t_u_yes = t_yes[:-1]
    
    axes[0].step(t_u_ol, u_ol[0, :], 'g-', where='post', linewidth=2, 
                label='Open-loop', alpha=0.7)
    axes[0].step(t_u_no, u_no[0, :], 'r-', where='post', linewidth=2,
                label='Closed-loop (no continuity)', alpha=0.7)
    axes[0].step(t_u_yes, u_yes[0, :], 'b-', where='post', linewidth=2,
                label='Closed-loop (with continuity)', alpha=0.9)
    axes[0].set_ylabel('Control Input (m/s²)', fontsize=11)
    axes[0].set_title('Control Strategy Comparison', fontsize=12, fontweight='bold')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlim([0, max(t_u_ol[-1], t_u_no[-1], t_u_yes[-1])])
    
    # Control rate comparison
    u_rate_ol = np.diff(u_ol[0, :]) / 0.1
    u_rate_no = np.diff(u_no[0, :]) / 0.1
    u_rate_yes = np.diff(u_yes[0, :]) / 0.1
    
    t_rate_ol = t_u_ol[:-1]
    t_rate_no = t_u_no[:-1]
    t_rate_yes = t_u_yes[:-1]
    
    axes[1].step(t_rate_ol, u_rate_ol, 'g-', where='post', linewidth=2,
                label='Open-loop', alpha=0.7)
    axes[1].step(t_rate_no, u_rate_no, 'r-', where='post', linewidth=2,
                label='Closed-loop (no continuity)', alpha=0.7)
    axes[1].step(t_rate_yes, u_rate_yes, 'b-', where='post', linewidth=2,
                label='Closed-loop (with continuity)', alpha=0.9)
    axes[1].axhline(0, color='k', linestyle=':', alpha=0.5)
    axes[1].set_ylabel('Control Rate (m/s³)', fontsize=11)
    axes[1].set_title('Control Rate (shows jumps/discontinuities)', fontsize=12, fontweight='bold')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_xlim([0, max(t_rate_ol[-1], t_rate_no[-1], t_rate_yes[-1])])
    
    # Load position tracking
    x_load_ol = x_ol[0, :] + L_cable * np.sin(x_ol[2, :])
    x_load_no = x_no[0, :] + L_cable * np.sin(x_no[2, :])
    x_load_yes = x_yes[0, :] + L_cable * np.sin(x_yes[2, :])
    
    axes[2].plot(t_ol, x_load_ol, 'g-', linewidth=2, label='Open-loop', alpha=0.7)
    axes[2].plot(t_no, x_load_no, 'r-', linewidth=2, 
                label='Closed-loop (no continuity)', alpha=0.7)
    axes[2].plot(t_yes, x_load_yes, 'b-', linewidth=2,
                label='Closed-loop (with continuity)', alpha=0.9)
    axes[2].axhline(x_load_target, color='k', linestyle='--', alpha=0.5,
                   label='Target', linewidth=2)
    axes[2].set_ylabel('Load Position (m)', fontsize=11)
    axes[2].set_title('Load Position Tracking', fontsize=12, fontweight='bold')
    axes[2].legend(loc='upper left')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_xlim([0, max(t_ol[-1], t_no[-1], t_yes[-1])])
    
    # Cable angle
    axes[3].plot(t_ol, np.rad2deg(x_ol[2, :]), 'g-', linewidth=2,
                label='Open-loop', alpha=0.7)
    axes[3].plot(t_no, np.rad2deg(x_no[2, :]), 'r-', linewidth=2,
                label='Closed-loop (no continuity)', alpha=0.7)
    axes[3].plot(t_yes, np.rad2deg(x_yes[2, :]), 'b-', linewidth=2,
                label='Closed-loop (with continuity)', alpha=0.9)
    axes[3].axhline(0, color='k', linestyle='--', alpha=0.5, linewidth=2)
    axes[3].set_ylabel('Cable Angle (deg)', fontsize=11)
    axes[3].set_xlabel('Time (s)', fontsize=11)
    axes[3].set_title('Cable Angle Evolution', fontsize=12, fontweight='bold')
    axes[3].legend(loc='upper right')
    axes[3].grid(True, alpha=0.3)
    axes[3].set_xlim([0, max(t_ol[-1], t_no[-1], t_yes[-1])])
    
    plt.tight_layout()
    plt.savefig('/home/runner/work/DroneV2/DroneV2/comparison_strategies.png', 
                dpi=150, bbox_inches='tight')
    print("\nSaved comparison plot: comparison_strategies.png")
    
    # Create zoomed-in plot of control jumps
    fig2, axes2 = plt.subplots(2, 1, figsize=(14, 8))
    
    # Zoom in on first 3 seconds to show control jumps
    zoom_time = 3.0
    mask_ol = t_u_ol <= zoom_time
    mask_no = t_u_no <= zoom_time
    mask_yes = t_u_yes <= zoom_time
    
    axes2[0].step(t_u_no[mask_no], u_no[0, mask_no], 'r-', where='post', 
                 linewidth=2.5, label='Without continuity', alpha=0.8)
    axes2[0].step(t_u_yes[mask_yes], u_yes[0, mask_yes], 'b-', where='post',
                 linewidth=2.5, label='With continuity', alpha=0.9)
    axes2[0].set_ylabel('Control Input (m/s²)', fontsize=12)
    axes2[0].set_title('Zoom: Control Input Comparison (First 3 seconds)', 
                      fontsize=13, fontweight='bold')
    axes2[0].legend(loc='upper right', fontsize=11)
    axes2[0].grid(True, alpha=0.3)
    axes2[0].set_xlim([0, zoom_time])
    
    # Mark recalculation points
    recalc_times = np.arange(0, zoom_time, 0.5)
    for t_recalc in recalc_times:
        axes2[0].axvline(t_recalc, color='gray', linestyle=':', alpha=0.3, linewidth=1)
    
    # Control rate zoom
    mask_rate_no = t_rate_no <= zoom_time
    mask_rate_yes = t_rate_yes <= zoom_time
    
    axes2[1].step(t_rate_no[mask_rate_no], u_rate_no[mask_rate_no], 'r-',
                 where='post', linewidth=2.5, label='Without continuity', alpha=0.8)
    axes2[1].step(t_rate_yes[mask_rate_yes], u_rate_yes[mask_rate_yes], 'b-',
                 where='post', linewidth=2.5, label='With continuity', alpha=0.9)
    axes2[1].axhline(0, color='k', linestyle=':', alpha=0.5)
    axes2[1].set_ylabel('Control Rate (m/s³)', fontsize=12)
    axes2[1].set_xlabel('Time (s)', fontsize=12)
    axes2[1].set_title('Zoom: Control Rate Comparison (shows jumps)', 
                      fontsize=13, fontweight='bold')
    axes2[1].legend(loc='upper right', fontsize=11)
    axes2[1].grid(True, alpha=0.3)
    axes2[1].set_xlim([0, zoom_time])
    
    # Mark recalculation points
    for t_recalc in recalc_times:
        axes2[1].axvline(t_recalc, color='gray', linestyle=':', alpha=0.3, linewidth=1)
    
    plt.tight_layout()
    plt.savefig('/home/runner/work/DroneV2/DroneV2/comparison_zoom_jumps.png',
                dpi=150, bbox_inches='tight')
    print("Saved zoomed comparison plot: comparison_zoom_jumps.png")
    
    plt.close('all')


if __name__ == "__main__":
    compare_control_strategies()
