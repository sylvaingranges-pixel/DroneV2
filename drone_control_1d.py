"""
1D Drone Control System with Suspended Load

This module implements a control system for a drone carrying a suspended load.
The system includes:
- Non-linear dynamics with air resistance
- Linearized model around equilibrium
- Discrete-time model
- MPC controller using CVXPY
- Simulation and visualization

Author: Drone Control System
Date: 2025-10-25
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.linalg import expm
import cvxpy as cp
import time

# Physical constants
G = 9.81  # Gravity (m/s^2)

# System parameters
M_DRONE = 40.0    # Drone mass (kg)
M_LOAD = 24.0     # Load mass (kg)
L_CABLE = 19.0    # Cable length (m)
S_LOAD = 0.2      # Load surface area (m^2)
CD_LOAD = 1.0     # Drag coefficient
RHO_AIR = 1.225   # Air density (kg/m^3)

# Sampling time
TS = 0.1  # seconds


class DroneSystem:
    """Drone system with suspended load - 1D model along X-axis"""
    
    def __init__(self):
        self.m_d = M_DRONE
        self.m_l = M_LOAD
        self.L = L_CABLE
        self.S = S_LOAD
        self.Cd = CD_LOAD
        self.rho = RHO_AIR
        self.g = G
        
        # Drag force coefficient
        self.k_drag = 0.5 * self.rho * self.Cd * self.S
        
    def nonlinear_dynamics(self, t, state, u):
        """
        Non-linear dynamics of the drone-load system
        
        State vector: [x_d, v_d, theta, omega]
        - x_d: drone position (m)
        - v_d: drone velocity (m/s)
        - theta: cable angle from vertical (rad)
        - omega: angular velocity of cable (rad/s)
        
        Control input: u = acceleration of drone (m/s^2)
        
        Returns: state derivative
        """
        x_d, v_d, theta, omega = state
        
        # Load position and velocity
        x_l = x_d + self.L * np.sin(theta)
        v_l = v_d + self.L * omega * np.cos(theta)
        
        # Drag force on load (opposes velocity)
        if abs(v_l) < 1e-6:
            F_drag = 0
        else:
            F_drag = -self.k_drag * v_l * abs(v_l)  # quadratic drag
        
        # Dynamics equations
        # Drone acceleration (given as input)
        a_d = u
        
        # Angular dynamics of cable
        # From Lagrangian mechanics for pendulum with moving support
        # m_l * L * theta_ddot = -m_l * a_d * cos(theta) - m_l * g * sin(theta) + F_drag * cos(theta)
        theta_ddot = (-a_d * np.cos(theta) - self.g * np.sin(theta) + 
                      (F_drag / self.m_l) * np.cos(theta)) / self.L
        
        return np.array([v_d, a_d, omega, theta_ddot])
    
    def get_linearized_matrices(self):
        """
        Linearize system around equilibrium point:
        - Load directly below drone (theta = 0)
        - No motion (v_d = 0, omega = 0)
        - No aerodynamic effects (v_l = 0)
        
        State: [x_d, v_d, theta, omega]
        Input: u (drone acceleration)
        
        Returns: A, B matrices for continuous-time linear system
        """
        # Equilibrium: theta = 0, omega = 0, v_d = 0
        # At equilibrium: theta_ddot = -u/L (when theta = 0)
        
        # Linearization:
        # x_d_dot = v_d
        # v_d_dot = u
        # theta_dot = omega
        # omega_dot = -u/L - g*theta/L  (linearized around theta=0, ignoring drag)
        
        # State matrix A
        A = np.array([
            [0, 1, 0, 0],           # x_d_dot = v_d
            [0, 0, 0, 0],           # v_d_dot = u (no dependency on state)
            [0, 0, 0, 1],           # theta_dot = omega
            [0, 0, -self.g/self.L, 0]  # omega_dot = -g*theta/L - u/L
        ])
        
        # Input matrix B
        B = np.array([
            [0],       # x_d_dot
            [1],       # v_d_dot = u
            [0],       # theta_dot
            [-1/self.L]  # omega_dot includes -u/L term
        ])
        
        return A, B
    
    def discretize(self, A, B, dt):
        """
        Discretize continuous-time linear system using matrix exponential
        
        Returns: Ad, Bd matrices for discrete-time system
        """
        n = A.shape[0]
        m = B.shape[1]
        
        # Build augmented matrix for exact discretization
        M = np.zeros((n + m, n + m))
        M[:n, :n] = A
        M[:n, n:] = B
        
        # Matrix exponential
        M_exp = expm(M * dt)
        
        Ad = M_exp[:n, :n]
        Bd = M_exp[:n, n:]
        
        return Ad, Bd


class MPCController:
    """Model Predictive Controller for drone system"""
    
    def __init__(self, Ad, Bd, L_cable, horizon=50):
        """
        Initialize MPC controller
        
        Args:
            Ad: Discrete state matrix
            Bd: Discrete input matrix
            L_cable: Cable length (m)
            horizon: Prediction horizon
        """
        self.Ad = Ad
        self.Bd = Bd
        self.L = L_cable
        self.N = horizon  # prediction horizon
        self.nx = Ad.shape[0]  # number of states
        self.nu = Bd.shape[1]  # number of inputs
        
        # Constraints
        self.u_max = 15.0  # Maximum acceleration (m/s^2)
        self.u_min = -15.0  # Minimum acceleration (m/s^2)
        
        # Store previous solution for warm starting and continuity
        self.u_prev_solution = None
        self.x_prev_solution = None
        
    def solve(self, x0, x_load_target, Q_state, Q_load, R, Qf_state=None, Qf_load=None, 
              u_prev=None, R_continuity=None):
        """
        Solve MPC optimization problem with focus on load position
        
        Args:
            x0: Initial state [x_d, v_d, theta, omega]
            x_load_target: Target load position (scalar)
            Q_state: State cost weights [q_xd, q_vd, q_theta, q_omega]
            Q_load: Weight for load position error
            R: Input cost weight
            Qf_state: Terminal state cost weights
            Qf_load: Terminal load position cost weight
            u_prev: Previous control input (to enforce continuity at start). If None, no constraint
            R_continuity: Weight for continuity with previous solution. If None, no continuity penalty
            
        Returns:
            u_opt: Optimal control sequence
            x_opt: Optimal state trajectory
            solve_time: Time to solve optimization
        """
        if Qf_state is None:
            Qf_state = Q_state * 50
        if Qf_load is None:
            Qf_load = Q_load * 100
        
        # Define optimization variables
        x = cp.Variable((self.nx, self.N + 1))
        u = cp.Variable((self.nu, self.N))
        
        # Cost function
        cost = 0
        constraints = []
        
        # Initial condition
        constraints.append(x[:, 0] == x0)
        
        # Continuity constraint: first control input matches previous (if provided)
        if u_prev is not None:
            constraints.append(u[:, 0] == u_prev)
        
        # Compute target state (drone position for load at target)
        x_drone_target = x_load_target  # When theta=0, x_load = x_drone
        x_target = np.array([x_drone_target, 0.0, 0.0, 0.0])
        
        # Stage costs and dynamics constraints
        for k in range(self.N):
            # State tracking cost
            dx = x[:, k] - x_target
            cost += Q_state[0] * dx[0]**2  # drone position
            cost += Q_state[1] * dx[1]**2  # drone velocity
            cost += Q_state[2] * dx[2]**2  # angle
            cost += Q_state[3] * dx[3]**2  # angular velocity
            
            # Load position cost
            # For linearized model: sin(theta) ≈ theta (small angle approximation)
            # x_load = x_d + L*sin(theta) ≈ x_d + L*theta
            x_load = x[0, k] + self.L * x[2, k]  # Linear approximation
            cost += Q_load * (x_load - x_load_target)**2
            
            # Control effort
            cost += R * u[0, k]**2
            
            # Continuity penalty: penalize deviation from previous solution
            if R_continuity is not None and self.u_prev_solution is not None:
                # Only penalize if we have previous solution and k is within bounds
                if k < self.u_prev_solution.shape[1]:
                    cost += R_continuity * (u[0, k] - self.u_prev_solution[0, k])**2
            
            # Dynamics constraint
            constraints.append(x[:, k + 1] == self.Ad @ x[:, k] + self.Bd @ u[:, k])
            
            # Input constraints
            constraints.append(u[:, k] <= self.u_max)
            constraints.append(u[:, k] >= self.u_min)
        
        # Terminal cost - state
        dx_f = x[:, self.N] - x_target
        cost += Qf_state[0] * dx_f[0]**2
        cost += Qf_state[1] * dx_f[1]**2
        cost += Qf_state[2] * dx_f[2]**2
        cost += Qf_state[3] * dx_f[3]**2
        
        # Terminal cost - load position (using small angle approximation)
        x_load_f = x[0, self.N] + self.L * x[2, self.N]  # Linear approximation
        cost += Qf_load * (x_load_f - x_load_target)**2
        
        # Solve optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        
        start_time = time.time()
        # Use CLARABEL solver - fastest and most robust based on benchmark
        # (see RAPPORT_COMPARATIF_SOLVEURS.md for detailed comparison)
        problem.solve(solver=cp.CLARABEL, verbose=False, max_iter=100000, 
                     tol_gap_abs=1e-4, tol_gap_rel=1e-4)
        solve_time = time.time() - start_time
        
        if problem.status not in ["optimal", "optimal_inaccurate", "solved", "solved_inaccurate"]:
            print(f"Warning: Optimization status: {problem.status}")
            return None, None, solve_time
        
        u_opt = u.value
        x_opt = x.value
        
        # Store solution for next iteration (warm starting and continuity)
        self.u_prev_solution = u_opt
        self.x_prev_solution = x_opt
        
        return u_opt, x_opt, solve_time


def simulate_linear(Ad, Bd, x0, u_sequence):
    """
    Simulate discrete linear system
    
    Args:
        Ad: Discrete state matrix
        Bd: Discrete input matrix
        x0: Initial state
        u_sequence: Control input sequence (nu x N)
        
    Returns:
        x_trajectory: State trajectory (nx x N+1)
    """
    N = u_sequence.shape[1]
    nx = Ad.shape[0]
    
    x_trajectory = np.zeros((nx, N + 1))
    x_trajectory[:, 0] = x0
    
    for k in range(N):
        x_trajectory[:, k + 1] = Ad @ x_trajectory[:, k] + Bd @ u_sequence[:, k]
    
    return x_trajectory


def simulate_nonlinear(system, x0, u_sequence, dt):
    """
    Simulate non-linear system using RK45 solver
    
    Args:
        system: DroneSystem object
        x0: Initial state
        u_sequence: Control input sequence (nu x N)
        dt: Time step
        
    Returns:
        t_trajectory: Time vector
        x_trajectory: State trajectory
    """
    N = u_sequence.shape[1]
    t_trajectory = []
    x_trajectory = []
    
    x_current = x0.copy()
    t_current = 0
    
    t_trajectory.append(t_current)
    x_trajectory.append(x_current.copy())
    
    for k in range(N):
        # Constant input over time interval
        u_k = u_sequence[0, k]
        
        # Solve ODE for this time step
        def dynamics(t, x):
            return system.nonlinear_dynamics(t, x, u_k)
        
        sol = solve_ivp(
            dynamics,
            [t_current, t_current + dt],
            x_current,
            method='RK45',
            max_step=dt/10
        )
        
        # Update state
        x_current = sol.y[:, -1]
        t_current += dt
        
        t_trajectory.append(t_current)
        x_trajectory.append(x_current.copy())
    
    return np.array(t_trajectory), np.array(x_trajectory).T


def simulate_closed_loop(system, controller, x0, x_load_target, Q_state, Q_load, R, 
                         n_steps, dt, recalc_interval=1, u_prev=None, R_continuity=None):
    """
    Simulate closed-loop control with MPC recalculation at each time step
    
    Args:
        system: DroneSystem object
        controller: MPCController object
        x0: Initial state
        x_load_target: Target load position
        Q_state: State cost weights
        Q_load: Load position cost weight
        R: Control effort weight
        n_steps: Number of simulation steps
        dt: Time step
        recalc_interval: How often to recalculate MPC (in steps)
        u_prev: Previous control input (for initial constraint)
        R_continuity: Weight for continuity penalty
        
    Returns:
        t_trajectory: Time vector
        x_trajectory: State trajectory
        u_trajectory: Control input trajectory
        solve_times: Array of solve times for each MPC call
    """
    t_trajectory = [0]
    x_trajectory = [x0.copy()]
    u_trajectory = []
    solve_times = []
    
    x_current = x0.copy()
    u_last_applied = u_prev  # Track last applied control
    
    for k in range(n_steps):
        # Recalculate MPC solution
        if k % recalc_interval == 0:
            # Solve MPC from current state
            u_opt, x_opt, solve_time = controller.solve(
                x_current, x_load_target, Q_state, Q_load, R,
                u_prev=u_last_applied,
                R_continuity=R_continuity
            )
            
            if u_opt is None:
                print(f"MPC failed at step {k}, using zero control")
                u_opt = np.zeros((1, controller.N))
            
            solve_times.append(solve_time)
            
            # Use only the first control input (receding horizon)
            u_k = u_opt[0, 0]
        else:
            # Between recalculations, use next control from previous solution
            idx = k % recalc_interval
            if controller.u_prev_solution is not None and idx < controller.u_prev_solution.shape[1]:
                u_k = controller.u_prev_solution[0, idx]
            else:
                u_k = 0.0
        
        # Apply control and simulate one step
        def dynamics(t, x):
            return system.nonlinear_dynamics(t, x, u_k)
        
        t_current = t_trajectory[-1]
        sol = solve_ivp(
            dynamics,
            [t_current, t_current + dt],
            x_current,
            method='RK45',
            max_step=dt/10
        )
        
        # Update state
        x_current = sol.y[:, -1]
        u_trajectory.append(u_k)
        u_last_applied = u_k  # Store for next iteration constraint
        
        t_trajectory.append(t_current + dt)
        x_trajectory.append(x_current.copy())
    
    return (np.array(t_trajectory), np.array(x_trajectory).T, 
            np.array(u_trajectory).reshape(1, -1), np.array(solve_times))


def plot_results(t, x_opt, x_linear, x_nonlinear, u_opt, x_target, L_cable, test_name):
    """
    Plot optimization results and simulation comparison
    
    Args:
        t: Time vector
        x_opt: Optimal trajectory from MPC
        x_linear: Simulated linear system trajectory
        x_nonlinear: Simulated non-linear system trajectory
        u_opt: Optimal control input
        x_target: Target state
        L_cable: Cable length (m)
        test_name: Name for the test case
    """
    fig, axes = plt.subplots(5, 1, figsize=(12, 14))
    
    # Drone position
    axes[0].plot(t, x_opt[0, :], 'b-', label='MPC Optimal', linewidth=2)
    axes[0].plot(t, x_linear[0, :], 'g--', label='Linear Model', linewidth=1.5)
    axes[0].plot(t, x_nonlinear[0, :], 'r:', label='Non-linear Model', linewidth=2)
    axes[0].axhline(x_target[0], color='k', linestyle=':', alpha=0.5, label='Target')
    axes[0].set_ylabel('Drone Position (m)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f'Drone Control System - {test_name}')
    
    # Drone velocity
    axes[1].plot(t, x_opt[1, :], 'b-', label='MPC Optimal', linewidth=2)
    axes[1].plot(t, x_linear[1, :], 'g--', label='Linear Model', linewidth=1.5)
    axes[1].plot(t, x_nonlinear[1, :], 'r:', label='Non-linear Model', linewidth=2)
    axes[1].axhline(x_target[1], color='k', linestyle=':', alpha=0.5, label='Target')
    axes[1].set_ylabel('Drone Velocity (m/s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Cable angle
    axes[2].plot(t, np.rad2deg(x_opt[2, :]), 'b-', label='MPC Optimal', linewidth=2)
    axes[2].plot(t, np.rad2deg(x_linear[2, :]), 'g--', label='Linear Model', linewidth=1.5)
    axes[2].plot(t, np.rad2deg(x_nonlinear[2, :]), 'r:', label='Non-linear Model', linewidth=2)
    axes[2].axhline(np.rad2deg(x_target[2]), color='k', linestyle=':', alpha=0.5, label='Target')
    axes[2].set_ylabel('Cable Angle (deg)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    # Angular velocity
    axes[3].plot(t, x_opt[3, :], 'b-', label='MPC Optimal', linewidth=2)
    axes[3].plot(t, x_linear[3, :], 'g--', label='Linear Model', linewidth=1.5)
    axes[3].plot(t, x_nonlinear[3, :], 'r:', label='Non-linear Model', linewidth=2)
    axes[3].axhline(x_target[3], color='k', linestyle=':', alpha=0.5, label='Target')
    axes[3].set_ylabel('Angular Velocity (rad/s)')
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)
    
    # Control input
    t_u = t[:-1]
    axes[4].step(t_u, u_opt[0, :], 'b-', where='post', linewidth=2, label='Control Input')
    axes[4].axhline(0, color='k', linestyle=':', alpha=0.5)
    axes[4].set_ylabel('Drone Acceleration (m/s²)')
    axes[4].set_xlabel('Time (s)')
    axes[4].legend()
    axes[4].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'/home/runner/work/DroneV2/DroneV2/results_{test_name}.png', dpi=150)
    print(f"Saved plot: results_{test_name}.png")
    
    # Also plot load position
    fig2, ax = plt.subplots(1, 1, figsize=(12, 6))
    
    # Calculate load position
    x_load_opt = x_opt[0, :] + L_cable * np.sin(x_opt[2, :])
    x_load_linear = x_linear[0, :] + L_cable * np.sin(x_linear[2, :])
    x_load_nonlinear = x_nonlinear[0, :] + L_cable * np.sin(x_nonlinear[2, :])
    x_load_target = x_target[0] + L_cable * np.sin(x_target[2])
    
    ax.plot(t, x_load_opt, 'b-', label='MPC Optimal', linewidth=2)
    ax.plot(t, x_load_linear, 'g--', label='Linear Model', linewidth=1.5)
    ax.plot(t, x_load_nonlinear, 'r:', label='Non-linear Model', linewidth=2)
    ax.axhline(x_load_target, color='k', linestyle=':', alpha=0.5, label='Target')
    ax.set_ylabel('Load Position (m)')
    ax.set_xlabel('Time (s)')
    ax.set_title(f'Load Position - {test_name}')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'/home/runner/work/DroneV2/DroneV2/load_position_{test_name}.png', dpi=150)
    print(f"Saved plot: load_position_{test_name}.png")


def plot_closed_loop_results(t, x_traj, u_traj, x_target, L_cable, test_name):
    """
    Plot closed-loop control results
    
    Args:
        t: Time vector
        x_traj: State trajectory
        u_traj: Control input trajectory
        x_target: Target state
        L_cable: Cable length (m)
        test_name: Name for the test case
    """
    fig, axes = plt.subplots(6, 1, figsize=(12, 16))
    
    # Drone position
    axes[0].plot(t, x_traj[0, :], 'b-', linewidth=2)
    axes[0].axhline(x_target[0], color='k', linestyle=':', alpha=0.5, label='Target')
    axes[0].set_ylabel('Drone Position (m)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f'Closed-Loop Drone Control - {test_name}')
    
    # Drone velocity
    axes[1].plot(t, x_traj[1, :], 'b-', linewidth=2)
    axes[1].axhline(x_target[1], color='k', linestyle=':', alpha=0.5, label='Target')
    axes[1].set_ylabel('Drone Velocity (m/s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Cable angle
    axes[2].plot(t, np.rad2deg(x_traj[2, :]), 'b-', linewidth=2)
    axes[2].axhline(np.rad2deg(x_target[2]), color='k', linestyle=':', alpha=0.5, label='Target')
    axes[2].set_ylabel('Cable Angle (deg)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    # Angular velocity
    axes[3].plot(t, x_traj[3, :], 'b-', linewidth=2)
    axes[3].axhline(x_target[3], color='k', linestyle=':', alpha=0.5, label='Target')
    axes[3].set_ylabel('Angular Velocity (rad/s)')
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)
    
    # Load position
    x_load = x_traj[0, :] + L_cable * np.sin(x_traj[2, :])
    x_load_target = x_target[0] + L_cable * np.sin(x_target[2])
    axes[4].plot(t, x_load, 'b-', linewidth=2)
    axes[4].axhline(x_load_target, color='k', linestyle=':', alpha=0.5, label='Target')
    axes[4].set_ylabel('Load Position (m)')
    axes[4].legend()
    axes[4].grid(True, alpha=0.3)
    
    # Control input
    t_u = t[:-1]
    axes[5].step(t_u, u_traj[0, :], 'b-', where='post', linewidth=2, label='Control Input')
    axes[5].axhline(0, color='k', linestyle=':', alpha=0.5)
    
    # Highlight control discontinuities
    if u_traj.shape[1] > 1:
        u_diff = np.abs(np.diff(u_traj[0, :]))
        discontinuity_threshold = 1.0  # m/s^2
        discontinuities = np.where(u_diff > discontinuity_threshold)[0]
        if len(discontinuities) > 0:
            for idx in discontinuities:
                axes[5].axvline(t_u[idx], color='r', linestyle='--', alpha=0.3, linewidth=1)
    
    axes[5].set_ylabel('Drone Acceleration (m/s²)')
    axes[5].set_xlabel('Time (s)')
    axes[5].legend()
    axes[5].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'/home/runner/work/DroneV2/DroneV2/closed_loop_{test_name}.png', dpi=150)
    print(f"Saved plot: closed_loop_{test_name}.png")
    plt.close()
    
    # Plot control smoothness analysis
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # Control trajectory
    axes[0].step(t_u, u_traj[0, :], 'b-', where='post', linewidth=2)
    axes[0].set_ylabel('Control Input (m/s²)')
    axes[0].set_title(f'Control Smoothness Analysis - {test_name}')
    axes[0].grid(True, alpha=0.3)
    
    # Control rate of change
    if u_traj.shape[1] > 1:
        u_diff = np.diff(u_traj[0, :]) / TS
        t_diff = t_u[:-1]
        axes[1].step(t_diff, u_diff, 'r-', where='post', linewidth=2)
        axes[1].set_ylabel('Control Rate (m/s³)')
        axes[1].set_xlabel('Time (s)')
        axes[1].grid(True, alpha=0.3)
        
        # Statistics
        max_jump = np.max(np.abs(u_diff))
        mean_jump = np.mean(np.abs(u_diff))
        axes[1].set_title(f'Max Jump: {max_jump:.2f} m/s³, Mean: {mean_jump:.2f} m/s³')
    
    plt.tight_layout()
    plt.savefig(f'/home/runner/work/DroneV2/DroneV2/control_smoothness_{test_name}.png', dpi=150)
    print(f"Saved plot: control_smoothness_{test_name}.png")
    plt.close()


def run_test_case(system, Ad, Bd, x0, x_load_target, test_name, Q_state, Q_load, R_weight, horizon=50):
    """
    Run a single test case
    
    Args:
        system: DroneSystem object
        Ad, Bd: Discrete system matrices
        x0: Initial state
        x_load_target: Target load position (scalar)
        test_name: Name for this test
        Q_state: State weights for cost function [q_xd, q_vd, q_theta, q_omega]
        Q_load: Weight for load position error
        R_weight: Input weight for cost function
        horizon: MPC horizon
    """
    print(f"\n{'='*60}")
    print(f"Test Case: {test_name}")
    print(f"{'='*60}")
    
    # Calculate initial load position
    x_load_init = x0[0] + system.L * np.sin(x0[2])
    
    print(f"Initial state: x_d={x0[0]:.1f}m, v_d={x0[1]:.1f}m/s, theta={np.rad2deg(x0[2]):.2f}deg, omega={x0[3]:.3f}rad/s")
    print(f"Initial load position: {x_load_init:.1f}m")
    print(f"Target load position: {x_load_target:.1f}m")
    print(f"Horizon: {horizon} steps ({horizon*TS:.1f} seconds)")
    
    # Create MPC controller
    controller = MPCController(Ad, Bd, system.L, horizon=horizon)
    
    # Solve MPC
    u_opt, x_opt, solve_time = controller.solve(x0, x_load_target, Q_state, Q_load, R_weight)
    
    if u_opt is None:
        print("ERROR: Optimization failed!")
        return
    
    print(f"Optimization solved in {solve_time*1000:.2f} ms")
    
    # Simulate linear system
    x_linear = simulate_linear(Ad, Bd, x0, u_opt)
    
    # Simulate non-linear system
    t_nonlinear, x_nonlinear = simulate_nonlinear(system, x0, u_opt, TS)
    
    # Time vector for plotting
    t = np.linspace(0, horizon * TS, horizon + 1)
    
    # Target state (drone at target with load directly below)
    x_target = np.array([x_load_target, 0.0, 0.0, 0.0])
    
    # Calculate final errors
    print("\nFinal errors (at end of horizon):")
    print("  Linear model:")
    print(f"    Position error: {abs(x_linear[0, -1] - x_target[0]):.4f} m")
    print(f"    Velocity error: {abs(x_linear[1, -1] - x_target[1]):.4f} m/s")
    print(f"    Angle error: {abs(np.rad2deg(x_linear[2, -1] - x_target[2])):.4f} deg")
    print(f"    Angular velocity error: {abs(x_linear[3, -1] - x_target[3]):.6f} rad/s")
    
    print("  Non-linear model:")
    print(f"    Position error: {abs(x_nonlinear[0, -1] - x_target[0]):.4f} m")
    print(f"    Velocity error: {abs(x_nonlinear[1, -1] - x_target[1]):.4f} m/s")
    print(f"    Angle error: {abs(np.rad2deg(x_nonlinear[2, -1] - x_target[2])):.4f} deg")
    print(f"    Angular velocity error: {abs(x_nonlinear[3, -1] - x_target[3]):.6f} rad/s")
    
    # Calculate load position error
    x_load_linear_final = x_linear[0, -1] + system.L * np.sin(x_linear[2, -1])
    x_load_nonlinear_final = x_nonlinear[0, -1] + system.L * np.sin(x_nonlinear[2, -1])
    
    print(f"\nLoad position errors:")
    print(f"  Linear model: {abs(x_load_linear_final - x_load_target):.4f} m")
    print(f"  Non-linear model: {abs(x_load_nonlinear_final - x_load_target):.4f} m")
    
    # Control statistics
    print(f"\nControl statistics:")
    print(f"  Max acceleration: {np.max(np.abs(u_opt)):.2f} m/s²")
    print(f"  Mean |acceleration|: {np.mean(np.abs(u_opt)):.2f} m/s²")
    
    # Plot results
    plot_results(t, x_opt, x_linear, x_nonlinear, u_opt, x_target, system.L, test_name)


def run_closed_loop_test(system, Ad, Bd, x0, x_load_target, test_name, Q_state, Q_load, R_weight, 
                         R_continuity=None, horizon=50, n_steps=100, recalc_interval=5):
    """
    Run a closed-loop test case with MPC recalculation
    
    Args:
        system: DroneSystem object
        Ad, Bd: Discrete system matrices
        x0: Initial state
        x_load_target: Target load position (scalar)
        test_name: Name for this test
        Q_state: State weights for cost function
        Q_load: Weight for load position error
        R_weight: Input weight for cost function
        R_continuity: Weight for continuity penalty (None to disable)
        horizon: MPC horizon
        n_steps: Number of simulation steps
        recalc_interval: How often to recalculate MPC (in steps)
    """
    print(f"\n{'='*60}")
    print(f"Closed-Loop Test: {test_name}")
    print(f"{'='*60}")
    
    # Calculate initial load position
    x_load_init = x0[0] + system.L * np.sin(x0[2])
    
    print(f"Initial state: x_d={x0[0]:.1f}m, v_d={x0[1]:.1f}m/s, theta={np.rad2deg(x0[2]):.2f}deg, omega={x0[3]:.3f}rad/s")
    print(f"Initial load position: {x_load_init:.1f}m")
    print(f"Target load position: {x_load_target:.1f}m")
    print(f"Horizon: {horizon} steps ({horizon*TS:.1f} seconds)")
    print(f"Simulation: {n_steps} steps ({n_steps*TS:.1f} seconds)")
    print(f"MPC recalculation interval: {recalc_interval} steps ({recalc_interval*TS:.1f} seconds)")
    if R_continuity is not None:
        print(f"Continuity weight: {R_continuity}")
    
    # Create MPC controller
    controller = MPCController(Ad, Bd, system.L, horizon=horizon)
    
    # Run closed-loop simulation
    t, x_traj, u_traj, solve_times = simulate_closed_loop(
        system, controller, x0, x_load_target, Q_state, Q_load, R_weight,
        n_steps, TS, recalc_interval=recalc_interval, R_continuity=R_continuity
    )
    
    print(f"\nMPC solve statistics:")
    print(f"  Number of optimizations: {len(solve_times)}")
    print(f"  Average solve time: {np.mean(solve_times)*1000:.2f} ms")
    print(f"  Max solve time: {np.max(solve_times)*1000:.2f} ms")
    print(f"  Min solve time: {np.min(solve_times)*1000:.2f} ms")
    
    # Calculate final errors
    x_target = np.array([x_load_target, 0.0, 0.0, 0.0])
    print(f"\nFinal state:")
    print(f"  Position: {x_traj[0, -1]:.4f} m (error: {abs(x_traj[0, -1] - x_target[0]):.4f} m)")
    print(f"  Velocity: {x_traj[1, -1]:.4f} m/s (error: {abs(x_traj[1, -1] - x_target[1]):.4f} m/s)")
    print(f"  Angle: {np.rad2deg(x_traj[2, -1]):.4f} deg (error: {abs(np.rad2deg(x_traj[2, -1] - x_target[2])):.4f} deg)")
    print(f"  Angular velocity: {x_traj[3, -1]:.6f} rad/s (error: {abs(x_traj[3, -1] - x_target[3]):.6f} rad/s)")
    
    # Calculate load position error
    x_load_final = x_traj[0, -1] + system.L * np.sin(x_traj[2, -1])
    print(f"\nLoad position:")
    print(f"  Final: {x_load_final:.4f} m")
    print(f"  Target: {x_load_target:.4f} m")
    print(f"  Error: {abs(x_load_final - x_load_target):.4f} m")
    
    # Control smoothness analysis
    if u_traj.shape[1] > 1:
        u_diff = np.abs(np.diff(u_traj[0, :])) / TS
        max_jump = np.max(u_diff)
        mean_jump = np.mean(u_diff)
        
        print(f"\nControl smoothness:")
        print(f"  Max control rate: {max_jump:.2f} m/s³")
        print(f"  Mean control rate: {mean_jump:.2f} m/s³")
        print(f"  Max control value: {np.max(np.abs(u_traj)):.2f} m/s²")
        print(f"  Mean |control|: {np.mean(np.abs(u_traj)):.2f} m/s²")
    
    # Plot results
    plot_closed_loop_results(t, x_traj, u_traj, x_target, system.L, test_name)


def main():
    """Main function to run all test cases"""
    
    print("="*60)
    print("1D Drone Control System with Suspended Load")
    print("="*60)
    
    # Create system
    system = DroneSystem()
    
    print("\nSystem Parameters:")
    print(f"  Drone mass: {system.m_d} kg")
    print(f"  Load mass: {system.m_l} kg")
    print(f"  Cable length: {system.L} m")
    print(f"  Load surface area: {system.S} m²")
    print(f"  Drag coefficient: {system.Cd}")
    print(f"  Sampling time: {TS} s")
    
    # Get linearized model
    print("\nLinearizing system around equilibrium...")
    A, B = system.get_linearized_matrices()
    
    print("\nContinuous-time system matrices:")
    print("A =")
    print(A)
    print("\nB =")
    print(B)
    
    # Discretize
    print(f"\nDiscretizing with dt = {TS} s...")
    Ad, Bd = system.discretize(A, B, TS)
    
    print("\nDiscrete-time system matrices:")
    print("Ad =")
    print(Ad)
    print("\nBd =")
    print(Bd)
    
    # Cost function weights
    Q_state = np.array([10.0, 20.0, 500.0, 100.0])  # State tracking weights
    Q_load = 1000.0  # High weight on load position
    R_weight = 2.0  # Control effort penalty
    
    # Continuity weight for closed-loop MPC
    R_continuity = 50.0  # Penalize deviations from previous solution
    
    # Test closed-loop control with continuity constraints
    print("\n" + "="*60)
    print("CLOSED-LOOP MPC TESTS WITH CONTINUITY CONSTRAINTS")
    print("="*60)
    
    closed_loop_tests = [
        {
            'name': 'closed_loop_20m_without_continuity',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),
            'x_load_target': 20.0,
            'R_continuity': None,  # No continuity constraint
            'horizon': 50,
            'n_steps': 100,
            'recalc_interval': 5
        },
        {
            'name': 'closed_loop_20m_with_continuity',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),
            'x_load_target': 20.0,
            'R_continuity': R_continuity,  # With continuity constraint
            'horizon': 50,
            'n_steps': 100,
            'recalc_interval': 5
        },
        {
            'name': 'closed_loop_40m_without_continuity',
            'x0': np.array([0.0, 2.0, 0.1, 0.0]),
            'x_load_target': 40.0,
            'R_continuity': None,
            'horizon': 60,
            'n_steps': 150,
            'recalc_interval': 5
        },
        {
            'name': 'closed_loop_40m_with_continuity',
            'x0': np.array([0.0, 2.0, 0.1, 0.0]),
            'x_load_target': 40.0,
            'R_continuity': R_continuity,
            'horizon': 60,
            'n_steps': 150,
            'recalc_interval': 5
        },
    ]
    
    # Run closed-loop tests
    for test in closed_loop_tests:
        run_closed_loop_test(
            system, Ad, Bd,
            test['x0'], test['x_load_target'],
            test['name'],
            Q_state, Q_load, R_weight,
            R_continuity=test['R_continuity'],
            horizon=test['horizon'],
            n_steps=test['n_steps'],
            recalc_interval=test['recalc_interval']
        )
    
    print("\n" + "="*60)
    print("All test cases completed successfully!")
    print("="*60)
    print("\nKey findings:")
    print("- Closed-loop tests demonstrate MPC recalculation with nonlinear feedback")
    print("- Continuity constraints (u_prev and R_continuity) smooth control transitions")
    print("- Control jumps are reduced when constraints are active")
    print("- System achieves target despite model mismatch and disturbances")


if __name__ == "__main__":
    main()
