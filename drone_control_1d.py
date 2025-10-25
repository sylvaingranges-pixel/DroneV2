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
        
    def solve(self, x0, x_load_target, Q_state, Q_load, R, Qf_state=None, Qf_load=None, 
              u_warmstart=None, x_warmstart=None, current_time=0.0):
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
            u_warmstart: Previous optimal control for warm start (optional)
            x_warmstart: Previous optimal state trajectory for warm start (optional)
            current_time: Current simulation time (for terminal constraint adaptation)
            
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
        
        # Warm start if previous solution provided
        if u_warmstart is not None and x_warmstart is not None:
            # Check if dimensions match, otherwise skip warm-start
            if u_warmstart.shape[1] == self.N and x_warmstart.shape[1] == self.N + 1:
                # Shift previous solution and use as initial guess
                x.value = x_warmstart
                u.value = u_warmstart
        
        # Cost function
        cost = 0
        constraints = []
        
        # Initial condition
        constraints.append(x[:, 0] == x0)
        
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


def simulate_closed_loop_mpc(system, controller, x0, x_load_target, Q_state, Q_load, R, 
                              total_horizon, replan_interval=30, dt=TS):
    """
    Simulate closed-loop MPC with receding horizon
    
    Args:
        system: DroneSystem object
        controller: MPCController object
        x0: Initial state
        x_load_target: Target load position
        Q_state: State cost weights
        Q_load: Load position cost weight
        R: Input cost weight
        total_horizon: Total simulation horizon in time steps
        replan_interval: How often to recompute MPC (in time steps)
        dt: Sampling time
        
    Returns:
        t_trajectory: Time vector
        x_trajectory: State trajectory
        u_trajectory: Control input trajectory
        solve_times: List of solve times at each replanning
    """
    # Storage for trajectory
    t_trajectory = [0.0]
    x_trajectory = [x0.copy()]
    u_trajectory = []
    solve_times = []
    
    # Current state
    x_current = x0.copy()
    t_current = 0.0
    
    # Previous solution for warm start
    u_prev = None
    x_prev = None
    
    # Simulate with replanning
    steps_taken = 0
    while steps_taken < total_horizon:
        # Determine horizon for this MPC solve
        # Remaining steps in simulation
        remaining_steps = total_horizon - steps_taken
        # MPC horizon should look ahead the full remaining time
        mpc_horizon = min(controller.N, remaining_steps)
        
        # Create temporary controller with current horizon if needed
        if mpc_horizon < controller.N:
            current_controller = MPCController(controller.Ad, controller.Bd, controller.L, horizon=mpc_horizon)
        else:
            current_controller = controller
        
        # Prepare warm start
        u_warmstart = None
        x_warmstart = None
        if u_prev is not None and x_prev is not None:
            # Shift previous solution: discard first replan_interval steps, 
            # append with zeros or last control
            if u_prev.shape[1] > replan_interval:
                # Shift and pad
                u_warmstart = np.hstack([
                    u_prev[:, replan_interval:],
                    np.tile(u_prev[:, -1:], (1, min(replan_interval, mpc_horizon - (u_prev.shape[1] - replan_interval))))
                ])
                x_warmstart = np.hstack([
                    x_prev[:, replan_interval:],
                    np.tile(x_prev[:, -1:], (1, min(replan_interval + 1, mpc_horizon + 1 - (x_prev.shape[1] - replan_interval))))
                ])
                # Adjust size to match current horizon
                if u_warmstart.shape[1] > mpc_horizon:
                    u_warmstart = u_warmstart[:, :mpc_horizon]
                if x_warmstart.shape[1] > mpc_horizon + 1:
                    x_warmstart = x_warmstart[:, :mpc_horizon + 1]
        
        # Solve MPC from current state
        # Current time is used to adapt terminal constraints in absolute time reference
        u_opt, x_opt, solve_time = current_controller.solve(
            x_current, x_load_target, Q_state, Q_load, R,
            u_warmstart=u_warmstart, x_warmstart=x_warmstart,
            current_time=t_current
        )
        
        if u_opt is None:
            print(f"Warning: MPC solve failed at t={t_current:.2f}s")
            # Use zero control if optimization fails
            u_opt = np.zeros((1, min(replan_interval, remaining_steps)))
        else:
            solve_times.append(solve_time)
            # Store for next warm start
            u_prev = u_opt
            x_prev = x_opt
        
        # Apply control for replan_interval steps or until end
        steps_to_apply = min(replan_interval, remaining_steps, u_opt.shape[1])
        
        for k in range(steps_to_apply):
            # Apply control
            u_k = u_opt[0, k]
            u_trajectory.append(u_k)
            
            # Simulate nonlinear system for one time step
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
            steps_taken += 1
            
            # Store trajectory
            t_trajectory.append(t_current)
            x_trajectory.append(x_current.copy())
    
    return (np.array(t_trajectory), 
            np.array(x_trajectory).T, 
            np.array(u_trajectory), 
            solve_times)


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


def plot_open_vs_closed_loop(t_open, x_open, u_open, t_closed, x_closed, u_closed, 
                               x_target, L_cable, test_name, solve_times_closed=None):
    """
    Plot comparison between open-loop and closed-loop MPC
    
    Args:
        t_open: Time vector for open-loop
        x_open: State trajectory for open-loop
        u_open: Control sequence for open-loop
        t_closed: Time vector for closed-loop
        x_closed: State trajectory for closed-loop
        u_closed: Control sequence for closed-loop
        x_target: Target state
        L_cable: Cable length (m)
        test_name: Name for the test case
        solve_times_closed: Solve times for closed-loop (optional)
    """
    fig, axes = plt.subplots(5, 1, figsize=(14, 16))
    
    # Drone position
    axes[0].plot(t_open, x_open[0, :], 'b-', label='Open-loop', linewidth=2, alpha=0.7)
    axes[0].plot(t_closed, x_closed[0, :], 'r-', label='Closed-loop', linewidth=2)
    axes[0].axhline(x_target[0], color='k', linestyle=':', alpha=0.5, label='Target')
    axes[0].set_ylabel('Drone Position (m)', fontsize=11)
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f'Open-loop vs Closed-loop MPC - {test_name}', fontsize=12)
    
    # Drone velocity
    axes[1].plot(t_open, x_open[1, :], 'b-', label='Open-loop', linewidth=2, alpha=0.7)
    axes[1].plot(t_closed, x_closed[1, :], 'r-', label='Closed-loop', linewidth=2)
    axes[1].axhline(x_target[1], color='k', linestyle=':', alpha=0.5, label='Target')
    axes[1].set_ylabel('Drone Velocity (m/s)', fontsize=11)
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.3)
    
    # Cable angle
    axes[2].plot(t_open, np.rad2deg(x_open[2, :]), 'b-', label='Open-loop', linewidth=2, alpha=0.7)
    axes[2].plot(t_closed, np.rad2deg(x_closed[2, :]), 'r-', label='Closed-loop', linewidth=2)
    axes[2].axhline(np.rad2deg(x_target[2]), color='k', linestyle=':', alpha=0.5, label='Target')
    axes[2].set_ylabel('Cable Angle (deg)', fontsize=11)
    axes[2].legend(fontsize=10)
    axes[2].grid(True, alpha=0.3)
    
    # Angular velocity
    axes[3].plot(t_open, x_open[3, :], 'b-', label='Open-loop', linewidth=2, alpha=0.7)
    axes[3].plot(t_closed, x_closed[3, :], 'r-', label='Closed-loop', linewidth=2)
    axes[3].axhline(x_target[3], color='k', linestyle=':', alpha=0.5, label='Target')
    axes[3].set_ylabel('Angular Velocity (rad/s)', fontsize=11)
    axes[3].legend(fontsize=10)
    axes[3].grid(True, alpha=0.3)
    
    # Control input
    t_u_open = t_open[:-1]
    t_u_closed = t_closed[:-1]
    axes[4].step(t_u_open, u_open[0, :], 'b-', where='post', linewidth=2, 
                 label='Open-loop', alpha=0.7)
    axes[4].step(t_u_closed, u_closed, 'r-', where='post', linewidth=2, 
                 label='Closed-loop')
    axes[4].axhline(0, color='k', linestyle=':', alpha=0.5)
    axes[4].set_ylabel('Drone Acceleration (m/s²)', fontsize=11)
    axes[4].set_xlabel('Time (s)', fontsize=11)
    axes[4].legend(fontsize=10)
    axes[4].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'/home/runner/work/DroneV2/DroneV2/comparison_open_closed_{test_name}.png', dpi=150)
    print(f"Saved plot: comparison_open_closed_{test_name}.png")
    
    # Plot load position comparison
    fig2, axes2 = plt.subplots(2, 1, figsize=(14, 10))
    
    # Calculate load positions
    x_load_open = x_open[0, :] + L_cable * np.sin(x_open[2, :])
    x_load_closed = x_closed[0, :] + L_cable * np.sin(x_closed[2, :])
    x_load_target = x_target[0] + L_cable * np.sin(x_target[2])
    
    # Load position
    axes2[0].plot(t_open, x_load_open, 'b-', label='Open-loop', linewidth=2, alpha=0.7)
    axes2[0].plot(t_closed, x_load_closed, 'r-', label='Closed-loop', linewidth=2)
    axes2[0].axhline(x_load_target, color='k', linestyle=':', alpha=0.5, label='Target')
    axes2[0].set_ylabel('Load Position (m)', fontsize=11)
    axes2[0].set_title(f'Load Position Comparison - {test_name}', fontsize=12)
    axes2[0].legend(fontsize=10)
    axes2[0].grid(True, alpha=0.3)
    
    # Load position error
    error_open = np.abs(x_load_open - x_load_target)
    error_closed = np.abs(x_load_closed - x_load_target)
    axes2[1].semilogy(t_open, error_open, 'b-', label='Open-loop', linewidth=2, alpha=0.7)
    axes2[1].semilogy(t_closed, error_closed, 'r-', label='Closed-loop', linewidth=2)
    axes2[1].set_ylabel('Load Position Error (m, log scale)', fontsize=11)
    axes2[1].set_xlabel('Time (s)', fontsize=11)
    axes2[1].legend(fontsize=10)
    axes2[1].grid(True, alpha=0.3)
    
    # Add text box with statistics
    final_error_open = error_open[-1]
    final_error_closed = error_closed[-1]
    mean_error_open = np.mean(error_open)
    mean_error_closed = np.mean(error_closed)
    max_error_open = np.max(error_open)
    max_error_closed = np.max(error_closed)
    
    stats_text = f'Final Error:\n  Open: {final_error_open:.4f}m\n  Closed: {final_error_closed:.4f}m\n\n'
    stats_text += f'Mean Error:\n  Open: {mean_error_open:.4f}m\n  Closed: {mean_error_closed:.4f}m\n\n'
    stats_text += f'Max Error:\n  Open: {max_error_open:.4f}m\n  Closed: {max_error_closed:.4f}m'
    
    if solve_times_closed:
        avg_solve_time = np.mean(solve_times_closed) * 1000
        stats_text += f'\n\nAvg MPC Solve Time:\n  {avg_solve_time:.1f}ms'
    
    axes2[1].text(0.02, 0.98, stats_text, transform=axes2[1].transAxes,
                  fontsize=9, verticalalignment='top',
                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig(f'/home/runner/work/DroneV2/DroneV2/load_comparison_{test_name}.png', dpi=150)
    print(f"Saved plot: load_comparison_{test_name}.png")
    
    plt.close('all')


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


def run_comparison_test(system, Ad, Bd, x0, x_load_target, test_name, 
                        Q_state, Q_load, R_weight, horizon=50, replan_interval=30):
    """
    Run comparison between open-loop and closed-loop MPC
    
    Args:
        system: DroneSystem object
        Ad, Bd: Discrete system matrices
        x0: Initial state
        x_load_target: Target load position (scalar)
        test_name: Name for this test
        Q_state: State weights for cost function
        Q_load: Weight for load position error
        R_weight: Input weight for cost function
        horizon: MPC horizon
        replan_interval: How often to recompute MPC in closed-loop (time steps)
    """
    print(f"\n{'='*70}")
    print(f"COMPARISON TEST: Open-loop vs Closed-loop - {test_name}")
    print(f"{'='*70}")
    
    x_load_init = x0[0] + system.L * np.sin(x0[2])
    
    print(f"Initial state: x_d={x0[0]:.1f}m, v_d={x0[1]:.1f}m/s, "
          f"theta={np.rad2deg(x0[2]):.2f}deg, omega={x0[3]:.3f}rad/s")
    print(f"Initial load position: {x_load_init:.1f}m")
    print(f"Target load position: {x_load_target:.1f}m")
    print(f"Horizon: {horizon} steps ({horizon*TS:.1f} seconds)")
    print(f"Replan interval: {replan_interval} steps ({replan_interval*TS:.1f} seconds)")
    
    # Create MPC controller
    controller = MPCController(Ad, Bd, system.L, horizon=horizon)
    
    # ========== OPEN-LOOP MPC ==========
    print(f"\n{'='*70}")
    print("OPEN-LOOP MPC (single optimization, full horizon)")
    print(f"{'='*70}")
    
    u_open, x_open_opt, solve_time_open = controller.solve(
        x0, x_load_target, Q_state, Q_load, R_weight
    )
    
    if u_open is None:
        print("ERROR: Open-loop optimization failed!")
        return
    
    print(f"Optimization solved in {solve_time_open*1000:.2f} ms")
    
    # Simulate open-loop with nonlinear model
    t_open, x_open = simulate_nonlinear(system, x0, u_open, TS)
    
    # Calculate final error
    x_load_open_final = x_open[0, -1] + system.L * np.sin(x_open[2, -1])
    error_open = abs(x_load_open_final - x_load_target)
    print(f"Final load position error: {error_open:.4f} m")
    
    # ========== CLOSED-LOOP MPC ==========
    print(f"\n{'='*70}")
    print(f"CLOSED-LOOP MPC (replanning every {replan_interval*TS:.1f}s)")
    print(f"{'='*70}")
    
    t_closed, x_closed, u_closed, solve_times_closed = simulate_closed_loop_mpc(
        system, controller, x0, x_load_target, Q_state, Q_load, R_weight,
        total_horizon=horizon, replan_interval=replan_interval, dt=TS
    )
    
    print(f"Number of MPC solves: {len(solve_times_closed)}")
    print(f"Average solve time: {np.mean(solve_times_closed)*1000:.2f} ms")
    print(f"Total computation time: {np.sum(solve_times_closed):.2f} s")
    
    # Calculate final error
    x_load_closed_final = x_closed[0, -1] + system.L * np.sin(x_closed[2, -1])
    error_closed = abs(x_load_closed_final - x_load_target)
    print(f"Final load position error: {error_closed:.4f} m")
    
    # ========== COMPARISON ==========
    print(f"\n{'='*70}")
    print("PERFORMANCE COMPARISON")
    print(f"{'='*70}")
    
    # Calculate statistics
    x_load_open_traj = x_open[0, :] + system.L * np.sin(x_open[2, :])
    x_load_closed_traj = x_closed[0, :] + system.L * np.sin(x_closed[2, :])
    
    error_open_traj = np.abs(x_load_open_traj - x_load_target)
    error_closed_traj = np.abs(x_load_closed_traj - x_load_target)
    
    print(f"\nLoad Position Tracking:")
    print(f"  Open-loop:")
    print(f"    Final error:   {error_open:.6f} m")
    print(f"    Mean error:    {np.mean(error_open_traj):.6f} m")
    print(f"    Max error:     {np.max(error_open_traj):.6f} m")
    print(f"    RMS error:     {np.sqrt(np.mean(error_open_traj**2)):.6f} m")
    
    print(f"  Closed-loop:")
    print(f"    Final error:   {error_closed:.6f} m")
    print(f"    Mean error:    {np.mean(error_closed_traj):.6f} m")
    print(f"    Max error:     {np.max(error_closed_traj):.6f} m")
    print(f"    RMS error:     {np.sqrt(np.mean(error_closed_traj**2)):.6f} m")
    
    improvement = (error_open - error_closed) / error_open * 100 if error_open > 0 else 0
    print(f"\n  Improvement (final error): {improvement:.1f}%")
    
    # Control effort comparison
    control_effort_open = np.sum(u_open[0, :]**2) * TS
    control_effort_closed = np.sum(u_closed**2) * TS
    
    print(f"\nControl Effort (integral of u²):")
    print(f"  Open-loop:  {control_effort_open:.2f}")
    print(f"  Closed-loop: {control_effort_closed:.2f}")
    
    # Max angle comparison
    max_angle_open = np.max(np.abs(x_open[2, :]))
    max_angle_closed = np.max(np.abs(x_closed[2, :]))
    
    print(f"\nMaximum Cable Angle:")
    print(f"  Open-loop:  {np.rad2deg(max_angle_open):.2f} deg")
    print(f"  Closed-loop: {np.rad2deg(max_angle_closed):.2f} deg")
    
    # Plot comparison
    x_target = np.array([x_load_target, 0.0, 0.0, 0.0])
    plot_open_vs_closed_loop(t_open, x_open, u_open, t_closed, x_closed, u_closed,
                              x_target, system.L, test_name, solve_times_closed)


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
    # Q_state: [drone position, drone velocity, cable angle, angular velocity]
    # Higher angle and angular velocity weights help dampen oscillations
    # Lower drone position weight since we focus on load position
    Q_state = np.array([10.0, 20.0, 500.0, 100.0])  # State tracking weights
    Q_load = 1000.0  # High weight on load position - this is the primary objective
    R_weight = 2.0  # Control effort penalty to smooth trajectory and avoid overshoot
    
    # Replan interval: every 3 seconds = 30 steps at 0.1s sampling
    replan_interval = 30
    
    # Define test cases for comparison
    comparison_test_cases = [
        {
            'name': 'comparison_20m_at_rest',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),  # Start at rest at origin
            'x_load_target': 20.0,  # Target load position: 20m
            'horizon': 100  # 10 seconds horizon
        },
        {
            'name': 'comparison_40m_at_rest',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),  # Start at rest at origin
            'x_load_target': 40.0,  # Target load position: 40m
            'horizon': 150  # 15 seconds horizon
        },
        {
            'name': 'comparison_20m_with_velocity',
            'x0': np.array([0.0, 2.0, 0.1, 0.0]),  # Start with velocity and angle
            'x_load_target': 20.0,  # Target load position: 20m
            'horizon': 100
        },
    ]
    
    # Run comparison tests (open-loop vs closed-loop)
    print("\n" + "="*70)
    print("RUNNING COMPARISON TESTS: OPEN-LOOP vs CLOSED-LOOP MPC")
    print("="*70)
    
    for test in comparison_test_cases:
        run_comparison_test(
            system, Ad, Bd,
            test['x0'], test['x_load_target'],
            test['name'],
            Q_state, Q_load, R_weight,
            horizon=test['horizon'],
            replan_interval=replan_interval
        )
    
    print("\n" + "="*70)
    print("All comparison tests completed successfully!")
    print("="*70)
    print("\nGenerated plots:")
    print("  - comparison_open_closed_*.png: State and control comparisons")
    print("  - load_comparison_*.png: Load position tracking and error analysis")
    print("="*60)
    
    plt.show()


if __name__ == "__main__":
    main()
