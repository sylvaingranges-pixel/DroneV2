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
    
    def __init__(self, Ad, Bd, horizon=50):
        """
        Initialize MPC controller
        
        Args:
            Ad: Discrete state matrix
            Bd: Discrete input matrix
            horizon: Prediction horizon
        """
        self.Ad = Ad
        self.Bd = Bd
        self.N = horizon  # prediction horizon
        self.nx = Ad.shape[0]  # number of states
        self.nu = Bd.shape[1]  # number of inputs
        
        # Constraints
        self.u_max = 15.0  # Maximum acceleration (m/s^2)
        self.u_min = -15.0  # Minimum acceleration (m/s^2)
        
    def solve(self, x0, x_target, Q, R, Qf=None):
        """
        Solve MPC optimization problem
        
        Args:
            x0: Initial state
            x_target: Target state
            Q: State cost matrix
            R: Input cost matrix
            Qf: Terminal state cost matrix (default: Q)
            
        Returns:
            u_opt: Optimal control sequence
            x_opt: Optimal state trajectory
            solve_time: Time to solve optimization
        """
        if Qf is None:
            Qf = Q * 50  # Much higher terminal cost to ensure convergence
        
        # Define optimization variables
        x = cp.Variable((self.nx, self.N + 1))
        u = cp.Variable((self.nu, self.N))
        
        # Cost function
        cost = 0
        constraints = []
        
        # Initial condition
        constraints.append(x[:, 0] == x0)
        
        # Stage costs and dynamics constraints
        for k in range(self.N):
            # State deviation from target
            dx = x[:, k] - x_target
            cost += cp.quad_form(dx, Q)
            
            # Control effort
            cost += cp.quad_form(u[:, k], R)
            
            # Dynamics constraint
            constraints.append(x[:, k + 1] == self.Ad @ x[:, k] + self.Bd @ u[:, k])
            
            # Input constraints
            constraints.append(u[:, k] <= self.u_max)
            constraints.append(u[:, k] >= self.u_min)
        
        # Terminal cost
        dx_f = x[:, self.N] - x_target
        cost += cp.quad_form(dx_f, Qf)
        
        # Terminal constraint (soft - via high terminal cost)
        # For hard constraint, uncomment:
        # constraints.append(x[:, self.N] == x_target)
        
        # Solve optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        
        start_time = time.time()
        problem.solve(solver=cp.OSQP, verbose=False, max_iter=10000, eps_abs=1e-5, eps_rel=1e-5)
        solve_time = time.time() - start_time
        
        if problem.status not in ["optimal", "optimal_inaccurate"]:
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


def plot_results(t, x_opt, x_linear, x_nonlinear, u_opt, x_target, test_name):
    """
    Plot optimization results and simulation comparison
    
    Args:
        t: Time vector
        x_opt: Optimal trajectory from MPC
        x_linear: Simulated linear system trajectory
        x_nonlinear: Simulated non-linear system trajectory
        u_opt: Optimal control input
        x_target: Target state
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
    x_load_opt = x_opt[0, :] + L_CABLE * np.sin(x_opt[2, :])
    x_load_linear = x_linear[0, :] + L_CABLE * np.sin(x_linear[2, :])
    x_load_nonlinear = x_nonlinear[0, :] + L_CABLE * np.sin(x_nonlinear[2, :])
    x_load_target = x_target[0] + L_CABLE * np.sin(x_target[2])
    
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


def run_test_case(system, Ad, Bd, x0, x_target, test_name, Q_weights, R_weight, horizon=50):
    """
    Run a single test case
    
    Args:
        system: DroneSystem object
        Ad, Bd: Discrete system matrices
        x0: Initial state
        x_target: Target state
        test_name: Name for this test
        Q_weights: State weights for cost function
        R_weight: Input weight for cost function
        horizon: MPC horizon
    """
    print(f"\n{'='*60}")
    print(f"Test Case: {test_name}")
    print(f"{'='*60}")
    print(f"Initial state: x_d={x0[0]:.1f}m, v_d={x0[1]:.1f}m/s, theta={np.rad2deg(x0[2]):.2f}deg, omega={x0[3]:.3f}rad/s")
    print(f"Target state: x_d={x_target[0]:.1f}m, v_d={x_target[1]:.1f}m/s, theta={np.rad2deg(x_target[2]):.2f}deg, omega={x_target[3]:.3f}rad/s")
    print(f"Horizon: {horizon} steps ({horizon*TS:.1f} seconds)")
    
    # Setup cost matrices
    Q = np.diag(Q_weights)
    R = np.diag([R_weight])
    Qf = Q * 20  # Higher terminal cost to ensure convergence
    
    # Create MPC controller
    controller = MPCController(Ad, Bd, horizon=horizon)
    
    # Solve MPC
    u_opt, x_opt, solve_time = controller.solve(x0, x_target, Q, R, Qf)
    
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
    x_load_target = x_target[0] + L_CABLE * np.sin(x_target[2])
    x_load_linear_final = x_linear[0, -1] + L_CABLE * np.sin(x_linear[2, -1])
    x_load_nonlinear_final = x_nonlinear[0, -1] + L_CABLE * np.sin(x_nonlinear[2, -1])
    
    print(f"\nLoad position errors:")
    print(f"  Linear model: {abs(x_load_linear_final - x_load_target):.4f} m")
    print(f"  Non-linear model: {abs(x_load_nonlinear_final - x_load_target):.4f} m")
    
    # Control statistics
    print(f"\nControl statistics:")
    print(f"  Max acceleration: {np.max(np.abs(u_opt)):.2f} m/s²")
    print(f"  Mean |acceleration|: {np.mean(np.abs(u_opt)):.2f} m/s²")
    
    # Plot results
    plot_results(t, x_opt, x_linear, x_nonlinear, u_opt, x_target, test_name)


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
    # Higher weights mean tighter tracking
    # Position, velocity, angle, angular velocity
    # Important: Higher position weight helps avoid overshoot
    # Higher angle and angular velocity weights help dampen oscillations
    Q_weights = [200.0, 50.0, 500.0, 100.0]  # Penalize position, velocity, angle, angular velocity
    R_weight = 1.0  # Higher penalty on control effort to smooth trajectory and avoid overshoot
    
    # Define test cases with longer horizons to ensure the load settles
    test_cases = [
        {
            'name': 'test_20m_at_rest',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),  # Start at rest at origin
            'x_target': np.array([20.0, 0.0, 0.0, 0.0]),  # Target: 20m, at rest
            'horizon': 100  # Longer horizon to ensure settling
        },
        {
            'name': 'test_40m_at_rest',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),  # Start at rest at origin
            'x_target': np.array([40.0, 0.0, 0.0, 0.0]),  # Target: 40m, at rest
            'horizon': 150  # Longer horizon to ensure settling
        },
        {
            'name': 'test_80m_at_rest',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),  # Start at rest at origin
            'x_target': np.array([80.0, 0.0, 0.0, 0.0]),  # Target: 80m, at rest
            'horizon': 200  # Longer horizon to ensure settling
        },
        {
            'name': 'test_20m_with_velocity',
            'x0': np.array([0.0, 2.0, 0.1, 0.0]),  # Start with velocity and angle
            'x_target': np.array([20.0, 0.0, 0.0, 0.0]),  # Target: 20m, at rest
            'horizon': 100
        },
        {
            'name': 'test_40m_with_velocity',
            'x0': np.array([0.0, 3.0, 0.15, 0.05]),  # Start with velocity and angle
            'x_target': np.array([40.0, 0.0, 0.0, 0.0]),  # Target: 40m, at rest
            'horizon': 150
        },
        {
            'name': 'test_80m_with_velocity',
            'x0': np.array([0.0, 4.0, 0.2, 0.1]),  # Start with velocity and angle
            'x_target': np.array([80.0, 0.0, 0.0, 0.0]),  # Target: 80m, at rest
            'horizon': 200
        }
    ]
    
    # Run all test cases
    for test in test_cases:
        run_test_case(
            system, Ad, Bd,
            test['x0'], test['x_target'],
            test['name'],
            Q_weights, R_weight,
            horizon=test['horizon']
        )
    
    print("\n" + "="*60)
    print("All test cases completed successfully!")
    print("="*60)
    
    plt.show()


if __name__ == "__main__":
    main()
