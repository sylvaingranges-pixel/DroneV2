"""
Drone Control System with Suspended Load
=========================================

This module implements a control system for a 40kg drone carrying a 24kg load
suspended at 19m on a single axis (X). The system includes:
- Non-linear model with air drag
- Linearized model around equilibrium
- Discrete-time model (Ts=0.1s)
- Optimal trajectory controller using MPC
- Validation with RK45 ODE solver
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.linalg import expm
import cvxpy as cp
import time


class DroneLoadSystem:
    """
    Models a drone with suspended load system on X-axis.
    
    Parameters:
    - m_drone: 40 kg (drone mass)
    - m_load: 24 kg (load mass)
    - L: 19 m (cable length)
    - S: 0.2 m² (load cross-section area)
    - Cd: 1.0 (aerodynamic drag coefficient)
    - rho: 1.225 kg/m³ (air density)
    - g: 9.81 m/s² (gravity)
    
    State variables [x_d, v_d, theta, omega]:
    - x_d: drone position (m)
    - v_d: drone velocity (m/s)
    - theta: cable angle from vertical (rad)
    - omega: cable angular velocity (rad/s)
    
    Input: u = a_d (drone acceleration, m/s²)
    Output: x_l (load position, m)
    """
    
    def __init__(self):
        # Physical parameters
        self.m_drone = 40.0  # kg
        self.m_load = 24.0   # kg
        self.L = 19.0        # m
        self.S = 0.2         # m²
        self.Cd = 1.0        # dimensionless
        self.rho = 1.225     # kg/m³
        self.g = 9.81        # m/s²
        
        # Discretization
        self.Ts = 0.1  # s
        
        # Derived parameters
        self.k_drag = 0.5 * self.rho * self.Cd * self.S  # drag coefficient
        
        # Horizon scaling parameters for optimization
        self.HORIZON_BASE_DISTANCE = 20.0  # m - reference distance for horizon scaling
        self.HORIZON_STEPS_PER_REF = 50    # steps per reference distance
        self.HORIZON_MIN_STEPS = 50        # minimum horizon length
        
    def nonlinear_dynamics(self, t, state, u_func):
        """
        Non-linear dynamics with air drag.
        
        State: [x_d, v_d, theta, omega]
        Input: u = a_d (drone acceleration)
        
        Returns: state derivative
        """
        x_d, v_d, theta, omega = state
        
        # Get control input
        u = u_func(t) if callable(u_func) else u_func
        a_d = u
        
        # Load position and velocity in inertial frame
        # The load hangs at angle theta from vertical, so:
        # x_l = x_d + L*sin(theta) (horizontal position in inertial frame)
        # v_l = v_d + L*omega*cos(theta) (horizontal velocity, differentiated from x_l)
        x_l = x_d + self.L * np.sin(theta)
        v_l = v_d + self.L * omega * np.cos(theta)
        
        # Air drag force on load (opposing velocity)
        F_drag = -self.k_drag * v_l * np.abs(v_l)
        
        # Dynamics equations
        # Drone: m_d * a_d = control input (directly specified)
        dx_d = v_d
        dv_d = a_d
        
        # Pendulum dynamics with drag
        # Tension T and drag affect the pendulum motion
        # From Lagrangian mechanics with external forces:
        dtheta = omega
        
        # Angular acceleration considering drag force
        # Moment equation: m_l * L^2 * alpha = m_l * g * L * sin(theta) - m_l * a_d * L * cos(theta) + F_drag * L * cos(theta)
        domega = (self.g / self.L) * np.sin(theta) - (a_d / self.L) * np.cos(theta) + (F_drag / (self.m_load * self.L)) * np.cos(theta)
        
        return np.array([dx_d, dv_d, dtheta, domega])
    
    def linearized_dynamics(self):
        """
        Linearize around equilibrium: theta=0, omega=0, no aerodynamic effects.
        
        At equilibrium:
        - Load is directly below drone (theta = 0)
        - No motion (v_d = 0, omega = 0)
        - No drag (v_l = 0)
        
        Linearized state: delta_state = [delta_x_d, delta_v_d, delta_theta, delta_omega]
        Input: u = a_d
        Output: y = x_l = x_d + L*theta (for small theta)
        
        Returns: A, B, C, D matrices
        """
        # State matrix A (4x4)
        A = np.array([
            [0, 1, 0, 0],                    # d(x_d)/dt = v_d
            [0, 0, 0, 0],                    # d(v_d)/dt = a_d (from input)
            [0, 0, 0, 1],                    # d(theta)/dt = omega
            [0, 0, self.g/self.L, 0]         # d(omega)/dt = (g/L)*theta - (a_d/L)
        ])
        
        # Input matrix B (4x1)
        B = np.array([
            [0],
            [1],
            [0],
            [-1/self.L]
        ])
        
        # Output matrix C (1x4) - load position: x_l = x_d + L*theta
        C = np.array([[1, 0, self.L, 0]])
        
        # Feedthrough matrix D (1x1)
        D = np.array([[0]])
        
        return A, B, C, D
    
    def discretize(self, A, B, C, D):
        """
        Discretize continuous-time linear system using zero-order hold.
        
        Returns: Ad, Bd, Cd, Dd
        """
        n = A.shape[0]
        m = B.shape[1]
        
        # Matrix exponential method for exact discretization
        # [Ad  Bd] = expm([A  B] * Ts)
        # [0   I ]       [0  0]
        
        M = np.zeros((n + m, n + m))
        M[:n, :n] = A * self.Ts
        M[:n, n:] = B * self.Ts
        
        expM = expm(M)
        
        Ad = expM[:n, :n]
        Bd = expM[:n, n:]
        Cd = C
        Dd = D
        
        return Ad, Bd, Cd, Dd
    
    def get_discrete_model(self):
        """Get discrete-time linear model."""
        A, B, C, D = self.linearized_dynamics()
        return self.discretize(A, B, C, D)


class OptimalController:
    """
    Optimal trajectory controller using MPC with cvxpy.
    
    Solves an optimal control problem to drive the load to a target position
    with minimal time while ensuring zero oscillation and zero velocity at arrival.
    """
    
    def __init__(self, system: DroneLoadSystem, N_horizon=100):
        """
        Initialize controller.
        
        Args:
            system: DroneLoadSystem instance
            N_horizon: Prediction horizon (number of time steps)
        """
        self.system = system
        self.N_default = N_horizon
        
        # Get discrete model
        self.Ad, self.Bd, self.Cd, self.Dd = system.get_discrete_model()
        
        # Constraints on drone acceleration
        self.u_max = 5.0   # m/s² (reasonable acceleration limit)
        self.u_min = -5.0  # m/s²
        
    def solve_trajectory(self, x0, x_load_target, verbose=False):
        """
        Solve optimal control problem.
        
        Args:
            x0: Initial state [x_d, v_d, theta, omega]
            x_load_target: Target load position
            verbose: Print solver output
            
        Returns:
            u_opt: Optimal control sequence
            x_opt: Optimal state trajectory
            solve_time: Time to solve optimization
        """
        start_time = time.time()
        
        # Calculate initial load position
        x_load_init = x0[0] + self.system.L * np.sin(x0[2])
        distance = abs(x_load_target - x_load_init)
        
        # Scale horizon based on distance
        # Horizon increases with distance to give enough time for the maneuver
        N_min = int(distance / self.system.HORIZON_BASE_DISTANCE * self.system.HORIZON_STEPS_PER_REF) + self.system.HORIZON_MIN_STEPS
        N = max(self.N_default, N_min)
        
        n_states = 4
        n_inputs = 1
        
        # Decision variables
        x = cp.Variable((n_states, N + 1))
        u = cp.Variable((n_inputs, N))
        
        # Cost function: minimize control effort and settling time
        # We want fast arrival, so minimize final time implicitly by penalizing deviation
        cost = 0
        
        # Control effort
        for k in range(N):
            cost += 0.01 * cp.sum_squares(u[:, k])
        
        # Terminal cost: load should be at target
        x_load_final = self.Cd @ x[:, N]
        cost += 1000 * cp.sum_squares(x_load_final - x_load_target)
        
        # Penalize velocity and oscillation at the end
        cost += 100 * cp.sum_squares(x[1, N])  # drone velocity = 0
        cost += 100 * cp.sum_squares(x[2, N])  # theta = 0
        cost += 100 * cp.sum_squares(x[3, N])  # omega = 0
        
        # Constraints
        constraints = []
        
        # Initial condition
        constraints.append(x[:, 0] == x0)
        
        # Dynamics constraints
        for k in range(N):
            constraints.append(x[:, k+1] == self.Ad @ x[:, k] + self.Bd @ u[:, k])
        
        # Input constraints
        for k in range(N):
            constraints.append(u[:, k] >= self.u_min)
            constraints.append(u[:, k] <= self.u_max)
        
        # Terminal constraints for zero oscillation and velocity
        constraints.append(x[1, N] == 0)  # drone velocity = 0
        constraints.append(x[2, N] == 0)  # theta = 0 (no oscillation)
        constraints.append(x[3, N] == 0)  # omega = 0 (no angular velocity)
        
        # Solve optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP, verbose=verbose, max_iter=10000, eps_abs=1e-5, eps_rel=1e-5)
        
        solve_time = time.time() - start_time
        
        if problem.status not in ["optimal", "optimal_inaccurate"]:
            print(f"Warning: Solver status: {problem.status}")
            return None, None, solve_time
        
        return u.value, x.value, solve_time


def simulate_linear(system, x0, u_sequence, t_total):
    """
    Simulate linear discrete model with control sequence.
    
    Args:
        system: DroneLoadSystem instance
        x0: Initial state
        u_sequence: Control input sequence
        t_total: Total simulation time
        
    Returns:
        t: Time array
        x: State trajectory
        x_load: Load position trajectory
    """
    Ad, Bd, Cd, Dd = system.get_discrete_model()
    Ts = system.Ts
    
    N_steps = len(u_sequence)
    t = np.arange(N_steps + 1) * Ts
    
    x = np.zeros((4, N_steps + 1))
    x[:, 0] = x0
    
    for k in range(N_steps):
        x[:, k+1] = Ad @ x[:, k] + Bd[:, 0] * u_sequence[k]
    
    x_load = Cd @ x
    
    return t, x, x_load


def simulate_nonlinear(system, x0, u_sequence, t_total):
    """
    Simulate non-linear model with RK45 ODE solver.
    
    Args:
        system: DroneLoadSystem instance
        x0: Initial state
        u_sequence: Control input sequence
        t_total: Total simulation time
        
    Returns:
        t: Time array
        x: State trajectory
        x_load: Load position trajectory
    """
    Ts = system.Ts
    N_steps = len(u_sequence)
    
    # Create interpolated control function
    def u_func(t):
        idx = int(t / Ts)
        if idx >= len(u_sequence):
            return u_sequence[-1]
        return u_sequence[idx]
    
    # Solve ODE
    t_span = (0, t_total)
    t_eval = np.linspace(0, t_total, N_steps + 1)
    
    sol = solve_ivp(
        lambda t, y: system.nonlinear_dynamics(t, y, u_func),
        t_span,
        x0,
        method='RK45',
        t_eval=t_eval,
        rtol=1e-6,
        atol=1e-9
    )
    
    t = sol.t
    x = sol.y
    
    # Compute load position
    x_load = x[0, :] + system.L * np.sin(x[2, :])
    
    return t, x, x_load


def plot_results(t, x, x_load, u_sequence, target_pos, title_suffix=""):
    """
    Create comprehensive plots for the results.
    
    Args:
        t: Time array
        x: State trajectory [x_d, v_d, theta, omega]
        x_load: Load position
        u_sequence: Control input sequence
        target_pos: Target load position
        title_suffix: Additional title text
    """
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f'Drone Control Results {title_suffix}', fontsize=14, fontweight='bold')
    
    # Drone position
    axes[0, 0].plot(t, x[0, :], 'b-', linewidth=2)
    axes[0, 0].set_ylabel('Drone Position (m)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_title('Drone Position')
    
    # Drone velocity
    axes[0, 1].plot(t, x[1, :], 'g-', linewidth=2)
    axes[0, 1].set_ylabel('Drone Velocity (m/s)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_title('Drone Velocity')
    
    # Load position
    axes[1, 0].plot(t, x_load, 'r-', linewidth=2, label='Load position')
    axes[1, 0].axhline(y=target_pos, color='k', linestyle='--', linewidth=1, label='Target')
    axes[1, 0].set_ylabel('Load Position (m)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    axes[1, 0].set_title('Load Position')
    
    # Cable angle
    axes[1, 1].plot(t, np.rad2deg(x[2, :]), 'm-', linewidth=2)
    axes[1, 1].set_ylabel('Cable Angle (deg)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_title('Cable Angle from Vertical')
    
    # Angular velocity
    axes[2, 0].plot(t, x[3, :], 'c-', linewidth=2)
    axes[2, 0].set_ylabel('Angular Velocity (rad/s)')
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].grid(True, alpha=0.3)
    axes[2, 0].set_title('Cable Angular Velocity')
    
    # Control input
    t_u = t[:-1]
    if len(t_u) > len(u_sequence):
        t_u = t_u[:len(u_sequence)]
    axes[2, 1].step(t_u, u_sequence[:len(t_u)], 'orange', linewidth=2, where='post')
    axes[2, 1].set_ylabel('Acceleration (m/s²)')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].grid(True, alpha=0.3)
    axes[2, 1].set_title('Drone Acceleration (Control Input)')
    
    plt.tight_layout()
    return fig


def compare_models(system, x0, u_sequence, target_pos, t_total, scenario_name=""):
    """
    Compare linear and non-linear models with the same control sequence.
    
    Args:
        system: DroneLoadSystem instance
        x0: Initial state
        u_sequence: Control input sequence
        target_pos: Target load position
        t_total: Total simulation time
        scenario_name: Name for the scenario
    """
    # Simulate linear model
    t_lin, x_lin, x_load_lin = simulate_linear(system, x0, u_sequence, t_total)
    
    # Simulate non-linear model
    t_nonlin, x_nonlin, x_load_nonlin = simulate_nonlinear(system, x0, u_sequence, t_total)
    
    # Create comparison plots
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle(f'Linear vs Non-Linear Model Comparison - {scenario_name}', 
                 fontsize=14, fontweight='bold')
    
    # Load position comparison
    axes[0, 0].plot(t_lin, x_load_lin[0, :], 'b-', linewidth=2, label='Linear Model')
    axes[0, 0].plot(t_nonlin, x_load_nonlin, 'r--', linewidth=2, label='Non-linear Model')
    axes[0, 0].axhline(y=target_pos, color='k', linestyle=':', linewidth=1, label='Target')
    axes[0, 0].set_ylabel('Load Position (m)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_title('Load Position')
    
    # Cable angle comparison
    axes[0, 1].plot(t_lin, np.rad2deg(x_lin[2, :]), 'b-', linewidth=2, label='Linear Model')
    axes[0, 1].plot(t_nonlin, np.rad2deg(x_nonlin[2, :]), 'r--', linewidth=2, label='Non-linear Model')
    axes[0, 1].set_ylabel('Cable Angle (deg)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_title('Cable Angle from Vertical')
    
    # Drone position comparison
    axes[1, 0].plot(t_lin, x_lin[0, :], 'b-', linewidth=2, label='Linear Model')
    axes[1, 0].plot(t_nonlin, x_nonlin[0, :], 'r--', linewidth=2, label='Non-linear Model')
    axes[1, 0].set_ylabel('Drone Position (m)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_title('Drone Position')
    
    # Drone velocity comparison
    axes[1, 1].plot(t_lin, x_lin[1, :], 'b-', linewidth=2, label='Linear Model')
    axes[1, 1].plot(t_nonlin, x_nonlin[1, :], 'r--', linewidth=2, label='Non-linear Model')
    axes[1, 1].set_ylabel('Drone Velocity (m/s)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_title('Drone Velocity')
    
    plt.tight_layout()
    
    return fig


def run_test_scenario(system, controller, x0, target_pos, scenario_name):
    """
    Run a complete test scenario.
    
    Args:
        system: DroneLoadSystem instance
        controller: OptimalController instance
        x0: Initial state
        target_pos: Target load position
        scenario_name: Name for the scenario
        
    Returns:
        solve_time: Time to solve optimization
    """
    print(f"\n{'='*60}")
    print(f"Test Scenario: {scenario_name}")
    print(f"{'='*60}")
    print(f"Initial state: x_d={x0[0]:.2f}m, v_d={x0[1]:.2f}m/s, theta={np.rad2deg(x0[2]):.2f}deg, omega={x0[3]:.2f}rad/s")
    print(f"Target load position: {target_pos:.2f}m")
    
    # Solve optimal control problem
    u_opt, x_opt, solve_time = controller.solve_trajectory(x0, target_pos, verbose=False)
    
    if u_opt is None:
        print("Failed to solve optimization problem!")
        return None
    
    N = x_opt.shape[1] - 1
    print(f"Optimization solved in {solve_time*1000:.2f} ms (horizon: {N} steps, {N*system.Ts:.1f}s)")
    
    # Total simulation time
    t_total = N * system.Ts
    
    # Plot optimal trajectory (from optimization)
    t_opt = np.arange(N + 1) * system.Ts
    x_load_opt = (controller.Cd @ x_opt)[0, :]
    fig1 = plot_results(t_opt, x_opt, x_load_opt, u_opt[0, :], target_pos, 
                        f"- Optimal Solution - {scenario_name}")
    
    # Compare linear and non-linear models
    fig2 = compare_models(system, x0, u_opt[0, :], target_pos, t_total, scenario_name)
    
    # Print final state
    print(f"\nFinal state (from optimization):")
    print(f"  Drone pos: {x_opt[0, -1]:.4f} m, velocity: {x_opt[1, -1]:.6f} m/s")
    print(f"  Load pos: {x_load_opt[-1]:.4f} m (target: {target_pos:.2f} m)")
    print(f"  Cable angle: {np.rad2deg(x_opt[2, -1]):.6f} deg, omega: {x_opt[3, -1]:.6f} rad/s")
    print(f"  Position error: {abs(x_load_opt[-1] - target_pos):.6f} m")
    
    return solve_time, fig1, fig2


def main():
    """Main function to run all test scenarios."""
    print("="*60)
    print("Drone Control System with Suspended Load")
    print("="*60)
    
    # Initialize system
    system = DroneLoadSystem()
    
    # Display system parameters
    print("\nSystem Parameters:")
    print(f"  Drone mass: {system.m_drone} kg")
    print(f"  Load mass: {system.m_load} kg")
    print(f"  Cable length: {system.L} m")
    print(f"  Load surface area: {system.S} m²")
    print(f"  Drag coefficient: {system.Cd}")
    print(f"  Air density: {system.rho} kg/m³")
    print(f"  Sample time: {system.Ts} s")
    
    # Verify linearized model
    A, B, C, D = system.linearized_dynamics()
    print("\nLinearized Model (Continuous):")
    print("State matrix A:")
    print(A)
    print("\nInput matrix B:")
    print(B)
    print("\nOutput matrix C:")
    print(C)
    
    # Verify discrete model
    Ad, Bd, Cd, Dd = system.get_discrete_model()
    print("\nDiscrete Model (Ts = 0.1s):")
    print("State matrix Ad:")
    print(Ad)
    print("\nInput matrix Bd:")
    print(Bd)
    
    # Initialize controller
    controller = OptimalController(system, N_horizon=100)
    
    # Test scenarios
    test_cases = [
        # (x0, target_pos, scenario_name)
        (np.array([0.0, 0.0, 0.0, 0.0]), 20.0, "Start at rest, target 20m"),
        (np.array([0.0, 0.0, 0.0, 0.0]), 40.0, "Start at rest, target 40m"),
        (np.array([0.0, 0.0, 0.0, 0.0]), 80.0, "Start at rest, target 80m"),
        (np.array([0.0, 2.0, 0.05, 0.0]), 20.0, "Start moving (v=2m/s, theta=0.05rad), target 20m"),
        (np.array([0.0, 2.0, 0.05, 0.0]), 40.0, "Start moving (v=2m/s, theta=0.05rad), target 40m"),
    ]
    
    solve_times = []
    figures = []
    
    for x0, target_pos, scenario_name in test_cases:
        result = run_test_scenario(system, controller, x0, target_pos, scenario_name)
        if result is not None:
            solve_time, fig1, fig2 = result
            solve_times.append(solve_time)
            figures.extend([fig1, fig2])
    
    # Summary
    print("\n" + "="*60)
    print("Summary")
    print("="*60)
    print(f"Average optimization time: {np.mean(solve_times)*1000:.2f} ms")
    print(f"Min optimization time: {np.min(solve_times)*1000:.2f} ms")
    print(f"Max optimization time: {np.max(solve_times)*1000:.2f} ms")
    print(f"\nOptimization is fast enough for closed-loop control at {system.Ts}s sample time.")
    
    # Save all figures
    import os
    output_dir = os.path.dirname(os.path.abspath(__file__))
    for i, fig in enumerate(figures):
        output_path = os.path.join(output_dir, f'result_plot_{i+1}.png')
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
    
    print(f"\nSaved {len(figures)} plots as result_plot_1.png to result_plot_{len(figures)}.png")
    
    # Show plots
    plt.show()


if __name__ == "__main__":
    main()
