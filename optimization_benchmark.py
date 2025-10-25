"""
Benchmark for Optimized MPC Solver Configurations

This script tests optimized configurations to achieve faster solve times
while maintaining the required precision:
- Position accuracy: 0.3m
- Velocity accuracy: 0.15m/s

Tests different:
- Solvers (CLARABEL, SCS, OSQP)
- Horizons (shorter for faster computation)
- Tolerance settings (relaxed)

Author: Drone Control System
Date: 2025-10-25
"""

import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp
import time
import pandas as pd
from drone_control_1d import DroneSystem
import warnings
warnings.filterwarnings('ignore')

TS = 0.1  # Sampling time (seconds)

# Required precision
REQUIRED_POS_ACCURACY = 0.3  # meters
REQUIRED_VEL_ACCURACY = 0.15  # m/s

# Baseline performance for comparison (from RAPPORT_COMPARATIF_SOLVEURS.md)
BASELINE_SOLVE_TIME_MS = 1180.6  # Average solve time with CLARABEL baseline

# Success status list for solution validity
SUCCESS_STATUSES = ["optimal", "optimal_inaccurate", "solved", "solved_inaccurate"]


def solve_mpc_optimized(system, Ad, Bd, x0, x_load_target, Q_state, Q_load, R,
                        horizon, solver_name, solver_params, Qf_state=None, Qf_load=None):
    """
    Solve MPC problem with specified configuration
    
    Returns:
        result dict with solve_time, status, errors, and solution
    """
    if Qf_state is None:
        Qf_state = Q_state * 50
    if Qf_load is None:
        Qf_load = Q_load * 100
    
    nx = Ad.shape[0]
    nu = Bd.shape[1]
    L = system.L
    u_max = 15.0
    u_min = -15.0
    
    # Define optimization variables
    x = cp.Variable((nx, horizon + 1))
    u = cp.Variable((nu, horizon))
    
    # Cost function
    cost = 0
    constraints = []
    
    # Initial condition
    constraints.append(x[:, 0] == x0)
    
    # Target state
    x_drone_target = x_load_target
    x_target = np.array([x_drone_target, 0.0, 0.0, 0.0])
    
    # Stage costs and dynamics
    for k in range(horizon):
        dx = x[:, k] - x_target
        cost += Q_state[0] * dx[0]**2
        cost += Q_state[1] * dx[1]**2
        cost += Q_state[2] * dx[2]**2
        cost += Q_state[3] * dx[3]**2
        
        x_load = x[0, k] + L * x[2, k]
        cost += Q_load * (x_load - x_load_target)**2
        cost += R * u[0, k]**2
        
        constraints.append(x[:, k + 1] == Ad @ x[:, k] + Bd @ u[:, k])
        constraints.append(u[:, k] <= u_max)
        constraints.append(u[:, k] >= u_min)
    
    # Terminal cost
    dx_f = x[:, horizon] - x_target
    cost += Qf_state[0] * dx_f[0]**2
    cost += Qf_state[1] * dx_f[1]**2
    cost += Qf_state[2] * dx_f[2]**2
    cost += Qf_state[3] * dx_f[3]**2
    
    x_load_f = x[0, horizon] + L * x[2, horizon]
    cost += Qf_load * (x_load_f - x_load_target)**2
    
    # Create and solve problem
    problem = cp.Problem(cp.Minimize(cost), constraints)
    
    try:
        start_time = time.time()
        problem.solve(solver=solver_params['solver'], verbose=False, **solver_params['params'])
        solve_time = time.time() - start_time
        
        # Check if solution was successful
        is_success = problem.status in SUCCESS_STATUSES
        
        if is_success and x.value is not None:
            x_opt = x.value
            u_opt = u.value
            
            # Calculate errors
            # Use sin for actual nonlinear calculation (validates against real system)
            x_load_final = x_opt[0, -1] + L * np.sin(x_opt[2, -1])
            pos_error = abs(x_load_final - x_load_target)
            # Velocity error: difference from target velocity (which is 0)
            vel_error = abs(x_opt[1, -1] - 0.0)
            
            # Check if meets requirements
            meets_req = pos_error <= REQUIRED_POS_ACCURACY and vel_error <= REQUIRED_VEL_ACCURACY
            
            result = {
                'solver': solver_name,
                'solve_time': solve_time,
                'status': problem.status,
                'success': True,
                'pos_error': pos_error,
                'vel_error': vel_error,
                'meets_requirements': meets_req,
                'x_opt': x_opt,
                'u_opt': u_opt
            }
        else:
            result = {
                'solver': solver_name,
                'solve_time': solve_time if solve_time < np.inf else np.inf,
                'status': problem.status,
                'success': False,
                'pos_error': np.inf,
                'vel_error': np.inf,
                'meets_requirements': False,
                'x_opt': None,
                'u_opt': None
            }
    except Exception as e:
        result = {
            'solver': solver_name,
            'solve_time': np.inf,
            'status': f'ERROR: {str(e)}',
            'success': False,
            'pos_error': np.inf,
            'vel_error': np.inf,
            'meets_requirements': False,
            'x_opt': None,
            'u_opt': None
        }
    
    return result


def main():
    """Run optimization benchmark"""
    print("="*80)
    print("BENCHMARK D'OPTIMISATION - VITESSE DE CALCUL MPC")
    print("="*80)
    print(f"\nPrécision requise:")
    print(f"  - Position: {REQUIRED_POS_ACCURACY} m")
    print(f"  - Vitesse: {REQUIRED_VEL_ACCURACY} m/s")
    
    # Create system
    system = DroneSystem()
    A, B = system.get_linearized_matrices()
    Ad, Bd = system.discretize(A, B, TS)
    
    # Cost function weights
    Q_state = np.array([10.0, 20.0, 500.0, 100.0])
    Q_load = 1000.0
    R_weight = 2.0
    
    # Define solver configurations to test
    solver_configs = [
        {
            'name': 'CLARABEL-Fast',
            'solver': cp.CLARABEL,
            'params': {'tol_gap_abs': 1e-2, 'tol_gap_rel': 1e-2, 'max_iter': 10000}
        },
        {
            'name': 'SCS-Fast',
            'solver': cp.SCS,
            'params': {'eps': 1e-2, 'max_iters': 10000}
        },
        {
            'name': 'OSQP-Fast',
            'solver': cp.OSQP,
            'params': {'eps_abs': 1e-2, 'eps_rel': 1e-2, 'max_iter': 10000, 'polish': True}
        },
        {
            'name': 'CLARABEL-Medium',
            'solver': cp.CLARABEL,
            'params': {'tol_gap_abs': 1e-3, 'tol_gap_rel': 1e-3, 'max_iter': 20000}
        },
        {
            'name': 'SCS-Medium',
            'solver': cp.SCS,
            'params': {'eps': 1e-3, 'max_iters': 20000}
        },
    ]
    
    # Define test cases - various scenarios
    test_cases = [
        {
            'name': '20m_repos',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),
            'x_load_target': 20.0,
            'horizon': 60  # Optimized shorter horizon
        },
        {
            'name': '40m_repos',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),
            'x_load_target': 40.0,
            'horizon': 80
        },
        {
            'name': '80m_repos',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),
            'x_load_target': 80.0,
            'horizon': 100
        },
        {
            'name': '20m_vitesse',
            'x0': np.array([0.0, 2.0, 0.1, 0.0]),
            'x_load_target': 20.0,
            'horizon': 60
        },
        {
            'name': '40m_vitesse',
            'x0': np.array([0.0, 3.0, 0.15, 0.05]),
            'x_load_target': 40.0,
            'horizon': 80
        },
    ]
    
    all_results = []
    
    print("\n" + "="*80)
    print("TESTS EN COURS...")
    print("="*80)
    
    for test_idx, test in enumerate(test_cases):
        test_name = test['name']
        x0 = test['x0']
        x_load_target = test['x_load_target']
        horizon = test['horizon']
        
        print(f"\n[{test_idx+1}/{len(test_cases)}] Test: {test_name}")
        print(f"  Horizon: {horizon} étapes ({horizon*TS:.1f}s), Cible: {x_load_target}m")
        
        for solver_config in solver_configs:
            solver_name = solver_config['name']
            print(f"    {solver_name:20s}...", end=' ', flush=True)
            
            result = solve_mpc_optimized(
                system, Ad, Bd, x0, x_load_target,
                Q_state, Q_load, R_weight,
                horizon, solver_name,
                {'solver': solver_config['solver'], 'params': solver_config['params']}
            )
            
            result['test_case'] = test_name
            result['horizon'] = horizon
            result['target_distance'] = x_load_target
            all_results.append(result)
            
            if result['success']:
                status = '✓' if result['meets_requirements'] else '✗'
                print(f"{result['solve_time']*1000:6.1f}ms | "
                      f"Pos: {result['pos_error']:.4f}m | "
                      f"Vel: {result['vel_error']:.4f}m/s | {status}")
            else:
                print(f"ÉCHEC - {result['status']}")
    
    # Create DataFrame
    df = pd.DataFrame(all_results)
    
    # Generate summary
    print("\n" + "="*80)
    print("RÉSUMÉ PAR SOLVEUR")
    print("="*80)
    
    summary = df[df['success'] == True].groupby('solver').agg({
        'solve_time': ['mean', 'std', 'min', 'max'],
        'pos_error': ['mean', 'max'],
        'vel_error': ['mean', 'max'],
        'meets_requirements': 'sum'
    }).round(4)
    
    for solver in df['solver'].unique():
        solver_df = df[df['solver'] == solver]
        n_success = solver_df['success'].sum()
        n_meets = solver_df['meets_requirements'].sum()
        
        if n_success > 0:
            avg_time = solver_df[solver_df['success']]['solve_time'].mean()
            avg_pos = solver_df[solver_df['success']]['pos_error'].mean()
            avg_vel = solver_df[solver_df['success']]['vel_error'].mean()
            
            print(f"\n{solver}:")
            print(f"  Taux de succès: {n_success}/{len(test_cases)} ({n_success/len(test_cases)*100:.0f}%)")
            print(f"  Conforme aux exigences: {n_meets}/{n_success} ({n_meets/n_success*100:.0f}%)")
            print(f"  Temps moyen: {avg_time*1000:.1f}ms")
            print(f"  Erreur position moyenne: {avg_pos:.4f}m")
            print(f"  Erreur vitesse moyenne: {avg_vel:.4f}m/s")
    
    # Find best configuration
    meeting_req = df[(df['success'] == True) & (df['meets_requirements'] == True)]
    if len(meeting_req) > 0:
        best_solver_df = meeting_req.groupby('solver')['solve_time'].mean().sort_values()
        best_solver = best_solver_df.index[0]
        best_time = best_solver_df.values[0]
        
        print("\n" + "="*80)
        print("RECOMMANDATION")
        print("="*80)
        print(f"\nMeilleure configuration: {best_solver}")
        print(f"  Temps moyen: {best_time*1000:.1f}ms")
        
        best_config = [c for c in solver_configs if c['name'] == best_solver][0]
        print(f"  Paramètres: {best_config['params']}")
        
        # Calculate speedup vs baseline
        speedup = BASELINE_SOLVE_TIME_MS / (best_time * 1000)
        print(f"\n  Accélération vs baseline: {speedup:.2f}x plus rapide")
    
    # Save results
    csv_file = 'optimization_benchmark_results.csv'
    df.to_csv(csv_file, index=False)
    print(f"\n✓ Résultats sauvegardés: {csv_file}")
    
    # Create visualization
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot 1: Solve time by solver
    ax = axes[0, 0]
    successful = df[df['success'] == True]
    if len(successful) > 0:
        solver_times = successful.groupby('solver')['solve_time'].mean().sort_values() * 1000
        colors = ['green' if df[(df['solver']==s) & (df['meets_requirements']==True)].shape[0] == len(test_cases) 
                  else 'orange' for s in solver_times.index]
        bars = ax.barh(solver_times.index, solver_times.values, color=colors, alpha=0.7, edgecolor='black')
        ax.set_xlabel('Temps Moyen (ms)', fontsize=11)
        ax.set_title('Temps de Résolution par Solveur', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3, axis='x')
        
        for bar in bars:
            width = bar.get_width()
            ax.text(width, bar.get_y() + bar.get_height()/2, f'{width:.1f}ms',
                   ha='left', va='center', fontsize=9, fontweight='bold')
    
    # Plot 2: Precision by solver
    ax = axes[0, 1]
    if len(successful) > 0:
        solver_errors = successful.groupby('solver')['pos_error'].mean().sort_values()
        colors = ['green' if e <= REQUIRED_POS_ACCURACY else 'red' for e in solver_errors.values]
        bars = ax.barh(solver_errors.index, solver_errors.values, color=colors, alpha=0.7, edgecolor='black')
        ax.axvline(x=REQUIRED_POS_ACCURACY, color='red', linestyle='--', linewidth=2, label=f'Requis: {REQUIRED_POS_ACCURACY}m')
        ax.set_xlabel('Erreur Position Moyenne (m)', fontsize=11)
        ax.set_title('Précision par Solveur', fontsize=12, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3, axis='x')
        
        for bar in bars:
            width = bar.get_width()
            ax.text(width, bar.get_y() + bar.get_height()/2, f'{width:.4f}m',
                   ha='left', va='center', fontsize=9, fontweight='bold')
    
    # Plot 3: Speed vs Precision
    ax = axes[1, 0]
    if len(successful) > 0:
        for solver in successful['solver'].unique():
            solver_df = successful[successful['solver'] == solver]
            ax.scatter(solver_df['solve_time']*1000, solver_df['pos_error'],
                      s=100, alpha=0.6, label=solver, edgecolors='black')
        
        ax.axhline(y=REQUIRED_POS_ACCURACY, color='red', linestyle='--', linewidth=2, alpha=0.5)
        ax.set_xlabel('Temps de Résolution (ms)', fontsize=11)
        ax.set_ylabel('Erreur Position (m)', fontsize=11)
        ax.set_title('Rapidité vs Précision', fontsize=12, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # Plot 4: Success rate and compliance
    ax = axes[1, 1]
    solver_stats = []
    for solver in df['solver'].unique():
        solver_df = df[df['solver'] == solver]
        success_rate = solver_df['success'].sum() / len(solver_df) * 100
        meets_rate = solver_df['meets_requirements'].sum() / len(solver_df) * 100
        solver_stats.append({'solver': solver, 'success': success_rate, 'meets': meets_rate})
    
    stats_df = pd.DataFrame(solver_stats)
    x = np.arange(len(stats_df))
    width = 0.35
    
    bars1 = ax.bar(x - width/2, stats_df['success'], width, label='Succès', color='blue', alpha=0.7)
    bars2 = ax.bar(x + width/2, stats_df['meets'], width, label='Conforme', color='green', alpha=0.7)
    
    ax.set_ylabel('Taux (%)', fontsize=11)
    ax.set_title('Robustesse et Conformité', fontsize=12, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(stats_df['solver'], rotation=45, ha='right')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    ax.set_ylim([0, 110])
    
    plt.tight_layout()
    plt.savefig('optimization_benchmark.png', dpi=150, bbox_inches='tight')
    print(f"✓ Graphiques sauvegardés: optimization_benchmark.png")
    
    print("\n" + "="*80)
    print("BENCHMARK TERMINÉ")
    print("="*80)


if __name__ == "__main__":
    main()
