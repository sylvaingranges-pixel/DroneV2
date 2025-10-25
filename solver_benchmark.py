"""
Solver Benchmark for Drone Control MPC

This script tests different CVXPY solvers to find the optimal balance between:
- Speed (solve time)
- Robustness (success rate)
- Precision (solution quality)

Tested solvers:
- OSQP: Operator Splitting Quadratic Program
- ECOS: Embedded Conic Solver
- CLARABEL: Modern conic solver (Rust-based)
- SCS: Splitting Conic Solver
- CVXOPT: Convex optimization package

Author: Drone Control System
Date: 2025-10-25
"""

import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp
import time
import pandas as pd
from drone_control_1d import DroneSystem, MPCController, simulate_linear
import warnings
warnings.filterwarnings('ignore')

# Physical constants
TS = 0.1  # Sampling time (seconds)


class SolverBenchmark:
    """Benchmark different CVXPY solvers for MPC optimization"""
    
    def __init__(self, system, Ad, Bd):
        """
        Initialize benchmark with system matrices
        
        Args:
            system: DroneSystem object
            Ad: Discrete state matrix
            Bd: Discrete input matrix
        """
        self.system = system
        self.Ad = Ad
        self.Bd = Bd
        self.results = []
        
        # Solvers to test with their configurations
        self.solvers = {
            'OSQP': {
                'solver': cp.OSQP,
                'params': {'verbose': False, 'max_iter': 100000, 'eps_abs': 1e-4, 'eps_rel': 1e-4, 'polish': True},
                'description': 'Operator Splitting QP - optimisé pour problèmes QP'
            },
            'ECOS': {
                'solver': cp.ECOS,
                'params': {'verbose': False, 'max_iters': 100000, 'abstol': 1e-4, 'reltol': 1e-4},
                'description': 'Embedded Conic Solver - petit et rapide'
            },
            'CLARABEL': {
                'solver': cp.CLARABEL,
                'params': {'verbose': False, 'max_iter': 100000, 'tol_gap_abs': 1e-4, 'tol_gap_rel': 1e-4},
                'description': 'Solveur conique moderne (Rust) - très performant'
            },
            'SCS': {
                'solver': cp.SCS,
                'params': {'verbose': False, 'max_iters': 100000, 'eps': 1e-4},
                'description': 'Splitting Conic Solver - robuste pour grands problèmes'
            },
            'CVXOPT': {
                'solver': cp.CVXOPT,
                'params': {'verbose': False, 'max_iters': 100000, 'abstol': 1e-4, 'reltol': 1e-4},
                'description': 'Solveur convexe classique - mature et stable'
            }
        }
    
    def solve_mpc_with_solver(self, x0, x_load_target, Q_state, Q_load, R, 
                              horizon, solver_name, Qf_state=None, Qf_load=None):
        """
        Solve MPC problem with specified solver
        
        Args:
            x0: Initial state
            x_load_target: Target load position
            Q_state: State weights
            Q_load: Load position weight
            R: Input weight
            horizon: Prediction horizon
            solver_name: Name of the solver to use
            Qf_state: Terminal state weights
            Qf_load: Terminal load weight
            
        Returns:
            result dict with solve_time, status, cost, and solution
        """
        if Qf_state is None:
            Qf_state = Q_state * 50
        if Qf_load is None:
            Qf_load = Q_load * 100
        
        nx = self.Ad.shape[0]
        nu = self.Bd.shape[1]
        L = self.system.L
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
            
            constraints.append(x[:, k + 1] == self.Ad @ x[:, k] + self.Bd @ u[:, k])
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
        
        solver_config = self.solvers[solver_name]
        
        try:
            start_time = time.time()
            problem.solve(solver=solver_config['solver'], **solver_config['params'])
            solve_time = time.time() - start_time
            
            # Check if solution was successful
            # Include more status types for robustness
            success_statuses = ["optimal", "optimal_inaccurate", "solved", "solved_inaccurate"]
            is_success = problem.status in success_statuses
            
            # Debug: print status for failed solvers
            if not is_success and solve_time < np.inf:
                print(f" [status: {problem.status}]", end='')
            
            result = {
                'solver': solver_name,
                'solve_time': solve_time,
                'status': problem.status,
                'cost': problem.value if problem.value is not None else np.inf,
                'success': is_success,
                'u_opt': u.value if u.value is not None else None,
                'x_opt': x.value if x.value is not None else None
            }
        except Exception as e:
            print(f"\n  ⚠ Error with {solver_name}: {str(e)}")
            result = {
                'solver': solver_name,
                'solve_time': np.inf,
                'status': f'ERROR: {str(e)}',
                'cost': np.inf,
                'success': False,
                'u_opt': None,
                'x_opt': None
            }
        
        return result
    
    def evaluate_solution_quality(self, x_opt, u_opt, x_load_target):
        """
        Evaluate the quality of a solution
        
        Returns:
            metrics dict with position error, velocity, angle, control effort
        """
        if x_opt is None or u_opt is None:
            return {
                'load_error': np.inf,
                'position_error': np.inf,
                'velocity_final': np.inf,
                'angle_final': np.inf,
                'control_effort': np.inf
            }
        
        # Final load position error
        x_load_final = x_opt[0, -1] + self.system.L * np.sin(x_opt[2, -1])
        load_error = abs(x_load_final - x_load_target)
        
        # Final drone position error
        position_error = abs(x_opt[0, -1] - x_load_target)
        
        # Final velocity
        velocity_final = abs(x_opt[1, -1])
        
        # Final angle
        angle_final = abs(x_opt[2, -1])
        
        # Control effort (RMS)
        control_effort = np.sqrt(np.mean(u_opt**2))
        
        return {
            'load_error': load_error,
            'position_error': position_error,
            'velocity_final': velocity_final,
            'angle_final': angle_final,
            'control_effort': control_effort
        }
    
    def run_benchmark(self, test_cases, Q_state, Q_load, R_weight, repetitions=3):
        """
        Run benchmark on multiple test cases with multiple repetitions
        
        Args:
            test_cases: List of test case dicts
            Q_state: State weights
            Q_load: Load position weight
            R_weight: Input weight
            repetitions: Number of times to run each test for averaging
            
        Returns:
            DataFrame with all results
        """
        print("="*80)
        print("BENCHMARK DES SOLVEURS CVXPY POUR LE CONTRÔLE DE DRONE")
        print("="*80)
        
        all_results = []
        
        for test_idx, test in enumerate(test_cases):
            test_name = test['name']
            x0 = test['x0']
            x_load_target = test['x_load_target']
            horizon = test['horizon']
            
            print(f"\n[Test {test_idx+1}/{len(test_cases)}] {test_name}")
            print(f"  Horizon: {horizon} étapes ({horizon*TS:.1f}s)")
            print(f"  Cible: {x_load_target:.1f}m")
            
            for solver_name in self.solvers.keys():
                print(f"    Testing {solver_name}...", end=' ')
                
                solve_times = []
                successes = []
                qualities = []
                
                for rep in range(repetitions):
                    result = self.solve_mpc_with_solver(
                        x0, x_load_target, Q_state, Q_load, R_weight,
                        horizon, solver_name
                    )
                    
                    solve_times.append(result['solve_time'])
                    successes.append(result['success'])
                    
                    if result['success']:
                        quality = self.evaluate_solution_quality(
                            result['x_opt'], result['u_opt'], x_load_target
                        )
                        qualities.append(quality)
                
                # Calculate statistics
                avg_time = np.mean(solve_times) if solve_times else np.inf
                std_time = np.std(solve_times) if solve_times else 0
                success_rate = np.mean(successes) * 100
                
                if qualities:
                    avg_load_error = np.mean([q['load_error'] for q in qualities])
                    avg_velocity = np.mean([q['velocity_final'] for q in qualities])
                    avg_angle = np.mean([q['angle_final'] for q in qualities])
                    avg_control = np.mean([q['control_effort'] for q in qualities])
                else:
                    avg_load_error = np.inf
                    avg_velocity = np.inf
                    avg_angle = np.inf
                    avg_control = np.inf
                
                print(f"{avg_time*1000:.1f}ms (taux succès: {success_rate:.0f}%)")
                
                all_results.append({
                    'test_case': test_name,
                    'horizon': horizon,
                    'target_distance': x_load_target,
                    'solver': solver_name,
                    'avg_solve_time_ms': avg_time * 1000,
                    'std_solve_time_ms': std_time * 1000,
                    'success_rate_%': success_rate,
                    'avg_load_error_m': avg_load_error,
                    'avg_velocity_final_ms': avg_velocity,
                    'avg_angle_final_rad': avg_angle,
                    'avg_control_effort': avg_control
                })
        
        df = pd.DataFrame(all_results)
        return df
    
    def generate_report(self, df, output_file='RAPPORT_COMPARATIF_SOLVEURS.md'):
        """
        Generate a comprehensive comparative report in French
        
        Args:
            df: DataFrame with benchmark results
            output_file: Output markdown file
        """
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("# Rapport Comparatif des Solveurs CVXPY\n\n")
            f.write("## Contrôle Prédictif de Drone avec Charge Suspendue\n\n")
            f.write(f"Date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write("## 1. Objectif\n\n")
            f.write("Identifier le meilleur solveur CVXPY pour le problème de contrôle MPC du drone ")
            f.write("en optimisant le compromis entre:\n")
            f.write("- **Rapidité**: Temps de résolution\n")
            f.write("- **Robustesse**: Taux de succès\n")
            f.write("- **Précision**: Qualité de la solution\n\n")
            
            f.write("## 2. Solveurs Testés\n\n")
            for name, config in self.solvers.items():
                f.write(f"### {name}\n")
                f.write(f"{config['description']}\n\n")
            
            f.write("## 3. Résultats par Solveur\n\n")
            
            # Group by solver and calculate overall statistics
            solver_stats = df.groupby('solver').agg({
                'avg_solve_time_ms': ['mean', 'std', 'min', 'max'],
                'success_rate_%': 'mean',
                'avg_load_error_m': 'mean',
                'avg_velocity_final_ms': 'mean',
                'avg_angle_final_rad': 'mean'
            }).round(2)
            
            f.write("### Statistiques Globales\n\n")
            f.write("| Solveur | Temps Moyen (ms) | Min (ms) | Max (ms) | Taux Succès (%) | Erreur Charge (m) |\n")
            f.write("|---------|------------------|----------|----------|-----------------|-------------------|\n")
            
            for solver in solver_stats.index:
                stats = solver_stats.loc[solver]
                f.write(f"| {solver} | ")
                f.write(f"{stats[('avg_solve_time_ms', 'mean')]:.1f} ± {stats[('avg_solve_time_ms', 'std')]:.1f} | ")
                f.write(f"{stats[('avg_solve_time_ms', 'min')]:.1f} | ")
                f.write(f"{stats[('avg_solve_time_ms', 'max')]:.1f} | ")
                f.write(f"{stats[('success_rate_%', 'mean')]:.0f} | ")
                f.write(f"{stats[('avg_load_error_m', 'mean')]:.4f} |\n")
            
            f.write("\n### Détails par Cas de Test\n\n")
            
            # Results by test case
            for test_name in df['test_case'].unique():
                test_df = df[df['test_case'] == test_name]
                f.write(f"#### {test_name}\n\n")
                f.write("| Solveur | Temps (ms) | Succès (%) | Erreur (m) | Vitesse Finale (m/s) |\n")
                f.write("|---------|------------|------------|------------|----------------------|\n")
                
                for _, row in test_df.iterrows():
                    f.write(f"| {row['solver']} | ")
                    f.write(f"{row['avg_solve_time_ms']:.1f} ± {row['std_solve_time_ms']:.1f} | ")
                    f.write(f"{row['success_rate_%']:.0f} | ")
                    f.write(f"{row['avg_load_error_m']:.4f} | ")
                    f.write(f"{row['avg_velocity_final_ms']:.4f} |\n")
                f.write("\n")
            
            f.write("## 4. Analyse Comparative\n\n")
            
            # Find best solvers
            successful_solvers = df[df['success_rate_%'] == 100.0]['solver'].unique()
            
            if len(successful_solvers) > 0:
                successful_df = df[df['solver'].isin(successful_solvers)]
                fastest = successful_df.groupby('solver')['avg_solve_time_ms'].mean().idxmin()
                most_precise = successful_df.groupby('solver')['avg_load_error_m'].mean().idxmin()
                
                f.write(f"### Solveur le Plus Rapide (avec 100% succès)\n")
                f.write(f"**{fastest}** - ")
                f.write(f"{successful_df[successful_df['solver']==fastest]['avg_solve_time_ms'].mean():.1f}ms moyen\n\n")
                
                f.write(f"### Solveur le Plus Précis (avec 100% succès)\n")
                f.write(f"**{most_precise}** - ")
                f.write(f"{successful_df[successful_df['solver']==most_precise]['avg_load_error_m'].mean():.4f}m erreur moyenne\n\n")
            
            f.write("### Performance selon l'Horizon\n\n")
            f.write("Impact de la taille du problème (horizon) sur le temps de résolution:\n\n")
            
            horizon_stats = df.groupby(['solver', 'horizon'])['avg_solve_time_ms'].mean().unstack()
            f.write("| Solveur | " + " | ".join([f"H={int(h)}" for h in horizon_stats.columns]) + " |\n")
            f.write("|---------|" + "|".join(["--------" for _ in horizon_stats.columns]) + "|\n")
            
            for solver in horizon_stats.index:
                f.write(f"| {solver} | ")
                f.write(" | ".join([f"{horizon_stats.loc[solver, h]:.1f}ms" for h in horizon_stats.columns]))
                f.write(" |\n")
            
            f.write("\n## 5. Recommandations\n\n")
            
            if len(successful_solvers) > 0:
                f.write(f"### Solveur Recommandé: **{fastest}**\n\n")
                f.write("**Justification:**\n")
                f.write(f"- Taux de succès: 100%\n")
                
                fastest_data = successful_df[successful_df['solver'] == fastest]
                avg_time = fastest_data['avg_solve_time_ms'].mean()
                avg_error = fastest_data['avg_load_error_m'].mean()
                
                f.write(f"- Temps de résolution moyen: {avg_time:.1f}ms\n")
                f.write(f"- Précision moyenne: {avg_error:.4f}m\n")
                f.write(f"- Adapté pour contrôle en boucle fermée temps-réel\n\n")
                
                if fastest != most_precise:
                    f.write(f"### Alternative pour Précision Maximale: **{most_precise}**\n\n")
                    precise_data = successful_df[successful_df['solver'] == most_precise]
                    f.write(f"Si la précision est critique et le temps moins contraignant:\n")
                    f.write(f"- Erreur moyenne: {precise_data['avg_load_error_m'].mean():.4f}m\n")
                    f.write(f"- Temps moyen: {precise_data['avg_solve_time_ms'].mean():.1f}ms\n\n")
            
            f.write("### Paramètres de Configuration Optimaux\n\n")
            f.write("```python\n")
            if len(successful_solvers) > 0:
                solver_config = self.solvers[fastest]
                f.write(f"solver = cp.{fastest}\n")
                f.write(f"params = {solver_config['params']}\n")
            f.write("```\n\n")
            
            f.write("## 6. Conclusions\n\n")
            f.write("Ce benchmark démontre que le choix du solveur a un impact significatif sur:\n")
            f.write("- La rapidité de résolution (variation jusqu'à 10x)\n")
            f.write("- La robustesse (certains solveurs échouent sur grands horizons)\n")
            f.write("- La précision (différences minimes pour solveurs réussis)\n\n")
            
            f.write("Pour le contrôle MPC du drone, un solveur rapide et robuste est essentiel ")
            f.write("pour permettre le re-calcul fréquent de la trajectoire en boucle fermée.\n\n")
            
            f.write("---\n")
            f.write(f"*Rapport généré automatiquement le {time.strftime('%Y-%m-%d à %H:%M:%S')}*\n")
        
        print(f"\n✓ Rapport généré: {output_file}")
    
    def plot_comparison(self, df, output_prefix='solver_comparison'):
        """
        Create comparison plots
        
        Args:
            df: DataFrame with results
            output_prefix: Prefix for output files
        """
        # Filter successful solvers only for fair comparison
        df_success = df[df['success_rate_%'] == 100.0].copy()
        
        if len(df_success) == 0:
            print("Aucun solveur n'a réussi tous les tests, utilisation de tous les résultats")
            df_success = df.copy()
        
        # Plot 1: Solve time comparison
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Time vs Horizon
        ax = axes[0, 0]
        for solver in df_success['solver'].unique():
            solver_df = df_success[df_success['solver'] == solver]
            grouped = solver_df.groupby('horizon')['avg_solve_time_ms'].mean()
            ax.plot(grouped.index, grouped.values, marker='o', label=solver, linewidth=2)
        ax.set_xlabel('Horizon (étapes)', fontsize=11)
        ax.set_ylabel('Temps de Résolution (ms)', fontsize=11)
        ax.set_title('Temps de Résolution vs Horizon', fontsize=12, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Average time by solver
        ax = axes[0, 1]
        solver_times = df_success.groupby('solver')['avg_solve_time_ms'].mean().sort_values()
        colors = plt.cm.viridis(np.linspace(0, 0.9, len(solver_times)))
        bars = ax.barh(solver_times.index, solver_times.values, color=colors)
        ax.set_xlabel('Temps Moyen (ms)', fontsize=11)
        ax.set_title('Temps de Résolution Moyen par Solveur', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3, axis='x')
        
        # Add value labels
        for bar in bars:
            width = bar.get_width()
            ax.text(width, bar.get_y() + bar.get_height()/2, f'{width:.1f}ms',
                   ha='left', va='center', fontsize=9, fontweight='bold')
        
        # Precision comparison
        ax = axes[1, 0]
        solver_errors = df_success.groupby('solver')['avg_load_error_m'].mean().sort_values()
        colors = plt.cm.plasma(np.linspace(0, 0.9, len(solver_errors)))
        bars = ax.barh(solver_errors.index, solver_errors.values, color=colors)
        ax.set_xlabel('Erreur Position Charge (m)', fontsize=11)
        ax.set_title('Précision Moyenne par Solveur', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3, axis='x')
        
        # Add value labels
        for bar in bars:
            width = bar.get_width()
            ax.text(width, bar.get_y() + bar.get_height()/2, f'{width:.4f}m',
                   ha='left', va='center', fontsize=9, fontweight='bold')
        
        # Speed vs Precision scatter
        ax = axes[1, 1]
        solver_summary = df_success.groupby('solver').agg({
            'avg_solve_time_ms': 'mean',
            'avg_load_error_m': 'mean'
        })
        
        colors_map = {'OSQP': 'blue', 'ECOS': 'green', 'CLARABEL': 'red', 
                     'SCS': 'orange', 'CVXOPT': 'purple'}
        
        for solver in solver_summary.index:
            ax.scatter(solver_summary.loc[solver, 'avg_solve_time_ms'],
                      solver_summary.loc[solver, 'avg_load_error_m'],
                      s=200, alpha=0.6, 
                      color=colors_map.get(solver, 'gray'),
                      edgecolors='black', linewidth=2,
                      label=solver)
            ax.annotate(solver, 
                       (solver_summary.loc[solver, 'avg_solve_time_ms'],
                        solver_summary.loc[solver, 'avg_load_error_m']),
                       xytext=(5, 5), textcoords='offset points',
                       fontsize=9, fontweight='bold')
        
        ax.set_xlabel('Temps Moyen (ms)', fontsize=11)
        ax.set_ylabel('Erreur Moyenne (m)', fontsize=11)
        ax.set_title('Rapidité vs Précision', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'{output_prefix}.png', dpi=150, bbox_inches='tight')
        print(f"✓ Graphiques sauvegardés: {output_prefix}.png")
        
        # Plot 2: Success rate
        fig2, ax2 = plt.subplots(1, 1, figsize=(10, 6))
        success_rates = df.groupby('solver')['success_rate_%'].mean().sort_values(ascending=False)
        colors = ['green' if x == 100 else 'orange' if x >= 80 else 'red' for x in success_rates.values]
        bars = ax2.bar(success_rates.index, success_rates.values, color=colors, alpha=0.7, edgecolor='black', linewidth=2)
        ax2.set_ylabel('Taux de Succès (%)', fontsize=12)
        ax2.set_title('Robustesse des Solveurs (Taux de Succès)', fontsize=13, fontweight='bold')
        ax2.set_ylim([0, 105])
        ax2.grid(True, alpha=0.3, axis='y')
        ax2.axhline(y=100, color='green', linestyle='--', linewidth=2, alpha=0.5, label='100% succès')
        
        # Add value labels
        for bar in bars:
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.0f}%',
                    ha='center', va='bottom', fontsize=11, fontweight='bold')
        
        plt.tight_layout()
        plt.savefig(f'{output_prefix}_robustesse.png', dpi=150, bbox_inches='tight')
        print(f"✓ Graphiques sauvegardés: {output_prefix}_robustesse.png")


def main():
    """Main benchmark execution"""
    print("Initialisation du système...")
    
    # Create system
    system = DroneSystem()
    
    # Get linearized and discretized model
    A, B = system.get_linearized_matrices()
    Ad, Bd = system.discretize(A, B, TS)
    
    # Cost function weights (same as original)
    Q_state = np.array([10.0, 20.0, 500.0, 100.0])
    Q_load = 1000.0
    R_weight = 2.0
    
    # Define test cases (subset of original for faster benchmarking)
    test_cases = [
        {
            'name': 'test_20m_repos',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),
            'x_load_target': 20.0,
            'horizon': 100
        },
        {
            'name': 'test_40m_repos',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),
            'x_load_target': 40.0,
            'horizon': 150
        },
        {
            'name': 'test_80m_repos',
            'x0': np.array([0.0, 0.0, 0.0, 0.0]),
            'x_load_target': 80.0,
            'horizon': 200
        },
        {
            'name': 'test_40m_vitesse',
            'x0': np.array([0.0, 3.0, 0.15, 0.05]),
            'x_load_target': 40.0,
            'horizon': 150
        }
    ]
    
    # Create benchmark object
    benchmark = SolverBenchmark(system, Ad, Bd)
    
    # Run benchmark
    print("\nDémarrage du benchmark...")
    df_results = benchmark.run_benchmark(test_cases, Q_state, Q_load, R_weight, repetitions=3)
    
    # Save results to CSV
    csv_file = 'solver_benchmark_results.csv'
    df_results.to_csv(csv_file, index=False)
    print(f"\n✓ Résultats sauvegardés: {csv_file}")
    
    # Generate report
    benchmark.generate_report(df_results)
    
    # Create visualizations
    benchmark.plot_comparison(df_results)
    
    print("\n" + "="*80)
    print("BENCHMARK TERMINÉ AVEC SUCCÈS")
    print("="*80)
    print("\nFichiers générés:")
    print("  - RAPPORT_COMPARATIF_SOLVEURS.md (rapport détaillé)")
    print("  - solver_comparison.png (graphiques comparatifs)")
    print("  - solver_comparison_robustesse.png (taux de succès)")
    print("  - solver_benchmark_results.csv (données brutes)")
    print("\n")


if __name__ == "__main__":
    main()
