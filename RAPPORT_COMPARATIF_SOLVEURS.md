# Rapport Comparatif des Solveurs CVXPY

## Contrôle Prédictif de Drone avec Charge Suspendue

Date: 2025-10-25 08:21:43

## 1. Objectif

Identifier le meilleur solveur CVXPY pour le problème de contrôle MPC du drone en optimisant le compromis entre:
- **Rapidité**: Temps de résolution
- **Robustesse**: Taux de succès
- **Précision**: Qualité de la solution

## 2. Solveurs Testés

### OSQP
Operator Splitting QP - optimisé pour problèmes QP

### ECOS
Embedded Conic Solver - petit et rapide

### CLARABEL
Solveur conique moderne (Rust) - très performant

### SCS
Splitting Conic Solver - robuste pour grands problèmes

### CVXOPT
Solveur convexe classique - mature et stable

## 3. Résultats par Solveur

### Statistiques Globales

| Solveur | Temps Moyen (ms) | Min (ms) | Max (ms) | Taux Succès (%) | Erreur Charge (m) |
|---------|------------------|----------|----------|-----------------|-------------------|
| CLARABEL | 1180.6 ± 365.8 | 765.1 | 1656.9 | 100 | 0.0000 |
| CVXOPT | 18294.5 ± 7856.4 | 10611.7 | 29253.9 | 25 | inf |
| ECOS | 1961.0 ± 688.4 | 1203.9 | 2870.8 | 100 | 0.0000 |
| OSQP | 2152.5 ± 602.8 | 1265.5 | 2591.4 | 100 | 0.0000 |
| SCS | 1304.0 ± 451.8 | 834.7 | 1921.4 | 100 | 0.0000 |

### Détails par Cas de Test

#### test_20m_repos

| Solveur | Temps (ms) | Succès (%) | Erreur (m) | Vitesse Finale (m/s) |
|---------|------------|------------|------------|----------------------|
| OSQP | 1265.5 ± 3.1 | 100 | 0.0007 | 0.0326 |
| ECOS | 1203.9 ± 6.8 | 100 | 0.0007 | 0.0326 |
| CLARABEL | 765.1 ± 6.3 | 100 | 0.0007 | 0.0326 |
| SCS | 834.7 ± 5.7 | 100 | 0.0007 | 0.0327 |
| CVXOPT | 10611.7 ± 32.7 | 100 | 0.0007 | 0.0326 |

#### test_40m_repos

| Solveur | Temps (ms) | Succès (%) | Erreur (m) | Vitesse Finale (m/s) |
|---------|------------|------------|------------|----------------------|
| OSQP | 2591.4 ± 10.2 | 100 | 0.0004 | 0.0058 |
| ECOS | 1953.0 ± 26.8 | 100 | 0.0004 | 0.0058 |
| CLARABEL | 1143.5 ± 12.7 | 100 | 0.0004 | 0.0058 |
| SCS | 1226.4 ± 10.9 | 100 | 0.0004 | 0.0056 |
| CVXOPT | 16077.1 ± 27.7 | 0 | inf | inf |

#### test_80m_repos

| Solveur | Temps (ms) | Succès (%) | Erreur (m) | Vitesse Finale (m/s) |
|---------|------------|------------|------------|----------------------|
| OSQP | 2448.4 ± 52.9 | 100 | 0.0002 | 0.0009 |
| ECOS | 2870.8 ± 12.1 | 100 | 0.0002 | 0.0009 |
| CLARABEL | 1656.9 ± 51.0 | 100 | 0.0002 | 0.0009 |
| SCS | 1921.4 ± 44.0 | 100 | 0.0002 | 0.0010 |
| CVXOPT | 29253.9 ± 2.9 | 0 | inf | inf |

#### test_40m_vitesse

| Solveur | Temps (ms) | Succès (%) | Erreur (m) | Vitesse Finale (m/s) |
|---------|------------|------------|------------|----------------------|
| OSQP | 2304.6 ± 39.7 | 100 | 0.0000 | 0.0028 |
| ECOS | 1816.3 ± 20.3 | 100 | 0.0000 | 0.0027 |
| CLARABEL | 1156.7 ± 9.0 | 100 | 0.0000 | 0.0028 |
| SCS | 1233.5 ± 9.1 | 100 | 0.0000 | 0.0030 |
| CVXOPT | 17235.5 ± 21.5 | 0 | inf | inf |

## 4. Analyse Comparative

### Solveur le Plus Rapide (avec 100% succès)
**CLARABEL** - 1180.6ms moyen

### Solveur le Plus Précis (avec 100% succès)
**SCS** - 0.0003m erreur moyenne

### Performance selon l'Horizon

Impact de la taille du problème (horizon) sur le temps de résolution:

| Solveur | H=100 | H=150 | H=200 |
|---------|--------|--------|--------|
| CLARABEL | 765.1ms | 1150.1ms | 1656.9ms |
| CVXOPT | 10611.7ms | 16656.3ms | 29253.9ms |
| ECOS | 1203.9ms | 1884.6ms | 2870.8ms |
| OSQP | 1265.5ms | 2448.0ms | 2448.4ms |
| SCS | 834.7ms | 1229.9ms | 1921.4ms |

## 5. Recommandations

### Solveur Recommandé: **CLARABEL**

**Justification:**
- Taux de succès: 100%
- Temps de résolution moyen: 1180.6ms
- Précision moyenne: 0.0003m
- Adapté pour contrôle en boucle fermée temps-réel

### Alternative pour Précision Maximale: **SCS**

Si la précision est critique et le temps moins contraignant:
- Erreur moyenne: 0.0003m
- Temps moyen: 1304.0ms

### Paramètres de Configuration Optimaux

```python
solver = cp.CLARABEL
params = {'verbose': False, 'max_iter': 100000, 'tol_gap_abs': 0.0001, 'tol_gap_rel': 0.0001}
```

## 6. Conclusions

Ce benchmark démontre que le choix du solveur a un impact significatif sur:
- La rapidité de résolution (variation jusqu'à 10x)
- La robustesse (certains solveurs échouent sur grands horizons)
- La précision (différences minimes pour solveurs réussis)

Pour le contrôle MPC du drone, un solveur rapide et robuste est essentiel pour permettre le re-calcul fréquent de la trajectoire en boucle fermée.

---
*Rapport généré automatiquement le 2025-10-25 à 08:21:43*

---

## Annexe: Détails Techniques

### Configuration du Problème

Le problème d'optimisation MPC est formulé comme un programme quadratique (QP) avec:
- **Variables**: 404 (pour horizon=100) à 804 (pour horizon=200)
- **Contraintes**: ~600-1200 contraintes linéaires
- **Type**: Problème convexe quadratique avec contraintes d'inégalité

### Critères d'Évaluation

1. **Rapidité**: Temps total de résolution (ms)
2. **Robustesse**: Pourcentage de problèmes résolus avec succès
3. **Précision**: Erreur de position finale de la charge (m)
4. **Scalabilité**: Variation du temps selon la taille du problème

### Méthodologie

- Chaque test répété 3 fois pour fiabilité statistique
- 4 cas de test différents (horizons 100-200, conditions initiales variées)
- Mêmes paramètres de tolérance pour tous les solveurs (1e-4)
- Limites d'itérations augmentées (100000) pour comparaison équitable

### Recommandations d'Utilisation

**Pour l'implémentation actuelle:**
```python
# Dans drone_control_1d.py, ligne 255-257
problem.solve(solver=cp.CLARABEL, verbose=False, max_iter=100000, 
             tol_gap_abs=1e-4, tol_gap_rel=1e-4)
```

**Pour contrôle en temps réel:**
- Utiliser CLARABEL comme solveur principal
- Horizon adaptatif: 50-100 étapes pour problèmes rapides
- Re-calcul toutes les 1-2 secondes en boucle fermée
- Monitoring du temps de calcul pour détecter problèmes

**En cas d'échec de CLARABEL:**
- Fallback vers SCS (très fiable, légèrement plus lent)
- Éviter CVXOPT (trop lent et instable pour ce problème)
