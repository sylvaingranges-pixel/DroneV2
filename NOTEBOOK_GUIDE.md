# Guide du Notebook Interactif de Réglage des Paramètres

## Vue d'ensemble

Le notebook `interactive_drone_tuning.ipynb` fournit une interface interactive pour régler les paramètres du contrôleur MPC (Model Predictive Control) du système drone-charge suspendue en 1D.

## Lancement

```bash
jupyter notebook interactive_drone_tuning.ipynb
```

Ou avec JupyterLab:
```bash
jupyter lab interactive_drone_tuning.ipynb
```

## Structure du Notebook

### 1. Section des Paramètres (Cellule Unique)

Tous les paramètres sont regroupés dans une seule section pour faciliter le réglage:

#### Paramètres de Simulation
- **TS**: Pas de temps (secondes) - défaut: 0.1s
- **HORIZON**: Horizon de prédiction MPC (nombre d'étapes) - défaut: 50

#### Conditions Initiales
- **X_DRONE_INIT**: Position initiale du drone (m) - défaut: 0.0 m
- **V_DRONE_INIT**: Vitesse initiale du drone (m/s) - défaut: 10.0 m/s
- **THETA_INIT**: Angle initial de la charge (degrés) - défaut: -30° (derrière le drone)
- **OMEGA_INIT**: Vitesse angulaire initiale (rad/s) - défaut: 0.0 rad/s

#### Consigne
- **X_LOAD_TARGET**: Position cible de la charge (m) - défaut: 10.0 m

#### Poids de la Fonction de Coût (États)
- **Q_XD**: Poids sur la position du drone - défaut: 10.0
- **Q_VD**: Poids sur la vitesse du drone - défaut: 20.0
- **Q_THETA**: Poids sur l'angle de la charge - défaut: 500.0
- **Q_OMEGA**: Poids sur la vitesse angulaire - défaut: 100.0
- **Q_LOAD**: Poids sur la position de la charge - défaut: 1000.0

#### Poids Terminaux (Multiplicateurs)
- **QF_XD_MULT**: Multiplicateur pour le poids terminal position drone - défaut: 50.0
- **QF_VD_MULT**: Multiplicateur pour le poids terminal vitesse drone - défaut: 50.0
- **QF_THETA_MULT**: Multiplicateur pour le poids terminal angle - défaut: 50.0
- **QF_OMEGA_MULT**: Multiplicateur pour le poids terminal vitesse angulaire - défaut: 50.0
- **QF_LOAD_MULT**: Multiplicateur pour le poids terminal position charge - défaut: 100.0

#### Autres Paramètres
- **R_CONTROL**: Poids sur l'effort de contrôle (pénalité accélération) - défaut: 2.0

### 2. Initialisation du Système

Cette section crée le système drone-charge, linéarise la dynamique autour de l'équilibre et discrétise le modèle.

### 3. Simulation et Visualisation

La fonction `run_simulation_and_plot()` exécute:
1. Résolution du problème MPC en boucle ouverte
2. Simulation du système linéaire avec la commande optimale
3. Simulation du système non-linéaire avec la commande optimale
4. Génération de 6 graphiques comparant les deux systèmes

### 4. Interface Interactive

Des widgets interactifs permettent d'ajuster tous les paramètres en temps réel:
- Sliders pour les valeurs numériques
- Sliders logarithmiques pour les poids de la fonction de coût
- Bouton "Mettre à jour et simuler" pour relancer la simulation

## Scénario de Test par Défaut

Le notebook implémente le scénario spécifié dans le problème:

### Conditions Initiales
- Position du drone: **0 m**
- Vitesse du drone: **10 m/s**
- Angle de la charge: **-30°** (derrière le drone)
- Vitesse angulaire: **0 rad/s**
- Position initiale de la charge: **-9.5 m** (calculée automatiquement)

### Consigne
- Position cible de la charge: **10 m**

### Résultat Attendu
Le MPC calcule une trajectoire optimale en boucle ouverte pour amener la charge de -9.5 m à 10 m tout en:
- Minimisant les oscillations (angle du câble)
- Réduisant l'effort de contrôle
- Stabilisant le système en fin de trajectoire

## Sorties

### Graphiques Générés

Le notebook génère un graphique composite avec 6 sous-graphiques:

1. **Position de la charge**: Compare la réponse linéaire (vert) et non-linéaire (rouge) par rapport à la cible (noir pointillé)
2. **Position du drone**: Trajectoire du drone pour les deux systèmes
3. **Vitesse du drone**: Évolution de la vitesse dans le temps
4. **Angle du câble**: Oscillations de la charge (en degrés)
5. **Vitesse angulaire**: Dérivée de l'angle
6. **Commande de contrôle**: Accélération du drone appliquée (bleu)

### Informations Textuelles

Le notebook affiche:
- État initial complet du système
- Temps de résolution de l'optimisation MPC
- Erreurs finales pour les systèmes linéaire et non-linéaire
- Position finale de la charge vs cible

## Exemple de Sortie

Lorsque vous exécutez le notebook avec les paramètres par défaut, vous obtiendrez un graphique composite montrant:

- **Ligne verte**: Réponse du système linéaire
- **Ligne rouge**: Réponse du système non-linéaire
- **Ligne noire pointillée**: Valeur cible

Le graphique comprend 6 sous-graphiques montrant la position de la charge, la position du drone, la vitesse, l'angle du câble, la vitesse angulaire et la commande de contrôle.

*Note: Exécutez le notebook pour générer les graphiques avec vos propres paramètres*

## Interprétation des Résultats

### Système Linéaire (Ligne Verte)
- Utilise une approximation aux petits angles: sin(θ) ≈ θ
- Valide pour des angles < 15-20°
- Plus rapide à simuler
- Erreur finale typique: < 0.1 m

### Système Non-Linéaire (Ligne Rouge)
- Modèle complet avec résistance de l'air quadratique
- Capture la dynamique réelle du système
- Plus précis pour de grands angles
- Peut diverger significativement du modèle linéaire pour le scénario avec -30° initial

### Divergence Observée
Pour le scénario par défaut (-30°), on observe:
- Le système linéaire atteint la cible avec ~0.06 m d'erreur
- Le système non-linéaire peut avoir une erreur importante (~13 m) car l'hypothèse de petits angles n'est pas valide

**Solution**: Réduire l'angle initial ou augmenter les poids Q_THETA et Q_LOAD pour forcer une trajectoire plus conservatrice.

## Guide d'Utilisation

### Réglage des Paramètres

1. **Pour améliorer le suivi de la position de la charge**:
   - Augmenter Q_LOAD (ex: 5000-10000)
   - Augmenter QF_LOAD_MULT (ex: 200-500)

2. **Pour réduire les oscillations**:
   - Augmenter Q_THETA (ex: 1000-5000)
   - Augmenter Q_OMEGA (ex: 500-1000)

3. **Pour des commandes plus douces**:
   - Augmenter R_CONTROL (ex: 10-50)
   - Note: cela peut ralentir la convergence

4. **Pour améliorer la convergence finale**:
   - Augmenter tous les multiplicateurs terminaux (QF_*)
   - Augmenter HORIZON (ex: 70-100)

5. **Pour tester différentes conditions**:
   - Modifier les conditions initiales (sliders interactifs)
   - Essayer différentes positions cibles
   - Tester des angles initiaux variés (-45° à +45°)

### Workflow Recommandé

1. Lancer le notebook avec les paramètres par défaut
2. Observer les graphiques et les erreurs
3. Identifier les aspects à améliorer (oscillations, convergence, effort)
4. Ajuster 1-2 paramètres à la fois
5. Cliquer sur "Mettre à jour et simuler"
6. Comparer avec le résultat précédent
7. Itérer jusqu'à obtenir le comportement désiré

## Limites et Considérations

### Limites du Modèle Linéaire
- Valide seulement pour des petits angles (< 15-20°)
- Ignore la résistance de l'air non-linéaire
- Peut sous-estimer les oscillations pour de grandes perturbations

### Contraintes du MPC
- Accélération maximale: ±15 m/s²
- Le solveur CLARABEL converge typiquement en 300-500 ms
- Des horizons trop longs (>100) peuvent ralentir la simulation

### Conditions d'Optimalité
- Le MPC optimise pour le modèle linéaire
- Les performances sur le système non-linéaire peuvent être dégradées
- Pour de meilleurs résultats réels, considérer un MPC non-linéaire (NMPC)

## Dépannage

### Le solveur ne converge pas
- Réduire HORIZON
- Réduire les poids (Q_*, R_CONTROL)
- Vérifier que les conditions initiales sont réalistes

### Erreur importante sur le système non-linéaire
- L'angle initial est probablement trop grand
- Réduire THETA_INIT à ±15° maximum
- Augmenter Q_THETA pour forcer le système à rester près de la verticale

### Oscillations importantes
- Augmenter Q_THETA et Q_OMEGA
- Augmenter les multiplicateurs terminaux
- Augmenter HORIZON

### Commandes trop agressives
- Augmenter R_CONTROL
- Réduire Q_LOAD légèrement
- Augmenter HORIZON pour plus de prévoyance

## Références

- `drone_control_1d.py`: Module principal avec les classes DroneSystem et MPCController
- `README.md`: Documentation générale du projet
- `CLOSED_LOOP_MPC_DOCUMENTATION.md`: Documentation du MPC en boucle fermée
- `RAPPORT_COMPARATIF_SOLVEURS.md`: Benchmark des solveurs d'optimisation
