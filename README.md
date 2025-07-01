# Régulateur PID en C++

## Présentation

Ce projet implémente un **régulateur PID** de base en C++ pour contrôler la vitesse d’un moteur.  
Le contrôleur PID (Proportionnel-Intégral-Dérivatif) est un algorithme de régulation en boucle largement utilisé en robotique et en automatique pour obtenir un contrôle stable et précis

---

## Fonctionnement

### Composantes PID

- **Proportionnel (P)** : réagit proportionnellement à l’erreur actuelle.
- **Intégral (I)** : cumule les erreurs passées pour éliminer les écarts persistants.
- **Dérivatif (D)** : anticipe les tendances futures de l’erreur pour amortir la réponse.

### Logique de l'algorithme

1. Mesurer la valeur actuelle (ex. : vitesse moteur).
2. Calculer l’erreur : `erreur = consigne - mesure`.
3. Intégrer l’erreur sur le temps.
4. Calculer la dérivée de l’erreur.
5. Combiner les trois termes (P, I, D) avec leurs gains (kp, ki, kd).
6. Appliquer la commande résultante au système.
7. Répéter périodiquement.

---

## Explication du code

- **Classe `PIDController`** :  
  Contient les paramètres PID, l’état interne et le calcul via `compute(consigne, mesure)`.

- **Gestion du temps** :  
  Utilise `std::chrono::steady_clock` pour calculer un `dt` précis entre appels successifs.

- **Windup intégral** :  
  Cette version n’intègre pas de protection contre l’accumulation excessive du terme intégral (à prévoir en cas réel).

- **Exemple `main()`** :  
  Simule une régulation de vitesse moteur avec réponse amortie et délais.

---

## Compilation et exécution

Assurez-vous d’avoir un compilateur compatible C++11 :

```bash
g++ -std=c++11 -o pid_controller pid_controller.cpp
./pid_controller
```

## Extensions possibles

- Ajouter une limitation du terme intégral (anti-windup)
- Ajouter des bornes de sortie (saturation d’actionneurs)
- Intégrer des capteurs/actionneurs réels (via ROS, par exemple)
- Automatiser le réglage des gains PID

## Références 

- Wikipédia – Contrôleur PID
- Ouvrages de systèmes de contrôle modernes
- Cours et tutoriels en robotique et automatisme
