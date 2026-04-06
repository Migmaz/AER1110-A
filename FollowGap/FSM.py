"""
CONTROL (FSM)
Gestion des états du robot.

Rôle :
- Décider quel comportement utiliser
- Gérer les transitions entre états

Exemples d’états :
- NAVIGATE
- SCAN
- ESCAPE
- RETURN

Entrées : données capteurs
Sortie : nom du behavior à exécuter
"""

# Ludo : À modifier les schémas de fonctions car peu clair et pas adapter

import numpy as np

theta_history = []
prev_dist_to_goal = None
stuck_counter = 0

def update_state(scan_true, theta_goal, prev_state, robot_pos, goal_pos):

    global theta_history, prev_dist_to_goal, stuck_counter

    distances = scan_true[:, 0]


    # État STOP
    if np.min(distances) <= 0.15:
        return "STOP"

    # --- État CUL-DE-SAC ---
    dist_to_goal = np.linalg.norm(np.array(goal_pos) - np.array(robot_pos))

    if prev_dist_to_goal is None:
        prev_dist_to_goal = dist_to_goal

    progress = prev_dist_to_goal - dist_to_goal
    prev_dist_to_goal = dist_to_goal

    if theta_goal is not None:
        theta_history.append(theta_goal)

    if len(theta_history) > 10:
        theta_history.pop(0)

    theta_var = np.var(theta_history) if len(theta_history) > 1 else 0

    free_ratio = np.sum(distances > 1.0) / len(distances)

    if (
        progress < 0.01
        and theta_var > 0.3
        and free_ratio < 0.3
    ):
        stuck_counter += 1
    else:
        stuck_counter = 0

    if stuck_counter > 5:
        return "CUL-DE-SAC"
    

    # État ESCAPE
    if prev_state == "CUL-DE-SAC" or prev_state == "STOP":
        return "ESCAPE"


    # État SCAN
    if dist_to_goal <= 1:
        return "SCAN"

    # État RETOUR_BASE
    if prev_state == "SCAN":

        return "RETOUR_BASE"

    return "NAVIGATE"