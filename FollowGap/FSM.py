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
from behaviors import emergency_stop

from behaviors import circle_object

from behaviors import navigate

from mapping import update_position

from mapping import save_path

import numpy as np

def update_state(scan, yaw, robot_pos, current_state, goal_pos, path):
    """
    Met à jour l'état du robot (FSM) et gère le chemin.

    Args:
        scan (list or np.ndarray): données LiDAR
        yaw (float): orientation actuelle
        robot_pos (tuple): position actuelle (x, y)
        current_state (str): état actuel
        goal_pos (tuple): position du goal (x, y)
        path (list): trajectoire déjà enregistrée

    Returns:
        dict: {
            "cmd": dict ou None,         # commande moteur
            "new_state": str,            # nouvel état
        }
    """

    # 🔹 Situation d'urgence
    if emergency_stop(scan) is not None:
        return {"cmd": {"linear": 0.0, "angular": 0.0},
                "new_state": "EMERGENCY_STOP",}

    # 🔹 Robot atteint le goal
    distance_to_goal = np.linalg.norm(np.array(goal_pos) - np.array(robot_pos))
    if distance_to_goal < 1 and current_state != "SCAN":
        path = save_path(path, *robot_pos)
        return {"cmd": None, "new_state": "SCAN", "path": path}

    # 🔹 Comportement dans l'état SCAN
    if current_state == "SCAN":
        cmd_vel = circle_object(scan, desired_distance=1.0)
        # Quand le scan est terminé, on peut retourner à NAVIGATE ou RETURN
        return {"cmd": cmd_vel, "new_state": "RETURN", "path": path}

    # 🔹 Comportement selon l'état actuel
    if current_state == "ESCAPE":
        cmd_vel = navigate(scan, yaw)
        return {"cmd": cmd_vel, "new_state": "NAVIGATE"}

    elif current_state == "RETURN":
        cmd_vel = navigate(scan, yaw)
        return {"cmd": cmd_vel, "new_state": "RETURN"}

    else:  # NAVIGATE
        cmd_vel = navigate(scan, yaw)
        return {"cmd": cmd_vel, "new_state": "NAVIGATE"}