
"""
BEHAVIORS
Comportements du robot.

Rôle :
- Combiner les fonctions de navigation pour produire des actions
- Générer les commandes moteur (linear + angular)

Contient :
- navigate() : déplacement principal (évitement + cible)
- circle_object() : tourner autour de l’objet
- escape() : sortir d’un blocage

Entrées : scan, yaw
Sorties : cmd_vel = {"linear": x, "angular": y}
"""
# Ludo : À modifier les schémas de fonctions car peu clair et pas adapter

from FTG import FollowGap #Ramène la classe FollowGap
import motor
def navigate(scan, yaw, k=0.4):
    """
    Comportement principal : navigation vers la cible avec évitement.

    Args:
        scan (list of float):
            Données LiDAR.
        yaw (float):
            Orientation actuelle (rad).
        k (float):
            Poids de l'influence de la cible.

    Returns:
        cmd_vel (dict):
            {
                "linear": float (m/s),
                "angular": float (rad/s)
            }
    """
    pass


def circle_object(scan, desired_distance=1.0):
    """
    Fait tourner le robot autour d'un objet.

    Args:
        scan (list of float):
            Données LiDAR.
        desired_distance (float):
            Distance cible à maintenir (m).

    Returns:
        cmd_vel (dict):
            Commande moteur pour mouvement circulaire.
    """
    pass


def escape():
    """
    Comportement d'urgence si le robot est bloqué.

    Returns:
        cmd_vel (dict):
            Commande pour tourner ou reculer.
    """
    return {"linear": 0.0, "angular": 0.8}

def emergency_stop():

    fg = FollowGap()
    
    scan_processed = fg.preprocess_lidar(scan)

    distances = scan_processed[:, 0]
    
    for i in range (len(distances)):

        if distances[i] < fg.bubble_radius:

            motor.arret = True

            return

    return

            