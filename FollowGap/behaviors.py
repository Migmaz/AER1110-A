
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

import numpy as np

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

     # 1. Vérifier danger en priorité
    cmd = emergency_stop(scan)

    if cmd is not None:
        return cmd 

    # 2. Sinon comportement normal
    fg = FollowGap()
    best_i, theta, scan_processed = fg.compute(scan, yaw)

    if best_i is None or theta is None:
        return escape(scan, theta)

    return {
        "linear": 0.5,
        "angular": theta
    }


def circle_object(scan, desired_distance=1.0):
    """
    Fait tourner le robot autour d'un objet à distance constante.
    
    Args:
        scan (np.ndarray Nx2): Scan LiDAR [distance, angle]
        desired_distance (float): Distance cible à maintenir (m)
    
    Returns:
        cmd_vel (dict): Commande moteur {"linear": ..., "angular": ...}
    """
    
    distances = scan[:, 0]
    angles = scan[:, 1]

    # Distance minimale et angle correspondant
    min_dist_idx = np.argmin(distances)
    min_dist = distances[min_dist_idx]
    theta_obj = angles[min_dist_idx]

    # Gains simples pour contrôler vitesse
    K_linear = 0.3
    K_angular = 0.5

    # Correction linéaire pour garder distance désirée
    linear = K_linear * (min_dist - desired_distance)

    # Correction angulaire pour tourner autour
    # Si l'objet est à gauche → tourner dans le sens horaire, etc.
    # On peut juste tourner autour avec une valeur fixe
    angular = K_angular * np.sign(theta_obj)  # positive = tourner gauche, negative = droite

    # Limiter vitesses
    linear = np.clip(linear, -0.3, 0.3) #la vitesse linéaire (avance/recul) est forcée à rester entre -0.3 m/s (recule) et +0.3 m/s (avance).
    angular = np.clip(angular, -0.5, 0.5) #la vitesse angulaire (rotation) est limitée entre -0.5 rad/s et +0.5 rad/s (rotation horaire ou antihoraire).

    return {"linear": linear, "angular": angular}


def escape(scan, theta):
    """
    Comportement d'urgence si le robot est bloqué.

    Returns:
        cmd_vel (dict):
            Commande pour tourner ou reculer.
    """
    fg = FollowGap()
    scan_processed = fg.preprocess_lidar(scan)

    distances = scan_processed[:, 0]
    min_dist = np.min(distances)

    # 1. Trop proche → reculer
    if min_dist < 0.35:
        return {"linear": -0.2, "angular": 0.3}

    # 2. Pas de direction → tourner doucement
    if theta is None:
        return {"linear": 0.0, "angular": 0.5}
    # 3. Direction valide → tourner vers ouverture
    return {"linear": 0.0, "angular": theta}
    
def emergency_stop(scan, distance_critique=0.15):
    """
    Détecte une situation dangereuse à partir du scan LiDAR.

    Args:
        scan (np.ndarray): Scan LiDAR Nx2 [distance, angle]
        distance_critique (float): distance minimale sécuritaire (m)

    Returns:
        dict ou None:
            {"linear": 0.0, "angular": 0.0} si danger
            None sinon
    """

    fg = FollowGap()
    scan_processed = fg.preprocess_lidar(scan)

    distances = scan_processed[:, 0]

    # Détection du danger
    danger = False

    for d in distances:
        if d < distance_critique:
            danger = True
            break

    # Action
    if danger:
        return {"linear": 0.0, "angular": 0.0}

    return None

            #Si tu vois ce code alors tut marche