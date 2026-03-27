"""
ACTUATION
Contrôle des moteurs.

Rôle :
- Convertir cmd_vel (linear, angular) en signaux moteurs
- Envoyer les commandes via PWM (ex: PCA9685)

Entrée :
- cmd_vel = {"linear": x, "angular": y}

Sortie :
- signaux moteurs
"""