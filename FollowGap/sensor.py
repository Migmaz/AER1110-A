"""
SENSOR
Gestion des capteurs.
 
Rôle :
- Lire les données du LiDAR (scan)
- Lire les données IMU (yaw / orientation)
- (Optionnel) Estimer la position (odometry)
 
Sorties standard :
- scan : liste de distances
- yaw : orientation du robot
"""
import numpy as np
import busio
import board
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
BNO_REPORT_LINEAR_ACCELERATION,
BNO_REPORT_GAME_ROTATION_VECTOR,
 
)
 
 
 
def get_lidar_scan():
    """
    Retourne un scan LiDAR.
 
    Returns:
        scan (list of float):
            Liste de distances en mètres.
            Taille typique : 180 à 360 points.
            Chaque index correspond à un angle relatif au robot.
            Exemple: scan[0] = -90°, scan[len/2] = 0°, scan[-1] = +90°
    """
    pass
 
 
def IMU(bno) -> tuple[np.ndarray, float]:
 
    try:
        ax, ay, az = bno.linear_acceleration
        qi, qj, qk, qr = bno.quaternion
    except:
        return np.zeros((3,1)), 0.0
 
    yaw = np.atan2(
        2 * (qr * qk + qi * qj),
        1 - 2 * (qj**2 + qk**2)
    )
 
    a = np.array([[ax], [ay], [az]])
 
    return a, yaw
 