"""
MAIN
Chef d’orchestre du projet.

Rôle :
- Boucle principale du robot
- Lit les données capteurs (sensor.py)
- Demande à control.py quel comportement utiliser
- Exécute le behavior correspondant (behaviors.py)
- Envoie la commande aux moteurs (actuation.py)
"""

# =================================================
# Importation de librairy générale
# =================================================
import time
import numpy as np


# =================================================
# Importation de librairy Périphérique
# =================================================
import board
import busio
from digitalio import DigitalInOut

from adafruit_pca9685 import PCA9685
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import(
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_ROTATION_VECTOR,
)


# =================================================
# Importation de fonctions
# =================================================

from FollowGap import FTG, FSM, tool, sensor

# =================================================
# Initialisation des capteurs
# =================================================

# IMU
i2c_IMU = busio.I2C(board.SCL, board.SDA, frequency=100000)
bno = BNO08X_I2C(i2c_IMU)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)


# PWM
i2c_PWM = board.I2C()
pca = PCA9685(i2c_PWM)
pca.frequency = 60

# =================================================
# Boucle principale
# =================================================
while True:
     # 1. Lire capteurs
    a, yaw = sensor.IMU(bno)

    # DEBUG
    print(f"Acc: {a.flatten()} | Yaw: {yaw:.2f}")

    time.sleep(0.01)

