#!/usr/bin/env python3
from ur5e_igm import *



home = [pi/2,-pi/2,pi/2,-pi/2,-pi/2,0]

orientation = [0.001,-3.137,0.002]

d_pose = [0.1324,-0.4921,0.487]

d_pose += orientation

print([d_pose])

print(inv_kin(d_pose,home,o_unit='d'))


import urx
import numpy as np

# Conexión al robot
robot = urx.Robot("192.168.0.102")  # Reemplaza con la IP del robot UR5e

# Obtener la pose actual del robot
current_pose = robot.get_pose()
print(current_pose)

# Obtener la matriz de transformación homogénea (IGM)
igm = 

# Imprimir la matriz de transformación homogénea (IGM)
print("Matriz de Transformación Homogénea (IGM):")
print(igm)

# Desconexión del robot
robot.close()



