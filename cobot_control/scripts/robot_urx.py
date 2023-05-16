import urx
import numpy as np
from ikpy.chain import Chain
from ikpy.link import URDFLink

# Crear una conexión con el robot UR5e
robot = urx.Robot("192.168.0.102")  # Reemplaza la dirección IP por la del robot UR5e

# Definir la cadena cinemática del robot UR5e
links = [
    URDFLink("base_link", [0, 0, 0], [0, 0, 0], [False, False, False]),
    URDFLink("shoulder_link", [0, 0, 0.089159], [0, 0, 0], [False, False, False]),
    URDFLink("upper_arm_link", [0, 0.425, 0], [0, 0, 0], [False, False, False]),
    URDFLink("forearm_link", [0, -0.39225, 0], [0, 0, 0], [False, False, False]),
    URDFLink("wrist_1_link", [0, 0, 0.10915], [0, 0, 0], [False, False, False]),
    URDFLink("wrist_2_link", [0, 0, 0.09465], [0, 0, 0], [False, False, False]),
    URDFLink("wrist_3_link", [0, 0, 0.0823], [0, 0, 0], [False, False, False]),
    URDFLink("ee_link", [0, 0, 0.081], [0, 0, 0], [False, False, False])
]

# Definir la cadena cinemática del robot UR5e con los enlaces correspondientes
chain = Chain(links)

# Definir la posición objetivo
target_position = [0.1324, -0.4921, 0.487]

# Calcular las variables articulares correspondientes a la posición objetivo
joint_angles = chain.inverse_kinematics(target_position)
print("Joint angles:", joint_angles)

# Cerrar la conexión con el robot
robot.close()
