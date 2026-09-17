#!/usr/bin/env python3

import rospy
import socket
import json
import os
import time
from scripts.adam import ADAM

# 1. Configuración del robot ADAM
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path, "..", "models", "robot", "rb1_base_description", "robots", "robotDummy.urdf")
adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=True)

# 2. Configuración del Socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target_address = ('127.0.0.1', 5005)

rospy.loginfo("🟢 Enviando pose del efector final usando adam.arm_kinematics...")

# Variables para control de impresión por consola
last_print_time = 0.0
PRINT_INTERVAL = 1.0  # Cada 1 segundo

# Bucle principal
while not rospy.is_shutdown():
    try:
        # 3. Obtener pose directamente con el módulo ArmsKinematics
        pos, quat = adam.arm_kinematics.get_arm_link_pose(arm='right', target_link='ee')

        # 4. Empaquetar en JSON
        pose_data = {
            "timestamp": rospy.get_time(),
            "ee_position": list(pos),      # [x, y, z]
            "ee_orientation": list(quat)   # [x, y, z, w]
        }

        # 5. Enviar a MuJoCo por UDP
        sock.sendto(json.dumps(pose_data).encode('utf-8'), target_address)

        # Imprimir telemetría local para debugging
        current_time = time.time()
        if (current_time - last_print_time) >= PRINT_INTERVAL:
            print("\n" + "="*50)
            print("🤖 [PYBULLET / ROS] Telemetría enviada:")
            print(f"📍 Posición EE (XYZ)       : {list(pos)}")
            print(f"🔄 Cuaternión (xyzw PyBullet): {list(quat)}")
            print("="*50)
            last_print_time = current_time

        # Avanzar el paso de simulación
        adam.step()

    except rospy.ROSInterruptException:
        break