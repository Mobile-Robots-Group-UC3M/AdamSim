from adam import ADAM
import pybullet as p
import time

# URDF robot path
robot_urdf_path = "/home/gonzalo/Projects/AdamBulletSimualator/paquetes_simulacion/rb1_base_description/robots/robot.urdf"

# Robot collision body STL path
robot_stl_path = "/home/gonzalo/Projects/AdamBulletSimualator/paquetes_simulacion/rb1_base_description/meshes/others/adam_model.stl"

# Create ADAM instance
adam = ADAM(robot_urdf_path, robot_stl_path, useSimulation=True, useRealTimeSimulation=False, used_fixed_base=True)

# Main simulation loop
while True:

    if (adam.useSimulation and adam.useRealTimeSimulation==0):
        p.stepSimulation()

    


    if not adam.useRealTimeSimulation:
        time.sleep(adam.t)