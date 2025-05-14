from adam import ADAM
import pybullet as p
import time

# URDF robot path
robot_urdf_path = "/home/gonzalo/Desktop/AdamBulletSimualator/paquetes_simulacion/rb1_base_description/robots/robotDummy.urdf"

# Create ADAM instance
adam = ADAM(robot_urdf_path, useSimulation=True, useRealTimeSimulation=False, used_fixed_base=True)

# Print robot information
adam.print_robot_info()



# Main simulation loop
while True:

    if (adam.useSimulation and not adam.useRealTimeSimulation):
        p.stepSimulation()

    # INSERT YOUR CODE HERE
    collision = adam.detect_autocollisions()
    print("Collision detected:", collision)

    if not adam.useRealTimeSimulation:
        time.sleep(adam.t)