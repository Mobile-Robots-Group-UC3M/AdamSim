#!/usr/bin/env python3

import rospy
import pybullet as p
from scripts.adam import ADAM
import os
import time

# URDF robot path and create ADAM instance
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","paquetes_simulacion", "rb1_base_description", "robots", "robotDummy.urdf")
adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=True)

# Print robot information
#adam.print_robot_info()


"""print('Initializing ADAMSim')
adam.ros.arm_real_to_sim()
adam.ros.wait(5)
print('ADAMSim ready!')"""

# Simulation loop
while not rospy.is_shutdown():
    try:

        # INSERT YOUR SIMULATION CODE HERE

        # Move simulated arm as real arm
        #adam.ros.arm_real_to_sim()

        # Move hand pose

        


        adam.step()

    except rospy.ROSInterruptException:
        break