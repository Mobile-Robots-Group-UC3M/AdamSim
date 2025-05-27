#!/usr/bin/env python3

import rospy
import pybullet as p
from adam import ADAM
import os
import time

# URDF robot path and create ADAM instance
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","paquetes_simulacion", "rb1_base_description", "robots", "robotDummy.urdf")
adam = ADAM(robot_urdf_path, useSimulation=False, useRealTimeSimulation=True, used_fixed_base=True)

# Print robot information
#adam.print_robot_info()
adam.teleop.create_sliders()

# ROS rate
rate = rospy.Rate(120)

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

        adam.ros.wait(3)

        adam.ros.call_set_angle('right', [1000, 0, 1000, 1000, 1000, 1000])
        adam.ros.call_set_angle('left', [1000, 1000, 1000, 1000, 1000, 1000])


        # Move to another hand pose
        adam.ros.wait(3)
        adam.ros.call_set_angle('right', [500, 500, 500, 500, 500, 0])
        adam.ros.call_set_angle('left', [500, 500, 500, 500, 500, 0])
        




        # Simulación
        if not adam.useRealTimeSimulation:
            p.stepSimulation()
            rospy.sleep(adam.t)
        else:
            p.stepSimulation()
            rate.sleep()

    except rospy.ROSInterruptException:
        break