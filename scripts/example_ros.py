#!/usr/bin/env python3

import rospy
import pybullet as p
from adam import ADAM

# URDF robot path and create ADAM instance
robot_urdf_path = "/home/adrian/Escritorio/ImitationLearning/SimuladorADAM/Adam_sim/paquetes_simulacion/rb1_base_description/robots/robotDummy.urdf"
adam = ADAM(robot_urdf_path, useSimulation=False, useRealTimeSimulation=True, used_fixed_base=False)

# Print robot information
adam.print_robot_info()
adam.teleop.create_sliders()

# ROS rate
rate = rospy.Rate(120)

# Simulation loop
while not rospy.is_shutdown():
    try:

        # INSERT YOUR SIMULATION CODE HERE

        # Move simulated arm as real arm
        #adam.ros.arm_real_to_sim()
        while True:
            adam.ros.teleop_real_base()
            # Simulación
            if not adam.useRealTimeSimulation:
                p.stepSimulation()
                rospy.sleep(adam.t)
            else:
                p.stepSimulation()
                rate.sleep()

    except rospy.ROSInterruptException:
        break