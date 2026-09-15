#!/usr/bin/env python3

import rospy
from scripts.adam import ADAM
import os

# Create ADAM instance
adam = ADAM(end_effector="inspire_hands", use_realtime=True, use_fixed_base=True, use_ros=True)

# Simulation loop
while not rospy.is_shutdown():
    try:

        # Move hand pose
        adam.step()

    except rospy.ROSInterruptException:
        break