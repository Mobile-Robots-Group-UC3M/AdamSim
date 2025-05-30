import rospy
import pybullet as p
from scripts.adam import ADAM
import os
import pandas as pd
import ast
import numpy as np
# URDF robot path and create ADAM instance
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","models","robot", "rb1_base_description", "robots", "robotDummy.urdf")
adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=True,use_ros=True)



# Simulation loop
data_left =[]
data_right =[]
try:
    adam.wait(0.1)
    print("Recollection of data...")
    while not rospy.is_shutdown():
        
        data_right.append(adam.ros.read_arm_joint_states(arm='right',mode='real'))
        data_left.append(adam.ros.read_arm_joint_states(arm='left',mode='real'))

        adam.step()
        

except:
    print("Exception cogida")
finally:
    print("Data collected")
    num_joints = len(data_right[0])
    headers = [f"joint_{i}" for i in range(num_joints)]

    # Crear DataFrame y guardar en CSV
    """ df = pd.DataFrame(data_right, columns=headers)
    df.to_csv("right.csv", index=False) """
    
    df = pd.DataFrame(data_left, columns=headers)
    df.to_csv("left3.csv", index=False)

