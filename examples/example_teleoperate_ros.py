from scripts.adam import ADAM
import pybullet as p
import math

# Create ADAM instance
adam = ADAM(end_effector="inspire_hands", use_realtime=True, use_fixed_base=False, use_ros=True)

adam.wait(1)

while True:

    # Teleoperate via ROS
    adam.ros.teleop_real_base()
    
    adam.step()
