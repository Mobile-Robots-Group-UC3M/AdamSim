import os
from scripts.adam import ADAM
import pybullet as p
adam = ADAM(useRealTimeSimulation=True, used_fixed_base=True, use_ros=False)
adam.print_robot_info(save=True, filename='robot_info.json')
while True:
    adam.step()


