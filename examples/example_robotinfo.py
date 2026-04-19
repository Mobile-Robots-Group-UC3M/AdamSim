import os
from scripts.adam import ADAM
import pybullet as p




# Create ADAM instance
adam = ADAM(useRealTimeSimulation=True, used_fixed_base=True, use_ros=False)

# Print robot information
adam.print_robot_info(save=True, filename='robot_info.json')


while True:

    # Enter code here

    adam.step()