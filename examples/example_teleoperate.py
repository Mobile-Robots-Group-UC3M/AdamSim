import os
from scripts.adam import ADAM
import pybullet as p




# Create ADAM instance
adam = ADAM(useRealTimeSimulation=True, used_fixed_base=False, use_ros=False)

# Print robot information
adam.print_robot_info(save=True, filename='robot_info.json')


while True:
    # adam.sensors.get_rgbd_image_from_link()
    adam.teleop.teleoperate_base(debug=True)

    adam.step()