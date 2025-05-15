from adam import ADAM
import pybullet as p
import time

# URDF robot path
robot_urdf_path = "/home/gonzalo/Desktop/AdamBulletSimualator/paquetes_simulacion/rb1_base_description/robots/robotDummy.urdf"

# Create ADAM instanceSliders
adam = ADAM(robot_urdf_path, useSimulation=True, useRealTimeSimulation=False, used_fixed_base=True)

# Print robot information
adam.print_robot_info()


# INSERT YOUR INITIALIZATION CODE HERE

adam.teleop.create_sliders()

pose = [[0.3422611951828003, 0.020709218457341194, 1.4573277235031128], [0.68709796667099, 0.11739179491996765, 0.6064856052398682, -0.382479727268219]]


# Main simulation loop
while True:

    if (adam.useSimulation and not adam.useRealTimeSimulation):
        p.stepSimulation()

    # INSERT YOUR SIMULATION CODE HERE

    # Example: Move the right arm to a specific position
    adam.arm_kinematics.move_arm_to_pose('right', pose, target_link='ee', accurate=False, visualize=True)
    current_pose = adam.arm_kinematics.get_arm_link_pose('right', target_link='ee')
    print("Current Pose:", current_pose)
    
    reached = adam.arm_kinematics.check_reached('right', target_pose=pose, target_link='ee')
    print("Reached:", reached)

    if not adam.useRealTimeSimulation:
        time.sleep(adam.t)