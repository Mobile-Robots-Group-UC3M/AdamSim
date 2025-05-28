from adam import ADAM
import pybullet as p
import time
import os

# Load URDF robot path
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","paquetes_simulacion", "rb1_base_description", "robots", "robotDummy.urdf")

print('Loading ADAM...')

# Create ADAM instanceSliders
adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=True, use_ros=False)
# adam.print_robot_info()

print('READY!')

# INSERT YOUR INITIALIZATION CODE HERE

#adam.teleop.create_sliders()

pose0 = [[0.1747363805770874, -0.5190918445587158, 1.5269604921340942], [0.13088582456111908, 0.13149239122867584, -0.6327640414237976, 0.7517901062965393]]
pose1 = [[0.5294547080993652, -0.5355535745620728, 1.1407946348190308], [0.09417623281478882, 0.7259091734886169, -0.05370301008224487, 0.6791926622390747]]
pose2 = [[0.5100053548812866, -0.06656599044799805, 1.1167795658111572], [-0.44266143441200256, 0.6868288516998291, -0.11991514265537262, 0.5638593435287476]]


adam.utils.draw_frame(pose0, axis_length=0.1, line_width=4)
adam.utils.draw_frame(pose1, axis_length=0.1, line_width=4)
adam.utils.draw_frame(pose2, axis_length=0.1, line_width=4)


# Main simulation loop
while True:


    # INSERT YOUR SIMULATION CODE HERE

    # Example: Move the right arm to a specific position
    
    #adam.teleop.apply_slider_values()
    adam.hand_kinematics.move_hand_to_dofs('right', [1000, 1000, 1000, 1000, 1000, 1000])
    adam.hand_kinematics.move_hand_to_dofs('left', [1000, 1000, 1000, 1000, 1000, 1000])
    
    adam.arm_kinematics.move_arm_to_pose('right', pose0, 'hand')
    adam.wait(3)
    adam.arm_kinematics.move_arm_to_pose('right', pose1, 'hand')
    adam.wait(3)
    adam.arm_kinematics.move_arm_to_pose('right', pose2, 'hand')
    adam.wait(3)

    current_pose = adam.arm_kinematics.get_arm_link_pose('right', target_link='hand')
    print("Current Pose:", current_pose)

    current_angles = adam.arm_kinematics.get_arm_joint_angles('right')
    # print("Current Angles:", current_angles)
    
    #adam.hand_kinematics.move_hand_to_dofs('right', [1000,1000,1000,1000,1000,0])
    #adam.hand_kinematics.move_hand_to_dofs('left', [1000,1000,1000,1000,1000,0])

    adam.step()
        