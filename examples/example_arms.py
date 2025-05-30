import os
from scripts.adam import ADAM


# URDF robot path
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","paquetes_simulacion", "rb1_base_description", "robots", "robotDummy.urdf")

# Create ADAM instance
adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=True, use_ros=False)

# Print robot information
# adam.print_robot_info()

# Define right arm poses
pose_r0 = [[0.1747363805770874, -0.5190918445587158, 1.5269604921340942], [0.13088582456111908, 0.13149239122867584, -0.6327640414237976, 0.7517901062965393]]
pose_r1 = [[0.5294547080993652, -0.5355535745620728, 1.1407946348190308], [0.09417623281478882, 0.7259091734886169, -0.05370301008224487, 0.6791926622390747]]
pose_r2 = [[0.5100053548812866, -0.06656599044799805, 1.1167795658111572], [-0.44266143441200256, 0.6868288516998291, -0.11991514265537262, 0.5638593435287476]]

# Define left arm poses
pose_l0 = [[0.08287623524665833, 0.6437807679176331, 1.4669008255004883], [0.16650719940662384, 0.26198387145996094, -0.7870264053344727, 0.5331315398216248]]
pose_l1 = [[0.5101651549339294, 0.5122880339622498, 1.1548899412155151], [-0.21365556120872498, 0.7626686692237854, -0.5657333731651306, 0.22942005097866058]]
pose_l2 = [[0.47146153450012207, 0.1964891403913498, 1.139336347579956], [-0.08194442838430405, -0.3476749360561371, 0.858890175819397, 0.3670353591442108]]

# Draw right arm frames
adam.utils.draw_frame(pose_r0, axis_length=0.1, line_width=4)
adam.utils.draw_frame(pose_r1, axis_length=0.1, line_width=4)
adam.utils.draw_frame(pose_r2, axis_length=0.1, line_width=4)

# Draw left arm frames
adam.utils.draw_frame(pose_l0, axis_length=0.1, line_width=4)
adam.utils.draw_frame(pose_l1, axis_length=0.1, line_width=4)
adam.utils.draw_frame(pose_l2, axis_length=0.1, line_width=4)

# Initialise hands
adam.hand_kinematics.move_hand_to_dofs('right', [1000, 1000, 1000, 1000, 1000, 0])
adam.hand_kinematics.move_hand_to_dofs('left', [1000, 1000, 1000, 1000, 1000, 0])
adam.wait(1)


while True:

    # INSERT YOUR SIMULATION LOOP CODE HERE

    adam.arm_kinematics.move_arm_to_multiple_poses(arm='both',
                                                   target_link='hand',
                                                   poses_arm1=[pose_r0, pose_r1, pose_r2],
                                                   poses_arm2=[pose_l0, pose_l1, pose_l2])
    
    adam.wait(1)

    # Close hands
    adam.hand_kinematics.move_hand_to_dofs('right', [500, 500, 500, 500, 500, 0])
    adam.hand_kinematics.move_hand_to_dofs('left', [500, 500, 500, 500, 500, 0])
    adam.wait(1)

    # Open hands
    adam.hand_kinematics.move_hand_to_dofs('right', [1000, 1000, 1000, 1000, 1000, 0])
    adam.hand_kinematics.move_hand_to_dofs('left', [1000, 1000, 1000, 1000, 1000, 0])
    adam.wait(1)

    adam.step()