from adam import ADAM
import pybullet as p
import time

# URDF robot path
robot_urdf_path = "/home/gonzalo/Desktop/AdamBulletSimualator/paquetes_simulacion/rb1_base_description/robots/robotDummy.urdf"

# Robot collision body STL path
robot_stl_path = "/home/gonzalo/Desktop/AdamBulletSimualator/paquetes_simulacion/rb1_base_description/meshes/others/adam_model.stl"

# Create ADAM instance
adam = ADAM(robot_urdf_path, robot_stl_path, useSimulation=True, useRealTimeSimulation=False, used_fixed_base=True)

# Print robot information
adam.print_robot_info()

angles = [0.1,0.2,0.3,0.4,0.5,0.6]

# adam.kinematics.move_arm_joints_to_angles('left', angles)

target_pose = [[0.5, 0.5, 0.5], [0.1, 0.2, 0.3]]
adam.kinematics.move_arm_to_pose('left', target_pose, 'hand')

# Main simulation loop
while True:

    if (adam.useSimulation and adam.useRealTimeSimulation==0):
        p.stepSimulation()

        '''print("Current pose (left):")
        pose = adam.kinematics.get_arm_end_effector_pose('left')
        print(pose)

        print("Current pose (right):")
        pose = adam.kinematics.get_arm_end_effector_pose('right')
        print(pose)

        print("PyKDL pose:")
        pose = adam.kinematics.pykdl.calculate_arm_forward_kinematics('left', angles)
        print(pose)'''

        pose = adam.kinematics.get_arm_link_pose('left', 'hand')
        print(pose)

    if not adam.useRealTimeSimulation:
        time.sleep(adam.t)