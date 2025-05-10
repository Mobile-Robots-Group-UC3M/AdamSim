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

        #pose = adam.kinematics.get_arm_link_pose('left', 'hand')
        #print(pose)

        rgb = adam.sensors.get_rgb_image_from_link(72, width=640, height=480, fov=80, near=0.01, far=5.0)
        

    if not adam.useRealTimeSimulation:
        time.sleep(adam.t)