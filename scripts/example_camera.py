from adam import ADAM
import pybullet as p
import time
import os

# URDF robot path
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","paquetes_simulacion", "rb1_base_description", "robots", "robotDummy.urdf")


# Create ADAM instance
adam = ADAM(robot_urdf_path, useSimulation=True, useRealTimeSimulation=False, used_fixed_base=True)

# Print robot information
adam.print_robot_info()

angles = [0.1,0.2,0.3,0.4,0.5,0.6]

# adam.kinematics.move_arm_joints_to_angles('left', angles)

target_pose = [[0.5, 0.5, 0.5], [0.1, 0.2, 0.3]]
adam.arm_kinematics.move_arm_to_pose('left', target_pose, 'hand')

camera_angle = 0
adam.sensors.move_camera_angle(camera_angle)
t = 0

# Main simulation loop
while True:

    if (adam.useSimulation and not adam.useRealTimeSimulation):
        p.stepSimulation()

    #pose = adam.kinematics.get_arm_link_pose('left', 'hand')
    #print(pose)

    keys = p.getKeyboardEvents()

    # Elevator Control (E key)
    if ord('o') in keys and keys[ord('o')] & p.KEY_WAS_TRIGGERED:
        
        camera_angle = -45 - camera_angle
        adam.sensors.move_camera_angle(camera_angle)
        print("Moving camera to angle:", camera_angle)

    rgb, depth = adam.sensors.get_rgbd_image_from_link(width=640, height=480, fov=60, near=0.01, far=5.0)
    print("RGB image:", rgb)
    print("Depth image:", depth)

    if not adam.useRealTimeSimulation:
        time.sleep(adam.t)
        t += adam.t