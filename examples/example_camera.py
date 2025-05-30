import os
from scripts.adam import ADAM
import pybullet as p
import pybullet_data


# URDF robot path
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","paquetes_simulacion", "rb1_base_description", "robots", "robotDummy.urdf")

# Create ADAM instance
adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=True, use_ros=False)

# Print robot information
# adam.print_robot_info()

# Load table located in pybullet_data/table
table_path = os.path.join(pybullet_data.getDataPath(), "table", "table.urdf")
p.loadURDF(table_path, basePosition=[0.9, 0, 0], baseOrientation=p.getQuaternionFromEuler([0, 0, 1.54]))

# Load object models
models = [
    "duck_vhacd.urdf",
    "teddy_vhacd.urdf",
    "random_urdfs/000/000.urdf",
    "random_urdfs/001/001.urdf",
    "random_urdfs/002/002.urdf",
    "random_urdfs/003/003.urdf",
]
positions = [
    [0.6, -0.2, 0.75],
    [0.8,  0.1, 0.75],
    [0.55,  0.2, 0.75],
    [0.9, -0.1, 0.75],
    [0.65, 0.3, 0.75],
    [0.75, -0.3, 0.75]
]
scale = 1.5

for pos, model in zip(positions, models):
    if model =="teddy_vhacd.urdf":
        # Teddy bear is a bit larger, so we scale it down
        scale = 4
    else: scale = 1.5
    p.loadURDF(model, basePosition=pos, baseOrientation=p.getQuaternionFromEuler([0,0,0]), globalScaling=scale)


# INSERT CODE PREVIOUS TO SIMULATION LOOP

initial_right_pose = [2.7354,-1.5973,-1.1237,-1.2453,0.4834,-0.1599]
initial_left_pose = [-2.0935,-2.0019,-0.8266,-2.6136,4.4560,5.1101]

# Move arms and hands to initial position
adam.arm_kinematics.move_arm_joints_to_angles(arm='right', angles=initial_right_pose)
adam.arm_kinematics.move_arm_joints_to_angles(arm='left', angles=initial_left_pose)
adam.hand_kinematics.move_hand_to_dofs(hand='both', dofs1=[1000, 1000, 1000, 1000, 1000, 1000], dofs2=[1000, 1000, 1000, 1000, 1000, 1000])

# Initialise camera angle
camera_angle = 0
adam.sensors.move_camera_angle(camera_angle)

print('Use "c" key to move camera')


# Main simulation loop
while True:

    # INSERT YOUR SIMULATION LOOP CODE HERE

    # Get rgb and depth image
    rgb, depth = adam.sensors.get_rgbd_image_from_link(width=640, height=480, fov=60, near=0.01, far=5.0)

    keys = p.getKeyboardEvents()

    # Press 'c' to move the camara between 0deg and -45deg
    if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
        
        camera_angle = -45 - camera_angle
        adam.sensors.move_camera_angle(camera_angle)
        print("Moving camera to angle:", camera_angle)

    # Press 's' to save image
    if ord('s') in keys and keys[ord('s')] & p.KEY_WAS_TRIGGERED:
        
        folder_path = os.path.join(base_path,"..","data", "images")
        adam.sensors.save_rgb_image(rgb_array=rgb, folder_path=folder_path, filename='scene.png')
        print("Saving RGB image.")

    adam.step()