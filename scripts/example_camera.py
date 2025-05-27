from adam import ADAM
import pybullet as p
import pybullet_data
import time
import os

# URDF robot path
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","paquetes_simulacion", "rb1_base_description", "robots", "robotDummy.urdf")


# Create ADAM instance
adam = ADAM(robot_urdf_path, useSimulation=True, useRealTimeSimulation=False, used_fixed_base=True,use_ros=False)

# Print robot information
adam.print_robot_info()
angles = [0.1,0.2,0.3,0.4,0.5,0.6]

# adam.kinematics.move_arm_joints_to_angles('left', angles)

target_pose = [[0.5, 0.5, 0.5], [0.1, 0.2, 0.3]]
#adam.arm_kinematics.move_arm_to_pose('left', target_pose, 'hand')

camera_angle = 0
adam.sensors.move_camera_angle(camera_angle)
t = 0

# Cargar la mesa (viene en pybullet_data/table)
table_path = os.path.join(pybullet_data.getDataPath(), "table", "table.urdf")
# Colocamos la mesa a 0.75 m de altura para que la superficie quede a ~0.7 m
p.loadURDF(table_path, basePosition=[0.6, 0, 0], baseOrientation=p.getQuaternionFromEuler([0, 0, 1.54]))

models = [
    "duck_vhacd.urdf",
    "teddy_vhacd.urdf",
    "random_urdfs/000/000.urdf",
    "random_urdfs/001/001.urdf",
    "random_urdfs/002/002.urdf",
    "random_urdfs/003/003.urdf",
]
positions = [
    [0.5, -0.2, 0.75],
    [0.7,  0.1, 0.75],
    [0.4,  0.2, 0.75],
    [0.8, -0.1, 0.75],
    [0.55, 0.3, 0.75],
    [0.65, -0.3, 0.75]
]
scale = 1.5

for pos, model in zip(positions, models):
    if model =="teddy_vhacd.urdf":
        # Teddy bear is a bit larger, so we scale it down
        scale = 4
    else:
        scale = 1.5
    p.loadURDF(model, 
               basePosition=pos, 
               baseOrientation=p.getQuaternionFromEuler([0,0,0]),
               globalScaling=scale)

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
        time.sleep(1.0/240.0)
        #t += adam.t