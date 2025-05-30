import os
from scripts.adam import ADAM
import pybullet as p
import pybullet_data
import math

# URDF robot path
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","paquetes_simulacion", "rb1_base_description", "robots", "robotDummy.urdf")

# Create ADAM instance
adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=False, use_ros=False)

# Print robot information
# adam.print_robot_info()

# Añadir obstáculo: una caja delante del robot
box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.5])
box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.5], rgbaColor=[1, 1, 0, 1])
box_id = p.createMultiBody(baseMass=0.1,
                        baseCollisionShapeIndex=box_collision,
                        baseVisualShapeIndex=box_visual,
                        basePosition=[2, 1.6, 0.5])  # Frente al robot

# Añadir obstáculo: una caja delante del robot
box_collision2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.5])
box_visual2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.5], rgbaColor=[0.7, 0, 45, 1])
box_id2 = p.createMultiBody(baseMass=0.1,
                        baseCollisionShapeIndex=box_collision2,
                        baseVisualShapeIndex=box_visual2,
                        basePosition=[1.5, -1.6, 0.5])  # Frente al robot    

adam.sensors.start_lidar()
print('Lidar started')

#adam.wait(10)

while True:

    # INSERT YOUR SIMULATION LOOP CODE HERE

    # Move base to pose
    pose = (3, 0, 0)
    
    adam.navigation.move_base_continuous(pose)

    adam.step()