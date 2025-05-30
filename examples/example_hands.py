import os
from scripts.adam import ADAM
import pybullet as p
import pybullet_data
import math

def import_object(path, obj_pose):
    '''
    Import objects in simulation.
    Args:
        path (str): path of the object file
        obj_pose (list): pose of the object as [position, quaternions]
    '''

    object_shape = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                        fileName=path,
                                        meshScale=[0.8, 0.8, 1])
    object_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                                fileName=path,
                                                meshScale=[0.8, 0.8, 1])  # Ajusta el escalado 
    object_id = p.createMultiBody(baseMass=0.05,              
                                        baseCollisionShapeIndex=object_shape,
                                        baseVisualShapeIndex=object_visual_shape,
                                        basePosition=obj_pose[0],
                                        baseOrientation=obj_pose[1])    # Cambia la posición


# URDF robot path
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","paquetes_simulacion", "rb1_base_description", "robots", "robotDummy.urdf")

# Create ADAM instance
adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=True, use_ros=False)

# Print robot information
# adam.print_robot_info()

# Load table located in pybullet_data/table
table_path = os.path.join(pybullet_data.getDataPath(), "table", "table.urdf")
p.loadURDF(table_path, basePosition=[0.8, 0, 0.1], baseOrientation=p.getQuaternionFromEuler([0, 0, 1.54]))

# Import object
object_path = os.path.join(base_path,"..","data", "models", "milk.stl")
import_object(object_path, [[0.5, 0, 0.85], p.getQuaternionFromEuler([0,0,0])])

pregrasp_pose = [[0.55, -0.23, 0.86], p.getQuaternionFromEuler([0, math.pi/2, math.pi/2])]
grasp_pose = [[0.55, -0.14, 0.86], p.getQuaternionFromEuler([0, math.pi/2, math.pi/2])]
lift_pose = [[0.56, -0.14, 0.95], p.getQuaternionFromEuler([0, math.pi/2, math.pi/2])]
adam.utils.draw_frame(grasp_pose, axis_length=0.1, line_width=4)
adam.utils.draw_frame(pregrasp_pose, axis_length=0.1, line_width=4)

# Open hands
adam.hand_kinematics.move_hand_to_dofs('right', [1000, 1000, 1000, 1000, 1000, 0])

adam.wait(10)

while True:

    # INSERT YOUR SIMULATION LOOP CODE HERE

    # Move to grasp pose
    adam.arm_kinematics.move_arm_to_pose('right', pregrasp_pose, 'hand')
    adam.wait(2)
    adam.arm_kinematics.move_arm_to_pose('right', grasp_pose, 'hand')
    adam.wait(2)

    # Close hands
    adam.hand_kinematics.move_hand_to_dofs('right', [300, 300, 300, 300, 300, 0])
    adam.wait(3)

    # Lift pose
    adam.arm_kinematics.move_arm_to_pose('right', lift_pose, 'hand')

    adam.wait(1000)

    adam.step()