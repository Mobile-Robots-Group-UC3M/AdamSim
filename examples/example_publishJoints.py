import rospy
import pybullet as p
from scripts.adam import ADAM
import os
import pandas as pd
import ast
import numpy as np
import scipy.io
# URDF robot path and create ADAM instance
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","models","robot", "rb1_base_description", "robots", "robotDummy.urdf")
adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=True,use_ros=True)


# Table
table_dim = [0.25, 0.5, 0.005]
table_position = [0.5, 0, 0.875]

collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=table_dim)
visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=table_dim, rgbaColor = [0.8, 0.4, 0.1, 1])
table_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape, baseVisualShapeIndex=visual_shape, basePosition=table_position)

#Box
box_dim = [0.26/2, 0.26/2, 0.17/2]

visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=box_dim, rgbaColor = [0.7, 0.3, 0.2, 1])
box1_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape, basePosition=[0.5,-0.1,0.875+0.085])



object_path = os.path.join(base_path, "..", "models", "vase_MD.stl")

object_shape = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=object_path, meshScale=[1.2,1.2,1])
object_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=object_path, meshScale=[1,1,1])
object_id = p.createMultiBody(baseMass=0.01, baseCollisionShapeIndex=object_shape, baseVisualShapeIndex=object_visual_shape, basePosition=[0.6, -0.42, 0.875 + 0.07], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

object_path2 = os.path.join(base_path, "..", "models", "vase_MD.stl")

object_shape2 = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=object_path2, meshScale=[1.1,1.1,1.5])
object_visual_shape2 = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=object_path2, meshScale=[1,1,1.5])
object2_id = p.createMultiBody(baseMass=0.001, baseCollisionShapeIndex=object_shape2, baseVisualShapeIndex=object_visual_shape2, basePosition=[0.51, 0.28, 0.875 + 0.13], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))



# Simulation loop
# Cargar el archivo .mat
""" mat = scipy.io.loadmat('dataRight4.mat')

data = mat['mean_blended']  # Reemplaza 'path' con el nombre real de la variable
print(data.shape)   # Debe ser algo como (N, 6) si son 6 joints

# Convertir a lista de listas
path_data = data.tolist()
# Inspeccionar las claves disponibles
print("Claves del .mat:", mat.keys())
print(path_data[0]) """

df = pd.read_csv('right1.csv')

# Mostrar el DataFrame para verificar
print(df.head())

# Convertir a lista de listas
path_data = df.values.tolist()

df2 = pd.read_csv('left1.csv')

# Mostrar el DataFrame para verificar
print(df2.head())

# Convertir a lista de listas
path_data2 = df2.values.tolist()

maximo = max(len(path_data),len(path_data2))

try:
    adam.wait(0.1)
    print("Publishing data...")
    for i in range(maximo):
        print('Next joint')
        if i < len(path_data):
            adam.ros.arm_publish_joint_trajectory(arm='right',joint_angles=path_data[i])
        if i< len(path_data2):
            adam.ros.arm_publish_joint_trajectory(arm='left',joint_angles=path_data2[i])
        adam.wait(0.01)

except:
    print("Exception cogida")
finally:
    print("Data sent")
    