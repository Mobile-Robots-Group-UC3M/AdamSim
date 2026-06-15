import os
from scripts.adam import ADAM
import pybullet as p
import math
import time

# URDF robot path
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","models","robot", "rb1_base_description", "robots", "robotDummy.urdf")
print(robot_urdf_path)


# Tiempo inicial
initial_time = time.time()
# Create ADAM instance
#t0 = time.time()
adam = ADAM(robot_urdf_path, useRealTimeSimulation=False, used_fixed_base=False, use_ros=False)
#t1 = time.time()
adam.environments.generate_home(
    seed=2, 
    max_home_regens=300, 
    room_config={
        "kitchen"   :1,
        "bathrooms" :1,
        "bedroom1"  :1,
        "bedroom2"  :1,
        "dining"    :1
    },
    floor_color=None, 
    wall_color=None, 
    show_info=True
)

#t2 = time.time()

adam.environments.create_movable_objects()
#t3 = time.time()

# Start LiDAR and configure visualizer
adam.sensors.start_lidar()
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

final_time = time.time()
#tiempo_adam = t1 - t0
#tiempo_generacion_entorno = t2 - t1
#tiempo_creacion_objetos = t3 - t2
#tiempo_toal = t3 - t0

total_time = final_time - initial_time

print(f"Tiempo de creación de la instancia de ADAM: {total_time:.2f} segundos")

while True:
    adam.teleop.teleoperate_base()
    rgb, _ = adam.sensors.get_rgbd_image_from_link(width=640, height=480, fov=60, near=0.01, far=5.0)
    obstacles = adam.sensors.get_lidar_points_world()

    #v, w = adam.planner.dwa_step(goal, obstacles)
    adam.step()



