from adam import ADAM
import time
import pybullet as p
import pybullet_data
import scipy.io
import os
import math

if __name__ == '__main__':
    base_path = os.path.dirname(__file__)
    robot_urdf_path = os.path.join(base_path,"..","paquetes_simulacion", "rb1_base_description", "robots", "robotDummy.urdf")
    
    adam = ADAM(robot_urdf_path,0,1, False, use_ros=False)
    
    adam.print_robot_info()
    #time.sleep(100)
    
    p.setRealTimeSimulation(adam.useRealTimeSimulation)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    """ initial_left_pose = [0.11,-1.96,-0.79,0.67,-0.08,-0.01]
    initial_right_pose = [0.66,-2.26,-0.70,-0.14,2.55,-1.15] """
    #initial_left_pose = [-0.85,-1.72,0.042,-1.036,1.0249,1.911]
    #initial_right_pose = [-0.43,-1.83,0.681,-1.43,1.908,-1.53]
    initial_right_pose = [2.7354,-1.5973,-1.1237,-1.2453,0.4834,-0.1599]
    initial_left_pose = [-2.0935,-2.0019,-0.8266,-2.6136,4.4560,5.1101]
    
    
    
    
    # Initial pose
    """ for _ in range(5):
        adam.arm_kinematics.initial_arm_pose("right",initial_right_pose)
        adam.arm_kinematics.initial_arm_pose("left",initial_left_pose) """
    
    adam.print_robot_info()
    #time.sleep(1000000)
    print("Dale chicha")
    
    """ p.setJointMotorControl2(adam.robot_id, 3, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(adam.robot_id, 4, p.VELOCITY_CONTROL, force=0) """
    #adam.navigation.move_base_continuous((-6,6,0.5),pos_tolerance=0.04,angle_tolerance=0.08)
    """ while(1):
        adam.teleop.teleoperate_base(debug=True) """
        
    # Añadir obstáculo: una caja delante del robot
    box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.5])
    box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.5], rgbaColor=[1, 1, 0, 1])
    box_id = p.createMultiBody(baseMass=0.1,
                            baseCollisionShapeIndex=box_collision,
                            baseVisualShapeIndex=box_visual,
                            basePosition=[3, 0, 0.5])  # Frente al robot
    
    # Añadir obstáculo: una caja delante del robot
    box_collision2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.28, 0.31])
    box_visual2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.5], rgbaColor=[0.7, 0, 45, 1])
    box_id2 = p.createMultiBody(baseMass=0.1,
                            baseCollisionShapeIndex=box_collision2,
                            baseVisualShapeIndex=box_visual2,
                            basePosition=[3, 0, 0.5])  # Frente al robot
    adam.teleop.create_sliders()
    while (1):
        adam.teleop.apply_slider_values()
        adam.teleop.teleoperate_base(debug=False)
        adam.sensors.simulated_lidar(ray_length=5)
        
    """ # Índice del joint donde está montado el lidar
    laser_joint_index = 8

    # Parámetros del LiDAR
    num_rays = 360
    ray_length = 10.0
    ray_hit_color = [1, 0, 0]
    ray_miss_color = [0, 1, 0]

    # Crear rayos inicialmente
    ray_ids = []
    for _ in range(num_rays):
        ray_ids.append(p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 1, 0]))

    while True:
        p.stepSimulation()

        # Obtener la pose del joint del LiDAR
        link_state = p.getLinkState(adam.robot_id, laser_joint_index)
        if not link_state:
            continue

        laser_pos = link_state[0]
        laser_ori = link_state[1]
        rot_matrix = p.getMatrixFromQuaternion(laser_ori)
        rot_matrix = [rot_matrix[0:3], rot_matrix[3:6], rot_matrix[6:9]]

        ray_from = []
        ray_to = []

        # Generar rayos en el plano XY del frame del LIDAR
        for i in range(num_rays):
            angle = 2 * math.pi * i / num_rays
            local_dir = [math.cos(angle), math.sin(angle), 0]

            # Convertir a coordenadas globales
            global_dir = [
                sum(rot_matrix[row][col] * local_dir[col] for col in range(3))
                for row in range(3)
            ]
            ray_from.append(laser_pos)
            ray_to.append([
                laser_pos[0] + ray_length * global_dir[0],
                laser_pos[1] + ray_length * global_dir[1],
                laser_pos[2] + ray_length * global_dir[2],
            ])

        # Lanzar rayos
        results = p.rayTestBatch(ray_from, ray_to)

        # Dibujar rayos
        for i in range(num_rays):
            if results[i][0] < 0:
                p.addUserDebugLine(ray_from[i], ray_to[i], ray_miss_color, lineWidth=1.0,
                                replaceItemUniqueId=ray_ids[i])
            else:
                hit_position = results[i][3]
                p.addUserDebugLine(ray_from[i], hit_position, ray_hit_color, lineWidth=1.0,
                                replaceItemUniqueId=ray_ids[i])

        time.sleep(1. / 240.0) """
        
        