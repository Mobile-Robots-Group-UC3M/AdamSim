import pybullet as p
import numpy as np
from PIL import Image
import os
import math
import time

class Sensors():
    def __init__(self, adam):
        self.adam = adam
        self.camera_angle = 0
        
        self.move_camera_angle(self.camera_angle)


    def start_lidar(self, num_rays=int(270/0.25)):
        '''
        Start Lidar rays
        '''

        self.num_rays = num_rays
        self.ray_ids = []

        for _ in range(self.num_rays): self.ray_ids.append(p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 1, 0]))


    def get_rgbd_image_from_link(self, width=640, height=480, fov=60, near=0.01, far=5.0):
        '''
        Get RGB and Depth image from a specific link of the robot.
        Args:
            width (int): The width of the image.
            height (int): The height of the image.
            fov (float): The field of view of the camera in degrees.
            near (float): The near clipping plane distance.
            far (float): The far clipping plane distance.
        Returns:
            rgb (numpy.ndarray): The captured RGB image.
            depth (numpy.ndarray): The captured depth image (in meters).
        '''

        # Get link world position and orientation
        link_state = p.getLinkState(self.adam.robot_id, self.adam.camera_link_index)
        cam_pos = link_state[0]
        cam_ori = link_state[1]

        # Rotation matrix from quaternion
        rot_matrix = np.array(p.getMatrixFromQuaternion(cam_ori)).reshape(3, 3)
        forward = rot_matrix @ np.array([0, 0, 1])  # Z forward
        up = rot_matrix @ np.array([0, 1, 0])       # Y up
        target = np.array(cam_pos) + forward

        # View and projection matrices
        view_matrix = p.computeViewMatrix(cam_pos, target, up)
        proj_matrix = p.computeProjectionMatrixFOV(fov, width / height, near, far)

        # Capture image (returns tuple with depth buffer)
        img = p.getCameraImage(
            width,
            height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # Extract and process RGB
        rgba = np.reshape(img[2], (height, width, 4))
        rgb = rgba[:, :, :3].astype(np.uint8)

        # Extract depth buffer and convert to actual depth in meters
        depth_buffer = np.reshape(img[3], (height, width)).astype(np.float32)
        depth = far * near / (far - (far - near) * depth_buffer)

        return rgb, depth


    def save_rgb_image(self, rgb_array, folder_path="./images", filename="camera_image.png"):
        '''
        Save RGB image to a file.
        Args:
            rgb_array (numpy.ndarray): The RGB image array.
            filename (str): The name of the file to save the image.
            folder_path (str): The path to save the image.
        '''
        
        os.makedirs(folder_path, exist_ok=True)
        path = os.path.join(folder_path, filename)
        image = Image.fromarray(rgb_array)
        image.save(path)
        print(f"Saved image to {path}")


    def move_camera_angle(self, angle):
        '''
        Move the camera to a specified angle.
        Args:
            angle (float): The camera angle in degrees. Must be between -45 and 45 degrees.
        '''
        self.camera_angle = angle
        angle_rad = np.deg2rad(self.camera_angle)

        if angle_rad > np.pi/4 or angle_rad < -np.pi/4: 
            raise ValueError("Angle must be between -pi/4 and pi/4")

        # Se añaden force y maxVelocity para garantizar el movimiento del motor
        p.setJointMotorControl2(
            bodyUniqueId=self.adam.robot_id, 
            jointIndex=self.adam.camera_joint_index, 
            controlMode=p.POSITION_CONTROL, 
            targetPosition=(np.pi/4 + angle_rad),
            force=50.0,
            maxVelocity=2.0
        )
        if not self.adam.use_realtime:
            p.stepSimulation()
        time.sleep(self.adam.t)

    def get_camera_angle(self):
        '''
        Get the camera angle.
        Returns:
            camera_angle (float): The camera angle in degrees.
        '''
        joint_state = p.getJointState(self.adam.robot_id, self.adam.camera_joint_index)
        return np.rad2deg(joint_state[0]) - 45
    
    def simulated_lidar(self,ray_length=10):
        '''
        Simulated LiDAR sensor using ray casting.
        Args:
            ray_length (float): The length of the rays in meters.
        '''

        if not self.num_rays: raise ValueError("LiDAR not initialized. Call start_lidar() first.")

        self.ray_length = ray_length
        self.ray_hit_color = [1, 0, 0]
        self.ray_miss_color = [0, 1, 0]

        p.stepSimulation()

        link_state = p.getLinkState(self.adam.robot_id, self.adam.laser_link_index  )

        laser_pos = link_state[0]
        laser_ori = link_state[1]
        rot_matrix = p.getMatrixFromQuaternion(laser_ori)
        rot_matrix = [rot_matrix[0:3], rot_matrix[3:6], rot_matrix[6:9]]

        self.ray_from = []
        self.ray_to = []

        for i in range(self.num_rays):
            #angle = 2 * math.pi * i / num_rays
            angle = -math.pi * 3/4 + (math.pi * 3/2) * i / self.num_rays
            local_dir = [math.cos(angle), math.sin(angle), 0]

            global_dir = [
                sum(rot_matrix[row][col] * local_dir[col] for col in range(3))
                for row in range(3)
            ]
            self.ray_from.append(laser_pos)
            self.ray_to.append([
                laser_pos[0] + ray_length * global_dir[0],
                laser_pos[1] + ray_length * global_dir[1],
                laser_pos[2] + ray_length * global_dir[2],
            ])

        results = p.rayTestBatch(self.ray_from, self.ray_to)

        for i in range(self.num_rays):
            if results[i][0] < 0:
                p.addUserDebugLine(self.ray_from[i], self.ray_to[i], self.ray_miss_color, lineWidth=1.0,
                                replaceItemUniqueId=self.ray_ids[i])
            else:
                hit_position = results[i][3]
                p.addUserDebugLine(self.ray_from[i], hit_position, self.ray_hit_color, lineWidth=1.0,
                                replaceItemUniqueId=self.ray_ids[i])

        time.sleep(self.adam.t)
    
    def get_lidar_points_world(self, ray_length=10):
        '''
        Simulated LiDAR sensor using ray casting.
        Updates debug lines and returns the hit points in world coordinates.
        
        Args:
            ray_length (float): The length of the rays in meters.
        Returns:
            np.array: An array of [x, y] coordinates where the LiDAR hit an obstacle.
        '''
        self.ray_length = ray_length
        self.ray_hit_color = [1, 0, 0]
        self.ray_miss_color = [0, 1, 0]
        
        # Trigger the LiDAR scan
        self.adam.sensors.simulated_lidar(ray_length=ray_length)

        # Lanzamos los rayos
        results = p.rayTestBatch(self.ray_from, self.ray_to)

        # Lista para guardar los obstáculos detectados
        hit_points = []

        # Procesamos los resultados
        for i in range(self.num_rays):
            if results[i][0] < 0:
                # No ha chocado con nada (Dibuja línea verde)
                p.addUserDebugLine(self.ray_from[i], self.ray_to[i], self.ray_miss_color, lineWidth=1.0,
                                replaceItemUniqueId=self.ray_ids[i])
            else:
                # HA CHOCADO CON ALGO
                hit_position = results[i][3] # <--- Coordenadas [x, y, z] del impacto
                
                # Añadimos solo X e Y a la lista de obstáculos (DWA funciona en 2D)
                hit_points.append([hit_position[0], hit_position[1]]) 
                
                # Dibuja línea roja hasta el punto de impacto
                p.addUserDebugLine(self.ray_from[i], hit_position, self.ray_hit_color, lineWidth=1.0,
                                replaceItemUniqueId=self.ray_ids[i])

        # Devolvemos los puntos como un array de Numpy para que sea más fácil de procesar en el planner
        return np.array(hit_points)