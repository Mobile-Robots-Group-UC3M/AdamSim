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
        self.camera_joint_index = 69
        self.link_index = 72
        self.laser_joint_index = 8
        
        self.ray_ids = []
        self.num_rays = int(270/0.25)

        self.move_camera_angle(self.camera_angle)


    def start_lidar(self):
        '''
        Start Lidar rays
        '''
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
        link_state = p.getLinkState(self.adam.robot_id, self.link_index)
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

    

    def save_rgb_image(self, rgb_array, filename="camera_image.png", directory="./images"):
        '''
        Save RGB image to a file.
        Args:
            rgb_array (numpy.ndarray): The RGB image array.
            filename (str): The name of the file to save the image.
            directory (str): The directory to save the image.
        '''
        
        os.makedirs(directory, exist_ok=True)
        path = os.path.join(directory, filename)
        image = Image.fromarray(rgb_array)
        image.save(path)
        print(f"Saved image to {path}")


    def move_camera_angle(self, angle):
        '''
        Move the camera to a specified angle.
        Args:
            angle (float): The camera angle in degrees. Must be between -45 and 45 degrees.
        '''

        # Save camera angle
        self.camera_angle = angle

        angle_rad = np.deg2rad(self.camera_angle)

        if angle_rad > np.pi/4 or angle_rad < -np.pi/4: raise ValueError("Angle must be between -pi/4 and pi/4")

        # Move camera joint to the specified angle
        p.setJointMotorControl2(self.adam.robot_id, self.camera_joint_index, p.POSITION_CONTROL, (np.pi/4 + angle_rad))
        p.stepSimulation()
        time.sleep(self.adam.t)


    def get_camera_angle(self):
        '''
        Get the camera angle.
        Returns:
            camera_angle (float): The camera angle in degrees.
        '''

        joint_state = p.getJointState(self.adam.robot_id, self.camera_joint_index)

        return np.rad2deg(joint_state[0]) - 45
    
    def simulated_lidar(self,ray_length=10):
        self.ray_length = ray_length
        self.ray_hit_color = [1, 0, 0]
        self.ray_miss_color = [0, 1, 0]

        # Crear rayos inicialmente
        """ if self.ray_ids ==[]:
            for _ in range(self.num_rays):
                self.ray_ids.append(p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 1, 0])) """

        p.stepSimulation()

        # Obtener la pose del joint del LiDAR
        link_state = p.getLinkState(self.adam.robot_id,self.laser_joint_index)

        laser_pos = link_state[0]
        laser_ori = link_state[1]
        rot_matrix = p.getMatrixFromQuaternion(laser_ori)
        rot_matrix = [rot_matrix[0:3], rot_matrix[3:6], rot_matrix[6:9]]

        self.ray_from = []
        self.ray_to = []

        # Generar rayos en el plano XY del frame del LIDAR
        for i in range(self.num_rays):
            #angle = 2 * math.pi * i / num_rays
            angle = -math.pi * 3/4 + (math.pi * 3/2) * i / self.num_rays
            local_dir = [math.cos(angle), math.sin(angle), 0]

            # Convertir a coordenadas globales
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

        # Lanzar rayos
        results = p.rayTestBatch(self.ray_from, self.ray_to)

        # Dibujar rayos
        for i in range(self.num_rays):
            if results[i][0] < 0:
                p.addUserDebugLine(self.ray_from[i], self.ray_to[i], self.ray_miss_color, lineWidth=1.0,
                                replaceItemUniqueId=self.ray_ids[i])
            else:
                hit_position = results[i][3]
                p.addUserDebugLine(self.ray_from[i], hit_position, self.ray_hit_color, lineWidth=1.0,
                                replaceItemUniqueId=self.ray_ids[i])

        time.sleep(self.adam.t)