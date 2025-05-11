import pybullet as p
import numpy as np
from PIL import Image
import os
import time

class Sensors:
    def __init__(self, adam):
        self.adam = adam


    def get_rgb_image_from_link(self, link_index, width=640, height=480, fov=60, near=0.01, far=5.0):

        # Get link world position and orientation
        link_state = p.getLinkState(self.adam.robot_id, link_index)
        cam_pos = link_state[0]
        cam_ori = link_state[1]

        # Get rotation matrix from quaternion
        rot_matrix = p.getMatrixFromQuaternion(cam_ori)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        # Camera direction (Z forward) and up vector (Y up)
        forward = rot_matrix @ np.array([0, 0, 1])
        up = rot_matrix @ np.array([0, 1, 0])
        target = np.array(cam_pos) + forward

        # View and projection matrices
        view_matrix = p.computeViewMatrix(cam_pos, target, up)
        proj_matrix = p.computeProjectionMatrixFOV(fov, width / height, near, far)

        # Capture image
        img = p.getCameraImage(
            width,
            height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # Extract RGB image (shape: H x W x 4), take only RGB
        rgba = np.reshape(img[2], (height, width, 4))
        rgb = rgba[:, :, :3].astype(np.uint8)

        return rgb
    

    def save_rgb_image(self, rgb_array, filename="camera_image.png", directory="./images"):
        
        os.makedirs(directory, exist_ok=True)
        path = os.path.join(directory, filename)
        image = Image.fromarray(rgb_array)
        image.save(path)
        print(f"Saved image to {path}")

    def move_camera_angle(self, angle):

        if angle > np.pi/4 or angle < -np.pi/4: raise ValueError("Angle must be between -pi/4 and pi/4")

        camera_joint_index = 69

        p.setJointMotorControl2(self.adam.robot_id, camera_joint_index, p.POSITION_CONTROL, (np.pi + angle))
        p.stepSimulation()
        time.sleep(self.adam.t)