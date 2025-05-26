import pybullet as p
import numpy as np
from scipy.io import loadmat
import math
import time


# Utility class for ADAM robot simulation
class Utils:
    def __init__(self, adam):
        self.adam = adam
        

    def draw_frame(self, pose, axis_length=0.1, line_width=2):
        '''
        Draws a coordinate frame at a given position and orientation in PyBullet.

        Args:
            pose (list): A list containing position and orientation [position, orientation].
                - position (list): The [x, y, z] coordinates of the frame origin.
                - orientation (list): The quaternion [x, y, z, w] representing the frame's orientation.
            axis_length (float): Length of each axis line.
            line_width (float): Thickness of the debug lines.
        '''

        # Rotation matrix from quaternion
        T_rotated = np.array(p.getMatrixFromQuaternion(pose[1])).reshape(3, 3)

        # Axis directions in world space
        x_axis = T_rotated @ np.array([axis_length, 0, 0])
        y_axis = T_rotated @ np.array([0, axis_length, 0])
        z_axis = T_rotated @ np.array([0, 0, axis_length])

        origin = np.array(pose[0])

        # Draw axes
        p.addUserDebugLine(origin, (origin + x_axis).tolist(), [1, 0, 0], line_width)  # X - red
        p.addUserDebugLine(origin, (origin + y_axis).tolist(), [0, 1, 0], line_width)  # Y - green
        p.addUserDebugLine(origin, (origin + z_axis).tolist(), [0, 0, 1], line_width)  # Z - blue
        
    