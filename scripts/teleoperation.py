import pybullet as p
import numpy as np

# Class for the sliders
class Teleop():
    
    def __init__(self, adam):
        self.adam = adam
        self.slider_ids = {
            'left_arm': [],
            'right_arm': [],
            'left_hand': [],
            'right_hand': [],
            'camera': None
        }
        
        self.forward = 0
        self.turn = 0
    
    def create_sliders(self):
        '''
        Create sliders for controlling the robot joints.
        Supports both "inspire_hands" (per finger DOFs) and "grippers" (single slider via set_pos).
        '''
        self.slider_ids = {
            'left_arm': [],
            'right_arm': [],
            'left_hand': [],
            'right_hand': [],
            'camera': None
        }

        joint_range = 2 * np.pi

        # 1. Sliders para el brazo izquierdo
        for i in self.adam.ur3_left_arm_joints:
            joint_info = p.getJointInfo(self.adam.robot_id, i)
            joint_name = joint_info[1].decode("utf-8")
            if joint_info[2] == p.JOINT_REVOLUTE:
                s_id = p.addUserDebugParameter(f"L_arm: {joint_name}", -joint_range, joint_range, 0)
                self.slider_ids['left_arm'].append((i, s_id))

        # 2. Sliders para el brazo derecho
        for i in self.adam.ur3_right_arm_joints:
            joint_info = p.getJointInfo(self.adam.robot_id, i)
            joint_name = joint_info[1].decode("utf-8")
            if joint_info[2] == p.JOINT_REVOLUTE:
                s_id = p.addUserDebugParameter(f"R_arm: {joint_name}", -joint_range, joint_range, 0)
                self.slider_ids['right_arm'].append((i, s_id))

        # 3. Sliders para manos o grippers
        if hasattr(self.adam, 'hand_kinematics') and self.adam.hand_kinematics is not None:
            # Opción 1: inspire_hands (un slider por articulación de dedo)
            for i in range(self.adam.hand_kinematics.num_dofs):
                joint_name = self.adam.hand_kinematics.finger_names[i]
                s_left = p.addUserDebugParameter(f"L_hand {joint_name}", 0, 1000, 0)
                self.slider_ids['left_hand'].append(s_left)
                
            for i in range(self.adam.hand_kinematics.num_dofs):
                joint_name = self.adam.hand_kinematics.finger_names[i]
                s_right = p.addUserDebugParameter(f"R_hand {joint_name}", 0, 1000, 0)
                self.slider_ids['right_hand'].append(s_right)

        elif hasattr(self.adam, 'grippers') and self.adam.grippers is not None:
            # Opción 2: grippers (un único slider normalizado [0-1] por pinza)
            s_left = p.addUserDebugParameter("L_gripper (0:closed, 1:open)", 0.0, 1.0, 0.0)
            s_right = p.addUserDebugParameter("R_gripper (0:closed, 1:open)", 0.0, 1.0, 0.0)
            self.slider_ids['left_hand'].append(s_left)
            self.slider_ids['right_hand'].append(s_right)

        # 4. Slider para la cámara
        if hasattr(self.adam, 'camera_joint_index') and self.adam.camera_joint_index is not None:
            self.slider_ids['camera'] = p.addUserDebugParameter('Camera joint', -45, 45, 0)

    def apply_slider_values(self):
        '''
        Apply the values of the sliders to the robot joints.
        '''
        # 1. Aplicar valores a los brazos
        left_arm_values = [p.readUserDebugParameter(s_id) for _, s_id in self.slider_ids['left_arm']]
        right_arm_values = [p.readUserDebugParameter(s_id) for _, s_id in self.slider_ids['right_arm']]

        if left_arm_values:
            self.adam.arm_kinematics.move_arm_joints_to_angles('left', left_arm_values)
        if right_arm_values:
            self.adam.arm_kinematics.move_arm_joints_to_angles('right', right_arm_values)

        # 2. Aplicar valores a las manos o grippers
        if hasattr(self.adam, 'hand_kinematics') and self.adam.hand_kinematics is not None:
            # Modo inspire_hands
            left_hand_values = [p.readUserDebugParameter(s_id) for s_id in self.slider_ids['left_hand']]
            right_hand_values = [p.readUserDebugParameter(s_id) for s_id in self.slider_ids['right_hand']]

            if left_hand_values:
                self.adam.hand_kinematics.move_hand_to_dofs('left', left_hand_values)
            if right_hand_values:
                self.adam.hand_kinematics.move_hand_to_dofs('right', right_hand_values)

        elif hasattr(self.adam, 'grippers') and self.adam.grippers is not None:
            # Modo grippers: usa set_pos para mover ambos dedos en simultáneo
            if self.slider_ids['left_hand']:
                val_left = p.readUserDebugParameter(self.slider_ids['left_hand'][0])
                self.adam.grippers.set_pos('left', val_left, normalised=True)
                
            if self.slider_ids['right_hand']:
                val_right = p.readUserDebugParameter(self.slider_ids['right_hand'][0])
                self.adam.grippers.set_pos('right', val_right, normalised=True)

        # 3. Aplicar valor a la cámara
        if self.slider_ids['camera'] is not None:
            camera_val = p.readUserDebugParameter(self.slider_ids['camera'])
            self.adam.sensors.move_camera_angle(camera_val)

    def teleoperate_base(self, debug=False, move_sim=True):
        leftWheelVelocity = 0
        rightWheelVelocity = 0
        speed = self.adam.navigation.linear_speed
        self.keys = p.getKeyboardEvents()
        
        for k, v in self.keys.items():
            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                self.turn = -1
            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
                self.turn = 0
            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                self.turn = 1
            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
                self.turn = 0

            if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                self.forward = 1
            if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
                self.forward = 0
            if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                self.forward = -1
            if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
                self.forward = 0

        rightWheelVelocity = (self.forward + self.turn) * speed
        leftWheelVelocity  = (self.forward - self.turn) * speed
        
        if debug:
            print("Right wheel velocity", rightWheelVelocity)
            print("Left wheel velocity", leftWheelVelocity)
        if move_sim:
            if self.turn == 0 and self.forward != 0:
                self.adam.navigation.move_wheels(leftWheelVelocity * 4.0, rightWheelVelocity * 4.0, force=50)
            else:
                self.adam.navigation.move_wheels(leftWheelVelocity, rightWheelVelocity, force=50)
        
        return leftWheelVelocity, rightWheelVelocity