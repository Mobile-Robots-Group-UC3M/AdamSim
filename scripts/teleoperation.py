import pybullet as p
import numpy as np
import math


#Class for the sliders
class Teleop():
    
    def __init__(self, adam):
        self.adam = adam
        self.slider_ids = []
        
        self.forward=0
        self.turn=0
    
    def create_sliders(self):
        '''
        Create sliders for controlling the robot joints.
        This function creates sliders for controlling the joints of the robot.
        '''

        # Crear sliders para controlar las articulaciones
        
        joint_range = 2*np.pi

        slider_joints = self.adam.ur3_left_arm_joints + self.adam.ur3_right_arm_joints

        # Create sliders for each joint
        for i in slider_joints:
            joint_info = p.getJointInfo(self.adam.robot_id, i)
            joint_name = joint_info[1].decode("utf-8")
            if joint_info[2] == p.JOINT_REVOLUTE:
                self.slider_ids.append(p.addUserDebugParameter(joint_name, -joint_range, joint_range, 0))

        # Crear sliders para las articulaciones del brazo izquierdo
        for arm in ['left', 'right']:
            for i in range(self.adam.hand_kinematics.num_dofs):
                joint_name = self.adam.hand_kinematics.finger_names[i]
                self.slider_ids.append(p.addUserDebugParameter(arm + ' ' + joint_name, 0, 1000, 0))

        self.slider_ids.append(p.addUserDebugParameter('Camera joint', -45, 45, 0))

            

    def apply_slider_values(self):
        '''
        Apply the values of the sliders to the robot joints.
        This function reads the values of the sliders and applies them to the robot joints.'''

        left_arm_values = []
        right_arm_values = []

        # Obtener los valores de los sliders para las articulaciones del brazo izquierdo
        for i in range(len(self.adam.ur3_left_arm_joints)):
            left_arm_values.append(p.readUserDebugParameter(self.slider_ids[i]))
            right_arm_values.append(p.readUserDebugParameter(self.slider_ids[i + len(self.adam.ur3_left_arm_joints)]))

        # Move arms
        self.adam.arm_kinematics.move_arm_joints_to_angles('left', left_arm_values)
        self.adam.arm_kinematics.move_arm_joints_to_angles('right', right_arm_values)

        # Move hands
        left_hand_values = []
        right_hand_values = []

        for i in range(self.adam.hand_kinematics.num_dofs):
            left_hand_values.append(p.readUserDebugParameter(self.slider_ids[i + len(self.adam.ur3_left_arm_joints) + len(self.adam.ur3_right_arm_joints)]))
            right_hand_values.append(p.readUserDebugParameter(self.slider_ids[i + len(self.adam.ur3_left_arm_joints) + len(self.adam.ur3_right_arm_joints) + self.adam.hand_kinematics.num_dofs]))

        # Move hands
        self.adam.hand_kinematics.move_hand_to_dofs('left', left_hand_values)
        self.adam.hand_kinematics.move_hand_to_dofs('right', right_hand_values)

        # Move camera
        self.adam.sensors.move_camera_angle(p.readUserDebugParameter(self.slider_ids[24]))
        
    def teleoperate_base(self, debug=False, move_sim = True):
        
        leftWheelVelocity=0
        rightWheelVelocity=0
        speed=self.adam.navigation.linear_speed
        self.keys = p.getKeyboardEvents()
        
        for k,v in self.keys.items():
                    if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                            self.turn = -1
                    if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                            self.turn = 0
                    if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                            self.turn = 1
                    if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
                            self.turn = 0

                    if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                            self.forward=1
                    if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
                            self. forward=0
                    if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                            self.forward=-1
                    if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
                            self.forward=0

        rightWheelVelocity = (self.forward + self.turn) * speed
        leftWheelVelocity  = (self.forward - self.turn) * speed
        
        if debug==True:
            print("Right wheel velocity", rightWheelVelocity)
            print("Left wheel velocity",leftWheelVelocity)
        if move_sim==True:
            #self.adam.navigation.move_wheels(leftWheelVelocity, rightWheelVelocity, force=50)
            if self.turn == 0 and self.forward != 0:
                self.adam.navigation.move_wheels(leftWheelVelocity*4, rightWheelVelocity*4, force=50)
            else:
                
                self.adam.navigation.move_wheels(leftWheelVelocity, rightWheelVelocity, force=50)
        
        return leftWheelVelocity, rightWheelVelocity
        
