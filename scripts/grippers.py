import pybullet as p
import time

class Grippers:
    def __init__(self,adam):
        
        self.adam = adam
        self.paddle_pos = 0.0

        # FRICTION SETTING FOR PADDLE LINKS

        paddle_link_names = {
            'right': ['right_gripper_right_pieza_movil',
                      'right_gripper_right_zip',
                      'right_gripper_right_paddle', 
                      'right_gripper_right_cover', 
                      'right_gripper_right_sensor',
                      'right_gripper_left_pieza_movil',
                      'right_gripper_left_zip',
                      'right_gripper_left_paddle', 
                      'right_gripper_left_cover', 
                      'right_gripper_left_sensor'
                      ],
            'left': ['left_gripper_right_pieza_movil',
                     'left_gripper_right_zip',
                     'left_gripper_right_paddle', 
                     'left_gripper_right_cover', 
                     'left_gripper_right_sensor',
                     'left_gripper_left_pieza_movil',
                     'left_gripper_left_zip',
                     'left_gripper_left_paddle', 
                     'left_gripper_left_cover', 
                     'left_gripper_left_sensor'
                     ]
        }

        # Look up link IDs explicitly from robot_info["links"]
        self.gripper_link_indices = {'right': [], 'left': []}

        for side in ['right', 'left']:
            for link_name in paddle_link_names.get(side, []):
                try:
                    link_id = self.adam.robot_info["links"][link_name]["id"]
                    self.gripper_link_indices[side].append(link_id)
                except KeyError:
                    print(f"[WARNING] Link '{link_name}' not found in robot_info['links']!")

        # Apply high friction to the retrieved paddle link IDs
        for side in ['right', 'left']:
            for link_id in self.gripper_link_indices[side]:
                p.changeDynamics(
                    self.adam.robot_id,
                    link_id,
                    lateralFriction=1000,
                    spinningFriction=1,
                    frictionAnchor=1
                )


    def set_pos(self, arm:str, target_pos:float, normalised=True):
        '''
        Move the gripper's paddles to an absolute position.
        Args:
            arm (str): The gripper to move ('left', 'right' or 'both').
            target_pos (float): The target position.
            normalised (bool): Style of introducing coordinates for a displacement.
                False: Position in metres.
                True: Position must be in range [0, 1], where 0 is closed and 1 is open.
        '''

        # Between 0 and 1
        if normalised == True: final_pos = target_pos*0.035

        # Between 0 and 35 mm
        elif normalised == False: final_pos = target_pos

        if arm == "right" or arm == "left":

            p.setJointMotorControl2(self.adam.robot_id, self.adam.hand_joint_indices[arm][0], p.POSITION_CONTROL, targetPosition=final_pos)
            p.setJointMotorControl2(self.adam.robot_id, self.adam.hand_joint_indices[arm][1], p.POSITION_CONTROL, targetPosition=-final_pos)
        
        elif arm== "both":
            p.setJointMotorControl2(self.adam.robot_id, self.adam.hand_joint_indices["right"][0], p.POSITION_CONTROL, targetPosition=final_pos)
            p.setJointMotorControl2(self.adam.robot_id, self.adam.hand_joint_indices["right"][1], p.POSITION_CONTROL, targetPosition=-final_pos)

            p.setJointMotorControl2(self.adam.robot_id, self.adam.hand_joint_indices["left"][0], p.POSITION_CONTROL, targetPosition=final_pos)
            p.setJointMotorControl2(self.adam.robot_id, self.adam.hand_joint_indices["left"][1], p.POSITION_CONTROL, targetPosition=-final_pos)

        self.paddle_pos = final_pos 

    def move_pos(self, arm:str, target_pos:float, direction:int, normalised=True):
        '''
        Move the gripper's paddles to a relative position by adding the relative position and directio to the absolute position.
        Args:
            arm (str): The gripper to move ('left', 'right' or 'both').
            target_pos(float): The target position.
            direction (bool): The direction of the movement (1 is outwards and -1 inwards)
            normalised (bool): Style of introducing coordinates for a displacement.
                False: Position in metres.
                True: Position must be in range [0, 1], where 0 is closed and 1 is open.
        '''

        # Between 0 and 1
        if normalised == True: final_pos = target_pos*0.035

        # Between 0 and 35 mm
        elif normalised == False: final_pos = target_pos

        # New position
        new_pos = self.paddle_pos + direction*final_pos

        if arm == "right" or arm == "left":
            p.setJointMotorControl2(self.robot_id, self.adam.hand_joint_indices[arm][0], p.POSITION_CONTROL, targetPosition=new_pos)
            p.setJointMotorControl2(self.robot_id, self.adam.hand_joint_indices[arm][1], p.POSITION_CONTROL, targetPosition=-new_pos)
        
        elif arm == "both":
            p.setJointMotorControl2(self.robot_id, self.adam.hand_joint_indices["right"][0], p.POSITION_CONTROL, targetPosition=new_pos)
            p.setJointMotorControl2(self.robot_id, self.adam.hand_joint_indices["right"][1], p.POSITION_CONTROL, targetPosition=-new_pos)

            p.setJointMotorControl2(self.robot_id, self.adam.hand_joint_indices["left"][0], p.POSITION_CONTROL, targetPosition=new_pos)
            p.setJointMotorControl2(self.robot_id, self.adam.hand_joint_indices["left"][1], p.POSITION_CONTROL, targetPosition=-new_pos)

        self.paddle_pos = new_pos


    def get_pos(self, arm: str):

        if arm in ["right", "left"]:
            r_pos = p.getJointState(self.adam.robot_id, self.adam.hand_joint_indices[arm][0])[0]
            l_pos = p.getJointState(self.adam.robot_id, self.adam.hand_joint_indices[arm][1])[0]
            print(f"The joint is in position: {r_pos}, {l_pos}\n")
        elif arm == "both":
            r_r_pos = p.getJointState(self.adam.robot_id, self.adam.hand_joint_indices["right"][0])[0]
            r_l_pos = p.getJointState(self.adam.robot_id, self.adam.hand_joint_indices["right"][1])[0]
            l_r_pos = p.getJointState(self.adam.robot_id, self.adam.hand_joint_indices["left"][0])[0]
            l_l_pos = p.getJointState(self.adam.robot_id, self.adam.hand_joint_indices["left"][1])[0]
            print(f"The joint is in position: {r_r_pos}, {r_l_pos}, {l_r_pos}, {l_l_pos}\n")


    def close_with_force_limit(self, arm: str, target_torque: float = 10.0, closing_speed: float = 0.05, max_steps: int = 1000):

        arms = ['right', 'left'] if arm == "both" else [arm]

        # Set persistent velocity control with capped force
        for a in arms:
            j0 = self.adam.hand_joint_indices[a][0]
            j1 = self.adam.hand_joint_indices[a][1]

            p.setJointMotorControl2(self.adam.robot_id, j0, p.VELOCITY_CONTROL, targetVelocity=closing_speed, force=target_torque)
            p.setJointMotorControl2(self.adam.robot_id, j1, p.VELOCITY_CONTROL, targetVelocity=-closing_speed, force=target_torque)

        # Monitor velocity until fingers stall against object
        stationary_counter = 0
        for step in range(max_steps):
            all_stopped = True
            for a in arms:
                j0, j1 = self.adam.hand_joint_indices[a][0], self.adam.hand_joint_indices[a][1]
                if abs(p.getJointState(self.adam.robot_id, j0)[1]) > 0.0005 or abs(p.getJointState(self.adam.robot_id, j1)[1]) > 0.0005:
                    all_stopped = False

            time.sleep(0.01)

            if step > 15 and all_stopped:
                stationary_counter += 1
                if stationary_counter >= 10:
                    break
            else:
                stationary_counter = 0

        # Lock position at contact state
        for a in arms:
            j0, j1 = self.adam.hand_joint_indices[a][0], self.adam.hand_joint_indices[a][1]
            pos0, pos1 = p.getJointState(self.adam.robot_id, j0)[0], p.getJointState(self.adam.robot_id, j1)[0]
            p.setJointMotorControl2(self.adam.robot_id, j0, p.POSITION_CONTROL, targetPosition=pos0, force=target_torque)
            p.setJointMotorControl2(self.adam.robot_id, j1, p.POSITION_CONTROL, targetPosition=pos1, force=target_torque)