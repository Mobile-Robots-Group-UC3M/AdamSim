import pybullet as p

class HandsKinematics():
    
    def __init__(self,adam):
        self.adam = adam

        self.joint_norm_values = {
            'thumb_MCP_joint1': 0.0011,
            'thumb_MCP_joint2': 0.0005,
            'thumb_PIP_joint': 0.001,
            'thumb_DIP_joint': 0.0012,
            'index_MCP_joint': 0.0017,
            'index_DIP_joint': 0.0016,
            'middle_MCP_joint': 0.0017,
            'middle_DIP_joint': 0.0016,
            'ring_MCP_joint': 0.0017,
            'ring_DIP_joint': 0.0016,
            'pink_MCP_joint': 0.0017,
            'pink_DIP_joint': 0.0016
        }

        self.joint_names = [
            'thumb_MCP_joint1',
            'thumb_MCP_joint2',
            'thumb_PIP_joint',
            'thumb_DIP_joint',
            'index_MCP_joint',
            'index_DIP_joint',
            'middle_MCP_joint',
            'middle_DIP_joint',
            'ring_MCP_joint',
            'ring_DIP_joint',
            'pink_MCP_joint',
            'pink_DIP_joint'
        ]

        self.finger_names = ['thumb_abd', 'thumb_flex', 'index', 'middle', 'ring', 'pink']

        self.num_hand_joints = len(self.joint_names)
        self.dof_joints_indices = [0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5]
        self.num_dofs = 6


    def move_hand_to_dofs(self, hand, dofs1, dofs2=None):
        '''
        Move the hand to the specified dof values.
        Args:
            hand (str): The hand to move. It can be 'right', 'left' or 'both'.
            dofs1 (list): The dof values of the first hand.
            dofs2 (list): The dof values of the second hand. It is only used if hand is 'both'.
        Returns:
            None
        '''

        if hand == 'right' or hand=='left':
            
            angles = self.dofs_to_joint_angles(dofs1)

            for i in range(self.num_hand_joints):
                p.setJointMotorControl2(self.adam.robot_id, self.adam.hand_joint_indices[hand][i], p.POSITION_CONTROL, angles[i])

        elif hand == 'both' and dofs2 is not None:
            angles1 = self.dofs_to_joint_angles(dofs1)
            angles2 = self.dofs_to_joint_angles(dofs2)

            for i in range(self.num_hand_joints):
                p.setJointMotorControl2(self.adam.robot_id, self.adam.hand_joint_indices['right'][i], p.POSITION_CONTROL, angles1[i])
                p.setJointMotorControl2(self.adam.robot_id, self.adam.hand_joint_indices['left'][i], p.POSITION_CONTROL, angles2[i])

        else: raise ValueError("El brazo debe ser 'right', 'left' o 'both', or you must provide dofs2 for 'both' hands.")


    def get_normalized_dofs(self, hand):
        '''
        Get the normalized angles of the hand joints.
        Args:
            hand (str): The hand to get the angles from. It can be 'right' or 'left'.
        Returns:
            angles (list): The normalized angles of the hand joints.
        '''

        # Obtain the joint indices for the specified hand
        try: joint_indices = self.adam.hand_joint_indices[hand]
        except: print('Hand must be "right" or "left"')

        # Get the joint angles for the specified hand
        angles = []

        for joint_index in joint_indices:
            joint_state = p.getJointState(self.adam.robot_id, joint_index)
            angles.append(joint_state[0])

        # Normalize the angles using the normalization values to a range [0-1000]

        angles_normalized = []

        for i in range(self.num_hand_joints): angles_normalized.append(angles[i] / self.joint_norm_values[self.joint_names[i]])

        # Average the normalized angles for each dof
        dof_sums = [0.0] * self.num_dofs
        dof_counts = [0] * self.num_dofs

        for joint_idx, dof_idx in enumerate(self.dof_joints_indices):
            dof_sums[dof_idx] += angles_normalized[joint_idx]
            dof_counts[dof_idx] += 1

        dof_values = [round(dof_sums[i]/dof_counts[i], 0) for i in range(self.num_dofs)]

        return dof_values


    def dofs_to_joint_angles(self, dofs):
        ''' 
        Convert the dof values to joint angles using the normalization values. 
        The dof values are normalized in a range [0-1000] and the joint angles are in radians.
        Args:
            dofs (list): The dof values of the hand link.
        Returns:
            joint_angles (list): The joint angles of the hand link.
        '''
        
        joint_angles = []

        # Convert the dof values to joint angles using the normalization values
        for i in range(self.num_hand_joints):
            joint_angles.append(dofs[self.dof_joints_indices[i]]*self.joint_norm_values[self.joint_names[i]])

        return joint_angles
