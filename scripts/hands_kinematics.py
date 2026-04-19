import pybullet as p
import json

class HandsKinematics():
    
    def __init__(self, adam):
        self.adam = adam
        
        # Load hand kinematics configuration
        with open(self.adam.hand_json_path, 'r', encoding='utf-8') as f:
            self.hand_kinematics_json = json.load(f)

        # Dynamic Attributes
        self.joint_norm_values = self.hand_kinematics_json["joint_norm_values"]
        self.joint_names = list(self.joint_norm_values.keys())
        self.dof_joints_indices = self.hand_kinematics_json["dof_joints_indices"]
        
        self.num_hand_joints = len(self.joint_names)
        self.num_dofs = 6
        self.dof_range = [0, 1000]

        # Process hand link information to create mappings for contact checking
        self.links_in_dofs = {'right': {}, 'left': {}}
        self.hand_link_indices = {'right': [], 'left': []}

        for side, prefix in [('right', 'R_'), ('left', 'L_')]:
            link_ids = []
            
            for finger, base_link_names in self.hand_kinematics_json["finger_base_link_names"].items():
                link_ids_for_finger = []

                for base_name in base_link_names:
                    full_link_name = prefix + base_name 

                    try:
                        link_id = self.adam.robot_info["links"][full_link_name]["id"]
                        link_ids.append(link_id)
                        link_ids_for_finger.append(link_id)
                    except KeyError:
                        print(f"[WARNING] Link {full_link_name} not found in robot_info.json!")
                
                self.links_in_dofs[side][finger] = link_ids_for_finger

            self.hand_link_indices[side] = link_ids

        # Set high friction for hand links to improve grasping stability
        for side in ['right', 'left']:
            for link_id in self.hand_link_indices[side]:
                p.changeDynamics(self.adam.robot_id, link_id, lateralFriction=1000, spinningFriction=1, frictionAnchor=1)


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
            joint_angles.append((1000 - dofs[self.dof_joints_indices[i]])*self.joint_norm_values[self.joint_names[i]])

        return joint_angles


    def close_hand(self, hand_side='right', thumb_abd_value=0, close_speed=10, force_threshold=2):
        """
        Closes the hand while keeping thumb abduction fixed for an antipodal grasp.
        Args:
            thumb_abd_value: 0-1000 value for thumb spread. 
                             (e.g., 500 for middle position)
        """
        # Start all flexion DOFs at 1000 (Open)
        # We only close indices 0-4 (Pinky, Ring, Middle, Index, Thumb Flex)
        current_dofs = [1000.0] * self.num_dofs
        
        # Set the Abduction DOF (index 5) to the fixed target immediately
        current_dofs[5] = thumb_abd_value
        
        # Track active flexion DOFs only (indices 0 to 4)
        dof_active = [True if i < 5 else False for i in range(self.num_dofs)]
        
        dof_to_finger_key = ['pinky', 'ring', 'middle', 'index', 'thumb', 'thumb']

        while any(dof_active):
            contact_points = p.getContactPoints(bodyA=self.adam.robot_id)
            
            for i in range(5): # Only iterate through flexion DOFs
                if not dof_active[i]:
                    continue

                finger_key = dof_to_finger_key[i]
                target_links = self.links_in_dofs[hand_side][finger_key]
                
                # Check force
                total_force = sum(c[9] for c in contact_points if c[3] in target_links)

                if total_force > force_threshold:
                    dof_active[i] = False
                elif current_dofs[i] <= self.dof_range[0]:
                    current_dofs[i] = self.dof_range[0]
                    dof_active[i] = False
                else:
                    current_dofs[i] -= close_speed # Closing speed

            # Move hand (DOF 5 remains constant at thumb_abd_value)
            self.move_hand_to_dofs(hand_side, current_dofs)

            self.adam.step()

        # Success check: Did the thumb and at least one other finger make contact?
        thumb_contact = not dof_active[4] and current_dofs[4] > 5
        other_contact = any(not dof_active[i] and current_dofs[i] > 5 for i in range(4))
        
        return thumb_contact and other_contact