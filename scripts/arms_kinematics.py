import pybullet as p
import time
import math
from pykdl_kinematics import Kinematics

# Class for kinematics
class ArmsKinematics:
    def __init__(self, adam):
        self.adam = adam
        self.adamDynamics = adam.dynamics
        self.pykdl = Kinematics(adam, adam.urdf_path)
    
    # Calculate the inverse kinematics for the specified arm
    def calculate_arm_inverse_kinematics(self, robot_id, ee_index, target_pose, null=True):
        '''
        Calculate the inverse kinematics for the specified arm.
        Args:
            robot_id (int): The ID of the robot in the simulation.
            ee_index (int): The index of the end effector.
            target_pose (list): The target position and orientation.
            null (bool): Whether to use null space or not.
        Returns:
            ik_solution: The inverse kinematics solution.
        '''
        
        # Get desired pose and orientation
        target_position = target_pose[0]
        target_orientation = target_pose[1]

        if null:
            ik_solution = p.calculateInverseKinematics(robot_id, ee_index, target_position, target_orientation,lowerLimits=self.adam.ll,
                                                    upperLimits=self.adam.ul,
                                                    jointRanges=self.adam.jr,
                                                    restPoses=self.adam.rp)
        else:
            ik_solution = p.calculateInverseKinematics(robot_id, ee_index, target_position, target_orientation,jointDamping=self.adam.jd,
                                                solver=0,
                                                maxNumIterations=100,
                                                residualThreshold=.01)
        return ik_solution
    
    def move_arm_to_pose(self, arm, target_pose, target_link, pos_act=None, vel_act=None, accurate=False, threshold=[0.01, 0.035], type="both"):
        '''
        Move the arm to the specified pose.
        Args:
            arm (str): The arm to move ('left' or 'right').
            target_pose (list): The target position and orientation.
            target_link (str): The target link ('ee', 'dummy' or 'hand').
            pos_act (list): The current position of the arm.
            vel_act (list): The current velocity of the arm.
            accurate (bool): Whether to use accurate inverse kinematics.
            threshold (list): The position and orientation thresholds.
            type (str): The type of check ('both', 'pos' or 'angle').
        Returns:   
            ik_solution (list): The inverse kinematics solution.
            arm_solution (list): The arm joint angles.
            vel_des (list): The desired velocity of the arm.
        '''

        # Get the joint indices and target link index
        joint_indices, rev_joint_indices = self.get_arm_joint_indices(arm)
        target_link_index = self.get_arm_link_index(arm, target_link)

        # Compute IK solution
        ik_solution = self.calculate_arm_inverse_kinematics(self.adam.robot_id, target_link_index, target_pose)
        arm_solution = [ik_solution[i] for i in rev_joint_indices]


        # Include dynamics
        if self.adam.Dynamics:

            # Compute inverse dynamics
            torque, vel_des, acc_des = self.adamDynamics.calculate_arm_inverse_dynamics(arm_solution, pos_act, vel_act, arm)

            for i, joint_id in enumerate(joint_indices):
                #set the joint friction
                p.setJointMotorControl2(self.adam.robot_id, joint_id, p.VELOCITY_CONTROL, targetVelocity=0, force=20)
                #apply a joint torque
                p.setJointMotorControl2(self.adam.robot_id, joint_id, p.TORQUE_CONTROL, force=torque[i])
            p.stepSimulation()

            #Calculamos la dinámica directa para obtener la aceleracion de las articulaciones al aplicar una fuerza sobre ellas
            acc = self.adam.calculate_arm_forward_dynamics(torque,arm)

        # Without dynamics
        else:

            # Calculo de la cinematica inversa precisa
            if accurate:

                while not closeEnough:
                    for i, joint_id in enumerate(joint_indices):
                        p.setJointMotorControl2(self.adam.robot_id, joint_id, p.POSITION_CONTROL, arm_solution[i])
                    
                    closeEnough, _, _ = self.adam.check_reached(arm, target_pose, target_link, threshold=threshold, type=type)

            # Calculo de la cinematica inversa sin precision
            else:
                for i, joint_id in enumerate(joint_indices):
                    p.setJointMotorControl2(self.adam.robot_id, joint_id, p.POSITION_CONTROL, arm_solution[i])

            vel_des = None
        
        return ik_solution, arm_solution, vel_des


    
    def move_arm_to_multiple_poses(self, arm, target_link, poses_arm1, poses_arm2=None, accurate=False, threshold=[0.01, 0.035]):
        '''
        Move the arm to multiple poses.
        Args:
            arm (str): The arm to move ('left', 'right' or 'both').
            target_link (str): The target link ('ee', 'dummy' or 'hand').
            poses_arm1 (list): A list of target positions and orientations for the first arm.
            poses_arm2 (list): A list of target positions and orientations for the second arm (only for 'both').
            accurate (bool): Whether to use accurate inverse kinematics.
            threshold (list): The position and orientation thresholds.
        Returns:
            None
        '''

        if arm == "left" or arm == "right":

            for pose in poses_arm1:

                # Check if collision is detected
                self.adam.detect_autocollisions()

                self.move_arm_to_pose(arm, poses_arm1, target_link=target_link, accurate=accurate, threshold=threshold)

                # Avanzar la simulación para que los movimientos se apliquen
                if not self.adam.useRealTimeSimulation:
                    p.stepSimulation()
                    time.sleep(self.adam.t)
            
        # Move both arms simultaneously
        if arm == "both":

            if poses_arm2 is None: raise ValueError("Debes proporcionar poses2 para mover ambos brazos")

            for pose_left, pose_right in zip(poses_arm1, poses_arm2):
                
                # Check if collision is detected
                self.adam.detect_autocollisions()
                
                self.move_arm_to_pose('right', poses_arm1, target_link=target_link, accurate=accurate, threshold=threshold)
                self.move_arm_to_pose('left', poses_arm2, target_link=target_link, accurate=accurate, threshold=threshold)

                # Avanzar la simulación para que los movimientos se apliquen
                if not self.adam.useRealTimeSimulation:
                    p.stepSimulation()
                    time.sleep(self.adam.t)
    

    def check_reached(self, arm, target_pose, target_link, threshold=[0.01, 0.035], type="both"):
        '''
        Check if the arm can reach the target pose.
        Args:
            arm (str): The arm to check ('left' or 'right').
            target_pose (list): The target position and orientation.
            target_link (str): The target link ('ee', 'dummy' or 'hand').
            threshold (list): The position and orientation thresholds.
            type (str): The type of check ('both', 'pos' or 'angle').
        Returns:
            bool: True if the arm can reach the target pose, False otherwise.
            pos_error (float): The position error.
            angle_error (float): The orientation error.
        '''

        # Initialize close reached flag
        reached = False
        
        # Calculate the current pose
        current_pose = self.get_arm_link_pose(arm, target_link)

        # Calculate pose error
        pos_error, angle_error = self.pykdl.get_pose_error(current_pose, target_pose)
        
        # Reachability check
        def within_threshold(pos, angle):
            if type == "both": return pos < threshold[0] and angle < threshold[1]
            elif type == "pos": return pos < threshold[0]
            elif type == "angle": return angle < threshold[1]
            else: raise ValueError("Type must be 'both', 'pos' or 'angle'.")

        reached = within_threshold(pos_error, angle_error)

        return reached, pos_error, angle_error
    

    def check_reachability(self, arm, target_pose, target_link, threshold=[0.01, 0.035], type="both", pregrasp=False, pregrasp_offset=[0, 0, 0]):
        '''
        Check if the arm can reach the target pose.
        Returns:
            bool: True if the arm can reach the target pose, False otherwise.
            pos_error (float): The position error.
            angle_error (float): The orientation error.
        '''
        target_link_index = self.get_arm_link_index(arm, target_link)
        _, rev_joint_indices = self.get_arm_joint_indices(arm)

        # Inverse kinematics and FK for target
        ik_solution_target = self.calculate_arm_inverse_kinematics(self.adam.robot_id, target_link_index, target_pose)
        arm_solution_target = [ik_solution_target[i] for i in rev_joint_indices]
        pykdl_target_pose = self.pykdl.calculate_arm_forward_kinematics(arm, arm_solution_target)
        pos_error, angle_error = self.pykdl.get_pose_error(pykdl_target_pose, target_pose) # Error between target pose and FK pose

        # If pregrasp, do the same
        if pregrasp:
            pregrasp_pose = self.pykdl.calculate_pregrasp_pose(target_pose, pregrasp_offset)
            ik_solution_pregrasp = self.calculate_arm_inverse_kinematics(self.adam.robot_id, target_link_index, pregrasp_pose)
            arm_solution_pregrasp = [ik_solution_pregrasp[i] for i in rev_joint_indices]
            pykdl_pregrasp_pose = self.pykdl.calculate_arm_forward_kinematics(arm, arm_solution_pregrasp)
            pos_error_pg, angle_error_pg = self.pykdl.get_pose_error(pykdl_pregrasp_pose, pregrasp_pose) # Error between target pose and FK pose
        else: pos_error_pg = angle_error_pg = 0  # dummy values if unused

        # Reachability check
        def within_threshold(pos, angle):
            if type == "both": return pos < threshold[0] and angle < threshold[1]
            elif type == "pos": return pos < threshold[0]
            elif type == "angle": return angle < threshold[1]
            else: raise ValueError("Type must be 'both', 'pos' or 'angle'.")

        reachable = within_threshold(pos_error, angle_error)
        if pregrasp: reachable = reachable and within_threshold(pos_error_pg, angle_error_pg)

        return reachable, pos_error, angle_error, pos_error_pg, angle_error_pg


    def move_arm_joints_to_angles(self, arm, angles):
        '''
        Move the arm joints to the specified angles.
        Args:
            arm (str): The arm to move ('left' or 'right').
            angles (list): A list of joint angles in radians.
        '''
        
        joint_indices, _ = self.get_arm_joint_indices(arm)

        #Asignar los ángulos a cada articulación del brazo
        for i, joint_id in enumerate(joint_indices):
            p.setJointMotorControl2(self.adam.robot_id, joint_id, p.POSITION_CONTROL, angles[i])

    
    def set_arm_pose(self, arm, type, pose, target_link="ee"):
        '''
        Set the arm pose.
        Args:
            arm (str): The arm to set the pose for ('left' or 'right').
            type (str): The type of pose ('joint' or 'pose').
            pose (list): The pose to set.
            target_link (str): The target link ('ee', 'dummy' or 'hand').
        '''

        # Get the joint indices and target link index
        joint_indices, rev_joint_indices = self.get_arm_joint_indices(arm)
        target_link_index = self.get_arm_link_index(arm, target_link)
        
        # Set joint angles
        if type == "joint" and len(pose) == len(joint_indices): angles = pose        
        
        # Set pose
        elif type == "pose" and len(pose) == 2 and len(pose[0]) == 3 and len(pose[1]) == 4:    
            ik_solution = self.calculate_arm_inverse_kinematics(self.adam.robot_id, target_link_index, pose[0], pose[1])
            angles = [ik_solution[i] for i in rev_joint_indices]            
        
        else: raise ValueError("Pose type must be 'joint' or 'pose' or pose format is incorrect.")

        # Set arm in simulation
        for i, joint_id in enumerate(joint_indices):
            p.resetJointState(self.adam.robot_id, joint_id, angles[i])

        if not self.adam.useRealTimeSimulation:
            p.stepSimulation()
            time.sleep(self.adam.t)


    def get_arm_link_pose(self, arm, target_link):
        '''
        Get the position and orientation of the specified arm link.
        Args:
            arm (str): The arm to get the link pose for ('left' or 'right').
            target_link (str): The target link ('ee', 'dummy' or 'hand').
        Returns:
            tuple: A tuple containing the position (x, y, z) and orientation (qx, qy, qz, qw).
        '''

        # Get the index of the specified arm link (ee, dummy or hand)
        target_link_index = self.get_arm_link_index(arm, target_link)
            
        # Get the link state
        link_state = p.getLinkState(self.adam.robot_id, target_link_index)
        
        return link_state[4],link_state[5]
    

    def get_arm_link_index(self, arm, target_link):
        '''
        Get the index of the specified arm link.
        Args:
            arm (str): The arm to get the link index for ('left' or 'right').
            target_link (str): The target link ('ee', 'dummy' or 'hand').
        Returns:
            int: The index of the specified arm link.
        '''

        if not arm == "left" and not arm == "right": raise ValueError("El brazo debe ser 'left' o 'right'.")

        if target_link == "ee": target_link_index = self.adam.ee_index[arm]
        elif target_link == "dummy": target_link_index = self.adam.dummy_index[arm]
        elif target_link == "hand": target_link_index = self.adam.hand_base_index[arm]
        else: raise ValueError("Invalid target_link: must be 'ee', 'dummy', or 'hand'.")
            
        return target_link_index
    

    def get_arm_joint_indices(self, arm):
        '''
        Get the joint indices of the specified arm.
        Args:
            arm (str): The arm to get the joint indices for ('left' or 'right').
        Returns:
            list: A list of joint indices.
        '''

        if arm == "left":
            joint_indices = self.adam.ur3_left_arm_joints
            rev_joint_indices = self.adam.ur3_left_arm_rev_joints
        elif arm == "right":
            joint_indices = self.adam.ur3_right_arm_joints
            rev_joint_indices = self.adam.ur3_right_arm_rev_joints
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")
        
        return joint_indices, rev_joint_indices