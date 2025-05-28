import PyKDL
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import numpy as np
import math

class Kinematics:

    def __init__(self, adam, urdf_path):
        
        # Initialize the Kinematics class with the ADAM robot and URDF path
        self.adam = adam

        # Load the URDF model
        self.robot_urdf = URDF.from_xml_file(urdf_path)

        # Convert URDF to KDL Tree
        ok, self.kdl_tree = treeFromUrdfModel(self.robot_urdf)
        if not ok:
            raise RuntimeError("Failed to parse URDF into a KDL tree")
        
        # RIGHT ARM
        right_base_link = "rb1_right_arm_base_link"
        right_end_effector = "R_hand_base_link"
        self.right_kdl_chain = self.kdl_tree.getChain(right_base_link, right_end_effector) # Build the kinematic chain of the right arm
        self.world_to_right_base_transform = self.get_transform_to_base(right_base_link) # Transform from robot base to right arm base link
        self.right_fk_solver = PyKDL.ChainFkSolverPos_recursive(self.right_kdl_chain) # Right arm FK solver

        # LEFT ARM
        left_base_link = "rb1_left_arm_base_link"
        left_end_effector = "L_hand_base_link"
        self.left_kdl_chain = self.kdl_tree.getChain(left_base_link, left_end_effector) # Build the kinematic chain of the left arm
        self.world_to_left_base_transform = self.get_transform_to_base(left_base_link) # Transform from robot base to left arm base link
        self.left_fk_solver = PyKDL.ChainFkSolverPos_recursive(self.left_kdl_chain) # Left arm FK solveryy


    def get_transform_to_base(self, base_link):
        '''
        Get the transform from the world to the base link of the arm.
        Args:
            base_link (str): The name of the base link of the arm.
        Returns:
            PyKDL.Frame: The transform from the world to the base link of the arm.
        '''    
    
        world_to_base_transform = PyKDL.Frame()

        current_link = base_link
        while current_link:
            for joint in self.robot_urdf.joints:
                if joint.child == current_link:
                    # Parse joint's origin (translation + rotation)
                    origin = joint.origin
                    translation = PyKDL.Vector(*origin.xyz)
                    rotation = PyKDL.Rotation.RPY(*origin.rpy)
                    joint_transform = PyKDL.Frame(rotation, translation)

                    # Pre-multiply to accumulate transform
                    world_to_base_transform = joint_transform * world_to_base_transform

                    # Move to the parent link
                    current_link = joint.parent
                    break
            else:
                # No more parents; reached the root
                break
        return world_to_base_transform


    def calculate_arm_forward_kinematics(self, arm, joint_angles):
        '''
        Calculate the forward kinematics of the specified arm.
        Args:
            arm (str): The arm to calculate the forward kinematics for ('left' or 'right').
            joint_angles (list): A list of joint angles in radians.
        Returns:
            tuple: A tuple containing the end effector position (x, y, z) and orientation (qx, qy, qz, qw).
        '''
        
        if arm == 'right':
            world_to_base_transform = self.world_to_right_base_transform
            kdl_chain = self.right_kdl_chain
            fk_solver = self.right_fk_solver

        elif arm == 'left':
            world_to_base_transform = self.world_to_left_base_transform
            kdl_chain = self.left_kdl_chain
            fk_solver = self.left_fk_solver

        # Get the number of joints in the kinematic chain
        num_joints = kdl_chain.getNrOfJoints()

        # Create PyKDL joints
        joint_positions = PyKDL.JntArray(num_joints)

        for i, angle in enumerate(joint_angles):
            joint_positions[i] = angle

        # Prepare a frame to store the result
        end_effector_frame = PyKDL.Frame()

        # Compute forward kinematics
        if fk_solver.JntToCart(joint_positions, end_effector_frame) >= 0:

            # Transform FK result to the world frame
            end_effector_in_world = world_to_base_transform * end_effector_frame

            position = end_effector_in_world.p  # Position as PyKDL.Vector
            orientation = end_effector_in_world.M  # Orientation as PyKDL.Rotation

            # Extract orientation as quaternions
            quaternions = orientation.GetQuaternion()
            
        else:
            print("Error computing forward kinematics")

        return [position, quaternions]


    def calculate_pregrasp_pose(self, grasp_pose, offset=[0, 0, 0]):
        '''
        Calculate the pregrasp pose based on the grasp pose.
        Args:
            grasp_pose (list): The grasp pose in the format [[x, y, z], [qx, qy, qz, qw]].
            offset (list): The offset to apply to the grasp pose in the format [dx, dy, dz].
        Returns:
            list: The pregrasp pose in the same format as the input.
        '''

        # Extract position and orientation from the grasp pose
        position = grasp_pose[0]  # [x, y, z]
        orientation = grasp_pose[1]  # [qx, qy, qz, qw]

        # Convert the position and orientation into a PyKDL Frame
        rotation = PyKDL.Rotation.Quaternion(*orientation)
        translation = PyKDL.Vector(*position)
        grasp_frame = PyKDL.Frame(rotation, translation)

        # Define the displacement along the local z-axis of the gripper frame
        offset_frame = PyKDL.Frame(PyKDL.Rotation.Identity(), PyKDL.Vector(offset[0], offset[1], offset[2]))

        # Apply the displacement in the gripper's local frame
        pregrasp_frame = grasp_frame * offset_frame

        # Extract the position and orientation from the pregrasp_frame
        pregrasp_position = [pregrasp_frame.p.x(), pregrasp_frame.p.y(), pregrasp_frame.p.z()]
        pregrasp_orientation = pregrasp_frame.M.GetQuaternion()

        # Return the pregrasp pose in the same format as the input
        return [pregrasp_position, pregrasp_orientation]


    def get_pose_error(self, current_pose, target_pose):
        '''
        Calculate the pose error between the current and target poses.
        Args:
            current_pose (list): The current pose in the format [[x, y, z], [qx, qy, qz, qw]].
            target_pose (list): The target pose in the same format.
        Returns:
            tuple: A tuple containing the position error and orientation error.
        '''

        position_error = [target_pose[0][0] - current_pose[0][0], target_pose[0][1] - current_pose[0][1], target_pose[0][2] - current_pose[0][2]]
        
            # Convert quaternions to PyKDL rotations
        current_rotation = PyKDL.Rotation.Quaternion(current_pose[1][0],
                                                    current_pose[1][1],
                                                    current_pose[1][2],
                                                    current_pose[1][3])
        target_rotation = PyKDL.Rotation.Quaternion(target_pose[1][0],
                                                    target_pose[1][1],
                                                    target_pose[1][2],
                                                    target_pose[1][3])

        # Compute the relative rotation from current to target
        relative_rotation = current_rotation.Inverse() * target_rotation

        # Convert the relative rotation to axis-angle representation
        angle, axis = relative_rotation.GetRotAngle()
        orientation_error = [angle * axis[0], angle * axis[1], angle * axis[2]]

        # Combine position and orientation errors
        error = np.zeros(6)  # 6D vector for 3D position and 3D orientation
        error[0:3] = position_error
        error[3:6] = orientation_error

        pos_error = math.sqrt(pow(error[0],2) + pow(error[1],2) + pow(error[2],2))
        angle_error = angle

        return pos_error, angle_error