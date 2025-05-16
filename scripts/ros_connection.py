import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pybullet as p

class ROSConnection:
    def __init__(self, adam):
        # Initialize the ROS node
        rospy.init_node('adam_ros_node', anonymous=True)

        # Create a subscriber for the robot's joint states
        self.joint_state_sub_left = rospy.Subscriber('/robot/left_arm/joint_states', JointState, self.arm_joint_state_callback, callback_args='left', queue_size=1)
        self.joint_state_sub_right = rospy.Subscriber('/robot/right_arm/joint_states', JointState, self.arm_joint_state_callback, callback_args='right', queue_size=1)

        # Create a publisher for the robot's pose
        self.arm_joint_pub = {'left': rospy.Publisher('/robot/left_arm/scaled_pos_traj_controller/command', JointTrajectory, queue_size=1),
                              'right': rospy.Publisher('/robot/right_arm/scaled_pos_traj_controller/command', JointTrajectory, queue_size=1)}
        
        self.adam = adam

        self.latest_arm_joint_states = {'right': None, 'left': None}

        self.last_arm_joint_state_log_time = rospy.Time.now()


    def arm_joint_state_callback(self, msg, arm):
        '''
        Callback function for joint state messages.
        Args:
            msg (JointState): The joint state message.
            arm (str): The arm for which the joint state is received ('left' or 'right').
        '''
        
        # Log the received joint state message
        rospy.loginfo(f"Received joint state message for {arm} arm: {msg.position}")

        joint_angles = list(msg.position)

        # Convert from robot to simulation joint angles
        joint_angles[0], joint_angles[2] = joint_angles[2], joint_angles[0]

        # Store the latest joint angles
        self.latest_arm_joint_states[arm] = joint_angles

    
    def arm_publish_joint_trajectory(self, arm, joint_angles):
        '''
        Publish the joint trajectory for the specified arm.
        Args:
            arm (str): The arm to publish the joint trajectory for ('left' or 'right').
            joint_angles (list): The joint angles to publish.
        '''
        
        # Create a JointTrajectory message
        traj_msg = self.convert_joints_to_msg(joint_angles)

        # Publish the message
        self.arm_joint_pub[arm].publish(traj_msg)
        
        rospy.loginfo("Published joint trajectory for %s arm", arm)

        
    def convert_joints_to_msg(self, joint_positions):
        """
        Crea un mensaje JointTrajectory a partir de una lista de nombres y posiciones.
        
        Args:
            joint_names (list of str): Nombres de las articulaciones.
            joint_positions (list of float): Posiciones deseadas para cada articulación.
            
        Returns:
            JointTrajectory: Mensaje listo para publicar.
        """
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['robot_left_arm_elbow_joint', 'robot_left_arm_shoulder_lift_joint', 'robot_left_arm_shoulder_pan_joint',
                                'robot_left_arm_wrist_1_joint', 'robot_left_arm_wrist_2_joint', 'robot_left_arm_wrist_3_joint']
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0, 0, 0, 0, 0, 0]
        point.accelerations = [0, 0, 0, 0, 0, 0]
        point.effort = [0.8682, 1.612, -1.5094, 0.3217, -0.2149, 0.0096]
        point.time_from_start.secs = 1  # Tiempo opcional para alcanzar la posición
        
        traj_msg.points.append(point)

        return traj_msg
    

    def arm_real_to_sim(self):
        '''
        Convert the latest arm joint states from the robot to the simulation.
        '''

        now = rospy.Time.now()

        if self.latest_arm_joint_states['right'] and self.latest_arm_joint_states['left']:

            rospy.loginfo(f"JointState del robot real left: {self.latest_arm_joint_states['left']}")
            rospy.loginfo(f"JointState del robot real right: {self.latest_arm_joint_states['right']}")

            # Convert from robot to simulation joint angles
            self.adam.arm_kinematics.move_arm_joints_to_angles('left', self.latest_arm_joint_states['left'])
            self.adam.arm_kinematics.move_arm_joints_to_angles('right', self.latest_arm_joint_states['right'])    

            self.last_arm_joint_state_log_time = now