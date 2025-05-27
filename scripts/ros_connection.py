import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pybullet as p

import sys
sys.path.insert(0, '/home/gonzalo/catkin_ws/devel/lib/python3/dist-packages')

from inspire_hand.srv import *


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
        

        self.hand_timer = rospy.Timer(rospy.Duration(0.02), self.hand_services_callback)
        self.real_to_sim_timer = rospy.Timer(rospy.Duration(1/120), self.real_to_sim)
        
        # Create a publisher for the base pose
        self.pub = rospy.Publisher('/robot/move_base/cmd_vel', Twist, queue_size=1)
        self.wheel_radius = 0.0762
        self.wheel_distance = 0.436
        
        self.adam = adam

        self.latest_arm_joint_states = {'right': None, 'left': None}
        self.latest_hand_dof_states = {'right': None, 'left': None}
        
        
    def send_velocity(self, linear_speed=0.0, angular_speed=0.0):
        '''
        Send velocity commands to the robot's base.
        '''
        
        twist = Twist()
        
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        
        self.pub.publish(twist)
        
        #rospy.loginfo("Published velocity command to robot base")

    def wait(self, secs):
        # ROS rate
        rate = rospy.Rate(120)

        # Simulación
        if not self.adam.useRealTimeSimulation:
            iter = round(secs/self.adam.t, 0)

            for i in range(iter):
                p.stepSimulation()
                rospy.sleep(self.adam.t)
        
        else:
            iter = secs*120

            for i in range(iter):
                p.stepSimulation()
                rate.sleep()

    def arm_joint_state_callback(self, msg, arm):
        '''
        Callback function for joint state messages.
        Args:
            msg (JointState): The joint state message.
            arm (str): The arm for which the joint state is received ('left' or 'right').
        '''
        
        # Log the received joint state message
        # rospy.loginfo(f"Received joint state message for {arm} arm: {msg.position}")

        joint_angles = list(msg.position)

        # Convert from robot to simulation joint angles
        joint_angles[0], joint_angles[2] = joint_angles[2], joint_angles[0]

        # Store the latest joint angles
        try: self.latest_arm_joint_states[arm] = joint_angles
        except: pass

    
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
        
        #rospy.loginfo("Published joint trajectory for %s arm", arm)

        
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
    

    def real_to_sim(self,event):
        '''
        Convert the latest arm joint states from the robot to the simulation.
        '''

        if self.latest_arm_joint_states['right'] and self.latest_arm_joint_states['left']:

            #rospy.loginfo(f"JointState del robot real left: {self.latest_arm_joint_states['left']}")
            #rospy.loginfo(f"JointState del robot real right: {self.latest_arm_joint_states['right']}")

            # Convert from robot to simulation joint angles
            self.adam.arm_kinematics.move_arm_joints_to_angles('left', self.latest_arm_joint_states['left'])
            self.adam.arm_kinematics.move_arm_joints_to_angles('right', self.latest_arm_joint_states['right'])    

        if self.latest_hand_dof_states['right'] and self.latest_hand_dof_states['left']:

            #rospy.loginfo(f"DofState del robot real left: {self.latest_hand_dof_states['left']}")
            #rospy.loginfo(f"DofState del robot real right: {self.latest_hand_dof_states['right']}")

            # Convert from robot to simulation joint angles
            self.adam.hand_kinematics.move_hand_to_dofs('left', self.latest_hand_dof_states['left'])
            self.adam.hand_kinematics.move_hand_to_dofs('right', self.latest_hand_dof_states['right'])


    def call_get_angle_act(self, arm):
        '''
        Call the get_angle_act service to get the joint angles for the specified hand.
        Args:
            arm (str): The arm to get the joint angles for ('left' or 'right').
        '''
        
        service_get_angle_act = f'/robot/{arm}/inspire_hand/get_angle_act'
        
        rospy.wait_for_service(service_get_angle_act)

        try:
            get_angle_set_service = rospy.ServiceProxy(service_get_angle_act, get_angle_act)
            response = get_angle_set_service()

            print(f"Joint angles in {arm} arm from service:", list(response.curangle))

            dofs = list(response.curangle)

            # Store the latest hand dof states
            self.latest_hand_dof_states[arm] = dofs

            return self.latest_hand_dof_states[arm]

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


    def call_set_angle(self, arm, dofs):
        '''
        Call the set_angle service to set the joint angles for the specified hand.
        Args:
            arm (str): The arm to set the joint angles for ('left' or 'right').
            dofs (list): The joint angles to set.
        '''

        # Check if the dofs list has exactly 6 elements
        if len(dofs) != 6:
            rospy.logerr("The dofs list must contain exactly 6 elements.")
            return

        service_set_angle = f'/robot/{arm}/inspire_hand/set_angle'
        rospy.wait_for_service(service_set_angle)
        
        try:
            # Call the service to set the angles
            set_angle_service = rospy.ServiceProxy(service_set_angle, set_angle)
            response = set_angle_service(dofs[0], dofs[1], dofs[2], dofs[3], dofs[4], dofs[5])
            print(f"Set angles in {arm} arm:", dofs)
        
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


    def hand_services_callback(self, event):
        '''
        Callback function for calling the hands services periodically.
        '''

        self.call_get_angle_act('left')
        self.call_get_angle_act('right')


    def teleop_real_base(self):
        '''
        Teleoperate the base using the keyboard.
        '''
        leftSpeed, rightSpeed = self.adam.teleop.teleoperate_base(move_sim=False)
        # Convertir velocidades de rueda (rad/s) a velocidad lineal y angular
        linear_x = self.wheel_radius * (rightSpeed + leftSpeed) / 2.0
        angular_z = self.wheel_radius * (rightSpeed - leftSpeed) / self.wheel_distance
        
        self.send_velocity(linear_x, angular_z)
        if angular_z == 0 and linear_x != 0:
            print("Left speed", leftSpeed)
            print("Right speed", rightSpeed)
            self.adam.navigation.move_wheels(leftSpeed*4, rightSpeed*4, force=50)
        else:
            
            self.adam.navigation.move_wheels(leftSpeed, rightSpeed, force=50)
