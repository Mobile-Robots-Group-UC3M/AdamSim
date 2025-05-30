import pybullet as p
import pybullet_data
from scripts.arms_dynamics import ArmsDynamics
from scripts.teleoperation import Teleop
from scripts.arms_kinematics import ArmsKinematics
from scripts.hands_kinematics import HandsKinematics
from scripts.sensors import Sensors
from scripts.navigation import Navigation
from scripts.utils import Utils
from scripts.ros_connection import ROSConnection
import time

# Class for ADAM robot
class ADAM:
    def __init__(self, urdf_path, useRealTimeSimulation, used_fixed_base=True, use_ros=True):
        
        # Load environment
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        if useRealTimeSimulation: p.setRealTimeSimulation(1)
        else: p.setRealTimeSimulation(0)

        # Load world plane
        self.plane_id = p.loadURDF("plane.urdf")
        p.setPhysicsEngineParameter(enableConeFriction=1)
        
        """ p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setPhysicsEngineParameter(enableConeFriction=1 """
        
        # Wheel indices
        self.left_wheel_joint = 4
        self.right_wheel_joint = 3

        # URDF path
        self.urdf_path = urdf_path

        # Spawn ADAM robot model
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=used_fixed_base, flags=p.URDF_USE_SELF_COLLISION)
        
        #Establecer fricción para las ruedas (índices de joint correctos)
        """ p.changeDynamics(self.robot_id, 3, lateralFriction=0.02)
        p.changeDynamics(self.robot_id, 4, lateralFriction=0.02)
        p.changeDynamics(self.robot_id, 5, lateralFriction=0.02)
        p.changeDynamics(self.robot_id, 6, lateralFriction=0.02)
        p.changeDynamics(self.robot_id, 7, lateralFriction=0.02)
        
        p.changeDynamics(self.robot_id, 3, linearDamping=0)
        p.changeDynamics(self.robot_id, 4, linearDamping=0)
        p.changeDynamics(self.robot_id, 5, linearDamping=0)
        p.changeDynamics(self.robot_id, 6, linearDamping=0)
        p.changeDynamics(self.robot_id, 7, linearDamping=0)
        
        p.changeDynamics(self.robot_id, 3, angularDamping=0)
        p.changeDynamics(self.robot_id, 4, angularDamping=0)
        p.changeDynamics(self.robot_id, 5, angularDamping=0)
        p.changeDynamics(self.robot_id, 6, angularDamping=0)
        p.changeDynamics(self.robot_id, 7, angularDamping=0)
        
        p.changeDynamics(self.robot_id, -1, lateralFriction=0.02)
        #p.changeDynamics(sphere,-1,rollingFriction=10)
        p.changeDynamics(self.robot_id, -1, linearDamping=0)
        p.changeDynamics(self.robot_id, -1, angularDamping=0) """

        # (Opcional) Establecer fricción para el cuerpo principal
        #p.changeDynamics(self.robot_id, base_link_index, lateralFriction=1.0)

        # Change simulation mode
        self.useRealTimeSimulation = useRealTimeSimulation
        self.use_ros = use_ros
        self.t = 0.01


        # Arm revolute joint indices
        self.ur3_right_arm_joints = list(range(20,26))  # Brazo derecho
        self.ur3_left_arm_joints = list(range(45,51)) # Brazo izquierdo

        self.ur3_right_arm_rev_joints = list(range(5,11))  # Brazo derecho
        self.ur3_left_arm_rev_joints = list(range(23,29)) # Brazo izquierdo

        # Hand revolute joint indices
        self.hand_joint_indices = {'right': list(range(30, 42)), 'left': list(range(55, 67))}

        # Torso link indices
        self.torso_link_indices = list(range(73, 76))

        # Other indices
        self.ee_index = {'right': 26, 'left': 51}
        self.hand_base_index = {'right': 28, 'left': 53}
        self.dummy_index = {'right': 29, 'left': 54}
        
        
        

        # ADAM MODULES
        self.arm_dynamics = ArmsDynamics(self)
        self.arm_kinematics = ArmsKinematics(self)
        self.hand_kinematics = HandsKinematics(self)
        if not used_fixed_base: self.navigation = Navigation(self)
        self.teleop = Teleop(self)
        self.sensors = Sensors(self)
        self.utils = Utils(self)
        if use_ros: self.ros = ROSConnection(self)      
        
        
        #Null space definition
        #lower limits for null space
        self.ll = [-6.28]*6
        #upper limits for null space
        self.ul = [6.28]*6
        #joint ranges for null space
        self.jr = [6.28]*6
        #restposes for null space
        self.rp = [0]*6
        #joint damping coefficents
        self.jd = [0.1]*21

        #Current pos, vel
        self.pos_act = []
        self.vel_act = []
        self.acc_joints = []

        # Dynamic and control
        self.Dynamics = False
        self.dt = None

        # Topics /right_joints /left_joints
        self.right_joints=[]
        self.left_joints=[]
        self.pub_right = False
        self.pub_left = False

        # Collision flag
        self.collision = False

        # Add autocollisions to the robot (DO NOT MODIFY)
        p.setCollisionFilterGroupMask(self.robot_id, -1, 0, 0)


    def step(self):
        '''
        Simulation step.
        Executes p.stepSimulation() in non real time simulations.
        Sleeps for adam.t using rospy if enabled.
        '''

        if not self.useRealTimeSimulation: p.stepSimulation()

        if self.use_ros: self.ros.sleep()
        else: time.sleep(self.t)


    def wait(self, secs):
        '''
        Wait for specified time in seconds
        '''

        iter = round(secs / self.t, 0)

        if self.use_ros:
            if not self.useRealTimeSimulation:
                for _ in range(iter):
                    p.stepSimulation()
                    self.ros.sleep()
            else:
                for _ in range(int(secs * 120)):
                    self.ros.sleep()
        else:
            for _ in range(int(iter)):
                if not self.useRealTimeSimulation: p.stepSimulation()
                time.sleep(self.t)

    #Collisions
    def detect_autocollisions(self):
        '''
        Detect self-collisions between the left and right arms of the robot.
        Returns:
            collision_left (bool): True if there is a collision in the left arm.
            collision_right (bool): True if there is a collision in the right arm.
        '''

        complete_left_arm_joints = self.ur3_left_arm_joints + self.hand_joint_indices['left']
        complete_right_arm_joints = self.ur3_right_arm_joints + self.hand_joint_indices['right']

        self.collision_left = False
        self.collision_right = False

        # Collisions between left arm and right arm
        for left_joint in complete_left_arm_joints:
            for right_joint in complete_right_arm_joints:
                contact_points = p.getClosestPoints(self.robot_id, self.robot_id, distance=0.0, linkIndexA=left_joint, linkIndexB=right_joint)
                if contact_points:
                    self.collision_left = True # Collision detected
                    self.collision_right = True # Collision detected

                    print('Collision in link:', left_joint)
                    print('Collision in link:', right_joint)

        # Collision with the body
        for left_joint in complete_left_arm_joints:
            for torso_index in self.torso_link_indices:

                contact_points = p.getContactPoints(self.robot_id, self.robot_id, linkIndexA=left_joint, linkIndexB=torso_index)
                if contact_points:
                    self.collision_left = True
                    print('Collision in link:', left_joint)

        # Collision with the body
        for right_joint in complete_right_arm_joints:
            for torso_index in self.torso_link_indices:
                contact_points = p.getClosestPoints(self.robot_id, self.robot_id, distance=0.0, linkIndexA=right_joint, linkIndexB=torso_index)
                if contact_points:
                    self.collision_right = True
                    print('Collision in link:', right_joint)


        return self.collision_left, self.collision_right
    
    def detect_collision_with_objects(self, object_id):
        '''
        Detect collisions between the robot's arms and a specified object.
        Returns:
            left_arm_collision (bool): True if there is a collision in the left arm.
            right_arm_collision (bool): True if there is a collision in the right arm.
            body_collision (bool): True if there is a collision with the body.'''
        
        left_arm_collision = False
        right_arm_collision = False
        body_collision = False

        # Check collisions of the left arm with the object
        for left_joint in self.ur3_left_arm_joints:
            contact_points = p.getClosestPoints(self.robot_id, object_id, distance=0, linkIndexA=left_joint)
            if len(contact_points) > 0:
                left_arm_collision = True
                self.collision = True

        # Check collisions of the right arm with the object
        for right_joint in self.ur3_right_arm_joints:
            contact_points = p.getClosestPoints(self.robot_id, object_id, distance=0, linkIndexA=right_joint)
            if len(contact_points) > 0:
                right_arm_collision = True
                self.collision = True


        # Check collisions of the body with the object
        for body_joint in self.body_joints:
            contact_points = p.getClosestPoints(self.robot_id, object_id, distance=0, linkIndexA=body_joint)
            if len(contact_points) > 0:
                body_collision = True
                self.collision = True

        #output: collision status for left arm, right arm, and body
        return left_arm_collision, right_arm_collision, body_collision


    def print_robot_info(self):
        '''
        Print the structure of the robot, including links and joints.
        '''

        num_joints = p.getNumJoints(self.robot_id)
        print(f"Robot ID: {self.robot_id}\n")
        
        print("=== ADAM LINKS ===")
        # Agregamos el link base manualmente (ID -1)
        print(f"Link ID: -1, Nombre: base_link (implícito)")
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode("utf-8")
            link_index = joint_info[0]  # También se puede usar i
            parent_index = joint_info[16]
            print(f"Link ID: {link_index}, Nombre: {link_name}, Parent Link ID: {parent_index}")
        
        print("\n=== ADAM JOINTS ===")
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_id = joint_info[0]
            joint_name = joint_info[1].decode("utf-8")
            joint_type = joint_info[2]
            
            joint_type_str = {
                p.JOINT_REVOLUTE: "Revolute",
                p.JOINT_PRISMATIC: "Prismatic",
                p.JOINT_SPHERICAL: "Spherical",
                p.JOINT_PLANAR: "Planar",
                p.JOINT_FIXED: "Fixed",
                p.JOINT_POINT2POINT: "Point2Point",
                p.JOINT_GEAR: "Gear"
            }.get(joint_type, "Unknown")
            
            print(f"ID: {joint_id}, Nombre: {joint_name}, Tipo: {joint_type_str}")

        print("\n=== ADAM REVOLUTE JOINTS ===")
        
        revolute_id = 0
        
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_id = joint_info[0]
            joint_name = joint_info[1].decode("utf-8")
            joint_type = joint_info[2]
            
            if joint_type == p.JOINT_REVOLUTE:
                print(f"Revolute ID: {revolute_id}, Nombre: {joint_name}")
                revolute_id += 1



