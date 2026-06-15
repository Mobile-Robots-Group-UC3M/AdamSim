import pybullet as p
import pybullet_data
from scripts.arms_dynamics import ArmsDynamics
from scripts.teleoperation import Teleop
from scripts.arms_kinematics import ArmsKinematics
from scripts.hands_kinematics import HandsKinematics
from scripts.sensors import Sensors
from scripts.navigation import Navigation
from scripts.planner import Planner
from scripts.utils import Utils

import time
import json
import os
# Class for ADAM robot
class ADAM:
    def __init__(self, urdf_path=None, info_json_path=None, semantic_json_path=None, hand_json_path=None, useRealTimeSimulation=True, used_fixed_base=True, use_ros=True):
        
        #Load JSON files
        
        base_path = os.path.dirname(__file__)
        self.robot_pkg_path = os.path.join(base_path, "..", "models", "robot", "rb1_base_description")
        self.config_dir = os.path.join(self.robot_pkg_path, "config")

        # URDF Path
        if urdf_path is None: self.urdf_path = os.path.join(self.robot_pkg_path, "robots", "robotDummy.urdf")
        else: self.urdf_path = urdf_path

        # JSON Config Paths
        if info_json_path is None: self.info_json_path = os.path.join(self.config_dir, "robot_info.json")
        else: self.info_json_path = info_json_path

        if semantic_json_path is None: self.semantic_json_path = os.path.join(self.config_dir, "semantic_groups.json")
        else: self.semantic_json_path = semantic_json_path

        if hand_json_path is None: self.hand_json_path = os.path.join(self.config_dir, "hand_kinematics.json")
        else: self.hand_json_path = hand_json_path
        
        # Load environment
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        if useRealTimeSimulation: p.setRealTimeSimulation(1)
        else: p.setRealTimeSimulation(0)

        # Load world plane
        self.plane_id = p.loadURDF("plane.urdf")
        p.setPhysicsEngineParameter(enableConeFriction=1)
        

        # Spawn ADAM robot model
        self.robot_id = p.loadURDF(self.urdf_path, useFixedBase=used_fixed_base, flags=p.URDF_USE_SELF_COLLISION)
        
        

        # Change simulation mode
        self.useRealTimeSimulation = useRealTimeSimulation
        self.use_ros = use_ros
        self.t = 0.01

        # Load indices from JSON
        self._load_dynamic_indices()
        
        
        
        
        # ADAM MODULES
        self.arm_dynamics = ArmsDynamics(self)
        self.arm_kinematics = ArmsKinematics(self)
        self.hand_kinematics = HandsKinematics(self)
        if not used_fixed_base: self.navigation = Navigation(self)
        self.teleop = Teleop(self)
        self.sensors = Sensors(self)
        self.utils = Utils(self)
        if use_ros: 
            from scripts.ros_connection import ROSConnection
            self.ros = ROSConnection(self)     
        self.planner =  Planner(self)
        
        
        # Null space definition
        # Lower limits for null space
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


    """ def wait(self, secs):
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
                time.sleep(self.t) """
    def wait(self, secs, callback_func=None):
        '''
        Wait for specified time in seconds. 
        If callback_func is provided, it runs in every step of the wait loop.
        '''
        iter = round(secs / self.t, 0)

        if self.use_ros:
            if not self.useRealTimeSimulation:
                for _ in range(int(iter)):
                    p.stepSimulation()
                    if callback_func: callback_func() 
                    self.ros.sleep()
            else:
                for _ in range(int(secs * 120)):
                    if callback_func: callback_func()
                    self.ros.sleep()
        else:
            for _ in range(int(iter)):
                if not self.useRealTimeSimulation: 
                    p.stepSimulation()
                
                if callback_func: 
                    callback_func() 
                
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


    def print_robot_info(self, save=False, filename="robot_info.json"):
        '''
        Print the structure of the robot, including links and joints.
        If save=True, exports this information to a JSON file for dynamic ID lookup.
        '''

        num_joints = p.getNumJoints(self.robot_id)
        
        # Dictionary to store all the data for the JSON export
        robot_data = {
            "links": {},
            "joints": {},
            "movable_joints": {}
        }

        print(f"Robot ID: {self.robot_id}\n")
        
        # --- LINKS ---
        print("=== ADAM LINKS ===")
        print(f"Link ID: -1, Nombre: base_link (implicit)")
        robot_data["links"]["base_link"] = {"id": -1, "parent_id": None}
        
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_index = joint_info[0]
            link_name = joint_info[12].decode("utf-8")
            parent_index = joint_info[16]
            
            print(f"Link ID: {link_index}, Nombre: {link_name}, Parent Link ID: {parent_index}")
            robot_data["links"][link_name] = {
                "id": link_index,
                "parent_id": parent_index
            }
        
        # --- JOINTS ---
        print("\n=== ADAM JOINTS ===")
        dof_index = 0 # Degree of Freedom index for IK matching
        
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
            robot_data["joints"][joint_name] = {
                "id": joint_id,
                "type": joint_type_str
            }
            
            # If it's a movable joint (Revolute or Prismatic), add it to the movable specific dictionary
            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                robot_data["movable_joints"][joint_name] = {
                    "dof_index": dof_index,
                    "joint_id": joint_id,
                    "type": joint_type_str
                }
                dof_index += 1

        # --- MOVABLE JOINTS ---
        print("\n=== ADAM MOVABLE JOINTS ===")
        for name, data in robot_data["movable_joints"].items():
            print(f"DoF Index: {data['dof_index']}, Nombre: {name}, Joint ID: {data['joint_id']}, Tipo: {data['type']}")

        # --- SAVE TO JSON ---
        if save:
            
            save_path = os.path.join(self.config_dir, filename)

            with open(save_path, "w", encoding="utf-8") as f:
                json.dump(robot_data, f, indent=4)
            print(f"\n[INFO] Robot structure successfully saved to '{filename}'.")

    def _load_dynamic_indices(self):
        '''
        Reads the auto-generated robot_info.json and the manual semantic_groups.json
        to populate all necessary joint and link indices dynamically.
        '''
        
        if not os.path.exists(self.info_json_path) or not os.path.exists(self.semantic_json_path):
            raise FileNotFoundError("[ERROR] Configuration JSONs not found. Please generate robot_info.json first.")

        with open(self.info_json_path, 'r', encoding='utf-8') as f:
            robot_info = json.load(f)
            self.robot_info = robot_info
            
        with open(self.semantic_json_path, 'r', encoding='utf-8') as f:
            semantics = json.load(f)
            self.semantics = semantics

        # 1. Arm global joint IDs
        self.ur3_right_arm_joints = [robot_info["joints"][name]["id"] for name in semantics["arms"]["right"]]
        self.ur3_left_arm_joints = [robot_info["joints"][name]["id"] for name in semantics["arms"]["left"]]

        # print("ur3_right_arm_joints", self.ur3_right_arm_joints)
        # print("ur3_left_arm_joints", self.ur3_left_arm_joints)


        # 2. Arm movable IDs (DoF Index for PyBullet IK arrays)
        self.ur3_right_arm_rev_joints = [robot_info["movable_joints"][name]["dof_index"] for name in semantics["arms"]["right"]]
        self.ur3_left_arm_rev_joints = [robot_info["movable_joints"][name]["dof_index"] for name in semantics["arms"]["left"]]

        # print("ur3_right_arm_rev_joints", self.ur3_right_arm_rev_joints)
        # print("ur3_left_arm_rev_joints", self.ur3_left_arm_rev_joints)

        # 3. Hand joints
        self.hand_joint_indices = {
            'right': [robot_info["joints"][name]["id"] for name in semantics["hands"]["right"]],
            'left': [robot_info["joints"][name]["id"] for name in semantics["hands"]["left"]]
        }

        # print("hand_joint_indices right", self.hand_joint_indices['right'])
        # print("hand_joint_indices left", self.hand_joint_indices['left'])

        # 4. Other indices
        self.left_wheel_joint = robot_info["joints"][semantics["wheels"]["left"]]["id"]
        self.right_wheel_joint = robot_info["joints"][semantics["wheels"]["right"]]["id"]
        
        # print("left_wheel_joint", self.left_wheel_joint)
        # print("right_wheel_joint", self.right_wheel_joint)

        self.torso_link_indices = [robot_info["links"][name]["id"] for name in semantics["torso"]]
        # print("torso_link_indices", self.torso_link_indices)

        self.ee_index = {
            'right': robot_info["links"][semantics["special_links"]["right_ee"]]["id"],
            'left': robot_info["links"][semantics["special_links"]["left_ee"]]["id"]
        }
        # print("ee_index right", self.ee_index['right'])
        # print("ee_index left", self.ee_index['left'])

        self.hand_base_index = {
            'right': robot_info["links"][semantics["special_links"]["right_hand_base"]]["id"],
            'left': robot_info["links"][semantics["special_links"]["left_hand_base"]]["id"]
        }
        # print("hand_base_index right", self.hand_base_index['right'])
        # print("hand_base_index left", self.hand_base_index['left'])
        
        self.dummy_index = {
            'right': robot_info["links"][semantics["special_links"]["right_dummy"]]["id"],
            'left': robot_info["links"][semantics["special_links"]["left_dummy"]]["id"]
        }
        # print("dummy_index right", self.dummy_index['right'])
        # print("dummy_index left", self.dummy_index['left'])

        self.camera_joint_index = robot_info["joints"][semantics["special_links"]["camera_joint"]]["id"]
        self.camera_link_index = robot_info["links"][semantics["special_links"]["camera_link"]]["id"]
        self.laser_link_index = robot_info["links"][semantics["special_links"]["laser_link"]]["id"]
        print("camera_joint_index", self.camera_joint_index)
        print("camera_link_index", self.camera_link_index)
        print("laser_link_index", self.laser_link_index)

