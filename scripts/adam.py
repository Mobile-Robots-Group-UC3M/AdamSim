import pybullet as p
import pybullet_data
import math
from arms_dynamics import ArmsDynamics
from sliders import Sliders
from arms_kinematics import ArmsKinematics
from hands_kinematics import HandsKinematics
from sensors import Sensors


# Class for ADAM robot
class ADAM:
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        
        # Load environment
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Load world plane
        self.plane_id = p.loadURDF("plane.urdf")

        # URDF path
        self.urdf_path = urdf_path

        # Spawn ADAM robot model
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT)

        # Change simulation mode
        self.useSimulation = useSimulation
        self.useRealTimeSimulation = useRealTimeSimulation
        self.t = 0.01
        
        base_position = [-0.10, 0.0, 0.54]
        base_orientation = p.getQuaternionFromEuler([0, 0, 0])
        if used_fixed_base: base_mass = 0
        else: base_mass = 80

        self.robot_shape = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                            fileName=robot_stl_path,
                                            meshScale=[1, 1, 1],
                                            flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        self.robot_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                                    fileName=robot_stl_path,
                                                    meshScale=[1, 1, 1])  # Ajusta el escalado 
        self.robot_stl_id = p.createMultiBody(baseMass = base_mass,              
                                            baseCollisionShapeIndex = self.robot_shape,
                                            baseVisualShapeIndex = self.robot_visual_shape,
                                            basePosition = base_position,
                                            baseOrientation = base_orientation)    # Cambia la posición 


        # Arm revolute joint indices
        self.ur3_right_arm_joints = list(range(20,26))  # Brazo derecho
        self.ur3_left_arm_joints = list(range(45,51)) # Brazo izquierdo

        self.ur3_right_arm_rev_joints = list(range(2,8))  # Brazo derecho
        self.ur3_left_arm_rev_joints = list(range(20,26)) # Brazo izquierdo

        # Hand revolute joint indices
        self.hand_joint_indices = {'right': list(range(30, 42)), 'left': list(range(55, 67))}

        # Other indices
        self.ee_index = {'right': 26, 'left': 51}
        self.hand_base_index = {'right': 28, 'left': 53}
        self.dummy_index = {'right': 29, 'left': 54}


        # ADAM MODULES
        self.arm_dynamics = ArmsDynamics(self)
        self.arm_kinematics = ArmsKinematics(self)
        self.hand_kinematics = HandsKinematics(self)
        self.sliders = Sliders(self)
        self.sensors = Sensors(self)      
        
        
        #Definir null space
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


        # Calculo de dinamica
        self.Dynamics = False
        self.dt = None

        # Topics /right_joints /left_joints
        self.right_joints=[]
        self.left_joints=[]
        self.pub_right = False
        self.pub_left = False

        # Señal de colision
        self.collision = False


    #Collisions
    def detect_autocollisions(self):
        # Colisiones entre los brazos
        for left_joint in self.ur3_left_arm_joints:
            for right_joint in self.ur3_right_arm_joints:
                contact_points = p.getClosestPoints(self.robot_id, self.robot_id, distance=0, linkIndexA=left_joint, linkIndexB=right_joint)
                if len(contact_points) > 0:
                    print("Colisión entre brazos detectada")
                    self.collision = True
                    return True

        # Colisiones cuerpo-brazo izquierdo
        for left_joint in self.ur3_left_arm_joints:
            contact_points = p.getClosestPoints(self.robot_id, self.robot_stl_id, distance=0.01, linkIndexA=left_joint)
            if len(contact_points) > 0:
                print("Colisión entre brazo izq-cuerpo")
                self.collision = True
                return True
        
        # Colisiones cuerpo-brazo derecho
        for right_joint in self.ur3_right_arm_joints:
            contact_points = p.getClosestPoints(self.robot_id, self.robot_stl_id, distance=0.01, linkIndexA=right_joint)
            if len(contact_points) > 0:
                print("Colisión entre brazo der-cuerpo")
                self.collision = True
                return True
            
        # Colisiones cuerpo-mano derecha
        for right_hand in self.right_hand_joints:
            contact_points = p.getClosestPoints(self.robot_id, self.robot_stl_id, distance=0.01, linkIndexA=right_hand)
            if len(contact_points) > 0:
                print("Colisión entre mano der-cuerpo")
                self.collision = True
                return True

        # Colisiones cuerpo-mano izquierda
        for left_hand in self.left_hand_joints:
            contact_points = p.getClosestPoints(self.robot_id, self.robot_stl_id, distance=0.01, linkIndexA=left_hand)
            if len(contact_points) > 0:
                print("Colisión entre mano izq-cuerpo")
                self.collision = True
                return True
            

        return False  # No hay colisiones
    
    def detect_collision_with_objects(self, object_id):
        #! TODO: Ver si se quiere dectectar la colision con el rest odel cuerpo
        # Detectar colisiones del brazo izquierdo o derecho con otros objetos en la escena
        left_arm_collision = False
        right_arm_collision = False
        body_collision = False

        # Comprobar colisiones del brazo izquierdo con el objeto
        for left_joint in self.ur3_left_arm_joints:
            contact_points = p.getClosestPoints(self.robot_id, object_id, distance=0, linkIndexA=left_joint)
            if len(contact_points) > 0:
                left_arm_collision = True
                self.collision = True

        # Comprobar colisiones del brazo derecho con el objeto
        for right_joint in self.ur3_right_arm_joints:
            contact_points = p.getClosestPoints(self.robot_id, object_id, distance=0, linkIndexA=right_joint)
            if len(contact_points) > 0:
                right_arm_collision = True
                self.collision = True


        # Comprobar objeto con cuerpo
        for body_joint in self.body_joints:
            contact_points = p.getClosestPoints(self.robot_id, object_id, distance=0, linkIndexA=body_joint)
            if len(contact_points) > 0:
                body_collision = True
                self.collision = True

        #Que nos devuelva puntos de contacto(articulaciones) y además un true o false
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



