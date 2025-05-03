import pybullet as p
import pybullet_data
import math
from Adam_sim.scripts.arms_dynamics import ArmsDynamics
from Adam_sim.scripts.sliders import Sliders
from Adam_sim.scripts.arms_kinematics import ArmsKinematics
from Adam_sim.scripts.hands_kinematics import HandsKinematics


#Class for ADAM info
class ADAM:
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        # Cargar las caractersitcias de ADAM
        self.dynamics = ArmsDynamics(self)
        self.kinematics = ArmsKinematics(self)
        self.handkinematics = HandsKinematics(self)
        self.sliders = Sliders(self)
        
        # Cargar el robot
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        #Cargar un plano para referencia
        self.planeId = p.loadURDF("plane.urdf")

        #Change simulation mode
        self.useSimulation = useSimulation
        self.useRealTimeSimulation = useRealTimeSimulation
        self.t = 0.1

        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT) #flags=p.URDF_USE_SELF_COLLISION,# flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT) # Cambiar la posición si es necesario
        
        #Creamos el objeto sin hombros
        #Orientacion del stl
        rotation_quaternion = p.getQuaternionFromEuler([0, 0, 0])

        self.robot_shape = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                            fileName=robot_stl_path,
                                            meshScale=[1, 1, 1],
                                            flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        self.robot_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                                    fileName=robot_stl_path,
                                                    meshScale=[1, 1, 1])  # Ajusta el escalado 
        self.robot_stl_id = p.createMultiBody(baseMass=0,              
                                            baseCollisionShapeIndex=self.robot_shape,
                                            baseVisualShapeIndex=self.robot_visual_shape,
                                            basePosition=[-0.10, 0.0, 0.54],
                                            baseOrientation = rotation_quaternion)    # Cambia la posición 

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

        # Definir los índices de los brazos (esto depende de tu URDF)
        self.ur3_right_arm_joints = list(range(20,26))  # Brazo derecho #! +1 ADRI
        self.ur3_left_arm_joints = list(range(45,51)) # Brazo izquierdo

        # Definir los indices de las manos
        self.right_hand_joints = list(range(30, 42)) #! +1 ADRI
        self.left_hand_joints = list(range(55,67)) #! +1 ADRI

        # Definir joints del cuerpo
        self.body_joints = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,26,27,28,41,50,51] #Cuerpo 
        self.joints=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51]

        self.arm_joints = 6


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

        self.joint_angle = 0.22


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
        num_joints = p.getNumJoints(self.robot_id)
        print(f"Robot ID: {self.robot_id}")
        print("Elementos del robot:")
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_id = joint_info[0]
            joint_name = joint_info[1].decode("utf-8")
            print(f"ID: {joint_id}, Nombre: {joint_name}")


