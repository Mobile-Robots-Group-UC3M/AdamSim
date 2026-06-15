import os
import csv
from scripts.adam import ADAM
import pybullet as p
import math
import time

# ---------------------------------------------------------
# 1. LEER EL ARCHIVO CSV Y GUARDAR LOS ÁNGULOS
# ---------------------------------------------------------
csv_file_path = "/home/alumnos/tfg-carmen/TFGenvironmentsAdamSim/data/coger_botella.csv"  # Reemplaza con la ruta real de tu CSV
right_arm_trajectory = []

with open(csv_file_path, mode='r') as file:
    csv_reader = csv.reader(file)
    # next(csv_reader) # Descomenta esta línea si tu CSV tiene una fila de cabecera
    for row in csv_reader:
        angles = [float(val) for val in row]
        right_arm_trajectory.append(angles)

# La pose inicial será el primer elemento del CSV
right_arm_home = right_arm_trajectory[0]

# ---------------------------------------------------------
# 2. CONFIGURACIÓN DEL ENTORNO Y ROBOT
# ---------------------------------------------------------
# URDF robot path
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","models","robot", "rb1_base_description", "robots", "robotDummy.urdf")
print(robot_urdf_path)

# Create ADAM instance
adam = ADAM(robot_urdf_path, useRealTimeSimulation=False, used_fixed_base=False, use_ros=False)

# Generate home environment
adam.environments.generate_home(
    seed=2, 
    max_home_regens=300, 
    room_config={
        "kitchen"   :1,
        "bathrooms" :1,
        "bedroom1"  :1,
        "bedroom2"  :1,
        "dining"    :1
    },
    floor_color=None, 
    wall_color=None, 
    show_info=True
)

# Wait for the environment to load
adam.wait(5)

# Generate objects
adam.environments.create_movable_objects()
adam.wait(5)

# Get room navigation points
room_points = adam.environments.room_pointd()
print("Puntos de navegación por habitaciones:", room_points)

#Poner aquí la cesta sin convex para la prueba de agarre
box_pos = room_points["Bedroom1_bed"]
box_pos = (box_pos[0] + 0.9, box_pos[1] + 0.7, 0.15)

box_size = [0.45, 0.35, 0.30]
box_half_extents = [box_size[0]/2, box_size[1]/2, box_size[2]/2]
box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_half_extents)
box_visual = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=box_half_extents, rgbaColor=[0.7, 0.0, 0.0, 1])
box_id = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=box_collision, baseVisualShapeIndex=box_visual, basePosition=box_pos, baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))


# Start LiDAR and configure visualizer
adam.sensors.start_lidar()
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# Move arms to initial positions (Brazo derecho usa el primer elemento del CSV)
adam.arm_kinematics.move_arm_joints_to_angles("right", right_arm_home)
adam.arm_kinematics.move_arm_joints_to_angles("left", [-3.175, -0.794, 0.00, -1.389, 3.307, -4.431]) 
adam.hand_kinematics.move_hand_to_dofs('right', [1000, 1000, 1000, 1000, 1000, 0])

# Wait for arms to move
adam.wait(2)



def add_orientation(point, theta = 0.0):
    return (point[0], point[1], theta)

goals = [
    add_orientation(room_points["Bathroom_door"], 0.0),
    add_orientation(room_points["Bathroom_center"], 0.0),
    add_orientation(room_points["Bathroom_lavabo"], 0.0),
    #add_orientation(room_points["Bathroom_center"], 0.0),
    add_orientation(room_points["Bathroom_door"], 0.0),
    #add_orientation(room_points["Home_center"], 0.0),
    add_orientation(room_points["Bedroom1_door"], 0.0),
    add_orientation(room_points["Bedroom1_center"], 0.0),
    add_orientation(room_points["Bedroom1_bed"], 0.0),
    add_orientation(room_points["Bedroom1_center"], 0.0),
    add_orientation(room_points["Bedroom1_door"], 0.0),
    add_orientation(room_points["Home_center"], 0.0)
]

lavabo_orientation_goal = room_points["Bathroom_lavabo_pose"]

goal_name =[
     "Puerta del baño",
     "Centro del baño",
     "Lavabo del baño",
     "Centro del baño",
     "Lavabo del baño",
     "Centro de la casa",
     "Puerta del dormitorio",
     "Centro del dormitorio",
     "Cama del dormitorio",
     "Centro de la casa"
]

estado = "INICIO" 
goal_index = 0
current_goal_ids = []  

#manipulation_done = False
#manipulation_point = 2 

grasp_point = 2
drop_point = 6

grasp_done = False
drop_done = False

step = True

backing = False
back_start_pos = None
returning_arm = False

flag = False

print("GOALS;")
for g in goals:
        print(g, "longitud =", len(g))

times = {}

t_total_start = None
t_nav_start = None
t_orientation_start = None
t_grasp_start = None
t_back_start = None
t_return_arm_start = None
t_drop_nav_start = None
t_drop_start = None

# ---------------------------------------------------------
# 3. BUCLE PRINCIPAL
# ---------------------------------------------------------
while True:
    
    if estado == "INICIO":
        print ("Estado:INICIO")
    
        goal_index = 0
        current_goal_ids = []
        grasp_done = False
        drop_done = False
        backing = False
        back_start_pos = None
        returning_arm = False

        print("Voy al primer punto:", goal_name[goal_index], "en coordenadas:", goals[goal_index])

        t_total_start = time.time()
        t_nav_start = time.time()

        estado = "NAVEGACION"

    elif estado == "NAVEGACION":
        print ("Estado:NAVEGACION")

        if backing:
            pos, _ = p.getBasePositionAndOrientation(adam.robot_id)
            
            if back_start_pos is None:
                back_start_pos = pos
                print ("Iniciando marcha atrás desde posición:", back_start_pos)
            dist_back = math.hypot(pos[0] - back_start_pos[0], pos[1] - back_start_pos[1])
           
            print("Distancia recorrida hacia atrás:", dist_back)
            if dist_back < 0.2:
                adam.navigation.send_velocity(-0.15, 0)  # Marcha atrás a velocidad constante
            else:
                adam.navigation.send_velocity(0, 0)  # Detener el robot después de retroceder
                print("Marcha atrás completada")

                times["Marcha atrás"] = time.time() - t_back_start

                backing = False
                back_start_pos = None
                returning_arm = True  # Activar el retorno del brazo después de la marcha atrás
                t_return_arm_start = time.time()
            adam.step()
            continue
        
        if returning_arm:
            print("Recogiendo el brazo con trayectoria inversa...")

            for joint_angles in reversed(right_arm_trajectory):
                adam.arm_kinematics.move_arm_joints_to_angles('right', joint_angles)
                adam.step()
                adam.wait(0.02)
            
            print("Brazo recogido. Continuando con la navegación.")

            times["Retorno del brazo"] = time.time() - t_return_arm_start
            
            returning_arm = False

            t_drop_nav_start = time.time()

            adam.step()
            continue


        if not current_goal_ids and goal_index < len(goals):
            g = goals[goal_index]
            current_goal_ids = adam.utils.draw_frame([[g[0], g[1], 0.1], p.getQuaternionFromEuler([0, 0, g[2]])])

        obstacles = adam.sensors.get_lidar_points_world() 
        goal = goals[goal_index]

        v, w = adam.planner.dwa_step(goal, obstacles)
        adam.navigation.send_velocity(v, w)

        pos, _ = p.getBasePositionAndOrientation(adam.robot_id)
        dist = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
        print("Posición actual:", pos, ". Posición del objetivo:", goal, ". Distancia al objetivo:", dist)
        print("Velocidad lineal:", v, "Velocidad angular:", w)

        if dist < 0.25:
            print("Goal reached.", goal)
            adam.navigation.send_velocity(0, 0)
            adam.utils.clear_frame(current_goal_ids)
            current_goal_ids = []

            if goal_index == grasp_point and not grasp_done:
                times["Navegación hasta el agarre"] = time.time() - t_nav_start
                print("Cambiando a estado de manipulación para agarrar")
                t_orientation_start = time.time()
                estado = "ORIENTACION"
            
            elif goal_index == drop_point and not drop_done:

                times["Navegación hasta el soltar"] = time.time() - t_drop_nav_start
                
                print("Cambiando a estado de manipulación para soltar")
                
                t_drop_start = time.time()
                
                estado = "DEJAR OBJETO"
                
            else:  
                goal_index += 1
                if goal_index >= len(goals):
                    print("RUTA COMPLETADA CORRECTAMENTE")
                    estado = "FIN"
                else:
                    print("Siguiente objetivo:", goal_name[goal_index], "en coordenadas:", goals[goal_index])

    elif estado == "ORIENTACION":
        print("Estado: ORIENTACION")
        
        if not flag:
            pos, orn = p.getBasePositionAndOrientation(adam.robot_id)
            x, y = pos[0], pos[1]
            _, _, theta = p.getEulerFromQuaternion(orn)
            theta = math.pi/2

            flag = True

        point = adam.navigation.move_base_continuous([x, y+0.1, theta])
        
        if point == False:
            adam.navigation.send_velocity(0, 0)
            times["Orientación"] = time.time() - t_orientation_start
            t_grasp_start = time.time()
            estado = "MANIPULACION"

    elif estado == "MANIPULACION":
        print ("Estado:MANIPULACION")
        print("End effector pose: ", adam.arm_kinematics.get_arm_link_pose('right', 'hand'))
        adam.navigation.send_velocity(0, 0)

        # 1. Abrir la mano
        adam.hand_kinematics.move_hand_to_dofs('right', [1000, 1000, 1000, 1000, 1000, 0])
        adam.wait(1)

        # 2. Ejecutar la trayectoria del CSV para pre-grasp y grasp (hasta el elemento 440)
        print("Ejecutando trayectoria de manipulación desde el CSV...")
        # Iteramos solo hasta el índice 440 (excluyéndolo, de 0 a 439)
        for joint_angles in right_arm_trajectory[:440]:
            adam.arm_kinematics.move_arm_joints_to_angles('right', joint_angles)
            adam.step()
            # Pausa breve para suavizar el movimiento en la simulación
            adam.wait(0.02) 

        # 3. Cerrar la mano para agarrar
        agarrado = adam.hand_kinematics.hand_close('right', close_speed=5)
        print("He agarrado el objeto?: ", agarrado)
        adam.wait(1)

        # 4. Levantar el objeto (Mantenemos la pose que tenías hardcodeada para levantar)
        for joint_angles in right_arm_trajectory[440:]:  # Desde el índice 440 hasta el final
            adam.arm_kinematics.move_arm_joints_to_angles('right', joint_angles)
            adam.step()
            adam.wait(0.02)
        adam.wait(2)

        print ("Manipulación completada")
        times["Manipulación"] = time.time() - t_grasp_start
        grasp_done = True
        goal_index += 1 

        #Activar marcha atrás
        backing = True
        back_start_pos = None
        t_back_start = time.time()

        if goal_index >= len(goals):
            estado = "FIN"
        else:
            estado = "NAVEGACION"
    
    elif estado == "DEJAR OBJETO":
        print("Estado: DEJAR OBJETO")
        adam.navigation.send_velocity(0, 0)

        # 1. Bajar el objeto 
        for joint_angles in (right_arm_trajectory[:440]):  # Desde el final hasta el índice 440
            adam.arm_kinematics.move_arm_joints_to_angles('right', joint_angles)
            adam.step()
            adam.wait(0.02)
        
        print ("Abriendo la mano para soltar el objeto...")
        
         # 2. Abrir la mano para soltar
        adam.hand_kinematics.move_hand_to_dofs('right', [1000, 1000, 1000, 1000, 1000, 0])
        adam.wait(3)

        times["Dejar objeto"] = time.time() - t_drop_start
        drop_done = True
        estado = "FIN"

    elif estado == "FIN":
        adam.navigation.send_velocity(0, 0)

        times["Tiempo total"] = time.time() - t_total_start

        if current_goal_ids:
            adam.utils.clear_frame(current_goal_ids)
            current_goal_ids = []
        print("Estado: FIN")
        print("Tiempos registrados:")
        print ("fase; tiempo (s)")
        for fase, tiempo in times.items():
            print(f"{fase}; {tiempo:.2f}")
        estado = "continuar"
        #break

    adam.step()