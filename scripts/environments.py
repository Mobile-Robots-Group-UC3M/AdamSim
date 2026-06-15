import pybullet as p
import time
import pybullet_data
import os
import random
import math 

# Configuraciones pybullet
#physicsClient = p.connect(p.GUI)
#p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
#p.setGravity(0,0,-10)
#planeId = p.loadURDF("plane.urdf")

class Environment:
    def __init__(self, adam, grid_n=3, step=3.0, z0=0.02, origin_x = 1.5, origin_y = 1.5, obj_dir =None, scale_file =None):
        self.adam = adam 
        self.grid_n = grid_n
        self.step = step
        self.z0 = z0
        self.origin_x = origin_x
        self.origin_y = origin_y

        #STL PATH
        if obj_dir is None:
            obj_dir = os.path.join(os.path.expanduser("~"), "tfg-carmen/TFGenvironmentsAdamSim", "models", "objects")
            #obj_dir = os.path.join(os.path.expanduser("~"), "AdamSim", "models", "objects")
            print(obj_dir)
        self.OBJ_DIR = obj_dir

        #self.WALL_STL = os.path.join(self.OBJ_DIR, "wall.2.stl")
        #self.WALL_DOOR_STL = os.path.join(self.OBJ_DIR, "wall_door2.stl")

        #Escalas
        self.Scale_default = 0.05
        self.Scale_file = scale_file or {
            #    "wall.2.stl"           : 0.026,
            #   "wall_door2.stl"       : 0.026,
        }

        self.Scale_file.update({
            "Washing_machine.stl"  : 0.03,
            "Wardrobe.stl"         : 0.0037,
            "Bed.stl"              : 0.02,
            "Table1.stl"           : 0.0075,
            "Table2.stl"           : 1.0,
            "Monitor2.stl"         : 0.7,
            "Chair.stl"            : 0.009,
            "Armchair.stl"         : 0.025,
            "wall.2.stl"           : 0.026,
            "wall_door2.stl"       : 0.026,
            "Ropero.stl"           : 0.0055,
            "MuebleC.stl"          : 0.03,
            "mesa_con_sillas1.stl" : 0.013,
            "MESA_centro.stl"      : 0.0055,
            "Cocina_leña.stl"      : 0.0055, #008
            "Escorial_Cocina.stl"  : 1.1,
            "Florero_v1.stl"       : 0.003,
            "camita.stl"           : 0.18,
            "lavavo.stl"           : 0.27,
            "comoda.stl"           : 0.16, 
            "mesa_escritorio.stl"  : 0.030,
            "Tocador.stl"          : 0.007,
            "silla_escritorio.stl" : 0.027,
            "cocina.stl"           : 8,
            "escritorio.stl"       : 0.3,
            "espejo.stl"           : 0.35,
            "Bowl.stl"             : 0.007,
            "cesta_baño_fijo.stl"  : 60,
            "lampara.stl"          : 160,
            "sofa.stl"             : 145,
        })

        #Configuración visual
        self.default_floor_color = [0.64, 0.39, 0.14, 1]
        self.default_wall_color = [0.93, 0.88, 0.82, 1]
        self.current_floor_color = list(self.default_floor_color)
        self.current_wall_color = list(self.default_wall_color)

        #Paredes
        self.wall_height = 2.4
        self.wall_thickness = 0.08

        #Puertas
        self.door_width = 1.0
        self.door_height = 2.00
        self.door_offset = -0.6

        #Ventana
        self.window_width = 1.00
        self.window_height = 0.8
        self.window_bottom = 1.00 #Distancia desde el suelo
        self.window_offset = 0.00

        #Cristal
        self.window_glass = True 
        self.window_color = [0.55, 0.8, 1.00, 0.3]

        
        #DATOS 
        self.data = None

        #Para las paredes
        self.cells = {} #diccionario con la posición (i, j), la rotación, size_xy y objetos 

        #Coordenadas posibles para cada tipo de habitación
        self.corner_coords           = [(0,0),(0,2),(2,0),(2,2)]
        self.edge_coords             = [(0,1),(1,0),(1,2),(2,1)]
        self.center_coords           = [(1,1)]
        self.non_center_coords       = set (self.corner_coords + self.edge_coords)

        self.sides = ["N", "E", "S", "W"]

        self.room = {}

        #Puntos para la navegación por habitaciones 
        #self.room_points = {} 
        self.start_point = None


        #Nombre de las habitaciones
        self.room_name= {
            0: "Kitchen",
            1: "Bathroom",
            2: "Bedroom1",
            3: "LivingLarge", #Habitación doble 
            4: "Office",
            5: "Entrance",
            6: "Bedroom2",
            7: "Dining",
        }
        self.default_room_config = {
            "kitchen"  :1,
            "bathrooms":1,
            "bedroom1" :1,
            "bedroom2" :1,
            "dining"   :1
        }
        self.allowed_door_sides_by_room_type = {
            0: {"N", "W"},           #Kitchen
            1: {"N", "E"},           #Bathroom
            2: {"E"},                #Bedroom1
            3: {"N", "E", "W"},      #Livinglarge
            4: {"N", "E", "W"},      #Office
            5: {"N", "E", "S", "W"}, #Entrance
            6: {"N", "W"},           #Bedroom2
            7: {"E"}                 #Dining 
        }
        #Muebles fijos
        self.fixed_objects_by_room_name = {
            0: [  # Kitchen LISTA
                #{"name":"escorial", "stl": "kitchen/Escorial_Cocina.stl", "pos": [ 0.3,  1.15, 0.0], "orn": self.qdeg(0,0,0),  "color":[0.66, 0.67, 0.68, 1]},
                {"name":"cocina",   "stl": "kitchen/cocina.stl",          "pos": [ 0.0, -0.55, 0.0],   "orn": self.qdeg(90,0,0), "color":[0.55, 0.62, 0.55, 1]},
                {"name":"mesa",     "stl": "kitchen/MESA_centro.stl",     "pos": [ 0.4,  0.0, 0.0],   "orn": self.qdeg(0,0,0),  "color":[0.74, 0.58, 0.40, 1]},         
            ],
            1: [  # Bathroom LISTA
                {"name":"lavabo",     "stl": "bedroom1/Table1.stl",         "pos": [ 1.05,  -0.7, 0.0],   "orn": self.qdeg(90,0,90),"color":[0.55, 0.36, 0.22, 1]}, # GONZALO MODIFICACION
                {"name":"bowl",       "stl": "bathroom/Bowl.stl",           "pos": [ 1.1,  0.5, 0.0],  "orn": self.qdeg(0,0,270), "color":[0.89, 0.85, 0.78, 1]},
                {"name":"cesta",      "stl": "bathroom/cesta_baño_fijo.stl","pos": [-1.0,  -1.0, 0.0],   "orn": self.qdeg(0,0,0), "color":[0.66, 0.50, 0.36, 1]},
                
            ],
            2: [  # Bedroom1   LISTA
                {"name":"cama",       "stl": "bedroom1/Bed.stl",             "pos": [ -0.7,  -1.0, 0.0],  "orn": self.qdeg(0,0,0),  "color":[0.58, 0.67, 0.58, 1]},
                #Cambiar la mesa para la prueba de agarre {"name":"Table1",     "stl": "bedroom1/Table1.stl",          "pos": [ -0.4,  1.1, 0.0],   "orn": self.qdeg(90,0,0), "color":[0.76, 0.62, 0.44, 1]}, 
                {"name":"Table1",     "stl": "bedroom1/Table1.stl",          "pos": [ -0.7,  1.0, 0.0],   "orn": self.qdeg(90,0,0), "color":[0.76, 0.62, 0.44, 1]},
                {"name":"armario",    "stl": "bedroom1/Wardrobe.stl",        "pos": [1.1, 1.2, 0.0],    "orn": self.qdeg(0,0,0),"color":[0.56, 0.50, 0.45, 1]},
            ],
            3: [  # LivingLarge HABITACIÓN GRANDE
                {"name":"butaca",     "stl": "livinglarge/Armchair.stl",          "pos": [ 1.3,  -0.85, 0.0],  "orn": self.qdeg(0,0,135),  "color":[0.55, 0.56, 0.57, 1]},
                {"name":"leña",       "stl": "livinglarge/Cocina_leña.stl",       "pos": [ 2.77,  1.4, 0.0],  "orn": self.qdeg(0,0,270), "color":[0.55, 0.18, 0.14, 1]},
                {"name":"sofa",       "stl": "livinglarge/sofa.stl",              "pos": [ 1.15, 0.4, 0.0],   "orn": self.qdeg(0,0,90),    "color":[0.73, 0.70, 0.66, 1]},
                {"name":"mesa2",       "stl": "livinglarge/mesa_con_sillas1.stl", "pos": [ -1.55, 0.0, 0.0],  "orn": self.qdeg(0,0,0),    "color":[0.82, 0.76, 0.68, 1]},
                {"name":"lampara",    "stl": "livinglarge/lampara.stl",           "pos": [ -2.65, -0.8, 0.0],   "orn": self.qdeg(0,0,0),    "color":[0.36, 0.52, 0.72, 1]},
            ],
            4: [  # Office HABITACIÓN DE MEDIO LISTA 
 
            ],
            5: [  # Entrance   

            ],
            6: [  # Bedroom2 LISTA
                {"name":"camita",       "stl": "bedroom2/camita.stl",         "pos": [ 0.6,  -0.7, 0.0],  "orn": self.qdeg(0,0,270), "color":[0.50, 0.61, 0.72, 1]},
                {"name":"ropero",       "stl": "bedroom2/Ropero.stl",         "pos": [ 1.3, 0.8, 0.0],   "orn": self.qdeg(0,0,270), "color":[0.43, 0.50, 0.45, 1]},
                {"name":"tocador",      "stl": "bedroom2/Tocador.stl",        "pos":[ -1.15, 0.5, 0.0],   "orn": self.qdeg(0,0,90), "color":[0.86, 0.82, 0.74, 1]},
            ],
            7: [  # Dining LISTA
                {"name":"silla",      "stl": "dining/silla_escritorio.stl", "pos": [ -0.42, 0.45, 0.0], "orn": self.qdeg(0,0,0), "color":[0.65, 0.20, 0.20, 1]},
                {"name":"mesa",       "stl": "dining/mesa_escritorio.stl",  "pos": [-0.5, 1.15, 0.0],  "orn": self.qdeg(0,0,0), "color":[0.64, 0.66, 0.69, 1]},
                {"name":"comoda",     "stl": "dining/comoda.stl",           "pos": [ 0.0, -1.1, 0.0],"orn": self.qdeg(90,0,0), "color":[0.68, 0.52, 0.56, 1]},
            ], 
        }

        self.movable_objects = [
            #Kitchen
            {"room_type_id":0, "name":"desayuno", "urdf": "kitchen/desayuno/desayuno.urdf",
            "pos":[0.4, 0.00, 0.0], "orn": self.qdeg(0, 0, 0),"support":"mesa", "clearance" : -0.02},

            {"room_type_id":0, "name":"tabla", "urdf": "kitchen/tabla_cocina/tabla_cocina.urdf",
            "pos":[-0.5, -1.01, 0.0], "orn": self.qdeg(0, 0, 180), "support":"cocina", "clearance" : -0.07},

            #{"room_type_id":0, "name":"tostadora", "urdf": "kitchen/tostadora/tostadora.urdf",
            #"pos":[0.4, 0.00, 0.0], "orn": self.qdeg(0, 0, 180), "support":"mesa", "clearance" : -0.07},

            #Bathdroom
            #{"room_type_id":1, "name":"secador", "urdf": "bathroom/secador/secador.urdf",
            #"pos":[1.05, -0.65, 0.0], "orn": self.qdeg(0, 0, 0), "support":"lavabo", "clearance" : -0.10},
            
            #{"room_type_id":1, "name":"taza", "urdf": "bathroom/taza/taza.urdf",
            #"pos":[0.9, -0.54, 0.0], "orn": self.qdeg(0, 0, 0), "support":"lavabo", "clearance" : -0.10},

            {"room_type_id":1, "name":"cola", "urdf": "bathroom/cola/cola.urdf",
            "pos":[0.82, -0.95, 0], "orn": self.qdeg(0, 0, 0), "support":"lavabo", "clearance" : 0.2},

            #{"room_type_id":1, "name":"altavoz", "urdf": "bathroom/altavoz/altavoz.urdf",
            #"pos":[1.05, -0.5, 0.0], "orn": self.qdeg(0, 0, 0), "support":"lavabo", "clearance" : -0.10},

            #Bedroom1 
            #{"room_type_id":2, "name":"manta", "urdf": "bedroom1/manta/manta.urdf",
            #"pos":[-0.7, -0.2, 0.0], "orn": self.qdeg(0,
            #  0, 90), "support":"cama", "clearance" : -0.10},

            #{"room_type_id":2, "name":"vaso", "urdf": "bedroom1/vaso/vaso.urdf",
            #"pos":[-0.22, 1.18, 0.6], "orn": self.qdeg(0, 0, 0), "support":"Table1", "clearance" : -0.005},
            
            #Igual cambiar el la posición del ordenador para la prueba de agarre "pos":[-0.22, 1.08, 0.0]
            {"room_type_id":2, "name":"ordenador", "urdf": "bedroom1/ordenador/ordenador.urdf",
            "pos":[-0.9, 1.08, 0.0], "orn": self.qdeg(0, 0, 0), "support":"Table1", "clearance" : -0.005},
            

            #{"room_type_id":2, "name":"reloj", "urdf": "bedroom1/reloj/reloj.urdf",
            #"pos":[-0.22, 1.08, 0.0], "orn": self.qdeg(0, 0, 0), "support":"Table1", "clearance" : -0.005},


            #{"room_type_id":2, "name":"reloj", "urdf": "bedroom1/reloj/reloj.urdf",
            #"pos":[-0.7, -0.2, 0.7], "orn": self.qdeg(0, 0, 0), "support":"Table1", "clearance" : -0.10},

            #{"room_type_id":2, "name":"tijeras", "urdf": "bedroom1/tijeras/tijeras.urdf",
            #"pos":[-0.50, 1.18, 0.6], "orn": self.qdeg(0, 0, 0), "support":"Table1", "clearance" : -0.005},

            #LivingLarge
            {"room_type_id":3, "name":"sandwich", "urdf": "livinglarge/sandwich/sandwich.urdf",
            "pos":[-1.05, 0.06, 0.0], "orn": self.qdeg(0, 0, 0), "support":"mesa2", "clearance" : -0.10},
            
            {"room_type_id":3, "name":"comida", "urdf": "livinglarge/vegetable/vegetable.urdf",
            "pos":[-1.10, -0.35, 0.0], "orn": self.qdeg(0, 0, 0), "support":"mesa2", "clearance" : -0.10},

            #{"room_type_id":3, "name":"coke", "urdf": "livinglarge/coke/coke.urdf",
            #"pos":[-1.30, -0.50, 0.0], "orn": self.qdeg(0, 0, 0), "support":"mesa2", "clearance" : -0.10},

            #{"room_type_id":3, "name":"comida", "urdf": "livinglarge/comida/comida.urdf",
            #"pos":[-1.30, -0.50, 0.8], "orn": self.qdeg(0, 0, 0), "support":"mesa2", "clearance" : -0.10},
            
            #Bedroom2
            {"room_type_id":6, "name":"peluche2", "urdf": "bedroom2/peluche/peluche.urdf",
            "pos":[0.65, -0.65, 0.0], "orn": self.qdeg(0, 0, 0), "support":"camita", "clearance" : -0.10},
            
            #{"room_type_id":6, "name":"monopoli", "urdf": "bedroom2/monopoli/monopoli.urdf",
            #"pos":[0.65, -0.45, 0.0], "orn": self.qdeg(0, 0, 0), "support":"camita", "clearance" : -0.10},

            #{"room_type_id":6, "name":"tijeras", "urdf": "bedroom2/tijeras/tijeras.urdf",
            #"pos":[-1.1, -0.15, 0.0], "orn": self.qdeg(0, 0, 0), "support":"tocador", "clearance" : -1.1},
            
            #Dining
            {"room_type_id":7, "name":"estuche", "urdf": "dining/estuche/estuche.urdf",
            "pos":[-0.10, -1.1, 0.0], "orn": self.qdeg(0, 0, 0), "support":"comoda", "clearance" : -0.008},
            
            #{"room_type_id":7, "name":"raton", "urdf": "dining/raton/raton.urdf",
            #"pos":[-0.10, -1.1, 0.0], "orn": self.qdeg(0, 0, 0), "support":"comoda", "clearance" : -0.008},

        ]
        #Para la habitación grande
        self.rot90_z = self.qdeg(0, 0, 90)
    
    #ORIENTACIONES
    #Cambiar las orientaciones     
    def qdeg (self,rx, ry, rz):
        return p.getQuaternionFromEuler([math.radians(rx), math.radians(ry),math.radians(rz)])
    
    #Para que la ventana o la puerta no salgan de la pared
    def limit(self, value, min_value, max_value):
        return max(min_value, min (max_value, value))
    
    def combine_quaternions (self, q_first, q_second):
        _, q_out = p.multiplyTransforms([0, 0, 0], q_first, [0, 0, 0], q_second)
        return q_out
    
    #STLs
    def get_stl_path(self, filename):
        return filename if os.path.isabs(filename) else os.path.join(self.OBJ_DIR, filename)
    
    def get_scale_for(self, path):
        base = os.path.basename(path)
        return self.Scale_file.get(base, self.Scale_file.get(base.lower(), self.Scale_default))

    def normalize_room_config(self, room_config=None):
        final_config = dict(self.default_room_config)
        if room_config is not None:
            final_config.update(room_config)
        kitchen_count  = final_config["kitchen"]
        bathroom_count = final_config["bathrooms"]
        bedroom1_count = final_config["bedroom1"]
        bedroom2_count = final_config["bedroom2"]
        dining_count   = final_config["dining"]

        if kitchen_count not in [0, 1]:
            raise ValueError("kitchen must be 0 or 1")
        if dining_count not in [0, 1]:
            raise ValueError("dining must be 0 or 1")
        if bathroom_count < 0 or bathroom_count > 2:
            raise ValueError("bathroom must be between 0 or 2")
        if bedroom1_count < 0:
            raise ValueError("bedroom1 must higher than 0")
        if bedroom2_count < 0:
            raise ValueError("bedroom2 must higher than 0")
        if bedroom1_count + bedroom2_count > 4:
            raise ValueError("Not possible")
        
        total_rooms = (kitchen_count + bathroom_count + bedroom1_count + bedroom2_count + dining_count)
        if total_rooms != 5:
            raise ValueError ("Not possible")
        return final_config

    #Función para importar objetos .stl
    def import_object(self, path, obj_pose, mass = 0.0):
        '''
        Import objects in simulation.
        Args:
           path (str): STl file path
          obj_pose (list): pose of the object as [position, quaternions]
        '''
        if not os.path.isfile(path):
            raise FileNotFoundError(f"No existe este archivo:{path}")

        print("Cargando objeto fijo:", path)

        s = self.get_scale_for(path)
        scale_vector = [s, s, s]

        base_name = os.path.basename(path).lower()
        is_wall = ("wall" in base_name)
        default_color = [1, 1, 1, 1] 
        rgba = self.current_wall_color if is_wall else default_color

        object_shape = p.createCollisionShape(shapeType=p.GEOM_MESH, 
                                            fileName=path,
                                            meshScale= scale_vector)  
        object_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                            fileName=path,
                                            meshScale= scale_vector,
                                            rgbaColor = rgba)  
        object_id = p.createMultiBody(baseMass = mass, #baseMass=0.00                                
                                            baseCollisionShapeIndex=object_shape,
                                            baseVisualShapeIndex=object_visual_shape,
                                            basePosition=obj_pose[0],
                                            baseOrientation=obj_pose[1]) 
        return object_id
    
    
    def frame_global(self, frame_pose, objects_dict, name, stl_file, local_pos, local_orn, mass=0.0): #ground=False
        frame_pos, frame_orn = frame_pose

        world_pos, world_orn = p.multiplyTransforms(frame_pos, frame_orn, local_pos, local_orn)
        body_id = self.import_object(self.get_stl_path(stl_file), [world_pos, world_orn], mass=mass)
        #if ground:
        #    self.place_body_on_floor(body_id, ground_z=0.0, offset=0.005)
        objects_dict[name] = body_id
        return body_id

    def create_floor(self):
        floor_size = self.grid_n * self.step
        floor_thickness = 0.01
        floor_height = -floor_thickness/2

        #Centro de la matriz
        i_mid = self.grid_n // 2
        j_mid = self.grid_n // 2
        center = self.position(i_mid, j_mid)

        floor_collision = p.createCollisionShape(
            p.GEOM_BOX, 
            halfExtents = [floor_size/2, floor_size /2, floor_thickness/2]
        )
        floor_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents = [floor_size/2, floor_size /2, floor_thickness/2],
            rgbaColor = self.current_floor_color
        )
        floor_id = p.createMultiBody(
            0, floor_collision, floor_visual, [center[0], center[1], floor_height]
        )
        return floor_id
      
    #Posición de la celda (i +X abajo, j +Y derecha) 
    def position(self, i, j):
        return [self.origin_x + i* self.step, self.origin_y + j*self.step, self.z0]

    #Global -> local
    def global_to_local_side(self, global_side, rotation_deg):
        steps = (rotation_deg // 90) %4
        index = self.sides.index(global_side)
        return self.sides[(index + steps) %4]

    def set_global_to_local_side(self, sides_global, rotation_deg):
        return{self.global_to_local_side(s, rotation_deg) for s in sides_global}  

    #Convierte el vector dx, dy global a local de la room
    def delta_global_to_local(self, dx, dy, rotation_deg):
        rotation_by_id = rotation_deg % 360
        if rotation_by_id == 0:
            return dx, dy
        if rotation_by_id == 90:
            return dy, -dx
        if rotation_by_id == 180:
            return -dx, -dy
        if rotation_by_id == 270:
            return -dy, dx
        return dx, dy

    def get_orthogonal_neighbors (self, i, j):
        return [( i-1, j), ( i+1, j), ( i, j-1), ( i, j+1)]
    
    # VECINOS ORTOGONALES CON SU LADO EN GLOBAL
    def get_neighbor_sides(self, i, j):
        neighbors = []
        if j + 1 < self.grid_n: neighbors.append(("N", (i, j+1)))
        if j - 1 >= 0:          neighbors.append(("S", (i, j-1)))
        if i + 1 < self.grid_n: neighbors.append(("E", (i+1, j)))
        if i - 1 >= 0:           neighbors.append(("W", (i-1, j)))
        return neighbors

    #Lados exteriores de la matriz
    def boundary_sides(self, i, j): 
        s = set()
        if j == self.grid_n - 1: s.add("N")
        if j == 0: s.add("S")
        if i == self.grid_n - 1: s.add("E")
        if i == 0: s.add("W")
        return s
    
    #ROTACIONES
    def find_rot_that_maps_boundary(self, boundary_global, desired_local_set):
        for rotation_by_id in [0, 90, 180, 270]:
            mapped = {self.global_to_local_side(s, rotation_by_id) for s in boundary_global}
            if mapped == desired_local_set:
                return rotation_by_id
        return 0

    def compute_rot_by_id_from_layout(self, home):
        rooms_by_id = {}
        for i in range(self.grid_n):
            for j in range(self.grid_n):
                room_id = home[i][j]
                rooms_by_id.setdefault(room_id, []).append((i, j))

        rotation_by_id = {}
        for room_id, cells in rooms_by_id.items():
            anchor_cell = next((c for c in cells if c in self.corner_coords), cells[0])
            outer_sides= self.boundary_sides(*anchor_cell)

            if len(outer_sides) == 2:
                rotation_by_id[room_id] = self.find_rot_that_maps_boundary(outer_sides, {"S", "W"})
            elif len(outer_sides) ==1:
                rotation_by_id[room_id] = self.find_rot_that_maps_boundary(outer_sides, {"S"})
            else:
                rotation_by_id[room_id] = 0
        return rotation_by_id
    
    #PUERTAS INTERIORES
    def door_builder_for_interior_edge(self, cell, side):
        i, j = cell
        if side == "E":
            return (i, j), "E"
        if side == "N":
            return (i, j), "N"
        if side == "W":
            return (i - 1, j), "E"
        if side == "S":
            return (i, j - 1), "N"
        raise ValueError("Side inválido")

    def plan_doors_closed_to_open(self, home, open_room_ids, entrance_rid, bathroom_room_ids, room_type_by_id, rot_by_id, rnd):
        door_sides_global = {}

        for i in range(self.grid_n):
            for j in range(self.grid_n):
                room_id = home[i][j]
                #SOLO HABITACIONES CERRADAS
                if room_id in open_room_ids:
                    continue

                room_type_id = room_type_by_id[room_id]
                allowed_local_sides = self.allowed_door_sides_by_room_type.get(room_type_id, {"N", "E", "S", "W"})

                
                front_door_candidates =[]
                avoided = []

                for side, (ni, nj) in self.get_neighbor_sides(i, j):
                    neighbor_room_id = home[ni][nj]

                    if neighbor_room_id not in open_room_ids:
                        continue
                    local_side = self.global_to_local_side(side, rot_by_id[room_id])
                    #Si ese lado local no está permitidio para esa habitación, no se usa 
                    if local_side not in allowed_local_sides:
                        continue

                    if room_id in bathroom_room_ids and neighbor_room_id == entrance_rid:
                        avoided.append(side)
                    else:
                        front_door_candidates.append(side)
                if not front_door_candidates and avoided:
                    front_door_candidates = avoided
                if not front_door_candidates:
                    return None

                chosen_side = rnd.choice(front_door_candidates)
                door_cell, door_side = self.door_builder_for_interior_edge((i, j), chosen_side)
                door_sides_global.setdefault(door_cell, set()).add(door_side)

        return door_sides_global

    def exterior_windows(self, home, large_room_id, door_sides_by_cell, random_generator, total_windows = 6):
        rooms_by_id = {}
        for i in range(self.grid_n):
            for j in range(self.grid_n):
                room_id = home[i][j]
                rooms_by_id.setdefault(room_id, []).append((i, j))
        exterior_window_room = {}

        for room_id, room_cells in rooms_by_id.items():
            for cell in room_cells:
                i, j = cell

                exterior_sides = self.boundary_sides(i, j)

                for side in exterior_sides:
                    #En la puerta de la calle no se coloca una ventana
                    if side in door_sides_by_cell.get(cell, set()):
                        continue 
                    exterior_window_room.setdefault(room_id, []).append((cell, side))
        window_sides_by_cell = {}

        def add_window(cell, side):
            window_sides_by_cell.setdefault(cell, set()).add(side)
        selected_window_count = 0

        #Poner dos ventanas en el salón
        large_room_candidates = exterior_window_room.get(large_room_id, [])
        random_generator.shuffle(large_room_candidates)

        if len(large_room_candidates) < 2:
            return None
        for cell, side in large_room_candidates[:2]:
            add_window(cell, side)
            selected_window_count += 1
        #Máximo 1 ventana por habitación
        left_rooms_ids = [
            room_id
            for room_id in exterior_window_room.keys()
            if room_id != large_room_id
        ]
        random_generator.shuffle(left_rooms_ids)

        for room_id in left_rooms_ids:
            if selected_window_count >= total_windows:
                break
            room_candidates = exterior_window_room.get(room_id, [])
            if not room_candidates:
                continue
            cell, side = random_generator.choice(room_candidates)

            add_window(cell, side)
            selected_window_count += 1
        if selected_window_count != total_windows:
            return None
        return window_sides_by_cell


    #SIDES POR CELDA
    def compute_open_sides_for_cell(self, i, j, home, open_room_ids):
        current_room_id = home [i][j]
        open_sides = set()

        for side, (ni, nj) in self.get_neighbor_sides(i, j):
            neighbor_room_id = home[ni][nj]
            if neighbor_room_id == current_room_id:
                open_sides.add(side)
            elif (current_room_id in open_room_ids) and (neighbor_room_id in open_room_ids):
                open_sides.add(side)
        return open_sides

    def compute_build_sides_global(self, i, j):
        build = set()
        if i < self.grid_n - 1: build.add("E")
        if j < self.grid_n - 1: build.add("N")
        #BORDES
        if i == 0: build.add("W")
        if j == 0: build.add("S")
        if i ==  self.grid_n - 1: build.add("E")
        if j ==  self.grid_n - 1: build.add("N")
        return build

    #MATRIZ
    def build_home_logic(self, rnd):
        #Matriz 3X3 vacía 
        home = [[-1, -1, -1],
                [-1, -1, -1],
                [-1, -1, -1]]

        large_room_id  = 10
        center_room_id = 20
        corner_room_ids = [30, 31, 32]
        entrance_rid = 40
        other_edge_room_ids =[41, 42]
    
        #Elegir aleatoriamente una coordenada que no sea el centro asignada al primer hueco de la habitación grande
        room1_large = rnd.choice(list(self.non_center_coords))

        #Nos quedamos con las coordenadas que NO estén en el centro y que No estén fuera de la matriz 
        valid_second_large_cells = [p for p in self.get_orthogonal_neighbors(*room1_large) if p in self.non_center_coords]
        room2_large =rnd.choice(valid_second_large_cells)

        #Colocar en la matriz los dos huecos de la habitación grande
        large_room_cells = {room1_large, room2_large}
        for (i,j) in large_room_cells:
            home[i][j] = large_room_id
        #Colocar la habitación central 
        for (i,j) in self.center_coords:
            home[i][j] = center_room_id

        #Ver las esquinas y lados que han quedado libres
        free_corners = [p for p in self.corner_coords if p not in large_room_cells]
        free_edges = [p for p in self.edge_coords if p not in large_room_cells]

        #borde aleatorio
        entrance_cell = rnd.choice(free_edges) 
        free_edges.remove(entrance_cell)

        for (i, j), room_id in zip(free_corners, corner_room_ids):
            home[i][j] = room_id
        #Rellenar la entrada    
        home[entrance_cell[0]][entrance_cell[1]] = entrance_rid

        for(i, j), room_id in zip(free_edges, other_edge_room_ids):
            home[i][j] = room_id

        return home, large_room_id, center_room_id, entrance_rid
    
    def generate_valid_house(self, seed=None, max_home_regens=300, room_config =None):
        rnd = random.Random(seed) if seed is not None else random.Random()
        
        final_room_config = self.normalize_room_config(room_config)
        
        for attempt in range(1, max_home_regens + 1): #El contador empiece en 1
            home, large_room_id, center_room_id, entrance_rid = self.build_home_logic(rnd)

            #Asignación de habitaciones
            room_type_by_id = {
                large_room_id :3,
                center_room_id : 4,
                entrance_rid :5
            }
            configurable_room_ids = []
            for i in range(self.grid_n):
                for j in range(self.grid_n):
                    room_id = home [i][j]
                    if room_id not in {large_room_id, center_room_id, entrance_rid}:
                        if room_id not in configurable_room_ids:
                            configurable_room_ids.append(room_id)
            room_type_list = (
                [0] * final_room_config["kitchen"] + [1] * final_room_config["bathrooms"] + [2] * final_room_config["bedroom1"] + [6] * final_room_config["bedroom2"] + [7] * final_room_config["dining"] 
            )
            rnd.shuffle(room_type_list)
            for room_id, room_type_id in zip (configurable_room_ids, room_type_list):
                room_type_by_id[room_id] = room_type_id
                        
            #Rotaciones
            rot_by_id = self.compute_rot_by_id_from_layout(home)

            #Qué room_id físico corresponde al baño
            bathroom_room_ids ={
                room_id for room_id, room_type_id in room_type_by_id.items()
                if room_type_id == 1
            }

            #Candidatos puerta extetior
            front_door_candidates = []
            for(i, j) in self.edge_coords:
                room_id = home[i][j]
                
                if room_id != entrance_rid: #La puerta exterior solo puede ser la habitación 5
                    continue
                outer_sides = self.boundary_sides(i, j)
                if len(outer_sides) == 1:
                    side = next(iter(outer_sides))
                    front_door_candidates.append(((i, j), side))

            if not front_door_candidates:
                continue

            rnd.shuffle(front_door_candidates)
            print(f"\nIntento {attempt}] Layout generado. Posibles puertas exteriores: {len(front_door_candidates)}")

            for (front_cell, front_side) in front_door_candidates:

                open_rooms_ids = {large_room_id, center_room_id, entrance_rid}

                door_sides_global = self.plan_doors_closed_to_open(home, open_rooms_ids, entrance_rid, bathroom_room_ids, room_type_by_id, rot_by_id, rnd)
                if door_sides_global is None:
                    continue
                
                door_sides_global.setdefault(front_cell, set()).add(front_side)

                window_sides_global = self.exterior_windows(
                    home = home, 
                    large_room_id = large_room_id,
                    door_sides_by_cell = door_sides_global,
                    random_generator = rnd, 
                    total_windows = 6
                )

                if window_sides_global is None:
                    continue

                return {
                    "home": home, 
                    "large_room"        : large_room_id, 
                    "center_room_id"    : center_room_id, 
                    "rot_by_id"         : rot_by_id, 
                    "front_cell"        : front_cell, 
                    "front_side"        : front_side, 
                    "entrance_rid"      : entrance_rid, 
                    "open_room_ids"     : open_rooms_ids, 
                    "door_sides_global" : door_sides_global,
                    "window_sides_global" : window_sides_global,
                    "regen_count"       : attempt, 
                    "seed_used"         : seed,
                    "room_type_by_id"   : room_type_by_id,
                    "room_config"       : final_room_config
                }
            print(f"Intento {attempt}] No hay casa válida")
        raise RuntimeError("No se pudo generar una casa válida tras muchos intentos")

    #GUARDAMOS CADA CELDA EN self. cells(i, j)
    def crear_celda(self, i, j, rotation_deg):
        room_center = self.position(i, j)
        room_orn = self.qdeg(0, 0, rotation_deg)
        self.cells[(i, j)] = {
            "pose"         : [room_center, room_orn],
            "rotation_deg" : rotation_deg,
            "size_xy"      : [self.step, self.step],
            "objs"         : {}  #Para guardar walls
        }
    
    def create_wall_box(self, room_pose, room_objects, object_name, local_position, half_box, color = None, collision=True):
        if color is None:
            color = self.current_wall_color
        room_position, room_orientation = room_pose
        
        world_position, world_orientation = p.multiplyTransforms(
            room_position, room_orientation, local_position, self.qdeg(0, 0, 0)
        )

        if collision:
            collision_shape = p.createCollisionShape(shapeType = p.GEOM_BOX, halfExtents = half_box)
        else:
            collision_shape = -1
        visual_shape = p.createVisualShape(shapeType = p.GEOM_BOX, halfExtents = half_box, rgbaColor = color)
        body_id = p.createMultiBody(baseMass = 0.0, 
                                    baseCollisionShapeIndex = collision_shape, 
                                    baseVisualShapeIndex = visual_shape,
                                    basePosition = world_position,
                                    baseOrientation = world_orientation)
        room_objects[object_name] = body_id
        return body_id

        
    def build_walls_for_cell(self, cell, open_local=None, door_local=None, window_local = None, build_local=None):
        if open_local is None:
            open_local = set()

        if door_local is None:
            door_local = set()

        if window_local is None:
            window_local = set()

        if build_local is None:
            build_local = {"N", "S", "E", "W"}
        
        room_width, room_depth = self.cells[cell]["size_xy"]

        half_room_width = room_width / 2
        half_room_depth = room_depth / 2
        wall_height = self.wall_height
        wall_thickness = self.wall_thickness
        
        room_pose = self.cells[cell]["pose"]
        room_objects = self.cells[cell]["objs"]

        north_y = half_room_depth
        south_y = -half_room_depth
        east_x  = half_room_width
        west_x  = -half_room_width

        def create_wall_piece(side, piece_name, axis_start, axis_end, z_star, z_end, color = None, collision = True):
            piece_length = axis_end - axis_start
            piece_height = z_end - z_star

            if piece_length <= 0.01 or piece_height <= 0.01:
                return
            axis_center = (axis_start + axis_end) / 2
            z_center = z_star + piece_height / 2 - self.z0

            if side in ["N", "S"]:
                y_position = north_y if side == "N" else south_y
                local_position = [axis_center, y_position, z_center]
                half_box = [piece_length / 2, wall_thickness / 2, piece_height / 2]
            else:
                x_position = east_x if side == "E" else west_x
                local_position = [x_position, axis_center, z_center]
                half_box = [wall_thickness / 2, piece_length / 2, piece_height / 2]
            self.create_wall_box(room_pose=room_pose, room_objects=room_objects, object_name=piece_name, local_position=local_position, half_box=half_box, color=color, collision=collision)
       
        def create_solid_wall(side):
            wall_length = room_width if side in ["N", "S"] else room_depth

            create_wall_piece(side=side, piece_name=f"wall_{side}_solid", axis_start=-wall_length / 2, axis_end=wall_length/2, z_star=0.0, z_end=wall_height)
        
        def create_wall_opening(side, opening_width, opening_bottom, opening_height, opening_offset, opening_name, glass=False):
            wall_length = room_width if side in ["N", "S"] else room_depth
            wall_start = -wall_length /2
            wall_end = wall_length /2

            max_offset = wall_length/2 - opening_width/2 - 0.05
            opening_center = self.limit(opening_offset, -max_offset, max_offset)

            opening_start = opening_center - opening_width / 2
            opening_end = opening_center + opening_width / 2
            opening_top = opening_bottom + opening_height

            create_wall_piece(side=side, piece_name=f"wall_{side}_{opening_name}_side_1", axis_start=wall_start, axis_end=opening_start, z_star=0.0, z_end=wall_height)
            create_wall_piece(side=side, piece_name=f"wall_{side}_{opening_name}_side_2", axis_start=opening_end, axis_end=wall_end, z_star=0.0, z_end=wall_height)
            create_wall_piece(side=side, piece_name=f"wall_{side}_{opening_name}_bottom", axis_start=opening_start, axis_end=opening_end, z_star=0.0, z_end=opening_bottom)
            create_wall_piece(side=side, piece_name=f"wall_{side}_{opening_name}_top", axis_start=opening_start, axis_end=opening_end, z_star=opening_top, z_end=wall_height)

            if glass:
                create_wall_piece(side=side, piece_name=f"wall_{side}_{opening_name}_glass", axis_start=opening_start, axis_end=opening_end, z_star=opening_bottom, z_end=opening_top, color=self.window_color, collision=False)
        for side in ["N", "S", "E", "W"]:
            if side not in build_local:
                continue
            if side in open_local:
                continue
            if side in door_local:
                create_wall_opening(side=side, opening_width = self.door_width, opening_bottom=0.0, opening_height=self.door_height, opening_offset=self.door_offset, opening_name="door", glass=False)
            elif side in window_local:
                create_wall_opening(side=side, opening_width = self.window_width, opening_bottom=self.window_bottom, opening_height=self.window_height, opening_offset=self.window_offset, opening_name="window", glass=self.window_glass)
            else:
                create_solid_wall(side)
    

    def create_rooms(self, home, rot_by_id, large_room_id, room_type_by_id):
        rooms_by_id = {}
        for i in range(self.grid_n):
            for j in range(self.grid_n):
                room_id = home[i][j]
                rooms_by_id.setdefault(room_id, []).append((i, j))
       
        self.room = {}

        for room_id, cells in rooms_by_id.items():
            rotation_deg = rot_by_id.get(room_id, 0)
            room_orn = self.qdeg(0, 0, rotation_deg)

            #Centro (En caso de la habitación grande se calcula la media)
            if room_id != large_room_id or len(cells) == 1:
                center = self.position(*cells[0])
            else:
                first_cell, second_cell = cells[0], cells[1]
                first_x, first_y, _ = self.position(*first_cell)
                second_x, second_y, _ = self.position(*second_cell)
                center = [0.5 * (first_x + second_x), 0.5 *(first_y + second_y), self.z0]
            
            room_type_id = room_type_by_id[room_id]

            room_data = {
                "name": self.room_name.get(room_type_id, f"Room{room_type_id}"),
                "room_type_id": room_type_id,
                "pose": [center, room_orn],
                "objs":{}, 
                "rotation_deg" : rotation_deg  #Se guarda la rotación en grados 
            }
            #Guardar si la habitación grande está en horizontal o en vertical
            if room_id == large_room_id and len(cells) == 2:
                (i1, j1), (i2, j2) = cells[0], cells[1]
                global_axis = "X" if (i1 != i2) else "Y"
                room_data["global_axis"] = global_axis

                global_dx, global_dy = (1,0) if global_axis == "X" else (0, 1)
                local_dx, local_dy = self.delta_global_to_local(global_dx, global_dy, rotation_deg)
                room_data["swap_axes"] = (local_dx == 0 and abs(local_dy) == 1)

            self.room[room_id] = room_data
    
    def room_pointd(self):
        """
        Devuelve los puntos de navegación en coordenadas globales. Los puntos se definen 
        manualmente en coordenadas locales de cada habitación.
        """
        points = {}

        home = self.data["home"]
        door_sides_global = self.data["door_sides_global"]

        home_center = self.position(self.grid_n // 2, self.grid_n // 2)
        points["Home_center"] = (home_center[0], home_center[1])

        for room_id, room_data in self.room.items():
            room_name = room_data["name"]
            room_pose = room_data["pose"]
            room_center = room_pose[0]
            room_orientation = room_pose[1]

            #Centro de las habitaciones
            points[room_name] = (room_center[0], room_center[1])
            points[f"{room_name}_center"] = (room_center[0], room_center[1])

            #Puntos del BAÑO
            if room_name == "Bathroom":

                lavabo_goal_local_pos = [0.57, -0.7, 0.0] #0.45, -0.35, 0.0
                lavabo_goal_world_pos, _ = p.multiplyTransforms(room_center, room_orientation, lavabo_goal_local_pos, self.qdeg(0, 0, 0))
                points["Bathroom_lavabo"] = (lavabo_goal_world_pos[0], lavabo_goal_world_pos[1])

                lavabo_object_local_pos = [1.3, -1.0, 0.0]
                lavabo_object_world_pos, _ = p.multiplyTransforms(room_center, room_orientation, lavabo_object_local_pos, self.qdeg(0, 0, 0))
                points["Bathroom_lavabo_object"] = (lavabo_object_world_pos[0], lavabo_object_world_pos[1])

                theta_lavabo = math.atan2(lavabo_object_world_pos[1] - lavabo_goal_world_pos[1],
                                            lavabo_object_world_pos[0] - lavabo_goal_world_pos[0])
                
                points["Bathroom_lavabo_pose"] = (lavabo_goal_world_pos[0], lavabo_goal_world_pos[1], theta_lavabo)

                """
                bathroom_points_local ={
                "Bathroom_door": [-0.6, 1.37, 0.0],
                "Bathroom_center": [0.0, 0.0, 0.0],
                "Bathroom_lavabo": [0.9, -0.40, 0.0]
                }

                for point_name, local_pos in bathroom_points_local.items():
                    world_pos, _ = p.multiplyTransforms(room_center, room_orientation, local_pos, self.qdeg(0, 0, 0))
                    points[point_name] = (world_pos[0], world_pos[1])
                """
            if room_name == "Bedroom1":
                
                bed_local = [-0.5, 0.5, 0.0]
                bed_world, _ = p.multiplyTransforms(room_center, room_orientation, bed_local, self.qdeg(0, 0, 0))
                points["Bedroom1_bed"] = (bed_world[0], bed_world[1])

                """
                bedroom1_points_local ={
                "Bedroom1_door": [1.05, -0.6, 0.0],
                "Bedroom1_center": [0.0, 0.0, 0.0],
                "Bedroom1_bed": [0.4, 0.4, 0.0]
                }

                for point_name, local_pos in bedroom1_points_local.items():
                    world_pos, _ = p.multiplyTransforms(room_center, room_orientation, local_pos, self.qdeg(0, 0, 0))
                    points[point_name] = (world_pos[0], world_pos[1])
                """

        def door_point(roon_name):
            target_id = None
            for room_id, room_data in self.room.items():
                if room_data["name"] == roon_name:
                    target_id = room_id
                    break
            if target_id is None:
                return ValueError(f"No se ha encontrado la habitación {roon_name}")
            
            for cell, sides in door_sides_global.items():
                i, j = cell
                room_id_cell = home[i][j]
                for side in sides:
                    if side == "N":
                        neighbor_cell = (i, j + 1)
                    elif side == "S":
                        neighbor_cell = (i, j - 1)
                    elif side == "E":
                        neighbor_cell = (i + 1, j)
                    else: #W
                        neighbor_cell = (i - 1, j)
                    room_id_neighbor = None
                    
                    if 0 <= neighbor_cell[0] < self.grid_n and 0 <= neighbor_cell[1] < self.grid_n:
                        room_id_neighbor = home[neighbor_cell[0]][neighbor_cell[1]]

                    if room_id_cell == target_id or room_id_neighbor == target_id:
                        rotation_deg = self.cells[cell]["rotation_deg"]
                        local_side = self.global_to_local_side(side, rotation_deg)

                        half = self.step / 2

                        if local_side == "N":
                            local_pos = [self.door_offset, half, 0.0]
                        elif local_side == "S":
                            local_pos = [self.door_offset, -half, 0.0]
                        elif local_side == "E":
                            local_pos = [half, self.door_offset, 0.0]
                        else: #W
                            local_pos = [-half, self.door_offset, 0.0]
                        
                        cell_pose = self.cells[cell]["pose"]
                        door_world, _ = p.multiplyTransforms(cell_pose[0], cell_pose[1], local_pos, self.qdeg(0, 0, 0))
                        return (door_world[0], door_world[1])
            raise ValueError(f"No se ha encontrado una puerta para la habitación {roon_name}")
        
        points["Bathroom_door"] = door_point("Bathroom")
        points["Bedroom1_door"] = door_point("Bedroom1")

       
        #La posición inicial del robot 
        if self.start_point is not None:
            points["Start"] = self.start_point
        return points 

    
    def create_fixed_objects(self):
        #Ver que posición grande está si horizontal o vertical
        large_room_id = self.data["large_room"] if self.data is not None else 3
        
        for room_id, room_data in self.room.items():
            room_type_id = room_data ["room_type_id"]
            items = self.fixed_objects_by_room_name.get(room_type_id, [])
        
            room_pose = self.room[room_id]["pose"]
            room_objects = self.room[room_id]["objs"]
            room_name = self.room[room_id]["name"]

            swap_axes = (room_id == large_room_id and self.room[room_id].get("swap_axes", False))

            for fixed_item in items:
                obj_name = f"{room_name}_{fixed_item['name']}"

                local_pos = fixed_item["pos"]
                local_orn = fixed_item["orn"]

                if swap_axes:
                    #Rotas (x, y) -> (-y, x)
                    x, y, z = local_pos
                    local_pos = [-y, x, z]
                    #Se rota junto con la habitación los objetos
                    local_orn = self.combine_quaternions(self.rot90_z, local_orn)
                #body_id = self.frame_global(room_pose, room_objects, obj_name, fixed_item["stl"], local_pos, local_orn, mass=0.0, ground=True)
                body_id = self.frame_global(room_pose, room_objects, obj_name, fixed_item["stl"], local_pos, local_orn, mass=0.0)#ground=False
                if "color" in fixed_item: 
                    p.changeVisualShape(body_id, -1, rgbaColor=fixed_item["color"])
    
    def create_movable_objects(self):
        large_room_id = self.data["large_room"] if self.data is not None else 3

        for room_id, room_data in self.room.items():
            room_type_id = room_data["room_type_id"]
            room_pose = self.room[room_id]["pose"]
            room_objects = self.room[room_id]["objs"]
            room_name = self.room[room_id]["name"]

            swap_axes = (room_id == large_room_id and room_data.get("swap_axes", False))
            movable_items =[
                item for item in self.movable_objects
                if item["room_type_id"] == room_type_id
            ]
            for item in movable_items:
                local_pos = item["pos"]
                local_orn = item["orn"]

                #Nuevo
                use_aabb = item.get("use_aabb", True)

                if use_aabb:
                    local_z_extra = local_pos[2]
                    local_pos[2] = 0.0
                else:
                    local_z_extra = 0.0
               
                if swap_axes:
                    x, y, z = local_pos
                    local_pos = [-y, x, z]
                    local_orn = self.combine_quaternions(self.rot90_z, local_orn)

                obj_name = f"{room_name}_{item['name']}"

                frame_pos, frame_orn = room_pose 

                world_pos, world_orn = p.multiplyTransforms (frame_pos, frame_orn, local_pos, local_orn)
                world_pos = list(world_pos)

                if item.get("support") is not None:
                    support_name = f"{room_name}_{item['support']}"
                    support_id = room_objects.get(support_name, None)
                    
                    if support_id is None:
                        print(f"No se ha encontrado el soporte{support_name}")
                        continue
                    _, support_max = p.getAABB(support_id)
                    world_pos[2] = support_max[2] + item.get("clearance", 0.003) + local_z_extra
                
                else:
                    world_pos[2] = 0.0 + item.get("clearance", 0.003) + local_z_extra


                if "urdf" in item:
                    #frame_pos, frame_orn = room_pose
                    #world_pos, world_orn = p.multiplyTransforms (frame_pos, frame_orn, local_pos, local_orn)
                    urdf_path = self.get_stl_path(item["urdf"])
                    print("Object urdf path: ", urdf_path)
                    print("Object world position: ", world_pos, "world orientation: ", world_orn)
                    body_id = p.loadURDF(urdf_path, basePosition = world_pos, baseOrientation = world_orn, useFixedBase=False)

                    #room_objects[obj_name] = body_id
                else:
                    body_id = self.import_object(self.get_stl_path(item["stl"]), [world_pos, world_orn], mass = 0.0)
                    #body_id = self.frame_global(room_pose, room_objects, obj_name, item["stl"], local_pos, local_orn, mass=0.0, ground = False)

                room_objects[obj_name] = body_id
                #if item ["support"] is not None:
                 #   support_name = f"{room_name}_{item['support']}"
                 #   support_id = room_objects.get(support_name, None)
                 #   if support_id is None:
                 #       continue
                 #   self.place_body_on_support(body_id, support_id, clearance=item.get("clearance", 0.003))
                #else:
                  #  self.place_body_on_floor(body_id, ground_z=0.0, offset=item.get("clearance", 0.003))
    
    def generate_objects (self, urdf_file, position, orientation_deg=(0, 0, 0), room_name = None, support=None, clearance=0.003):
        obj_orn = self.qdeg(orientation_deg[0], orientation_deg[1], orientation_deg[2])
        urdf_path = self.get_stl_path(urdf_file)

        local_pos = list(position)
        local_z_extra = local_pos[2]
        local_pos[2] = 0.0

        if room_name is not None:
            room_id_found = None

            for room_id, room_data in self.room.items():
                if room_data["name"] == room_name:
                    room_id_found = room_id 
                    break
            if room_id_found is None:
                raise ValueError(f"No existe la habitación{room_name}")
            
            room_pose = self.room[room_id_found]["pose"]
            room_objects = self.room[room_id_found]["objs"]

            world_pos, world_orn = p.multiplyTransforms(
                room_pose[0], room_pose[1], position, obj_orn)

            world_pos = list(world_pos)
            
            if support is not None:
                support_name = f"{room_name}_{support}"
                support_id = room_objects.get(support_name)

                if support_id is None:
                    raise ValueError(f"no existe {support_name}")
                _, support_max = p.getAABB(support_id)
                world_pos[2] = support_max[2] + clearance + local_z_extra
            else:
                world_pos[2] = 0.0 + clearance + local_z_extra
        

            body_id = p.loadURDF(urdf_path, basePosition=world_pos, baseOrientation =world_orn, useFixedBase=False)

            object_name = os.path.splitext(os.path.basename(urdf_file))[0]
            full_name = f"{room_name}_{object_name}"
            room_objects[full_name] = body_id

            return body_id

            #if support is not None:
             #   support_name = f"{room_name}_{support}"
             #   support_id =room_objects.get(support_name)

            #    if support_id is None:
            #        raise ValueError("No existe el soporte {support_name}")
            #    self.place_body_on_support(body_id, support_id, clearance)
            #else:
             #   self.place_body_on_floor(body_id, ground_z=0.0, offset=clearance)
            #return body_id
        
        world_pos = list(position)
        local_z_extra = world_orn[2]
        world_pos[2] = 0.0 + clearance + local_z_extra
        
        body_id = p.loadURDF(urdf_path, basePosition=position, baseOrientation=obj_orn, useFixedBase = False)
        #self.place_body_on_floor(body_id, ground_z=0.0, offset=clearance)
        return body_id
                

    def get_rooms_with_objects(self):
        rooms_summary = []
        for room_id, room_data in self.room.items():
            room_name = room_data["name"]
            prefix = room_name + "_"
            objects_names = []

            for full_name in room_data["objs"].keys():
                if full_name.startswith(prefix):
                    objects_names.append(full_name[len(prefix):])
                else:
                    objects_names.append(full_name)
            rooms_summary.append({
                "room_id": room_id,
                "room_name": room_name,
                "objects": objects_names
            })
        return rooms_summary
    
    #MAIN
    def generate_home(self, seed=None, max_home_regens=300, room_config=None, floor_color =None, wall_color =None, show_info=True):

        self.current_floor_color = list(floor_color) if floor_color is not None else list(self.default_floor_color)
        self.current_wall_color = list(wall_color) if wall_color is not None else list(self.default_wall_color)
        self.data = self.generate_valid_house(seed=seed, max_home_regens=max_home_regens, room_config=room_config)
        

        self.create_floor()

        home                = self.data["home"]
        rot_by_id           = self.data["rot_by_id"]
        open_room_ids       = self.data["open_room_ids"]
        door_sides_global   = self.data["door_sides_global"]
        window_sides_global = self.data["window_sides_global"]
        
        if show_info:
            print ("\n ===HOME válida===")
            print("Intentos necesarios:", self.data["regen_count"])
            print("Puerta exterior:", self.data["front_cell"], self.data["front_side"], "| entrance_rid =", self.data["entrance_rid"])
            print("Habitaciones abiertas:", open_room_ids)
            print ("Habitaciones asignadas:", self.data["room_type_by_id"])
            print ("Número de habitaciones:", self.data["room_config"])
        
        #Crear celdas
        self.cells = {}
        for i in range(self.grid_n):
            for j in range(self.grid_n):
                room_id = home[i][j]
                angle = rot_by_id.get(room_id, 0)
                self.crear_celda(i, j, angle)
        #Construir las paredes
        for i in range(self.grid_n):
            for j in range(self.grid_n):
                cell = (i, j)
                rotation_deg = self.cells[cell]["rotation_deg"]

                open_global = self.compute_open_sides_for_cell(i, j, home, open_room_ids)
                door_global = door_sides_global.get((i, j), set())
                window_global = window_sides_global.get((i, j), set())
                build_global = self.compute_build_sides_global(i, j)

                open_local = self.set_global_to_local_side(open_global, rotation_deg)
                door_local = self.set_global_to_local_side(door_global, rotation_deg)
                window_local = self.set_global_to_local_side(window_global, rotation_deg)
                build_local = self.set_global_to_local_side(build_global,rotation_deg)

                self.build_walls_for_cell(cell, open_local=open_local, door_local=door_local, window_local= window_local, build_local=build_local)
 
        self.create_rooms(home, rot_by_id, self.data["large_room"], self.data["room_type_by_id"])
        
        #Recolocar ADAM
        entrance_room = self.room[self.data["entrance_rid"]]
        room_center = entrance_room["pose"][0]
        front_side = self.data["front_side"]
        
        offset = self.step * 0.25
        current_pos, _ = p.getBasePositionAndOrientation(self.adam.robot_id)
        if front_side == "N":
            robot_pos = [room_center[0], room_center[1] - offset, current_pos[2]]
            robot_orn = self.qdeg(0, 0, 270)
        elif front_side == "S":
            robot_pos = [room_center[0], room_center[1] + offset, current_pos[2]]
            robot_orn = self.qdeg(0, 0, 180)
        elif front_side == "E":
            robot_pos = [room_center[0] - offset, room_center[1], current_pos[2]]
            robot_orn = self.qdeg(0, 0, 90)
        else: #W
            robot_pos = [room_center[0]+ offset, room_center[1], current_pos[2]]
            robot_orn = self.qdeg(0, 0, 0)
        p.resetBasePositionAndOrientation(self.adam.robot_id, robot_pos, robot_orn)
        
        #Nuevo
        self.start_point = (robot_pos[0], robot_pos[1])


        self.create_fixed_objects()
        # self.create_movable_objects()
        if show_info:
            print ("\nHabitaciones y objetos:")
            for room_info in self.get_rooms_with_objects():
                object_list = room_info["objects"]
                text = ",".join(object_list) if object_list else "No hay objetos"
                print (f"-{room_info['room_name']} (id = {room_info}): {text}")
        
        #p.loadURDF(os.path.join(self.OBJ_DIR, "kitchen/chica/FemaleVisitor.urdf"), basePosition=[4.0, 7.0, 0], baseOrientation=self.qdeg(0, 0, 0), useFixedBase=True)

        return self.data
    


if __name__ == "__main__":
    env = Environment()
    
    env.generate_home(
        seed=None, 
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

    env.generate_objects (
        urdf_file="bedroom1/vaso/vaso.urdf",
        room_name="Bedroom1",
        position= [-0.2, 1.1, 0.0],
        orientation_deg= (0, 0, 0),
        support="Table1",
        clearance=-0.005
    )
    while True:
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

        