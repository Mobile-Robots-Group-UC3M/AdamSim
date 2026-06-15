import os
from scripts.adam import ADAM
import pybullet as p
import math
from ultralytics import YOLO
import time
import cv2


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

# Start LiDAR and configure visualizer
#adam.sensors.start_lidar()
#p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# Move arms to initial positions
adam.arm_kinematics.move_arm_joints_to_angles("right", [-3.307, -0.794, 0.00, -1.521, 0.00, -1.257]) #-2.712, -0.661, 0, -1.521, -0.661, -1.521   -2.712, -0.5, 0.0, -1.1, -0.661, -1.521
adam.arm_kinematics.move_arm_joints_to_angles("left", [-3.175, -0.794, 0.00, -1.389, 3.307, -4.431]) #-0.066, -2.116, -0.265, -1.587, 0.132, 1.852
adam.hand_kinematics.move_hand_to_dofs('right', [1000, 1000, 1000, 1000, 1000, 0])

# Wait for arms to move
adam.wait(2)

#Initialise camera
camera_angle = 0
adam.sensors.move_camera_angle(camera_angle)

# Load YOLO model
model = YOLO("yolo26n.pt")  # load an official model

#Carpeta donde se guardan las imágenes
image_folder = "/home/alumnos/tfg-carmen/TFGenvironmentsAdamSim/data/images"

last_capture = time.time()
frame_id = 0



print("simulation started")
print("frame; tiempo_yolo_s")


while True:
    #Move robot with keyboard 
    adam.teleop.teleoperate_base()
    keys = p.getKeyboardEvents()
    # Press 'c' to move the camara between 0deg and -45deg
    if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
        
        camera_angle = -45 - camera_angle
        adam.sensors.move_camera_angle(camera_angle)
        print("Moving camera to angle:", camera_angle)

    current_time = time.time()

    if current_time - last_capture >= 2.0: #Capturar la imagen cada 2 segundos

        rgb, _ = adam.sensors.get_rgbd_image_from_link(width=640, height=480, fov=60, near=0.01, far=5.0)

        filename = f"camera_image_{frame_id}.png"

        #adam.sensors.save_rgb_image(rgb_array=rgb, folder_path=image_folder, filename=filename)
        start_yolo = time.time()

        results = model(rgb)

        end_yolo = time.time()
        yolo_time = end_yolo - start_yolo
        
        annotated_frame = results[0].plot()
        cv2.imshow("YOLO Camera view", annotated_frame)
        cv2.waitKey(1)  # Display the window for 1 ms

        if results[0].boxes is not None and len(results[0].boxes) > 0:
            cv2.imwrite(os.path.join(image_folder, filename), annotated_frame)
            print(f"{filename}; {yolo_time:.4f} s")
        else:
            print(f"{filename}; {yolo_time:.4f} s - No objects detected.")

        frame_id += 1
        last_capture = current_time

        #for result in results:
        #    #result.show()
        #    annotated_frame = result.plot()
        #    cv2.imshow("YOLO Camera view", annotated_frame)  
        #    cv2.waitKey(1)  # Display the window for 1 ms
#
        #    if result.boxes is not None and len(result.boxes) > 0:
        #        
        #        filename = f"camera_yolo_{frame_id}.png"
        #        cv2.imwrite(os.path.join(image_folder, filename), annotated_frame)
        #        print(f"Saved detection image: {filename}")
#
        #    frame_id += 1
        #last_capture = current_time


            
            
    adam.step()

            #if result.boxes is None or len(result.boxes) == 0:
            #    print("No objects detected.")
            #else:
            #    for box in result.boxes:
            #        cls_id = int(box.cls[0])  
            #        conf = float(box.conf[0])
            #        name = result.names[cls_id]
            #        print(f"Detected object: {name} with confidence {conf:.2f}")
        #



  
