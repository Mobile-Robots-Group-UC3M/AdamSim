#!/usr/bin/env python3

from sliders import Sliders
from arms_kinematics import ArmsKinematics
from hands_kinematics import HandsKinematics
import time
import pybullet as p
import pybullet_data
import scipy.io
from node_connection import Node
import rospy

#Class for simulation
class Simulation(Sliders, ArmsKinematics, HandsKinematics):
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True)

    #! START SIMULATION
    def run_simulation(self):

        p.setRealTimeSimulation(self.useRealTimeSimulation)

        #Initial_arm_poses
        initial_left_pose = [0.11,-1.96,-0.79,0.67,-0.08,-0.01]
        initial_right_pose = [0.66,-2.26,-0.70,-0.14,2.55,-1.15]

        #Cargamos el archivo de matlab
        positions_data = scipy.io.loadmat('/home/adrian/Escritorio/ImitationLearning/GaussianBelief/src/Adam_sim/simulaciones_reales/positions.mat')
        positions = positions_data['positionR']
        orientations_data = scipy.io.loadmat('/home/adrian/Escritorio/ImitationLearning/GaussianBelief/src/Adam_sim/simulaciones_reales/orientations.mat')
        orientations = orientations_data['quaternionR']
        
        poses = [
        [(positions[i, 0], positions[i, 1], positions[i, 2]), (orientations[i, 1], orientations[i, 2], orientations[i, 3], orientations[i, 0])]
        for i in range(positions.shape[0])]

        poses2=poses
        print("len de poses: ",len(poses))


        # Initial pose
        for _ in range(100):
            self.initial_arm_pose("right",initial_right_pose)
            self.initial_arm_pose("left",initial_left_pose)
        
        # #Initialize subscribers
        # node.read_joints("right")

        
        #Inicializamos el tiempo de simulacion
        if (self.useSimulation and self.useRealTimeSimulation==0):
            p.stepSimulation()
        
        for pose in poses:
            self.move_arm_to_pose("right", pose)

            if (self.useSimulation and self.useRealTimeSimulation==0):
                p.stepSimulation()
                time.sleep(self.t)

        self.hand_forward_kinematics("both", [800,800,800,800,800], [800,800,800,800,800])

        if (self.useSimulation and self.useRealTimeSimulation==0):
            p.stepSimulation()
            time.sleep(self.t)

        #time.sleep(200)


        # self.print_robot_info()

        # while(True):
        ## ROS

        # # Publicamos la trayectoria en el topic
        # if self.pub_left and self.pub_right:
        #     pub_arm = "both"
        # elif self.pub_left:
        #     pub_arm = "left"
        # elif self.pub_right:
        #     pub_arm = "right"
        # else:
        #     raise ValueError("No se publicará ningun mensaje")

        # #frecuencia de publicacion
        # rate = rospy.Rate(10)  # 10 Hz

        # for joint in self.right_joints:
        #     node.publish_joints(pub_arm, joint)

        #     rate.sleep()

        # self.pub_left, self.pub_right = False, False

            # self.apply_slider_values()
            # if self.detect_autocollisions():
            #     break



        if not self.useRealTimeSimulation:
            time.sleep(self.t)


# URDF robot
robot_urdf_path = "/home/adrian/Escritorio/ImitationLearning/GaussianBelief/src/Adam_sim/paquetes_simulacion/rb1_base_description/robots/robot.urdf"

# Robot_body STL
robot_stl_path = "/home/adrian/Escritorio/ImitationLearning/GaussianBelief/src/Adam_sim/paquetes_simulacion/rb1_base_description/meshes/others/adam_model.stl"


adam_robot = Simulation(robot_urdf_path,robot_stl_path,1,0)
#adam_robot.create_sliders()

# node = Node() 

adam_robot.run_simulation()
