from scripts.adam import ADAM
import time
import pybullet as p
import pybullet_data
import scipy.io
import os
import math

if __name__ == '__main__':
    base_path = os.path.dirname(__file__)
    robot_urdf_path = os.path.join(base_path,"..","models","robot", "rb1_base_description", "robots", "robotDummy.urdf")
    
    adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=False, use_ros=True)
    adam.wait(0.1)
    #adam.print_robot_info()
    adam.sensors.start_lidar()
    
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    """ initial_left_pose = [0.11,-1.96,-0.79,0.67,-0.08,-0.01]
    initial_right_pose = [0.66,-2.26,-0.70,-0.14,2.55,-1.15] """
    #initial_left_pose = [-0.85,-1.72,0.042,-1.036,1.0249,1.911]
    #initial_right_pose = [-0.43,-1.83,0.681,-1.43,1.908,-1.53]
    initial_right_pose = [2.7354,-1.5973,-1.1237,-1.2453,0.4834,-0.1599]
    initial_left_pose = [-2.0935,-2.0019,-0.8266,-2.6136,4.4560,5.1101]
    
    
    
    
    # Initial pose
    """ for _ in range(5):
        adam.arm_kinematics.initial_arm_pose("right",initial_right_pose)
        adam.arm_kinematics.initial_arm_pose("left",initial_left_pose) """
    
    print("Dale chicha")
    
    """ p.setJointMotorControl2(adam.robot_id, 3, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(adam.robot_id, 4, p.VELOCITY_CONTROL, force=0) """
    #adam.navigation.move_base_continuous((-6,6,0.5),pos_tolerance=0.04,angle_tolerance=0.08)
    """ while(1):
        adam.teleop.teleoperate_base(debug=True) """
        
    # Añadir obstáculo: una caja delante del robot
    box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.275, 0.05, 0.17])
    box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.275, 0.05, 0.17], rgbaColor=[1, 1, 0, 1])
    box_id = p.createMultiBody(baseMass=0.1,
                            baseCollisionShapeIndex=box_collision,
                            baseVisualShapeIndex=box_visual,
                            basePosition=[4.4, -1.4, 0.17],
                            baseOrientation=p.getQuaternionFromEuler([0,0,math.pi/2]))  # Frente al robot
    
    # Añadir obstáculo: una caja delante del robot
    box_collision2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.13, 0.25])
    box_visual2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.25, 0.13, 0.25], rgbaColor=[0.7, 0, 45, 1])
    box_id2 = p.createMultiBody(baseMass=0.1,
                            baseCollisionShapeIndex=box_collision2,
                            baseVisualShapeIndex=box_visual2,
                            basePosition=[3, 0.7, 0.25],
                            baseOrientation=p.getQuaternionFromEuler([0,0,math.pi/2]))  # Frente al robot
    step=True
    con = 0
    points =[(2,-1.5,0),(3.5,-0.6,0.5),(5,0,0)]
    """ adam.utils.draw_frame(([points[0][0],points[0][1],0],p.getQuaternionFromEuler([0,0,points[0][2]])),axis_length=0.5,line_width=6)
    adam.utils.draw_frame(([points[1][0],points[1][1],0],p.getQuaternionFromEuler([0,0,points[1][2]])),axis_length=0.5,line_width=6)
    adam.utils.draw_frame(([points[2][0],points[2][1],0],p.getQuaternionFromEuler([0,0,points[2][2]])),axis_length=0.5,line_width=6) """
    #time.sleep(5)
    while step:
        #adam.teleop.teleoperate_base(debug=False)
        #adam.ros.teleop_real_base()
        
        point = adam.navigation.move_base_continuous(points[con],pos_tolerance=0.05,angle_tolerance=0.5,orient_tolerance=0.5,use_lidar=True,use_ros=False)
        if point == False:
            con=con+1
        if con == 3:
            step=False
        adam.step()
