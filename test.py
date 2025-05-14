from scripts.adam import ADAM
import time
import pybullet as p
import pybullet_data
import scipy.io

if __name__ == '__main__':
    robot_urdf_path = "/home/adrian/Escritorio/ImitationLearning/SimuladorADAM/Adam_sim/paquetes_simulacion/rb1_base_description/robots/robotDummy.urdf"
    
    adam = ADAM(robot_urdf_path,0,1, False)
    
    adam.print_robot_info()
    #time.sleep(100)
    
    p.setRealTimeSimulation(adam.useRealTimeSimulation)
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
    
    adam.print_robot_info()
    #time.sleep(1000000)
    print("Dale chicha")
    forward=0
    turn=0
    """ p.setJointMotorControl2(adam.robot_id, 3, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(adam.robot_id, 4, p.VELOCITY_CONTROL, force=0) """
    
    adam.navigation.move_base_continuous((-6,6,0.5),pos_tolerance=0.01,angle_tolerance=0.03)
    
    """ while (1):
        keys = p.getKeyboardEvents()
        
        leftWheelVelocity=0
        rightWheelVelocity=0
        speed=8
        
        for k,v in keys.items():
                    if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                            turn = -1
                    if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                            turn = 0
                    if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                            turn = 1
                    if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
                            turn = 0

                    if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                            forward=1
                    if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
                            forward=0
                    if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                            forward=-1
                    if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
                            forward=0

        rightWheelVelocity = (forward + turn) * speed
        leftWheelVelocity  = (forward - turn) * speed
        print("Rueda R", rightWheelVelocity)
        print("Rueda L",leftWheelVelocity)
        
        
        
        p.setJointMotorControl2(adam.robot_id,4,p.VELOCITY_CONTROL,targetVelocity=leftWheelVelocity,force=100)
        p.setJointMotorControl2(adam.robot_id,3,p.VELOCITY_CONTROL,targetVelocity=rightWheelVelocity,force=100) """
        
        
    #time.sleep(100000)