#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pybullet as p
from adam import ADAM

# Variable global para guardar el último mensaje de /joint_states
latest_joint_state_left = None
latest_joint_state_right = None

def joint_state_right_callback(msg):
    global latest_joint_state_right, last_joint_state_log_time

    last_joint_state_log_time = rospy.Time.now()
    latest_joint_state_right = msg

def joint_state_left_callback(msg):
    global latest_joint_state_left
    latest_joint_state_left = msg

def convert_pose_to_msg(pose_list):
    """Convierte una pose [position, orientation] a geometry_msgs/Pose"""
    position, orientation = pose_list
    pose_msg = Pose()
    pose_msg.position.x = position[0]
    pose_msg.position.y = position[1]
    pose_msg.position.z = position[2]
    pose_msg.orientation.x = orientation[0]
    pose_msg.orientation.y = orientation[1]
    pose_msg.orientation.z = orientation[2]
    pose_msg.orientation.w = orientation[3]
    return pose_msg

def convert_joints_to_msg(joint_positions):
    """
    Crea un mensaje JointTrajectory a partir de una lista de nombres y posiciones.
    
    Args:
        joint_names (list of str): Nombres de las articulaciones.
        joint_positions (list of float): Posiciones deseadas para cada articulación.
        
    Returns:
        JointTrajectory: Mensaje listo para publicar.
    """
    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['robot_left_arm_elbow_joint', 'robot_left_arm_shoulder_lift_joint', 'robot_left_arm_shoulder_pan_joint',
                            'robot_left_arm_wrist_1_joint', 'robot_left_arm_wrist_2_joint', 'robot_left_arm_wrist_3_joint']
    
    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.velocities = [0, 0, 0, 0, 0, 0]
    point.accelerations = [0, 0, 0, 0, 0, 0]
    point.effort = [0.8682, 1.612, -1.5094, 0.3217, -0.2149, 0.0096]
    point.time_from_start.secs = 1  # Tiempo opcional para alcanzar la posición
    
    traj_msg.points.append(point)
    return traj_msg

def main():
    rospy.init_node('adam_node', anonymous=True)

    # Publisher
    pose_pub = rospy.Publisher('/right_hand_pose', Pose, queue_size=10)
    joints_pub = rospy.Publisher('/robot/left_arm/scaled_pos_traj_controller/command', JointTrajectory, queue_size=10)

    # Subscriber
    rospy.Subscriber('/robot/left_arm/joint_states', JointState, joint_state_left_callback, queue_size=1)
    rospy.Subscriber('/robot/right_arm/joint_states', JointState, joint_state_right_callback, queue_size=1)

    # Ruta del URDF
    robot_urdf_path = "/home/gonzalo/Desktop/AdamBulletSimualator/paquetes_simulacion/rb1_base_description/robots/robotDummy.urdf"
    adam = ADAM(robot_urdf_path, useSimulation=False, useRealTimeSimulation=True, used_fixed_base=True)

    adam.print_robot_info()
    adam.teleop.create_sliders()

    # Pose de destino
    pose = [[0.4211849355697632, -0.3960678040981293, 1.4412695169448853], [0, 0, 0, 1]]
    adam.utils.draw_frame(pose, axis_length=0.1, line_width=2)

    rate = rospy.Rate(60)  # 30 Hz
    
    last_joint_state_log_time = rospy.Time.now()

    real_joints_left = [0.321898, -0.28845, 1.263628, -2.468249, -1.148432, 7.416554]
    command_joints_left = [0.321898, -0.28845, 1.263628, -2.468249, -1.148432, 3.416554]
    #command_joints_left = real_joints_left


    while not rospy.is_shutdown():
        try:
            # Mover brazo simulado
            #_, closeEnough, _ = adam.arm_kinematics.move_arm_to_pose('right', pose, target_link='hand', accurate=True)

            # Obtener pose actual y publicarla
            # current_pose = adam.arm_kinematics.get_arm_link_pose('right', target_link='hand')
            #joint_msg = convert_joints_to_msg(command_joints_left)
            #joints_pub.publish(joint_msg)

            #rospy.loginfo_throttle(1.0, f"Joints msg: {joint_msg}")

            # Leer /joint_states cada 1 segundo
            now = rospy.Time.now()

            if latest_joint_state_left and latest_joint_state_right and (now - last_joint_state_log_time).to_sec() > 0.1:
                rospy.loginfo(f"JointState del robot real left: {latest_joint_state_left.position}")

                corrected_position_left = list(latest_joint_state_left.position)
                # Intercambiar elementos 0 y 2
                corrected_position_left[0], corrected_position_left[2] = corrected_position_left[2], corrected_position_left[0]
                adam.arm_kinematics.move_arm_joints_to_angles('left', corrected_position_left)


                rospy.loginfo(f"JointState del robot real right: {latest_joint_state_right.position}")

                corrected_position_right = list(latest_joint_state_right.position)
                # Intercambiar elementos 0 y 2
                corrected_position_right[0], corrected_position_right[2] = corrected_position_right[2], corrected_position_right[0]
                adam.arm_kinematics.move_arm_joints_to_angles('right', corrected_position_right)
                last_joint_state_log_time = now

                print(adam.detect_autocollisions())

            # Simulación
            if not adam.useRealTimeSimulation:
                p.stepSimulation()
                rospy.sleep(adam.t)
            else:
                p.stepSimulation()
                rate.sleep()

        except rospy.ROSInterruptException:
            break

if __name__ == '__main__':
    main()
