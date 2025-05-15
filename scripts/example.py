from adam import ADAM
import pybullet as p
import time

# URDF robot path
robot_urdf_path = "/home/gonzalo/Desktop/AdamBulletSimualator/paquetes_simulacion/rb1_base_description/robots/robotDummy.urdf"

# Create ADAM instanceSliders
adam = ADAM(robot_urdf_path, useSimulation=False, useRealTimeSimulation=True, used_fixed_base=True)

# Print robot information
adam.print_robot_info()


# INSERT YOUR INITIALIZATION CODE HERE

adam.teleop.create_sliders()




right_initial_pose = [2.7354, -1.5973, -1.1237, -1.2453, 0.4834, -0.1599]
left_initial_pose = [-2.0935, -2.0019, -0.8266, -2.6136, 4.4560, 5.1101]

#adam.arm_kinematics.move_arm_joints_to_angles('right', right_initial_pose)
#adam.arm_kinematics.move_arm_joints_to_angles('left', left_initial_pose)

'''for i in range(500):
    p.stepSimulation()
    time.sleep(adam.t)'''

'''
Current Pose: ((0.3811849355697632, -0.2960678040981293, 1.4412695169448853), (0.06631585210561752, -0.8540112376213074, 0.014523339457809925, 0.5158063769340515))
Current Angles: [-1.3216045350115093, 1.0978111577306686, -1.438960327420289, -0.7712279280331018, -0.6670987730930772, 3.077457820186016]


'''

pose = [[0.4211849355697632, -0.3960678040981293, 1.4412695169448853], [0, 0, 0, 1]]

adam.utils.draw_frame(pose, axis_length=0.1, line_width=2)

# Main simulation loop
while True:


    # INSERT YOUR SIMULATION CODE HERE

    # Example: Move the right arm to a specific position
    
    #adam.teleop.apply_slider_values()
    #Test
    _, closeEnough, _ = adam.arm_kinematics.move_arm_to_pose('right', pose, target_link='hand', accurate=True)
    print("Close enough:", closeEnough)

    current_pose = adam.arm_kinematics.get_arm_link_pose('right', target_link='hand')
    print("Current Pose:", current_pose)

    current_angles = adam.arm_kinematics.get_arm_joint_angles('right')
    print("Current Angles:", current_angles)    
    

    if not adam.useRealTimeSimulation:
        p.stepSimulation()
        time.sleep(adam.t)
        