import os
from scripts.adam import ADAM
import pybullet as p
import pybullet_data
import os
import math

# Create ADAM instance
adam = ADAM(end_effector="inspire_hands", use_realtime=True, use_fixed_base=False, use_ros=False)
# adam = ADAM(end_effector="grippers", use_realtime=True, use_fixed_base=True, use_ros=False) # With inspire hands

adam.wait(0.1)

initial_right_pose = [2.7354,-1.5973,-1.1237,-1.2453,0.4834,-0.1599]
initial_left_pose = [-2.0935,-2.0019,-0.8266,-2.6136,4.4560,5.1101]

adam.arm_kinematics.move_arm_joints_to_angles(arm='right', angles=initial_right_pose)
adam.arm_kinematics.move_arm_joints_to_angles(arm='left', angles=initial_left_pose)

# Add obstacle: a box in front of the robot
box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.275, 0.05, 0.17])
box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.275, 0.05, 0.17], rgbaColor=[1, 1, 0, 1])
box_id = p.createMultiBody(baseMass=0.1,
                        baseCollisionShapeIndex=box_collision,
                        baseVisualShapeIndex=box_visual,
                        basePosition=[4.4, -1.4, 0.17],
                        baseOrientation=p.getQuaternionFromEuler([0,0,math.pi/2]))  # In front of the robot

# Add obstacle: a box in front of the robot
box_collision2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.13, 0.25])
box_visual2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.25, 0.13, 0.25], rgbaColor=[0.7, 0, 45, 1])
box_id2 = p.createMultiBody(baseMass=0.1,
                        baseCollisionShapeIndex=box_collision2,
                        baseVisualShapeIndex=box_visual2,
                        basePosition=[3, 0.7, 0.25],
                        baseOrientation=p.getQuaternionFromEuler([0,0,math.pi/2]))  # In front of the robot

step=True
con = 0
points =[(2,-1.5,0),(3.5,-0.6,0.5),(5,0,0)]

adam.utils.draw_frame(([points[0][0],points[0][1],0],p.getQuaternionFromEuler([0,0,points[0][2]])),axis_length=0.5,line_width=6)
adam.utils.draw_frame(([points[1][0],points[1][1],0],p.getQuaternionFromEuler([0,0,points[1][2]])),axis_length=0.5,line_width=6)
adam.utils.draw_frame(([points[2][0],points[2][1],0],p.getQuaternionFromEuler([0,0,points[2][2]])),axis_length=0.5,line_width=6)

adam.wait(5)

while step:
    # 1. Read the camera (now in real time!)
    rgb, depth = adam.sensors.get_rgbd_image_from_link(width=640, height=480, fov=60, near=0.01, far=5.0)
    
    # 2. Calculate control and move the robot for one step
    is_moving = adam.navigation.move_base_continuous(
        points[con],
        pos_tolerance=0.1,
        angle_tolerance=0.5,
        orient_tolerance=0.5,
        use_lidar=False,
        use_ros=False,
        debug=False # Set it to False to avoid flooding the terminal on each iteration
    )
    
    # 3. Logic for switching to the next point
    if is_moving == False:
        con = con + 1
        adam.wait(5.0, callback_func=lambda: adam.sensors.get_rgbd_image_from_link(width=640, height=480, fov=60, near=0.01, far=5.0))
        if con == len(points): # Better to use len() in case you add more points
            step = False
            
    # 4. Advance the simulation by one step
    adam.step()
