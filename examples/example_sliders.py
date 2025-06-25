from scripts.adam import ADAM
import os

# Load URDF robot path
base_path = os.path.dirname(__file__)
robot_urdf_path = os.path.join(base_path,"..","models","robot", "rb1_base_description", "robots", "robotDummy.urdf")


# Create ADAM robot in PyBullet
adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=True, use_ros=False)
# adam.print_robot_info()

print('READY!')

# INSERT YOUR INITIALIZATION CODE HERE

adam.teleop.create_sliders()


# Main simulation loop
while True:


    # INSERT YOUR SIMULATION CODE HERE

    # Apply slider values    
    adam.teleop.apply_slider_values()

    # Get arm poses
    current_pose_right = adam.arm_kinematics.get_arm_link_pose('right', target_link='hand')
    current_pose_left = adam.arm_kinematics.get_arm_link_pose('left', target_link='hand')
    #print("Current right hand pose:", current_pose_right)
    print("Current left hand pose:", current_pose_left)

    # Get arm joint angles
    current_angles = adam.arm_kinematics.get_arm_joint_angles('right')
    #print("Current Angles:", current_angles)

    # Get rgb and depth image
    # rgb, depth = adam.sensors.get_rgbd_image_from_link(width=640, height=480, fov=60, near=0.01, far=5.0)
    
    adam.step()
        