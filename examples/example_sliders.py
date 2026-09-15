from scripts.adam import ADAM

# Create ADAM instance
# adam = ADAM(end_effector="grippers", use_realtime=True, use_fixed_base=True, use_ros=False)
adam = ADAM(end_effector="inspire_hands", use_realtime=True, use_fixed_base=True, use_ros=False) # With inspire hands

adam.teleop.create_sliders()

# Main simulation loop
while True:

    # Apply slider values    
    adam.teleop.apply_slider_values()

    # Get arm poses
    current_pose_right = adam.arm_kinematics.get_arm_link_pose('right', target_link='hand')
    current_pose_left = adam.arm_kinematics.get_arm_link_pose('left', target_link='hand')

    print("---")
    print("Current right hand pose:", current_pose_right)
    print("Current left hand pose:", current_pose_left)
    
    adam.step()
        