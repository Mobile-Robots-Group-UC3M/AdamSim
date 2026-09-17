from scripts.adam import ADAM

# Create ADAM instance
# adam = ADAM(end_effector="grippers", use_realtime=True, use_fixed_base=False, use_ros=False)
adam = ADAM(end_effector="inspire_hands", use_realtime=True, use_fixed_base=False, use_ros=False) # With inspire hands


while True:
    # adam.sensors.get_rgbd_image_from_link()
    adam.teleop.teleoperate_base(debug=True)

    adam.step()