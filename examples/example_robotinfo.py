from scripts.adam import ADAM

# Create ADAM instance
adam = ADAM(end_effector="grippers", use_realtime=True, use_fixed_base=True, use_ros=False)
# adam = ADAM(end_effector="inspire_hands", use_realtime=True, use_fixed_base=True, use_ros=False) # With inspire hands

adam.print_robot_info()

while True:
    adam.step()


