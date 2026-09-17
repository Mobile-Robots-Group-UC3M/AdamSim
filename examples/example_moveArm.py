import os
from scripts.adam import ADAM
import pybullet as p
import pybullet_data

# Create ADAM instance
adam = ADAM(end_effector="inspire_hands", use_realtime=True, use_fixed_base=True, use_ros=False)
# adam = ADAM(end_effector="grippers", use_realtime=True, use_fixed_base=True, use_ros=False) # With inspire hands

# Poses
pose0 = [[0.1747363805770874, -0.5190918445587158, 1.5269604921340942], [0.13088582456111908, 0.13149239122867584, -0.6327640414237976, 0.7517901062965393]]
pose1 = [[0.5294547080993652, -0.5355535745620728, 1.1407946348190308], [0.09417623281478882, 0.7259091734886169, -0.05370301008224487, 0.6791926622390747]]
pose2 = [[0.5100053548812866, -0.06656599044799805, 1.1167795658111572], [-0.44266143441200256, 0.6868288516998291, -0.11991514265537262, 0.5638593435287476]]

adam.utils.draw_frame(pose0, axis_length=0.1, line_width=4)
adam.utils.draw_frame(pose1, axis_length=0.1, line_width=4)
adam.utils.draw_frame(pose2, axis_length=0.1, line_width=4)


# Auxiliary functions

def background_tasks():
    """This contains everything that should always run (sensors, closed hands...)"""
    adam.hand_kinematics.move_hand_to_dofs('right', [1000, 1000, 1000, 1000, 1000, 1000])
    adam.hand_kinematics.move_hand_to_dofs('left', [1000, 1000, 1000, 1000, 1000, 1000])
    rgb, depth = adam.sensors.get_rgbd_image_from_link(width=640, height=480, fov=60, near=0.01, far=5.0)

def go_to_pose(target_pose):
    """Loop that moves the arm until it reaches the target while updating the simulation without freezing it"""
    reached = False
    while not reached:
        # 1. Move the arm one step
        _, reached, _ = adam.arm_kinematics.move_arm_to_pose_continuous(
            arm='right', target_pose=target_pose, target_link='dummy', accurate=True
        )
        # 2. Run background tasks (hands, sensors)
        background_tasks()
        # 3. Advance simulation
        adam.step()

def maintain_pose_callback(target_pose):
    """Callback so that during wait() the arm maintains force in place"""
    adam.arm_kinematics.move_arm_to_pose_continuous(
        arm='right', target_pose=target_pose, target_link='dummy', accurate=False
    )
    background_tasks()


while True:
    rgb, depth = adam.sensors.get_rgbd_image_from_link(width=640, height=480, fov=60, near=0.01, far=5.0)
    print("Moviendo a pose 0...")
    go_to_pose(pose0)
    print("Esperando 3 segundos...")
    adam.wait(0.2, callback_func=lambda: maintain_pose_callback(pose0))

    print("Moviendo a pose 1...")
    go_to_pose(pose1)
    print("Esperando 3 segundos...")
    adam.wait(0.2, callback_func=lambda: maintain_pose_callback(pose1))

    print("Moviendo a pose 2...")
    go_to_pose(pose2)
    print("Esperando 3 segundos...")
    adam.wait(0.2, callback_func=lambda: maintain_pose_callback(pose2))

    # Retrieve current pose and joint angles
    current_pose = adam.arm_kinematics.get_arm_link_pose('right', target_link='hand')
    print("Current Pose:", current_pose)

    current_angles = adam.arm_kinematics.get_arm_joint_angles('right')
    # print("Current Angles:", current_angles)