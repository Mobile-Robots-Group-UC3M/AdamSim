import os
from scripts.adam import ADAM
import pybullet as p
import pybullet_data

# Create ADAM instance
adam = ADAM(end_effector="grippers", use_realtime=True, use_fixed_base=False, use_ros=False, use_plane=False)
# adam = ADAM(end_effector="inspire_hands", use_realtime=True, use_fixed_base=False, use_ros=False, use_plane=False) # With inspire hands

adam.environments.generate_home(
    seed=2, 
    max_home_regens=300, 
    room_config={
        "kitchen"   :1,
        "bathrooms" :1,
        "bedroom1"  :1,
        "bedroom2"  :1,
        "dining"    :1
    },
    floor_color=None, 
    wall_color=None, 
    show_info=True
)

adam.environments.create_movable_objects()

adam.sensors.start_lidar(num_rays=40)


while True:

    # Teleoperate base
    adam.teleop.teleoperate_base()

    # Get RGB and depth image from camera
    rgb, _ = adam.sensors.get_rgbd_image_from_link(width=640, height=480, fov=60, near=0.01, far=5.0)

    # Get LiDAR points in world coordinates
    obstacles = adam.sensors.get_lidar_points_world()

    adam.step()



