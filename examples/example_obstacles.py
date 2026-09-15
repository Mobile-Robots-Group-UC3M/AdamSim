import os
from scripts.adam import ADAM
import pybullet as p
import pybullet_data
import math


# Create person dynamic obstacle
def add_dynamic_person(path_points, z=0.0, speed=0.4):
    base_path = os.path.dirname(os.path.abspath(__file__))
    person_urdf_path = os.path.join(base_path, "..", "models", "objects", "person", "person.urdf")
    
    x0, y0 = path_points[0]

    person_id = p.loadURDF(person_urdf_path, basePosition=[x0, y0, z], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=False)
    
    return {
        "id": person_id,
        "path": path_points,
        "target_index": 1,
        "z": z,
        "speed": speed,
    }

# Move person along its path
def move_person(obs, dt):
    obs_id = obs["id"]
    pos, _ = p.getBasePositionAndOrientation(obs_id)

    x, y, _ = pos
    target = obs["path"][obs["target_index"]]
    tx, ty  = target

    dx = tx - x
    dy = ty - y
    dist = math.hypot(dx, dy)

    if dist < 0.05:
        obs["target_index"] = (obs["target_index"] + 1) % len(obs["path"])
        return
    vx = (dx / dist) 
    vy = (dy / dist) 
    new_x = x + vx * obs["speed"] * dt
    new_y = y + vy * obs["speed"] * dt

    yaw = math.atan2(vy, vx)
    orn = p.getQuaternionFromEuler([0, 0, yaw])
    p.resetBasePositionAndOrientation(obs_id, [new_x, new_y, obs["z"]], orn)


# Create ADAM instance
adam = ADAM(end_effector="inspire_hands", use_realtime=True, use_fixed_base=False, use_ros=False, use_plane=False)
# adam = ADAM(end_effector="grippers", use_realtime=True, use_fixed_base=False, use_ros=False, use_plane=False) # With inspire hands

# Generate home environment
adam.environments.generate_home(
    seed=1, 
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

# Wait for the environment to load
adam.wait(5)

# Start LiDAR and configure visualizer
adam.sensors.start_lidar(40)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# Move arms to initial positions
adam.arm_kinematics.move_arm_joints_to_angles("right", [-3.307, -0.794, 0.00, -1.521, 0.00, -1.257]) #-2.712, -0.661, 0, -1.521, -0.661, -1.521   -2.712, -0.5, 0.0, -1.1, -0.661, -1.521
adam.arm_kinematics.move_arm_joints_to_angles("left", [-3.175, -0.794, 0.00, -1.389, 3.307, -4.431]) #-0.066, -2.116, -0.265, -1.587, 0.132, 1.852
adam.hand_kinematics.move_hand_to_dofs('right', [1000, 1000, 1000, 1000, 1000, 0])

# Wait for arms to move
adam.wait(5)


# Create dynamic obstacle
dynamic_obstacles = []
person_path_points = [[5.0, 7],[1.7, 7]]
dynamic_obstacles.append(add_dynamic_person(path_points=person_path_points, z=0.0, speed=0.1))

# Define goal position
goal = (4.0, 8.2, 0.0)
adam.utils.draw_frame([[goal[0], goal[1], 0.1], p.getQuaternionFromEuler([0, 0, goal[2]])])

dt = 0.05

while True:

    # Get LiDAR points in world coordinates
    obstacles = adam.sensors.get_lidar_points_world()

    # Get velocity commands from DWA planner
    v, w = adam.planner.dwa_step(goal, obstacles)

    # Send velocity commands to the robot
    adam.navigation.send_velocity(v, w)

    # Check if the robot has reached the goal
    robot_pos, _ = p.getBasePositionAndOrientation(adam.robot_id)
    dist = math.hypot(goal[0] - robot_pos[0], goal[1] - robot_pos[1])
    
    if dist < 0.2:
        print("Goal reached.")
        #adam.navigation.send_velocity(0, 0)
        break

    for obs in dynamic_obstacles:
        move_person(obs, dt)

    adam.step()

adam.navigation.send_velocity(0, 0)