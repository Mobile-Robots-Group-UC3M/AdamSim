import time
import math
import os
from scripts.adam import ADAM
import pybullet as p
import pybullet_data


# Auxiliary function to create a dynamic obstacle
def add_dynamic_obstacle(x, y_start, y_end, z=0.17, color=[0, 1, 0, 1], speed=1.5):
    col_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.275, 0.05, 0.17])
    vis_shape = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[0.275, 0.05, 0.17], rgbaColor=color
    )
    obstacle_id = p.createMultiBody(
        baseMass=1.0,
        baseCollisionShapeIndex=col_shape,
        baseVisualShapeIndex=vis_shape,
        basePosition=[x, y_start, z],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    )
    return {
        "id": obstacle_id,
        "x": x,
        "y_min": min(y_start, y_end),
        "y_max": max(y_start, y_end),
        "z": z,
        "dir": 1,
        "speed": speed,
    }



# Create ADAM instance
adam = ADAM(end_effector="grippers", use_realtime=True, use_fixed_base=False, use_ros=False)
# adam = ADAM(end_effector="inspire_hands", use_realtime=True, use_fixed_base=False, use_ros=False) # With inspire hands

adam.wait(0.1)
adam.sensors.start_lidar()


dynamic_obstacles = []
dynamic_obstacles.append(add_dynamic_obstacle(4.0, -2.0, 2.0, color=[0, 1, 0, 1], speed=1.0))
dynamic_obstacles.append(add_dynamic_obstacle(5.0, -4.0, -1.0, color=[1, 0, 0, 1], speed=0.8))
dynamic_obstacles.append(add_dynamic_obstacle(6.0, 0.0, 1.0, color=[0, 0, 1, 1], speed=0.5))
dynamic_obstacles.append(add_dynamic_obstacle(7.0, 2.0, -2.0, color=[1, 0.5, 0, 1], speed=1.0))
dynamic_obstacles.append(add_dynamic_obstacle(4.5, -1.0, 4.5, color=[0.5, 0, 0.5, 1], speed=0.6))

goal = (8, -3)


while True:

    # Get LiDAR points in world coordinates
    obstacles = adam.sensors.get_lidar_points_world()

    # Compute the next velocity command using DWA
    v, w = adam.planner.dwa_step(goal, obstacles)

    # Send the velocity command to ADAM
    adam.navigation.send_velocity(v, w)

    # Check if the goal is reached
    pos, _ = p.getBasePositionAndOrientation(adam.robot_id)
    dist = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
    if dist < 0.2:
        print("Goal reached.")
        break

    # Update the position of each dynamic obstacle
    for obs in dynamic_obstacles:
        obs_id = obs["id"]
        pos, orn = p.getBasePositionAndOrientation(obs_id)
        y = pos[1]
        new_y = y + obs["dir"] * obs["speed"] * 0.05

        if new_y > obs["y_max"]:
            new_y = obs["y_max"]
            obs["dir"] = -1
        elif new_y < obs["y_min"]:
            new_y = obs["y_min"]
            obs["dir"] = 1

        p.resetBasePositionAndOrientation(obs_id, [obs["x"], new_y, obs["z"]], orn)

    adam.step()

adam.navigation.send_velocity(0, 0)
