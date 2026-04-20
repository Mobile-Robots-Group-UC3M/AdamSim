import sys
import os
import time
import pybullet as p
import pybullet_data
import math
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from scripts.adam import ADAM


if __name__ == '__main__':
    base_path = os.path.dirname(__file__)
    robot_urdf_path = os.path.join(
        base_path, "..", "models", "robot", "rb1_base_description", "robots", "robotDummy.urdf"
    )

    adam = ADAM(robot_urdf_path, useRealTimeSimulation=True, used_fixed_base=False, use_ros=False)
    adam.wait(0.1)
    adam.sensors.start_lidar()
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    dynamic_obstacles = []

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

    dynamic_obstacles.append(add_dynamic_obstacle(4.0, -2.0, 2.0, color=[0, 1, 0, 1], speed=1.0))
    dynamic_obstacles.append(add_dynamic_obstacle(5.0, -4.0, -1.0, color=[1, 0, 0, 1], speed=0.8))
    dynamic_obstacles.append(add_dynamic_obstacle(6.0, 0.0, 1.0, color=[0, 0, 1, 1], speed=0.5))
    dynamic_obstacles.append(add_dynamic_obstacle(7.0, 2.0, -2.0, color=[1, 0.5, 0, 1], speed=1.0))
    dynamic_obstacles.append(add_dynamic_obstacle(4.5, -1.0, 4.5, color=[0.5, 0, 0.5, 1], speed=0.6))

    goal = (8, -3)

    while True:
        obstacles = adam.sensors.get_lidar_points_world()

        v, w = adam.planner.dwa_step(goal, obstacles)

        adam.navigation.send_velocity(v, w)

        pos, _ = p.getBasePositionAndOrientation(adam.robot_id)
        dist = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
        if dist < 0.2:
            print("Goal reached.")
            break

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
        time.sleep(0.05)

    adam.navigation.send_velocity(0, 0)
