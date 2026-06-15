import pybullet as p
import numpy as np
import math

# Class for planner
class Planner:
    def __init__(self, adam):
        self.adam = adam

    ## DWA planner
    def simulate_trajectory(self, x, y, theta, v, w, dt=0.1, t_total=1.0):
        """
        Simulates the robot's motion using unicycle model.
        Returns a list of (x, y) points along the simulated path.
        """
        traj = []
        t = 0.0
        while t < t_total:
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt
            traj.append((x, y))
            t += dt
        return traj
    
    def compute_obstacle_cost(self, traj, obstacles, robot_radius=0.2):
        """
        Returns a soft cost based on proximity to obstacles. Collisions get infinite cost.
        """
        min_dist = float('inf')
        total_penalty = 0.0

        for x, y in traj:
            for ox, oy in obstacles:
                dist = math.hypot(ox - x, oy - y)

                if dist < robot_radius:
                    return float('inf')  # hard collision
                elif dist < robot_radius + 0.5:  # within "influence radius"
                    # Soft exponential penalty
                    penalty = math.exp(-dist + robot_radius)
                    total_penalty += penalty

                min_dist = min(min_dist, dist)

        return total_penalty

    
    def compute_goal_cost(self, traj, goal):
        """
        Returns the distance from the end of the trajectory to the goal.
        """
        x_end, y_end = traj[-1]
        return math.hypot(goal[0] - x_end, goal[1] - y_end)
    
    def dwa_step(self, goal, obstacles, max_v=1.0, max_w=1.5, v_samples=5, w_samples=5):
        """
        Selects the best (v, w) pair using DWA logic and returns it.
        """
        # Get current robot pose
        pos, orn = p.getBasePositionAndOrientation(self.adam.robot_id)
        x, y = pos[0], pos[1]
        _, _, theta = p.getEulerFromQuaternion(orn)

        best_score = float('inf')
        best_v = 0.0
        best_w = 0.0

        for v in np.linspace(0, max_v, v_samples):
            for w in np.linspace(-max_w, max_w, w_samples):
                traj = self.simulate_trajectory(x, y, theta, v, w)
                obs_cost = self.compute_obstacle_cost(traj, obstacles)
                goal_cost = self.compute_goal_cost(traj, goal)
                total_cost = goal_cost + 5.0 * obs_cost  # weights

                if total_cost < best_score:
                    best_score = total_cost
                    best_v = v
                    best_w = w

        return best_v, best_w
