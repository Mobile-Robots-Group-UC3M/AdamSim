import pybullet as p
import time
import math


#Class for dynamics
class Navigation():
    def __init__(self,adam):
        self.adam = adam
        
        self.start_pos = [0, 0, 0] # x,y,theta
        self.wheel_radius=0.0762
        self.wheel_distance=0.436
        self.linear_speed=6.55
        self.angular_speed=1.0
        
        self.K_lin = 1.0
        self.K_ang = 3.0
        
    def shortest_angle_diff(self, a, b):
        return math.atan2(math.sin(b - a), math.cos(b - a))
        
    def move_wheels(self, left_wheel_speed, right_wheel_speed,force = 100):
        '''
        Generates a movement of the robot by setting the wheel speeds.
        Args:
            left_wheel_speed (float): Speed of the left wheel in m/s.
            right_wheel_speed (float): Speed of the right wheel in m/s.
            force (int): Force applied to the wheels.
        '''
        p.setJointMotorControl2(self.adam.robot_id, self.adam.left_wheel_joint,
                                p.VELOCITY_CONTROL, targetVelocity=left_wheel_speed, force=force)
        p.setJointMotorControl2(self.adam.robot_id, self.adam.right_wheel_joint,
                                p.VELOCITY_CONTROL, targetVelocity=right_wheel_speed, force=force)
    

        
    def move_base_continuous(self, target_pos, pos_tolerance=0.01, angle_tolerance=0.02, orient_tolerance=0.1, debug=True):
        '''
        Generates a movement of the robot towards a target position and orientation continuously.
        Args:
            target_pos (tuple): Target position and orientation as (x, y, theta).
            pos_tolerance (float): Position tolerance to stop the robot.
            angle_tolerance (float): Orientation tolerance to stop the robot.
            orient_tolerance (float): Orientation tolerance to start moving towards the target.
            debug (bool): If True, prints debug information.
        Returns:
            bool: True if the movement was successful, False otherwise.
        '''
        x_goal, y_goal, theta_goal = target_pos
        # PID Gains
        K_lin = self.K_lin
        K_ang = self.K_ang
        step = True

        self.adam.utils.draw_frame(([x_goal,y_goal,0],p.getQuaternionFromEuler([0,0,theta_goal])),axis_length=0.5,line_width=6)
        
        while step:

            self.adam.sensors.simulated_lidar() # Lidar during navigation

            # 1) Estado actual
            pos, orn = p.getBasePositionAndOrientation(self.adam.robot_id)
            x, y = pos[0], pos[1]
            _, _, theta = p.getEulerFromQuaternion(orn)

            # 2) Errors
            dx = x_goal - x
            dy = y_goal - y
            dist = math.hypot(dx, dy)
            path_angle = math.atan2(dy, dx)
            alpha = self.shortest_angle_diff(theta, path_angle)
            dtheta_final = self.shortest_angle_diff(theta, theta_goal)

            # Debug
            if debug:
                print(f"step={step}, dist={dist:.3f}, α={alpha:.3f}, Δθ_final={dtheta_final:.3f}")

            # 4) Control logic
            if dist > pos_tolerance:
                # far from target position:
                if abs(alpha) >= orient_tolerance:
                    # 4.1) Turn in place
                    v = 0.0
                    w = K_ang * alpha
                else:
                    # 4.2) Go ahead and fix orientation
                    if dist <= 2.0:
                        v = K_lin *dist
                    else:
                        v = K_lin *2
                    w = K_ang * alpha
                    
            else:
                # 4.3) near target (dist <= pos_tolerance): just orientation
                v = 0.0
                w = K_ang * dtheta_final

            v = max(-self.linear_speed, min(self.linear_speed, v))
            w = max(-self.angular_speed, min(self.angular_speed, w))

            v_l = (2 * v - w * self.wheel_distance) / (2 * self.wheel_radius)
            v_r = (2 * v + w * self.wheel_distance) / (2 * self.wheel_radius)
            
            p.setJointMotorControl2(self.adam.robot_id, self.adam.left_wheel_joint,
                                    p.VELOCITY_CONTROL, targetVelocity=v_l, force=100)
            p.setJointMotorControl2(self.adam.robot_id, self.adam.right_wheel_joint,
                                    p.VELOCITY_CONTROL, targetVelocity=v_r, force=100)
            
            p.stepSimulation()
            time.sleep(self.adam.t)
            step += 1
            
            #Check if reached final position
            if dist < pos_tolerance and abs(dtheta_final) < angle_tolerance:
                print("Final position reached")
                print("Final position and orientation:", pos, theta)
                print("Target position and orientation:", target_pos)
                step = False
        # Stop the robot
        p.setJointMotorControl2(self.adam.robot_id, self.adam.left_wheel_joint,
                                p.VELOCITY_CONTROL, targetVelocity=0, force=10)
        p.setJointMotorControl2(self.adam.robot_id, self.adam.right_wheel_joint,
                                p.VELOCITY_CONTROL, targetVelocity=0, force=10)
        
        return True

