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
        self.linear_speed=8
        self.angular_speed=1.0
        
        self.K_lin = 1.0
        self.K_ang = 3.0
        
    def shortest_angle_diff(self, a, b):
        return math.atan2(math.sin(b - a), math.cos(b - a))
        
    def move_wheels(self, left_wheel_speed, right_wheel_speed,force = 100):
        # Set the wheel speeds
        p.setJointMotorControl2(self.adam.robot_id, self.adam.left_wheel_joint,
                                p.VELOCITY_CONTROL, targetVelocity=left_wheel_speed, force=force)
        p.setJointMotorControl2(self.adam.robot_id, self.adam.right_wheel_joint,
                                p.VELOCITY_CONTROL, targetVelocity=right_wheel_speed, force=force)
    

        
    def move_base_continuous(self, target_pos, pos_tolerance=0.01, angle_tolerance=0.02, orient_tolerance=0.1, debug=True):
        x_goal, y_goal, theta_goal = target_pos
        # Ganancias
        K_lin = self.K_lin
        K_ang = self.K_ang
        step = True

        self.adam.utils.draw_frame(([x_goal,y_goal,0],p.getQuaternionFromEuler([0,0,theta_goal])),axis_length=0.5,line_width=6)
        
        while step:
            # 1) Estado actual
            pos, orn = p.getBasePositionAndOrientation(self.adam.robot_id)
            x, y = pos[0], pos[1]
            _, _, theta = p.getEulerFromQuaternion(orn)

            # 2) Errores
            dx = x_goal - x
            dy = y_goal - y
            dist = math.hypot(dx, dy)
            path_angle = math.atan2(dy, dx)
            alpha = self.shortest_angle_diff(theta, path_angle)
            dtheta_final = self.shortest_angle_diff(theta, theta_goal)

            # Debug
            if debug:
                print(f"step={step}, dist={dist:.3f}, α={alpha:.3f}, Δθ_final={dtheta_final:.3f}")

            # 4) Ley de control
            if dist > pos_tolerance:
                # Mientras estamos lejos del objetivo:
                if abs(alpha) >= orient_tolerance:
                    # 4.1) GIRAR IN SITU hacia la dirección al target
                    v = 0.0
                    w = K_ang * alpha
                else:
                    # 4.2) AVANZAR y corregir heading fino
                    print("UPSSS ME HE PASADO")
                    if dist <= 2.0:
                        v = K_lin *dist
                    else:
                        v = K_lin *2
                    w = K_ang * alpha
                    
            else:
                # 4.3) Estamos cerca (dist <= pos_tolerance): sólo orientación final
                v = 0.0
                w = K_ang * dtheta_final

            # 5) Limitación de velocidades
            v = max(-self.linear_speed, min(self.linear_speed, v))
            w = max(-self.angular_speed, min(self.angular_speed, w))

            # 6) Convertir a velocidades de rueda
            v_l = (2 * v - w * self.wheel_distance) / (2 * self.wheel_radius)
            v_r = (2 * v + w * self.wheel_distance) / (2 * self.wheel_radius)
            

            # 7) Enviar comandos
            p.setJointMotorControl2(self.adam.robot_id, self.adam.left_wheel_joint,
                                    p.VELOCITY_CONTROL, targetVelocity=v_l, force=100)
            p.setJointMotorControl2(self.adam.robot_id, self.adam.right_wheel_joint,
                                    p.VELOCITY_CONTROL, targetVelocity=v_r, force=100)
            
            # 8) Simular un paso
            p.stepSimulation()
            time.sleep(self.adam.t)
            step += 1
            
            # --- 3) Chequeo de parada estricta ---
            if dist < pos_tolerance and abs(dtheta_final) < angle_tolerance:
                print("Final position reached")
                print("Final position and orientation:", pos, theta)
                print("Target position and orientation:", target_pos)
                step = False

        # 9) Parar el robot
        p.setJointMotorControl2(self.adam.robot_id, self.adam.left_wheel_joint,
                                p.VELOCITY_CONTROL, targetVelocity=0, force=10)
        p.setJointMotorControl2(self.adam.robot_id, self.adam.right_wheel_joint,
                                p.VELOCITY_CONTROL, targetVelocity=0, force=10)
        
        return True

