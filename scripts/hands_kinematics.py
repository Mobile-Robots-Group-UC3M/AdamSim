import pybullet as p
import pybullet_data 
import time

class HandsKinematics():
    """ def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True) """
    
    def __init__(self,adam):
        self.adam = adam


# Cinematica directa
    def hand_forward_kinematics(self, hand, q_joints, q_joints2=False):

        # q_joints is an array with the angle of each finger base_joint
        # q_joints = [q_thumb1, q_thumb2, q_index, q_middle, q_ring, q_pinky] (Normalized in a range [0-1000])
        # For every finger base link we have a factor X of relationship between the real range and the normalized range.
        # The normalized valor is going to be the same for all the finger joints, however, the real angle of each joint of the finger is not in the same range. 

        # The X relationship calculated for each joint: 
        thumb_MCP_joint1=0.0011; thumb_MCP_joint2=0.0005; thumb_PIP_joint=0.001; thumb_DIP_joint=0.0012
        index_MCP_joint=0.0017; index_DIP_joint=0.0016; middle_MCP_joint=0.0017; middle_DIP_joint=0.0016; ring_MCP_joint=0.0017; ring_DIP_joint=0.0016
        pink_MCP_joint=0.0017; pink_DIP_joint=0.0016


        def Calculate_real_angles(q_joints):
            # Calculate the real angle of each joints before moving them. 
            q_real_joints=[]
            # Thumb1
            q_real_joints.append(q_joints[0]*thumb_MCP_joint1)
            q_real_joints.append(q_joints[0]*thumb_MCP_joint2)
            # Thumb2
            q_real_joints.append(q_joints[1]*thumb_PIP_joint)
            q_real_joints.append(q_joints[1]*thumb_DIP_joint)
            # Index
            q_real_joints.append(q_joints[2]*index_MCP_joint)
            q_real_joints.append(q_joints[2]*index_DIP_joint)
            # Middle
            q_real_joints.append(q_joints[3]*middle_MCP_joint)
            q_real_joints.append(q_joints[3]*middle_DIP_joint)
            # Ring
            q_real_joints.append(q_joints[4]*ring_MCP_joint)
            q_real_joints.append(q_joints[4]*ring_DIP_joint)
            # Pink
            q_real_joints.append(q_joints[5]*pink_MCP_joint)
            q_real_joints.append(q_joints[5]*pink_DIP_joint)
            return q_real_joints
        
        q_real_joints = Calculate_real_angles(q_joints)
        q_real_joints2 = Calculate_real_angles(q_joints2) if q_joints2 else []



        if hand == "right":
            if q_joints2:
                raise ValueError("Solo debes introducir el valor de una mano")
            else:
                joint_indices = self.adam.right_hand_joints
                num_joints = len(joint_indices)
                vals = [0] * num_joints
                iter_steps = [q_real / 20 for q_real in q_real_joints]

                # Fase 1: Mover solo q_joints[0] (Thumb1)
                while vals[0] < q_real_joints[0]:
                    p.setJointMotorControl2(self.adam.robot_id, joint_indices[0], p.POSITION_CONTROL, vals[0])
                    vals[0] += iter_steps[0]
                    if self.adam.useSimulation and self.adam.useRealTimeSimulation == 0:
                        p.stepSimulation()
                        time.sleep(self.adam.t)

                # Fase 2: Mover el resto de las articulaciones
                while any(val < q_real for val, q_real in zip(vals[1:], q_real_joints[1:])):
                    for idx, joint_val in enumerate(joint_indices[1:], start=1):
                        if vals[idx] < q_real_joints[idx]:
                            p.setJointMotorControl2(self.adam.robot_id, joint_val, p.POSITION_CONTROL, vals[idx])
                            vals[idx] += iter_steps[idx]
                    if self.adam.useSimulation and self.adam.useRealTimeSimulation == 0:
                        p.stepSimulation()
                        time.sleep(self.adam.t)

        elif hand == "left":
            if q_joints2:
                raise ValueError("Solo debes introducir el valor de una mano")
            else:
                joint_indices = self.adam.left_hand_joints
                num_joints = len(joint_indices)
                vals = [0] * num_joints
                iter_steps = [q_real / 20 for q_real in q_real_joints]

                # Fase 1: Mover solo q_joints[0] (Thumb1)
                while vals[0] < q_real_joints[0]:
                    p.setJointMotorControl2(self.adam.robot_id, joint_indices[0], p.POSITION_CONTROL, vals[0])
                    vals[0] += iter_steps[0]
                    if self.adam.useSimulation and self.adam.useRealTimeSimulation == 0:
                        p.stepSimulation()
                        time.sleep(self.adam.t)

                # Fase 2: Mover el resto de las articulaciones
                while any(val < q_real for val, q_real in zip(vals[1:], q_real_joints[1:])):
                    for idx, joint_val in enumerate(joint_indices[1:], start=1):
                        if vals[idx] < q_real_joints[idx]:
                            p.setJointMotorControl2(self.adam.robot_id, joint_val, p.POSITION_CONTROL, vals[idx])
                            vals[idx] += iter_steps[idx]
                    if self.adam.useSimulation and self.adam.useRealTimeSimulation == 0:
                        p.stepSimulation()
                        time.sleep(self.adam.t)

        elif hand == "both":
            # Selección de articulaciones según la mano
            joint_indices_right = self.adam.right_hand_joints
            joint_indices_left = self.adam.left_hand_joints

            # Inicializar valores de cada articulación
            vals_right = [0] * len(joint_indices_right)
            iter_steps_right = [q_real / 20 for q_real in q_real_joints]
            vals_left = [0] * len(joint_indices_left)
            iter_steps_left = [q_real / 20 for q_real in q_real_joints2]

            # Fase 1: Mover solo q_joints[0] (Thumb1) en ambas manos
            while vals_right[0] < q_real_joints[0] or vals_left[0] < q_real_joints2[0]:
                if vals_right[0] < q_real_joints[0]:
                    p.setJointMotorControl2(self.adam.robot_id, joint_indices_right[0], p.POSITION_CONTROL, vals_right[0])
                    vals_right[0] += iter_steps_right[0]
                if vals_left[0] < q_real_joints2[0]:
                    p.setJointMotorControl2(self.adam.robot_id, joint_indices_left[0], p.POSITION_CONTROL, vals_left[0])
                    vals_left[0] += iter_steps_left[0]
                if self.adam.useSimulation and self.adam.useRealTimeSimulation == 0:
                    p.stepSimulation()
                    time.sleep(self.adam.t)

            # Fase 2: Mover el resto de las articulaciones en ambas manos
            while (any(val < q_real for val, q_real in zip(vals_right[1:], q_real_joints[1:])) or
                any(val < q_real for val, q_real in zip(vals_left[1:], q_real_joints2[1:]))
            ):
                for idx, joint_val in enumerate(joint_indices_right[1:], start=1):
                    if vals_right[idx] < q_real_joints[idx]:
                        p.setJointMotorControl2(self.adam.robot_id, joint_val, p.POSITION_CONTROL, vals_right[idx])
                        vals_right[idx] += iter_steps_right[idx]
                for idx, joint_val in enumerate(joint_indices_left[1:], start=1):
                    if vals_left[idx] < q_real_joints2[idx]:
                        p.setJointMotorControl2(self.adam.robot_id, joint_val, p.POSITION_CONTROL, vals_left[idx])
                        vals_left[idx] += iter_steps_left[idx]
                if self.adam.useSimulation and self.adam.useRealTimeSimulation == 0:
                    p.stepSimulation()
                    time.sleep(self.adam.t)

        else:
            raise ValueError("El brazo debe ser 'right', 'left' o 'both'.")

"""     # Cinematica directa
    def hand_forward_kinematics(self, hand, q_joints, q_joints2=False):

        # q_joints is an array with the angle of each finger base_joint
        # q_joints = [q_thumb, q_index, q_middle, q_ring, q_pinky] (Normalized in a range [0-1000])
        # For every finger base link we have a factor X of relationship between the real range and the normalized range.
        # The normalized valor is going to be the same for all the finger joints, however, the real angle of each joint of the finger is not in the same range. 

        # The X relationship calculated for each joint: 
        thumb_MCP_joint1=0.0011; thumb_MCP_joint2=0.0005; thumb_PIP_joint=0.001; thumb_DIP_joint=0.0012
        index_MCP_joint=0.0017; index_DIP_joint=0.0016; middle_MCP_joint=0.0017; middle_DIP_joint=0.0016; ring_MCP_joint=0.0017; ring_DIP_joint=0.0016
        pink_MCP_joint=0.0017; pink_DIP_joint=0.0016


        def Calculate_real_angles(q_joints):
            # Calculate the real angle of each joints before moving them. 
            q_real_joints=[]
            # Thumb
            q_real_joints.append(q_joints[0]*thumb_MCP_joint1)
            q_real_joints.append(q_joints[0]*thumb_MCP_joint2)
            q_real_joints.append(q_joints[0]*thumb_PIP_joint)
            q_real_joints.append(q_joints[0]*thumb_DIP_joint)
            # Index
            q_real_joints.append(q_joints[1]*index_MCP_joint)
            q_real_joints.append(q_joints[1]*index_DIP_joint)
            # Middle
            q_real_joints.append(q_joints[2]*middle_MCP_joint)
            q_real_joints.append(q_joints[2]*middle_DIP_joint)
            # Ring
            q_real_joints.append(q_joints[3]*ring_MCP_joint)
            q_real_joints.append(q_joints[3]*ring_DIP_joint)
            # Pink
            q_real_joints.append(q_joints[4]*pink_MCP_joint)
            q_real_joints.append(q_joints[4]*pink_DIP_joint)
            return q_real_joints
        
        q_real_joints = Calculate_real_angles(q_joints)
        q_real_joints2 = Calculate_real_angles(q_joints2) if q_joints2 else []



        if hand == "right":            
            if q_joints2:
                raise ValueError("Solo debes introducir el valor de una mano")

            else:
                joint_indices = self.adam.right_hand_joints
                num_joints = len(joint_indices)
                vals = [0] * num_joints
                iter_steps = [q_real / 20 for q_real in q_real_joints] 

                # Realizar movimiento simultáneo
                while any(val < q_real for val, q_real in zip(vals, q_real_joints)):
                    for idx, joint_val in enumerate(joint_indices):
                        if vals[idx] < q_real_joints[idx]:  # Solo mover si no ha alcanzado el valor objetivo
                            p.setJointMotorControl2(self.adam.robot_id, joint_val, p.POSITION_CONTROL, vals[idx])
                            vals[idx] += iter_steps[idx]  # Incrementar el valor de la articulación




        elif hand == "left":
            if q_joints2:
                raise ValueError("Solo debes introducir el valor de una mano")


            else:
                joint_indices = self.adam.left_hand_joints
                num_joints = len(joint_indices)
                vals = [0] * num_joints
                iter_steps = [q_real / 20 for q_real in q_real_joints] 

                # Realizar movimiento simultáneo
                while any(val < q_real for val, q_real in zip(vals, q_real_joints)):
                    for idx, joint_val in enumerate(joint_indices):
                        if vals[idx] < q_real_joints[idx]:  # Solo mover si no ha alcanzado el valor objetivo
                            p.setJointMotorControl2(self.adam.robot_id, joint_val, p.POSITION_CONTROL, vals[idx])
                            vals[idx] += iter_steps[idx]  # Incrementar el valor de la articulación



        elif hand == "both":
        # Selección de articulaciones según la mano
            joint_indices_right = self.adam.right_hand_joints
            joint_indices_left = self.adam.left_hand_joints

            # Inicializar valores de cada articulación
            vals_right = [0] * len(joint_indices_right)
            iter_steps_right = [q_real / 20 for q_real in q_real_joints] 
            vals_left = [0] * len(joint_indices_left)
            iter_steps_left = [q_real / 20 for q_real in q_real_joints2]

            # Movimiento simultáneo de ambas manos
            while (
                any(val < q_real for val, q_real in zip(vals_right, q_real_joints)) or 
                any(val < q_real for val, q_real in zip(vals_left, q_real_joints2))
            ):
                # Mover articulaciones de la mano derecha
                for idx, joint_val in enumerate(joint_indices_right):
                    if vals_right[idx] < q_real_joints[idx]:
                        p.setJointMotorControl2(self.adam.robot_id, joint_val, p.POSITION_CONTROL, vals_right[idx])
                        vals_right[idx] += iter_steps_right[idx]

                # Mover articulaciones de la mano izquierda
                for idx, joint_val in enumerate(joint_indices_left):
                    if vals_left[idx] < q_real_joints2[idx]:
                        p.setJointMotorControl2(self.adam.robot_id, joint_val, p.POSITION_CONTROL, vals_left[idx])
                        vals_left[idx] += iter_steps_left[idx]

                # Actualizar la simulación en cada paso
                if self.adam.useSimulation and self.adam.useRealTimeSimulation == 0:
                    p.stepSimulation()
                    time.sleep(self.adam.t)


        else:
            raise ValueError("El brazo debe ser 'right', 'left' o 'both'.") """




    # def get_hand_ee_pose(self, hand, finger):

    #     if hand == "right":
    #         end_effector_index = 
    #     elif hand == "left":
    #         end_effector_index = 
    #     else:
    #         raise ValueError("La mano debe ser 'left' o 'right'")
        
    #     link_state = p.getLinkState(self.robot_id, end_effector_index)
    #     return link_state[4], link_state[5]



        



        
