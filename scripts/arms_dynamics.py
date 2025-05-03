import pybullet as p
import time


#Class for dynamics
class ArmsDynamics():

    """ def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True) """
    def __init__(self,adam):
        self.adam = adam

    
    def get_arm_joints_pos_vel(self, arm):

        joint_positions = []
        joint_velocities = []

        # Obtener los índices del brazo seleccionado
        if arm == "left":
            joint_indices = self.adam.ur3_left_arm_joints
        elif arm == "right":
            joint_indices = self.adam.ur3_right_arm_joints
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")

        # Leer las posiciones articulares
        for joint_id in joint_indices:
            joint_state = p.getJointState(self.adam.robot_id, joint_id)
            joint_positions.append(joint_state[0])  # La posición de la articulación está en el índice 0
            joint_velocities.append(joint_state[1]) # La velocidad de la articulacion en el indice 1

        return joint_positions, joint_velocities

    
    # Forward dynamics
    def calculate_arm_forward_dynamics(self, torques, arm):
        if arm == "right":
            joints = self.adam.ur3_right_arm_joints

        elif arm == "left":
            joints = self.adam.ur3_left_arm_joints

        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")

        #Se obtienen las posiciones y velocidades actuales
        self.adam.pos_act,self.adam.vel_act = self.adam.get_joints_pos_vel(arm)

        #Obtener los términos de la dinámica del sistema
        zero_acc = [0.0] * len(joints)
        zero_vel = zero_acc
        #Termino gravitacional
        tau_g = p.calculateInverseDynamics(self.adam.robot_id,self.adam.pos_act,zero_vel,zero_acc, flags=1)

        #Terminos de coriolis y centrifugo
        #Para calcular el torque de coriolis y centrifugas despreciamos la gravedad
        tau_c_c_g = p.calculateInverseDynamics(self.adam.robot_id,self.adam.pos_act, self.adam.vel_act,zero_acc, flags=1)
    
        # Restar los términos gravitacionales de los combinados
        tau_c_c = [tau_c_c_g[i] - tau_g[i] for i in range(len(joints))]
        
        #Calculo de la aceleracion
        joints_acc=[]
        for j, i in enumerate(joints):
            #Matriz de inercia M
            dynamics_info = p.getDynamicsInfo(self.adam.robot_id,i)
            # Obtener el valor de inercia para el eje z
            I_z = dynamics_info[2][2]

            
            if I_z>0:
                joints_acc.append((torques[j]-tau_c_c[j]-tau_g[j])/I_z)
            else: 
                joints_acc.append(0.0)

        return joints_acc
    
    # Inverse dynamics
    def calculate_arm_inverse_dynamics (self, pos_des, pos_act, vel_act, arm):
        
        
        #Calculate current pos, vel 
        if arm == "right" or arm == "left":
            # pos_act, vel_act = self.get_joints_pos_vel(arm)
            pass
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")
        
        vel_des = []
        acc_des = []

        for i in range(len(pos_des)):
            vel_des.append((pos_des[i]-pos_act[i])/self.adam.dt)
            acc_des.append((vel_des[i]-vel_act[i])/self.adam.dt)
            # print(f"aceleración deseada: {acc_des[i]}")
            print(f"pos_des:   {pos_des[i]}")
            print(f"pos_act:   {pos_act[i]}")
        
        
        
        #Calculate inverse dynamics
        try:
            torque_IK = []
            torque_all = list(p.calculateInverseDynamics(self.adam.robot_id, pos_act, vel_act, acc_des, flags=1))
            # for i in range (len(torque_all)):
                # print(f"torque en articulacion {i} es de {torque_all[i]}")
            for i in range (len(pos_des)):
                torque_IK.append(torque_all[i+15])
                # torque_IK.append(20)

                # print(f"torque aplicado en las articulaciones: {torque_IK[i]}")
                
        except Exception as e:
            raise SystemError(f"Error en calculateInverseDynamics: {e}")

        return torque_IK, vel_des, acc_des

