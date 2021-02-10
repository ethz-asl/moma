import pybullet as p
import numpy as np
from numpy import linalg as LA

import math

from highlevel_planning.tools.door_opening_util import *

from quadprog import solve_qp

from highlevel_planning.sim.controllers.controller import ControllerTemplate

EPS = 1e-6
DEBUG = True

class Controller(ControllerTemplate):

    def __init__(self, scene, robot, time_step, noCollision=True, Npolygon=8):
        
        super(Controller, self).__init__(scene, robot, time_step)
    
        self.noCollision = noCollision
        self.Npolygon = Npolygon
        
        if self.noCollision:
            self.mode_name = 'moving_base_no_collision_max_mob_control_LCQP_'+str(Npolygon)
        else:
            self.mode_name = 'moving_base_max_mob_control_LCQP_'+str(Npolygon)
 
        self.vLinBase_b = None
        self.vAngBase_b = None
        
#-------
    def PrepareTask1(self, J_b_ee, vdesEE_b, M, b, q, q_dot, tau_prev):
    
        Kp = np.zeros((7,7))
        Kp[0, 0] = 600
        Kp[1, 1] = 600
        Kp[2, 2] = 600
        Kp[3, 3] = 600
        Kp[4, 4] = 250
        Kp[5, 5] = 150
        Kp[6, 6] = 50
        
        Kd = np.zeros((7,7))
        Kd[0, 0] = 50
        Kd[1, 1] = 50
        Kd[2, 2] = 50
        Kd[3, 3] = 20
        Kd[4, 4] = 20
        Kd[5, 5] = 20
        Kd[6, 6] = 10
        
        Aux = Kp*self.dt + Kd
        
        A = J_b_ee
        y = vdesEE_b
        
        G1 = np.matmul(np.transpose(A), A) + 0.0001**2*np.eye(len(q))
        a1 = np.matmul(np.transpose(A), y)        
        
        C1 = np.transpose(np.concatenate((Aux, -Aux, np.eye(len(q)), -np.eye(len(q))), axis=0))
        
        ineq1 = -self.torque_max[:len(q)] - b + np.matmul(Kd, q_dot)
        ineq2 = -self.torque_max[:len(q)] + b - np.matmul(Kd, q_dot)
        ineq3 = np.maximum(np.maximum(self.q_dot_min[:len(q)], (1/self.dt)*(self.q_min[:len(q)] - q)), self.dt*self.q_dot_dot_min + q_dot)
        ineq4 = -np.minimum(np.minimum(self.q_dot_max[:len(q)], (1/self.dt)*(self.q_max[:len(q)] - q)), self.dt*self.q_dot_dot_max + q_dot)
        
        b1 = np.concatenate((ineq1, ineq2, ineq3, ineq4), axis=0)

        return G1, a1, C1, b1
    
#-------
    def PrepareTask2(self, M, b, q, q_dot, tau_prev, sol1, Null1):
    
        Kp = np.zeros((7,7))
        Kp[0, 0] = 600
        Kp[1, 1] = 600
        Kp[2, 2] = 600
        Kp[3, 3] = 600
        Kp[4, 4] = 250
        Kp[5, 5] = 150
        Kp[6, 6] = 50
        
        Kd = np.zeros((7,7))
        Kd[0, 0] = 50
        Kd[1, 1] = 50
        Kd[2, 2] = 50
        Kd[3, 3] = 20
        Kd[4, 4] = 20
        Kd[5, 5] = 20
        Kd[6, 6] = 10
        
        A = Null1
        
        Aux = np.matmul((Kp*self.dt + Kd), Null1)
        
        G2 = np.matmul(np.transpose(A), A) + 0.0001**2*np.eye(len(q))
        a2 = np.matmul(np.transpose(A), q_dot - sol1) 
        
        C2 = np.transpose(np.concatenate((Aux, -Aux, Null1, -Null1), axis=0))
        
        ineq1 = -self.torque_max[:len(q)] - b + np.matmul(Kd, q_dot - sol1) - self.dt*np.matmul(Kp, np.array(sol1).reshape(-1, 1))
        ineq2 = -self.torque_max[:len(q)] + b - np.matmul(Kd, q_dot - sol1) + self.dt*np.matmul(Kp, np.array(sol1).reshape(-1, 1))
        ineq3 = np.maximum(np.maximum(self.q_dot_min[:len(q)], (1/self.dt)*(self.q_min[:len(q)] - q)), self.dt*self.q_dot_dot_min + q_dot) - sol1
        ineq4 = -np.minimum(np.minimum(self.q_dot_max[:len(q)], (1/self.dt)*(self.q_max[:len(q)] - q)), self.dt*self.q_dot_dot_max + q_dot) + sol1
        
        b2 = np.concatenate((ineq1, ineq2, ineq3, ineq4), axis=0)
        
        return G2, a2, C2, b2
        
#-------
    def CalculateDesiredJointVel(self, veldesEE_ee, J_b_ee, M, b, q, q_dot, C_O_b, C_O_ee, tau_prev, minTorque):
    
        vLindesEE_ee = np.squeeze(veldesEE_ee[:3])
        vAngdesEE_ee = np.squeeze(veldesEE_ee[3:])
        
        vLindesEE_b = np.squeeze(np.array((C_O_b.inv() * C_O_ee).apply(vLindesEE_ee)))
        vAngdesEE_b = np.squeeze(np.array((C_O_b.inv() * C_O_ee).apply(vAngdesEE_ee)))
        
        vdesEE_b = np.concatenate((vLindesEE_b, vAngdesEE_b), axis=0)
        
        G1, a1, C1, b1 = self.PrepareTask1(J_b_ee, vdesEE_b[:J_b_ee.shape[0]], M, b, q, q_dot, tau_prev)
        
        sol1,_,_,_,_,_ = solve_qp(G1, a1, C1, b1)
        
        q_dot_optimal = np.array(sol1)
        
        if minTorque:
        
            try:
                Null1 = NullProjection(J_b_ee)
                G2, a2, C2, b2 = self.PrepareTask2(M, b, q, q_dot, tau_prev, sol1, Null1)
                
                sol2,_,_,_,_,_ = solve_qp(G2, a2, C2, b2)
                q_dot_optimal += np.squeeze(np.matmul(Null1, np.array(sol2))) 
            
            except Exception as e:
                
                print("FAILED TORQUE MINIMIZATION")
                print(e)
            
        return np.squeeze(q_dot_optimal)

#-------
    def PrepareTask3(self, scaleFactor, vdes, vmean, scaledV):
    
        G3 = (1 + scaleFactor**2) * np.eye(len(vdes))
        a3 = scaleFactor * vdes + vmean
        
        C3 = []
        
        for i in range(self.Npolygon):
        
            n = [-np.cos(i*2*math.pi/self.Npolygon), -np.sin(i*2*math.pi/self.Npolygon)]
            C3.append(n)
        
        C3 = np.transpose(np.array(C3))
        b3 = scaledV * np.array(self.Npolygon * [-1.0])
        
        return G3, a3, C3, b3
        
#-------
    def SplitVelocity(self, veldesEE_ee, J_b_ee, C_O_b, C_O_ee, r_O_ee, v, q, Rlim=0.1, alpha=2.0):

        vLindesEE_O = np.array(C_O_ee.apply(veldesEE_ee[:3]))
        vAngdesEE_O = np.array(C_O_ee.apply(veldesEE_ee[3:]))
        
        vLindesEE_b = C_O_b.inv().apply(vLindesEE_O)
        vAngdesEE_b = C_O_b.inv().apply(vAngdesEE_O)        
        
        r_rob_ee = np.array(self.robot.convert_pos_to_robot_frame(r_O_ee))
        R = LA.norm(r_rob_ee[:2])
        
        scaleFactor = np.sign(R - Rlim)*(1.0 - np.exp(-alpha*np.abs(R - Rlim)))
        
        if not self.noCollision:
                
            scaleFactor = 1.0
        
        q_dot_des = []
        gamma = 0.1
        
        for i in range(len(self.q_mean[:6])):
            
            q_k = q[i]
            q_mean = self.q_mean[i]
            sign = np.sign(q_mean - q_k)
            
            if sign<0:
                
                q_dot_abs = min([abs(self.q_dot_min[i]), gamma*(1/self.dt)*abs(q_mean - q_k)])
                
            else:
                
                q_dot_abs = min([abs(self.q_dot_max[i]), gamma*(1/self.dt)*abs(q_mean - q_k)])

                
            q_dot_des.append(sign*q_dot_abs)
            
        q_dot_des = np.array(q_dot_des)
        
        vmean = np.matmul(J_b_ee[:2, :6], q_dot_des[:6])
        
        if abs(scaleFactor)>0:
            
            vmean = 1/scaleFactor * vmean        
            vdes = vLindesEE_b[:2]
            
            scaledV = 0.5*v
            
            if scaledV <= 0:
                
                scaledV = 0.001
            
            G3, a3, C3, b3 = self.PrepareTask3(scaleFactor, vdes, vmean, scaledV)            
            sol_lin,_,_,_,_,_ = solve_qp(G3, a3, C3, b3)
            
            vLinEE_b = np.array([scaleFactor*sol_lin[0], scaleFactor*sol_lin[1], vLindesEE_b[2]])

            vLinEE_ee = np.array((C_O_ee.inv() * C_O_b).apply(vLinEE_b))
            vAngEE_ee = np.array((C_O_ee.inv() * C_O_b).apply(vAngdesEE_b))             
            vEE_ee = np.concatenate((vLinEE_ee, vAngEE_ee), axis=0)
                    
            vLinBase_b = vLindesEE_b - vLinEE_b            
            vAngBase_b = np.array([0.0]*3)
                        
        else:
            vLinEE_b = np.array([0.0, 0.0, vLindesEE_b[2]])
            
            vLinEE_ee = np.array((C_O_ee.inv() * C_O_b).apply(vLinEE_b))
            vAngEE_ee = np.array((C_O_ee.inv() * C_O_b).apply(vAngdesEE_b))             
            vEE_ee = np.concatenate((vLinEE_ee, vAngEE_ee), axis=0)
                        
            vLinBase_b = vLindesEE_b - vLinEE_b            
            vAngBase_b = np.array([0.0]*3)
                
        return vLinBase_b, vAngBase_b, vEE_ee                
    
#-------            
    def PerformOneStep(self, veldesEE_ee, infoTuple=None):
    
        M = infoTuple[0]
        b = infoTuple[1]
        J_b_ee = infoTuple[2]
        q = infoTuple[3]
        q_dot = infoTuple[4]
        C_O_b = infoTuple[5]
        C_O_ee = infoTuple[6]
        r_O_ee = infoTuple[7]
        v = infoTuple[8]
        tau_prev = infoTuple[9]
        
        q = q[:7]
        q_dot = q_dot[:7]
        M = M[:7, :7]
        b = b[:7]
        J_b_ee = J_b_ee[:, :7]
        
        vLinBase_b, vAngBase_b, vEE_ee = self.SplitVelocity(veldesEE_ee, J_b_ee, C_O_b, C_O_ee, r_O_ee, v, q)
        
        self.q_dot_optimal = self.CalculateDesiredJointVel(vEE_ee, J_b_ee, M, b, q, q_dot, C_O_b, C_O_ee, tau_prev, True)
        
        p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_fingers, p.TORQUE_CONTROL, forces=[-25.0]*2)
        
        q_dot_dot_optimal = (1/self.dt)*(self.q_dot_optimal - q_dot)
        
        tau = np.matmul(M, q_dot_dot_optimal) + b 
        
        #p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm, p.TORQUE_CONTROL, forces=list(tau[:7]))
        
        p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm, p.VELOCITY_CONTROL, targetVelocities=list(self.q_dot_optimal[:7]))
        self.robot.update_velocity(vLinBase_b, vAngBase_b[2])                             
        self.robot.velocity_setter()        
        
        self.robot._world.step_one()
        self.robot._world.sleep(self.robot._world.T_s)
        
                
        
        
    
        
        
        
        
         
        
        
        
        
        
        
        
                         
                         
                
        
