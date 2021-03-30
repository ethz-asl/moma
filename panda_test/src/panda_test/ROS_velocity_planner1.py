import numpy as np

from panda_test.ROS_door_opening_util import *

from quadprog import solve_qp

EPS = 1e-6
DEBUG = True

#----- Description -----

# This is the class for the fixed base version of the algorithm. The controller 
# performs the optimization procedure needed for issuing the joint velocity commands
# in compliance with the hardware imposed constraints.

#-----------------------

class Controller:

    def __init__(self, time_step):
    
        self.dt = time_step
        
        self.mode_name = 'fixed_base_torque_control'
        
        #----- Constraints obtained from the data sheet of the robot -----

        self.torque_max = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
        self.torque_dot_max = np.array([1000.0]*7)
        self.torque_dot_min = np.array([-1000.0]*7)
        
        self.q_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        self.q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        
        self.q_dot_max = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
        self.q_dot_min = np.array([-2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100])
        
        self.q_dot_dot_max = np.array([15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])
        self.q_dot_dot_min = -np.array([15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])
        
        self.q_mean = np.copy(0.5*(self.q_max+self.q_min))
        
        #----- Init Values -----
        
        self.q_dot_optimal = np.array(7*[0.0])  
        self.q_dot_dot_optimal = np.array(7*[0.0]) 
        self.tau_optimal = None
        
        self.vLinBase_b = [0.0, 0.0, 0.0]
        self.vAngBase_b = [0.0, 0.0, 0.0]
        
        self.sol_lin_previous = [0.0]*7
        
#-------
    def PrepareTask1(self, J_b_ee, vdesEE_b, M, b, q, q_dot, tau_prev):
        
        #----- Joint impedance parameters taken from the franka_ros package -----
    
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
        
        #----- Description -----
        
        # This is a task with a second highest priority solved in the null space
        # of the original task of issuing the optimal joint velocity commands within
        # the hardware constraints. It is not used in the solution presented in the
        # thesis but is left here in case some future work finds it useful.
        
        #-----------------------
    
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
        
        ineq1 = -self.torque_max[:len(q)] - b + np.matmul(Kd, q_dot - sol1) - self.dt*np.matmul(Kp, np.array(sol1))
        ineq2 = -self.torque_max[:len(q)] + b - np.matmul(Kd, q_dot - sol1) + self.dt*np.matmul(Kp, np.array(sol1))
        ineq3 = np.maximum(np.maximum(self.q_dot_min[:len(q)], (1/self.dt)*(self.q_min[:len(q)] - q)), self.dt*self.q_dot_dot_min + q_dot) - sol1
        ineq4 = -np.minimum(np.minimum(self.q_dot_max[:len(q)], (1/self.dt)*(self.q_max[:len(q)] - q)), self.dt*self.q_dot_dot_max + q_dot) + sol1
        
        b2 = np.concatenate((ineq1, ineq2, ineq3, ineq4), axis=0)
        
        return G2, a2, C2, b2
        
#-------
    def CalculateDesiredJointVel(self, veldesEE_ee, J_b_ee, M, b, q, q_dot, C_b_ee, tau_prev, minTorque):
    
        vLindesEE_ee = np.squeeze(veldesEE_ee[:3])
        vAngdesEE_ee = np.squeeze(veldesEE_ee[3:])
        
        vLindesEE_b = np.squeeze(np.array(C_b_ee.apply(vLindesEE_ee)))
        vAngdesEE_b = np.squeeze(np.array(C_b_ee.apply(vAngdesEE_ee)))
        
        vdesEE_b = np.concatenate((vLindesEE_b, vAngdesEE_b), axis=0)
        
        try:
        
            G1, a1, C1, b1 = self.PrepareTask1(J_b_ee, vdesEE_b[:J_b_ee.shape[0]], M, b, q, q_dot, tau_prev)
        
            sol1,_,_,_,_,_ = solve_qp(G1, a1, C1, b1)
            
            if sol1 is None:
                
                return self.q_dot_optimal
            
            else:
        
                q_dot_optimal = np.array(sol1)

            #----- This is if the null space projection control is included ------
            
            # In the end, it was not used in the solution presented in the thesis 
            # but is left here as an option to be later included if needed. 
        
            if minTorque:
        
                try:
                    Null1 = NullProjection(J_b_ee)
                    G2, a2, C2, b2 = self.PrepareTask2(M, b, q, q_dot, tau_prev, sol1, Null1)
                
                    sol2,_,_,_,_,_ = solve_qp(G2, a2, C2, b2)
                    
                    if sol2 is not None:
                        q_dot_optimal += np.squeeze(np.matmul(Null1, np.array(sol2))) 
            
                except Exception as e:
                
                    print("FAILED TORQUE MINIMIZATION")
                    print(e)
            
            return np.squeeze(q_dot_optimal)
    
        except Exception as e:
            
            print(10*"*" + " ERROR MESSAGE " + 10*"*")
            print(e)
            print("Optimization failed: Keeping the previous commanded values")
            print(30*"*")
            return self.q_dot_optimal
        
#-------
    def GetCurrOptSol(self):
    
        return self.q_dot_optimal        
                                        
#-------            
    def PerformOneStep(self, veldesEE_ee, infoTuple=None):
        
        M = infoTuple[0]
        b = infoTuple[1]
        J_b_ee = infoTuple[2]
        q = infoTuple[3]
        q_dot = infoTuple[4]
        C_O_b = infoTuple[5]
        C_O_ee = infoTuple[6]
        C_b_ee = infoTuple[7]
        r_b_ee = infoTuple[8]
        v = infoTuple[9]
        tau_prev = infoTuple[10]
        
        M = M[:7, :7]
        b = b[:7]
        J_b_ee = J_b_ee[:, :7]
        
        try:
            self.q_dot_optimal = self.CalculateDesiredJointVel(veldesEE_ee, J_b_ee, M, b, q, q_dot, C_b_ee, tau_prev, False)  
            self.q_dot_dot_optimal = (1/self.dt)*(self.q_dot_optimal - q_dot)        
            self.tau_optimal = np.matmul(M, self.q_dot_dot_optimal) + b
        except:
            pass
        
        return True
                
        
        
    
        
        
        
        
         
        
        
        
        
        
        
        
                         
                         
                
        
