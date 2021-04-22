import numpy as np
from numpy import linalg as LA

import math

from highlevel_planning.tools.door_opening_util import *

from highlevel_planning.sim.direction_estimators.direction_estimation import SkillUnconstrainedDirectionEstimation
from sklearn.gaussian_process import GaussianProcessRegressor
from scipy.stats import norm

EPS = 1e-6
DEBUG = True

#----- Description -----

# This class performs the online direction estimation update procedure. It performs 
# the haptic based and the fixed-grasp based updates and outputs the combined 
# estimate. For convenience, this class also automatically gives the total 
# desired end effector velocity (linear + angular) and transfers it to the velocity
# planner module that performs the velocity split and planning within hardware 
# constraints.

#-----------------------

class Estimator(SkillUnconstrainedDirectionEstimation):

    def __init__(self, scene, robot, time_step, buffer_length, init_direction, initN=100, fd1=0.1):
        
        super(Estimator, self).__init__(scene, robot, time_step, buffer_length, init_direction, initN=100)
    
        #----- Butterworth filter parameters -----
        
        self.fd1 = fd1
        
        self.y_k_1 = init_direction
        self.y_k_2 = init_direction
        self.x_k_1 = init_direction
        self.x_k_2 = init_direction
        
        self.x_k_coeff = None
        self.x_k_1_coeff = None
        self.x_k_2_coeff = None
        
        self.y_k_1_coeff = None
        self.y_k_2_coeff = None
        
        self.SetupButterworthFilter()
        
#-------
    def SetupButterworthFilter(self):
        
        wd1 = 2*math.pi*self.fd1

        a = np.tan(0.5*wd1*self.dt)

        self.x_k_coeff = a**2/(a**2+a*2**0.5+1)
        self.x_k_1_coeff = 2*self.x_k_coeff
        self.x_k_2_coeff = self.x_k_coeff
        self.y_k_1_coeff = -(2*a**2-2)/(a**2+a*2**0.5+1)
        self.y_k_2_coeff = -(a**2-a*2**0.5+1)/(a**2+a*2**0.5+1)    
                        
#-------
    def GetDirectionFromPoses(self):
        
        #----- Fixed-grasp based estimator -----
        
        sample = np.copy(np.array(self.objPoseBuffer[0]).reshape(1,3))
        
        X = np.copy(sample)
        
        for i in range(1,len(self.objPoseBuffer)):
            
            sample = np.copy(np.array(self.objPoseBuffer[i]).reshape(1,3))
            X = np.concatenate((X, sample), axis=0)
        
        data_mean = np.mean(X, axis=0)    
        X_centered = X - data_mean
        u, s, vh = LA.svd(X_centered,  full_matrices=False)
        vh = vh.T
        
        v1 = vh[:,0]
        
        e = np.sign(np.dot(v1, self.vec))*v1
        
        return e
        
#-------
    def UpdateEstimate(self, f_wristframe, alpha, C_O_ee, smooth=False, mixCoeff=0.5):
        
        #----- Haptic based estimator -----
    
        self.counter +=1
        f_wristframe = f_wristframe.reshape(3,1)
        
        gravity_dir = np.array(C_O_ee.inv().apply(np.array([0.0, 0.0, 1.0]))).reshape(-1, 1)
        
        orthoProjMatGravity = OrthoProjection(gravity_dir)
        orthoProjMat = OrthoProjection(self.directionVector)    
              
        if self.counter == self.initN:
        
            self.vec = - np.array(self.objPoseBuffer[0]) + np.array(self.objPoseBuffer[len(self.objPoseBuffer) - 1])
        
        error = np.matmul(orthoProjMatGravity, np.matmul(orthoProjMat, f_wristframe)) - self.fDesired
        error = error/LA.norm(error)
        
        #-----------------------------------  
        
        if self.counter>self.initN:
            
            eFromPoses = self.GetDirectionFromPoses()
            eFromPoses = np.array(C_O_ee.inv().apply(eFromPoses))
            eFromPoses = eFromPoses.reshape(3,1)
            
            eFromForces = self.directionVector - alpha*error 
            eFromForces = eFromForces/LA.norm(eFromForces)
            eFromForces = eFromForces.reshape(3,1)
            
            e = mixCoeff * eFromForces + (1.0 - mixCoeff)*eFromPoses
            newDirVec = e/LA.norm(e)
            
        else:

            newDirVec = self.directionVector - alpha*error 
            newDirVec = newDirVec/LA.norm(newDirVec)
        
        if self.counter>3:            
            self.directionVector = self.x_k_coeff * newDirVec + self.x_k_1_coeff * self.x_k_1 + self.x_k_2_coeff * self.x_k_2 + self.y_k_1_coeff * self.y_k_1 + self.y_k_2_coeff * self.y_k_2
        else:
            self.directionVector = newDirVec
            
        self.x_k_2 = self.x_k_1
        self.x_k_1 = newDirVec
        
        self.y_k_2 = self.y_k_1
        self.y_k_1 = self.directionVector
                    
        self.directionVector = self.directionVector/LA.norm(self.directionVector)
        
#-------
    def GetPlannedVelocities(self, v, calcAng=False, kAng=1):
    
        vdesEE_ee = v * self.directionVector
        
        if calcAng and self.counter>self.initN:
            theta_des = 0.0
            
            nObj_ee = np.squeeze(self.directionVector)
            n_w_des_ee = np.cross(np.array([0.0, 0.0, 1.0]), -nObj_ee)
            theta = np.arccos(np.dot(np.array([0.0, 0.0, 1.0]), -nObj_ee))
            wdesEE_ee = kAng*(theta - theta_des)*n_w_des_ee
            
        else:
            
            wdesEE_ee = np.array([0.0]*3)
   
        veldesEE_ee = np.concatenate((np.squeeze(vdesEE_ee), np.squeeze(wdesEE_ee)), axis=0)
        
        return veldesEE_ee
    
#-------
    def CalculateInitialDirections(self, Nx=5, Ny=5):
        
        #----- Calculate candidate initial directions -----
        
        list_n = []
        
        list_of_x = list(np.linspace(-1.0, 1.0, num=Nx))
        list_of_y = list(np.linspace(-1.0, 1.0, num=Ny))
        
        for x in list_of_x:
            for y in list_of_y:
                
                if x**2 + y**2 <=1:
                    
                    n = [x, y, -(1.0 - x**2 - y**2)**0.5]
                    list_n.append(n)
        
        return list_n
    
#-------    
    def EstimateBestInitialDirection(self, X_data, Y_data, C_O_ee):
        
        #----- Calculating the expected initial unconstrained direction of motion -----
        
        X_data = np.array(X_data).reshape(len(X_data), -1)
        Y_data = np.array(Y_data)
        
        print("X_data: ", X_data)
        print("Y_data: ", Y_data)
        
        Z = np.sum(Y_data)
        Y_data = Y_data/Z
        
        for i in range(X_data.shape[0]):
            
            X_data[i, :] = Y_data[i]*X_data[i, :]
        
        X_data = np.sum(X_data, axis=0)
        X_data = np.squeeze(X_data)
        
        estimated_direction = np.array([X_data[0], X_data[1], -(1.0 - X_data[0]**2 - X_data[1]**2)**0.5])
        
        gravity_dir = np.array(C_O_ee.inv().apply(np.array([0.0, 0.0, 1.0]))).reshape(-1, 1)
        
        orthoProjMatGravity = OrthoProjection(gravity_dir)        
        
        estimated_direction = np.squeeze(np.matmul(orthoProjMatGravity, estimated_direction.reshape(-1,1)))
        
        estimated_direction = estimated_direction/LA.norm(estimated_direction)
        
        print("ESTIMATED DIR: ", estimated_direction)
        
        self.y_k_1 = estimated_direction.reshape(3,1)
        self.y_k_2 = estimated_direction.reshape(3,1)
        self.x_k_1 = estimated_direction.reshape(3,1)
        self.x_k_2 = estimated_direction.reshape(3,1)
        
        self.initDir = estimated_direction.reshape(3,1)
        self.directionVector = estimated_direction.reshape(3,1)

        
        
        
        
        
                
            
                
        
        
            
        
        
        
        
        
        
    
        
        
        
         
        
        
        
        
        
        
        
                         
                         
                
        
