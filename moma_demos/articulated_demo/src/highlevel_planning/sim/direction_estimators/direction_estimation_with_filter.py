import numpy as np
from numpy import linalg as LA

import math

from highlevel_planning.tools.door_opening_util import *

from highlevel_planning.sim.direction_estimators.direction_estimation import SkillUnconstrainedDirectionEstimation

EPS = 1e-6
DEBUG = True

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
        #X_projected = np.matmul(X_centered, e)
        
        return e
        
#-------
    def UpdateEstimate(self, f_wristframe, alpha, C_O_ee, smooth=False, mixCoeff=0.5):
    
        self.counter +=1
        f_wristframe = f_wristframe.reshape(3,1)
        
        orthoProjMat = OrthoProjection(self.directionVector)    
              
        if self.counter == self.initN:
        
            self.vec = - np.array(self.objPoseBuffer[0]) + np.array(self.objPoseBuffer[len(self.objPoseBuffer) - 1])
        
        error = np.matmul(orthoProjMat, f_wristframe) - self.fDesired
        error = error/LA.norm(error)
                
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
        
        
            
        
        
        
        
        
        
    
        
        
        
         
        
        
        
        
        
        
        
                         
                         
                
        
