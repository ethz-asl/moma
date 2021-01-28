import numpy as np
from numpy import linalg as LA
import math

from panda_test.ROS_door_opening_util import *

from collections import deque

EPS = 1e-6
DEBUG = True

class SkillUnconstrainedDirectionEstimation:

    def __init__(self, time_step, buffer_length, init_direction, initN=100, fd1=0.1):
    
        self.dt = time_step
        self.bufferLength = buffer_length
        self.initDir = init_direction
        self.initN = initN
        self.fd1 = fd1
        
        self.vec = None
        
        self.y_k_1 = init_direction
        self.y_k_2 = init_direction
        self.x_k_1 = init_direction
        self.x_k_2 = init_direction
        
        self.x_k_coeff = None
        self.x_k_1_coeff = None
        self.x_k_2_coeff = None
        
        self.y_k_1_coeff = None
        self.y_k_2_coeff = None
        
        
        self.fDesired = np.array([0.0]*3).reshape(3,1)
        
        #----- Init values -----
        
        self.directionVector = init_direction
        self.SetupButterworthFilter()
        self.counter = 0
        
        self.objPoseBuffer = deque(maxlen=buffer_length)
        self.measuredForcesBuffer = deque(maxlen=buffer_length)
        
        self.sufficiently_filled = False
        
#-------
    def reset(self):
    
        self.directionVector = self.initDir    
        self.counter = 0    
        self.objPoseBuffer = deque(maxlen=self.bufferLength)
        self.measuredForcesBuffer = deque(maxlen=self.bufferLength)        
            
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
        X_projected = np.matmul(X_centered, e)
        
        return e
        
#-------
    def UpdateEstimate(self, f_wristframe, alpha, C_O_ee, smooth=False, mixCoeff=0.1):
    
        self.counter +=1
        f_wristframe = f_wristframe.reshape(3,1)
        
        orthoProjMat = OrthoProjection(self.directionVector)    
                
        if self.counter > self.initN and len(self.objPoseBuffer)>1 and not(self.sufficiently_filled):
        
            self.vec = - np.array(self.objPoseBuffer[0]) + np.array(self.objPoseBuffer[len(self.objPoseBuffer) - 1])
            self.sufficiently_filled = True
        
        error = np.matmul(orthoProjMat, f_wristframe) - self.fDesired
        error = error/LA.norm(error)
                
        if self.sufficiently_filled:
            
            dirFromPoses = self.GetDirectionFromPoses()
            dirFromPoses = np.array(C_O_ee.inv().apply(dirFromPoses))
            dirFromPoses = dirFromPoses.reshape(3,1)
            
            dirFromForces = self.directionVector - alpha*error
            dirFromForces = dirFromForces/LA.norm(dirFromForces)
            dirFromForces = dirFromForces.reshape(3,1)
            
            e = mixCoeff * dirFromForces + (1.0 - mixCoeff)*dirFromPoses
            
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
    def GetCurrEstimate(self):
        
        return np.squeeze(self.directionVector)
        
#-------
    def UpdateBuffers(self, f_wristframe, pose):
        
        if len(self.objPoseBuffer)>1:
            
            if LA.norm(np.array(self.objPoseBuffer[len(self.objPoseBuffer) - 1]) - pose)>1e-6:
                
                self.objPoseBuffer.append(pose)
                self.measuredForcesBuffer.append(np.array(f_wristframe).reshape(3,1))
                
        else:
            
            self.objPoseBuffer.append(pose)
            self.measuredForcesBuffer.append(np.array(f_wristframe).reshape(3,1))
            
        
            
        
        
        
        
        
        
    
        
        
        
         
        
        
        
        
        
        
        
                         
                         
                
        
