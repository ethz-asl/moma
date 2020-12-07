import numpy as np
from numpy import linalg as LA

from highlevel_planning.tools.door_opening_util import *

from highlevel_planning.sim.direction_estimators.direction_estimation import SkillUnconstrainedDirectionEstimation

EPS = 1e-6
DEBUG = True

class Estimator(SkillUnconstrainedDirectionEstimation):

    def __init__(self, scene, robot, time_step, buffer_length, init_direction, initN=100):
        
        super(Estimator, self).__init__(scene, robot, time_step, buffer_length, init_direction, initN=100)
        
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
    def UpdateEstimate(self, f_wristframe, alpha, C_O_ee, smooth=False):
    
        self.counter +=1
        f_wristframe = f_wristframe.reshape(3,1)
        
        orthoProjMat = OrthoProjection(self.directionVector)    
        
        if self.counter == self.initN:
        
            self.vec = - np.array(self.objPoseBuffer[0]) + np.array(self.objPoseBuffer[len(self.objPoseBuffer) - 1])
            
        if self.counter>self.initN:
            
            e = self.GetDirectionFromPoses()
            e = np.array(C_O_ee.inv().apply(e))
            self.directionVector = e/LA.norm(e)
            
        else:

            if smooth:
            
                if len(self.measuredForcesBuffer)>0:
                    f_mean = np.zeros((3,1))
        
                    for f in self.measuredForcesBuffer:
                        f_mean +=f
                    f_mean = (1/len(self.measuredForcesBuffer)) * f_mean        
            
                    error = np.matmul(orthoProjMat, f_mean) - self.fDesired
                
                else:
                    error = np.matmul(orthoProjMat, f_wristframe) - self.fDesired
                
            
            else:
                error = np.matmul(orthoProjMat, f_wristframe) - self.fDesired
        
            error = error/LA.norm(error)
            self.directionVector = self.directionVector - alpha*error
            self.directionVector = self.directionVector/LA.norm(self.directionVector)

#-------
    def GetPlannedVelocities(self, v, calcAng=False, kAng=1):
    
        vdesEE_ee = v * self.directionVector
        
        if calcAng:
            
            theta_des = 0.0
            
            nObj_ee = np.squeeze(self.directionVector)
            n_w_des_ee = np.cross(np.array([0.0, 0.0, 1.0]), -nObj_ee)
            theta = np.arccos(np.dot(np.array([0.0, 0.0, 1.0]), -nObj_ee))
            wdesEE_ee = kAng*(theta - theta_des)*n_w_des_ee
            
        else:
            
            wdesEE_ee = np.array([0.0]*3)
            
        veldesEE_ee = np.concatenate((np.squeeze(vdesEE_ee), np.squeeze(wdesEE_ee)), axis=0)
        
        return veldesEE_ee
        
        
            
        
        
        
        
        
        
    
        
        
        
         
        
        
        
        
        
        
        
                         
                         
                
        
