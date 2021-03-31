import numpy as np

from highlevel_planning.tools.door_opening_util import *

from collections import deque

EPS = 1e-6
DEBUG = True

#----- Description -----

# This is the base class for the online direction estimation update procedure. 
# The 'UpdateEstimate' function needs to be implemented so that the online 
# direction estimation module can be plugged-in and used within the door opening 
# skill. This function should obtain the haptic based and fixed-grasp based estimates
# and combine them.

#-----------------------

class SkillUnconstrainedDirectionEstimation:

    def __init__(self, scene, robot, time_step, buffer_length, init_direction, initN=100):
    
        self.scene = scene
        self.robot = robot
        self.dt = time_step
        self.bufferLength = buffer_length
        self.initDir = init_direction
        self.initN = initN
        
        self.vec = None
        
        self.fDesired = np.array([0.0]*3).reshape(3,1)
        
        #----- Init values -----
        
        self.directionVector = init_direction
        
        self.counter = 0
        
        self.objPoseBuffer = deque(maxlen=buffer_length)
        self.measuredForcesBuffer = deque(maxlen=buffer_length)
        
#-------
    def reset(self):
    
        self.directionVector = self.initDir    
        self.counter = 0    
        self.objPoseBuffer = deque(maxlen=self.bufferLength)
        self.measuredForcesBuffer = deque(maxlen=self.bufferLength)        
        
#-------
    def SetupEstimationProcedure(self, approached, target_name, link_idx, grasp_id, sk_grasp, sk_nav=None, nav_min_dist=1.0):
    
        if not approached:
            
            sk_nav.move_to_object(target_name, nav_min_dist)
            
        res = sk_grasp.grasp_object(target_name, link_idx, grasp_id)
        
        if not res:
            
            print('Initial grasping failed')
            return False
            
        else:
            
            return True
            
#-------
    def UpdateEstimate(self, f_wristframe, alpha, C_O_ee, smooth=False):
    
        raise NotImplementedError

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
        
#-------
    def GetCurrEstimate(self):
        
        return np.squeeze(self.directionVector)
        
#-------
    def UpdateBuffers(self, f_wristframe, pose):
        print("Pose: "+str(pose))
        self.objPoseBuffer.append(pose)
        self.measuredForcesBuffer.append(np.array(f_wristframe).reshape(3,1))
            
        
            
        
        
        
        
        
        
    
        
        
        
         
        
        
        
        
        
        
        
                         
                         
                
        
