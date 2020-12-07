import pybullet as p
import numpy as np
from numpy import linalg as LA

from scipy.spatial.transform import Rotation as R
from highlevel_planning.tools.util import homogenous_trafo, invert_hom_trafo

from highlevel_planning.tools.door_opening_util import *

from highlevel_planning.tools.util import IKError
from collections import deque

EPS = 1e-6
DEBUG = True

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
	def GetDirectionFromPoses(self):
	
		N_samples = len(self.objPoseBuffer)
		
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
		
#-------
	def GetCurrEstimate(self):
		
		return np.squeeze(self.directionVector)
		
#-------
	def UpdateBuffers(self, f_wristframe, pose):
		
		self.objPoseBuffer.append(pose)
		self.measuredForcesBuffer.append(np.array(f_wristframe).reshape(3,1))
			
		
			
		
		
		
		
		
		
	
		
		
		
		 
		
		
		
		
		
		
		
						 
						 
				
		
