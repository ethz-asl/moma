import pybullet as p
import numpy as np
from numpy import linalg as LA
from scipy.linalg import lstsq

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
		self.n_O = None
		
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
	def FitPlane(self):
		
		N_samples = len(self.objPoseBuffer)
		
		sample = np.copy(np.array(list(self.objPoseBuffer[0]) + [1.0]).reshape(1,4))
		
		A = np.copy(sample)
		
		
		for i in range(1,len(self.objPoseBuffer)):
			
			sample = np.copy(np.array(list(self.objPoseBuffer[i]) + [1.0]).reshape(1,4))
			A = np.concatenate((A, sample), axis=0)
		
		y = np.array(len(self.objPoseBuffer)*[0.0]).reshape(-1, 1)
		print(A)
		u, s, vh = LA.svd(A,  full_matrices=False)
		
		v1 = vh.T[:, vh.shape[1]-1]
		
		A = v1[0]
		B = v1[1]
		C = v1[2]
		
		n = np.array([A, B, C])
		
		n = np.sign(np.dot(n, np.array([0.0, 0.0, 1.0]))) * n
		n = n/LA.norm(n)
		return n		
			
#-------
	def UpdateEstimate(self, f_wristframe, alpha, C_O_ee, smooth=False, mixCoeff=0.1):
	
		self.counter +=1
		f_wristframe = f_wristframe.reshape(3,1)
		
		orthoProjMat = OrthoProjection(self.directionVector)	
		
		if self.counter == 100:
		
			self.n_O = self.FitPlane()
			print(self.n_O)
		
		if self.counter == self.initN:
		
			self.vec = - np.array(self.objPoseBuffer[0]) + np.array(self.objPoseBuffer[len(self.objPoseBuffer) - 1])
			
		
		if self.counter>self.initN:
		
			n_ee = np.array(C_O_ee.inv().apply(self.n_O))
			n_ee = n_ee.reshape(3,1)
			orthoProjMatPlane = OrthoProjection(n_ee)
			f_wristframe = np.matmul(orthoProjMatPlane, f_wristframe)
		
			
		error = np.matmul(orthoProjMat, f_wristframe) - self.fDesired
		error = error/LA.norm(error)
				
		#if self.counter>self.initN:
			
		#	eFromPoses = self.GetDirectionFromPoses()
		#	eFromPoses = np.array(C_O_ee.inv().apply(eFromPoses))
		#	eFromPoses = eFromPoses.reshape(3,1)
			
		#	eFromForces = error
			
		#	e = mixCoeff * eFromForces + (1.0 - mixCoeff)*eFromPoses
		#	newDirectionVector = e/LA.norm(e)
			
		#else:

		#	newDirectionVector = self.directionVector - alpha*error
		#	newDirectionVector = newDirectionVector/LA.norm(newDirectionVector)
		
		newDirectionVector = self.directionVector - alpha*error
		newDirectionVector = newDirectionVector/LA.norm(newDirectionVector)
			
		self.directionVector = newDirectionVector
		self.directionVector = self.directionVector/LA.norm(self.directionVector)		

#-------
	def GetPlannedVelocities(self, v, calcAng=False, kAng=1):
		print('direction vector: ', self.directionVector)
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
			
		
			
		
		
		
		
		
		
	
		
		
		
		 
		
		
		
		
		
		
		
						 
						 
				
		
