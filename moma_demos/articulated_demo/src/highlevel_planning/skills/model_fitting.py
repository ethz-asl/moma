import pybullet as p
import numpy as np
from numpy import linalg as LA

from math import log

from scipy.spatial.transform import Rotation as R
from highlevel_planning.tools.util import homogenous_trafo, invert_hom_trafo
from matplotlib import pyplot as plt
from scipy.linalg import lstsq
from sklearn.metrics import mean_squared_error

from highlevel_planning.tools.util import IKError
from collections import deque

EPS = 1e-6
DEBUG = True

def ortho_projection(direction):
    assert np.abs(np.linalg.norm(direction) - 1.0) < EPS
    projection = np.matmul(direction, direction.T)
    return np.eye(3) - projection


class SkillModelIdentification:
	def __init__(self, scene_, robot_, buffer_length, time_step):
	
		self.scene = scene_
		self.robot = robot_
		self.buffer_length = buffer_length
		self.dt = time_step
		
		self.direction_vector = None
		
		self.obj_pose_buffer = deque(maxlen=buffer_length)
		
	def init_trajectory(self, desired_velocity, sk_grasp, sk_nav, observation_available=True, approached=True, target_name="cupboard", link_idx=0, grasp_id=0):


		if self.buffer_length>5000:
			it_max = self.buffer_length
			
		else:
			it_max = 5000
		
		if not approached:
		
			sk_nav.move_to_object("cupboard", nav_min_dist=1.0)
			
		it = 0
		
		while len(self.obj_pose_buffer)<self.buffer_length and it<it_max:
			
			if observation_available:
		
				obj_info = self.scene.objects[target_name]
				target_id = obj_info.model.uid
			
				if len(obj_info.grasp_links) == 0:
					raise SkillExecutionError("No grasps defined for this object")
				link_id = obj_info.grasp_links[link_idx]

				num_grasps = len(obj_info.grasp_pos[link_id])
				if num_grasps == 0:
					raise SkillExecutionError("No grasps defined for this object")
				if grasp_id >= num_grasps:
					raise SkillExecutionError("Invalid grasp ID")

        			# Get the object pose
				if link_id == -1:
					temp = p.getBasePositionAndOrientation(target_id)
					target_pos = np.array(temp[0]).reshape((-1, 1))

				else:
					temp = p.getLinkState(target_id, link_id)
					target_pos = np.array(temp[4]).reshape((-1, 1))
					
			else:
			
				link_pos_and_vel = p.getLinkState(self.robot.model.uid, linkIndex=self.robot.link_name_to_index["panda_default_EE"])	
				target_pos = link_pos_and_vel[4]		
			
			if len(self.obj_pose_buffer)>0:
			
				if np.linalg.norm(np.array(self.obj_pose_buffer[len(self.obj_pose_buffer)-1])- np.array(target_pos))>EPS:
					self.obj_pose_buffer.append(np.array(target_pos))	
					
			else:
				self.obj_pose_buffer.append(np.array(target_pos))
				
			velocity_rotation = np.array([0.0, 0.0, 0.0])
			self.robot.task_space_velocity_control(np.squeeze(desired_velocity*np.array([0, 0, -1])), np.squeeze(velocity_rotation), 1)
				
			it +=1 

		start_pose = np.copy(self.obj_pose_buffer[0].reshape(1,3))
		stop_pose = np.copy(self.obj_pose_buffer[len(self.obj_pose_buffer)-1].reshape(1,3))
		self.direction_vector = stop_pose - start_pose
			
		print("Initial trajectory finished")	
		
	def FitPrismaticModel(self):
	
		N_samples = len(self.obj_pose_buffer)
		
		sample = np.copy(self.obj_pose_buffer[0].reshape(1,3))
		
		X = np.copy(sample)
		
		for i in range(1,len(self.obj_pose_buffer)):
			
			sample = np.copy(self.obj_pose_buffer[i].reshape(1,3))
			X = np.concatenate((X, sample), axis=0)
		
		data_mean = np.mean(X, axis=0)	
		X_centered = X - data_mean
		u, s, vh = LA.svd(X_centered,  full_matrices=False)
		vh = vh.T
		
		v1 = vh[:,0]
		
		e = np.sign(np.dot(v1, np.squeeze(self.direction_vector)))*v1
		
		X_projected = np.matmul(X_centered, e)
		
		return e, X_projected, data_mean, X
		 
	def FitRevoluteModel(self):
	
		N_samples = len(self.obj_pose_buffer)
		
		sample = np.copy(self.obj_pose_buffer[0].reshape(3,1))
		
		A = np.zeros([2,7])
		A[0,0] = sample[0,0]
		A[0,1] = sample[1,0]
		A[0,2] = sample[2,0]
		A[0,3] = 1
		A[1,4] = 1
		A[1,5] = 2*sample[0,0]
		A[1,6] = 2*sample[1,0]
		
		B = np.zeros([2,1])
		B[1,0] = sample[0,0]*sample[0,0]+sample[1,0]*sample[1,0]
		
		A_tilda = np.copy(A)
		B_tilda = np.copy(B)

		for i in range(1,len(self.obj_pose_buffer)):
			
			sample = np.copy(self.obj_pose_buffer[i].reshape(3,1))
			
			A = np.zeros([2,7])
			A[0,0] = sample[0,0]
			A[0,1] = sample[1,0]
			A[0,2] = sample[2,0]
			A[0,3] = 1
			A[1,4] = 1
			A[1,5] = 2*sample[0,0]
			A[1,6] = 2*sample[1,0]
			
			B = np.zeros([2,1])
			B[1,0] = sample[0,0]*sample[0,0]+sample[1,0]*sample[1,0]
			
			A_tilda = np.concatenate((A_tilda, A), axis=0)
			B_tilda = np.concatenate((B_tilda, B), axis=0)
			
		p, res, rnk, s = lstsq(A_tilda, B_tilda)
		 
		return p, A_tilda, B_tilda	
			 
	def PickModel(self):
	
		#print(self.obj_pose_buffer)
		p_r, A_r, B_r = self.FitRevoluteModel()
		e, X_projected, data_mean, X = self.FitPrismaticModel()
		
		k_p = len(e)+len(data_mean)
		k_r = len(p_r)
		
		B_pred_r = np.matmul(A_r, p_r)
		X_pred_p = np.outer(X_projected, e) + data_mean
		
		mse_r = mean_squared_error(B_r, B_pred_r)
		mse_p = mean_squared_error(X, X_pred_p)
		
		print("Revolute: ", mse_r)
		print("Prismatic: ", mse_p)
		print(e)
		
		if mse_r<mse_p:
			
			model_type = 'revolute'
			
			A = p_r[0]
			B = p_r[1]
			C = p_r[2]
			D = p_r[3]
			
			n = np.array([A, B, C])
			
			x_c = p_r[5]
			y_c = p_r[6]
			z_c = -1.0/C*(A*x_c+B*y_c+D)
			
			C = np.array([x_c, y_c, z_c])
			
			r_sqr = p_r[4]+x_c**2+y_c**2
			r = r_sqr**0.5 
			
			return model_type, (n, C, r)
			
		else:
		
			model_type = 'prismatic'
			n = e
			C = data_mean
			return model_type, (n, C)
			

		
		
		
		
		
		
		
						 
						 
				
		
