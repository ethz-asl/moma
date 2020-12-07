import pybullet as p
import numpy as np
import pinocchio as pin
import time
from pinocchio.robot_wrapper import RobotWrapper
from scipy.optimize import minimize
from pinocchio.utils import *

from numpy import linalg as LA

from math import log
import math

from scipy.spatial.transform import Rotation as R
from highlevel_planning.tools.util import homogenous_trafo, invert_hom_trafo
from matplotlib import pyplot as plt
from scipy.linalg import lstsq
from sklearn.metrics import mean_squared_error

from highlevel_planning.tools.util import IKError
from collections import deque
import matplotlib.pyplot as plt

from quadprog import solve_qp

import os

EPS = 1e-4
DEBUG = True

#------------------- Helper functions ---------------------------------------
    
def ortho_projection(direction):
	
	assert np.abs(np.linalg.norm(direction) - 1.0) < EPS
	projection = np.matmul(direction, direction.T)
	
	return np.eye(3) - projection
	
def Null_proj(A):
	
	return np.eye(A.shape[1])-np.matmul(LA.pinv(A), A) 

def getJointStates(robot):

	joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
	joint_positions = [state[0] for state in joint_states]
	joint_velocities = [state[1] for state in joint_states]
	joint_torques = [state[3] for state in joint_states]
	
	return joint_positions, joint_velocities, joint_torques


def getMotorJointStates(robot):

	joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
	joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
	joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
	joint_positions = [state[0] for state in joint_states]
	joint_velocities = [state[1] for state in joint_states]
	joint_torques = [state[3] for state in joint_states]
	
	return joint_positions, joint_velocities, joint_torques
	
def LinearVelocityObjective(x, *args):

	u1_lin = args[0]
	lin_vdesEE_b_arm = args[1]
	scale_coeff = args[2]
	
	temp = np.array([scale_coeff*(lin_vdesEE_b_arm[0] - x[0]), scale_coeff*(lin_vdesEE_b_arm[1] - x[1]), lin_vdesEE_b_arm[2]])
	temp = temp/LA.norm(temp)
	
	u1_lin = u1_lin/LA.norm(u1_lin)
	
	return -np.dot(temp, u1_lin)
	
def LVO_constraint1(x, *args):
	
	v = args[0]
	
	return v**2 - x[0]**2 - x[1]**2
	
def AngularVelocityObjective(x, *args):

	u1_ang = args[0]
	ang_vdesEE_b_arm = args[1]
	
	temp = np.array([ang_vdesEE_b_arm[0], ang_vdesEE_b_arm[1], ang_vdesEE_b_arm[2] - x])
	temp = temp/LA.norm(temp)
	
	return -np.dot(temp, u1_ang)
				
#------------------- Model based planning skill class -----------------------------

class SkillTrajectoryPlanning:

	def __init__(self, scene_, robot_, sk_mID_, time_step):
	
		self.scene = scene_
		self.robot = robot_
		self.sk_mID = sk_mID_
		self.dt = time_step
		
		#----- Constraints -----
		
		self.torque_max = []
		self.q_max = []
		self.q_min = []
		self.q_dot_max = []
		self.q_dot_min = []
		self.q_mean_value = []
		
		#----- Additional values needed for planning -----
		
		self.f_desired = np.array([0.0, 0.0, 0.0]).reshape(-1, 1)
		self.force_integral = np.array([0.0, 0.0, 0.0]).reshape(-1, 1)
		self.C_O_obj_k = None
		self.q_dot_optimal_previous = None
		
		#----- Log data for plotting -----
		
		self.log_q = []
		self.log_q_dot = []
		self.log_tau = []
		
		self.log_estimated_dir = []			# Stored to check model ID performance
		self.log_actual_dir = []			# Actual direction
								
		self.log_exec_time = []
		self.log_lin_vdesEE_O = []
		self.log_ang_vdesEE_O = []
		
		self.log_theta = []
		self.log_lin_vEE_O_meas = []
		self.log_ang_vEE_O_meas = []
		self.log_manipulability_meas = []
		
		self.log_lin_vBase_O_meas = []
		self.log_ang_vBase_O_meas = []
		
		self.log_q_dot_optimal = []
		self.log_relative_v = []
		
		self.log_f_wristframe = []
		self.log_t_wristframe = []
		self.log_actual_drawer_pos = []
			
		#----- Init values -----
		
		self.PrintRobotJointInfo()
		self.ExtractMotorizedJointConstraints()

#-------		
	def PrintRobotJointInfo(self):
	
		num_joints = p.getNumJoints(self.robot.model.uid)
		
		print("***** Robot joint information *****")
		for i in range(num_joints):
		
			info = p.getJointInfo(self.robot.model.uid, i)
			joint_name = info[1] if type(info[1]) is str else info[1].decode("utf-8")
			print("i: ", i, " | name: ", joint_name, " | type: ", info[2], " | parentIndex: ", info[16], " | linkName: ", info[12], " | min and max: ", 
				info[8], info[9], info[10], info[11]
			)
					
#-------			
	def ExtractMotorizedJointConstraints(self):
	
		num_joints = p.getNumJoints(self.robot.model.uid)
		
		for i in range(num_joints):
			
			info = p.getJointInfo(self.robot.model.uid, i)
			if i in self.robot.joint_idx_arm or i in self.robot.joint_idx_fingers:
				
				self.q_dot_max.append(info[11])
				self.q_dot_min.append(-info[11])
				self.torque_max.append(info[10])
				self.q_max.append(info[9])
				self.q_min.append(info[8])
		
		self.q_dot_max = np.array(self.q_dot_max)
		self.q_dot_min = np.array(self.q_dot_min)
		self.torque_max = np.array(self.torque_max)
		self.q_max = np.array(self.q_max)
		self.q_min = np.array(self.q_min)
		
		self.q_mean = np.copy(0.5*(self.q_max+self.q_min))
		
#-------
	def ResetArmAndBaseJointDampings(self):
	
		num_joints = p.getNumJoints(self.robot.model.uid)
			
		p.changeDynamics(self.robot.model.uid, -1, linearDamping=0, angularDamping=0, jointDamping=0)
		for i in self.robot.joint_idx_arm:
			
			p.changeDynamics(self.robot.model.uid, i, linearDamping=0, angularDamping=0, jointDamping=0)
				
		p.setJointMotorControlArray(self.robot.model.uid, range(num_joints), p.VELOCITY_CONTROL, forces = [0.0]*num_joints)
		
#-------
	def ResetFingerJointDampings(self):
	
		for i in self.robot.joint_idx_fingers:
			p.changeDynamics(self.robot.model.uid, i, linearDamping=0, angularDamping=0, jointDamping=0)
		
		p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_fingers, p.VELOCITY_CONTROL, forces = [0.0]*len(self.robot.joint_idx_fingers))
		
#-------
	def getInitialObjectOri(self, target_name=None, link_idx=None, grasp_id=0):
	
		_,_,self.C_O_obj_k,_ = self.CalculateGraspedObjectPosOriNormal(target_name, link_idx, grasp_id)
	
#-------	
	def CalculateGraspedObjectPosOriNormal(self, target_name=None, link_idx=None, grasp_id=0):
	
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
		
		if link_id == -1:			
			temp = p.getBasePositionAndOrientation(target_id)
			pos_obj  = np.array(temp[0]).reshape((-1, 1))
			ori_obj = np.array(temp[1])
		else:
			temp = p.getLinkState(target_id, link_id, computeForwardKinematics = True)
			pos_obj = np.array(temp[4]).reshape((-1, 1))
			ori_obj = np.array(temp[5])	
		
		r_O_obj = np.squeeze(pos_obj)
		quat = np.squeeze(ori_obj)
		C_O_obj = R.from_quat(ori_obj)
		
		nobj_O = np.squeeze(np.array(C_O_obj.as_matrix())[:, 0])		# This is true if grasped object's x axis is aligned with the object's plane normal
		
		return r_O_obj, quat, C_O_obj, nobj_O
		
#-------
	def GetMeasurements(self):
	
		link_pos_and_vel = p.getLinkStates(self.robot.model.uid, linkIndices=[self.robot.arm_base_link_idx, self.robot.link_name_to_index["panda_default_EE"]], 
			computeLinkVelocity = 1, computeForwardKinematics = True
		)
		
		f_wristframe, t_wristframe = self.robot.get_wrist_force_torque()
		
		C_O_b = R.from_quat(link_pos_and_vel[0][5])
		r_O_b = link_pos_and_vel[0][4]
		
		C_O_ee = R.from_quat(link_pos_and_vel[1][5])
		r_O_ee = link_pos_and_vel[1][4]
		
		lin_vEE_O = link_pos_and_vel[1][6]
		ang_vEE_O = link_pos_and_vel[1][7]
		
		lin_vBase_O, ang_vBase_O = p.getBaseVelocity(self.robot.model.uid)
		
		q, q_dot, mtorq = getMotorJointStates(self.robot.model.uid)
		zero_vec = [0.0]*len(q)
		
		b = np.array(p.calculateInverseDynamics(self.robot.model.uid, list(q), list(q_dot), zero_vec))
		M = np.array(p.calculateMassMatrix(self.robot.model.uid, list(q)))		
		
		lin, ang = p.calculateJacobian(self.robot.model.uid, self.robot.arm_ee_link_idx, [0.0, 0.0, 0.0], list(q), zero_vec, zero_vec)
		lin = np.array(lin)
		ang = np.array(ang)
			
		J_b_ee = np.concatenate((lin, ang), axis=0)
		
		return M, b, J_b_ee, q, q_dot, C_O_b, r_O_b, C_O_ee, r_O_ee, mtorq, lin_vEE_O, ang_vEE_O, lin_vBase_O, ang_vBase_O, f_wristframe, t_wristframe	
		 		
#-------
	def CalculateDesiredEEVelocity(self, model_type, parameters, v, C_O_b_k, C_O_ee_k, r_O_ee_k, r_O_obj_k, C_O_obj_k, nobj_O_k):
		 
		if model_type == 'prismatic':
		
			e = np.squeeze(parameters[0])
			C = np.squeeze(parameters[1])
			
			rdesEE_O = np.squeeze(r_O_obj_k + v*self.dt*e)		# Desired EE position at the start of the k+1-th time step expressed in world frame
			
			e_k_plus_1 = e
			
			theta_rot = 0
			
			C_obj_k_obj_k_plus_1 = R.from_matrix(np.eye(3))
			
		else:
		
			n = np.squeeze(parameters[0])
			C = np.squeeze(parameters[1])
			r = parameters[2]
			
			e = np.squeeze(np.cross(r_O_obj_k - np.squeeze(C), np.squeeze(n)))
			e = np.sign(np.dot(np.squeeze(self.sk_mID.direction_vector), e))*e
			e = np.squeeze(e/LA.norm(e))
			
			rdesEE_O = np.squeeze(r_O_obj_k + v*self.dt*e)		# Desired EE position at the start of the k+1-th time step expressed in world frame
		
			e_k_plus_1 = np.squeeze(np.cross(rdesEE_O - C, n))
			e_k_plus_1 = np.sign(np.dot(np.squeeze(self.sk_mID.direction_vector), e_k_plus_1))*e_k_plus_1
			e_k_plus_1 = e_k_plus_1/LA.norm(e_k_plus_1)
			
			theta_rot = v*self.dt/(r*np.sign(np.dot(np.squeeze(self.sk_mID.direction_vector), nobj_O_k)))
			
			C_obj_k_obj_k_plus_1 = R.from_matrix([[np.cos(theta_rot), 0, np.sin(theta_rot)],[0, 1, 0],[-np.sin(theta_rot), 0, np.cos(theta_rot)]])					
		
		#----- Log estimated dir -----
		
		self.log_estimated_dir.append(e)
		
		#-----------------------------
			
		lin_vdesEE_O = v*e_k_plus_1							# Desired lin EE velocity at the start of the k+1-th time step expressed in world frame 
		
		C_obj_k_plus_1_ee_k_plus_1 = R.from_matrix([[0,0,-1],[0,1,0],[1,0,0]])
		
		C_ee_k_ee_k_plus_1 = C_O_ee_k.inv() * C_O_obj_k * C_obj_k_obj_k_plus_1 * C_obj_k_plus_1_ee_k_plus_1
		
		K_ang = 1
		ang_vdesEE_ee_k = K_ang*C_ee_k_ee_k_plus_1.as_rotvec()			# Desired ang EE velocity at the start of the k+1-th time step expressed in EE frame
		ang_vdesEE_O = C_O_ee_k.apply(ang_vdesEE_ee_k)				# Desired ang EE velocity at the start of the k+1-th time step expressed in world frame
		
		#----- Log values -----
		
		self.log_lin_vdesEE_O.append(lin_vdesEE_O)
		self.log_ang_vdesEE_O.append(ang_vdesEE_O)
		self.log_theta.append(LA.norm(np.array(ang_vdesEE_O)))
		
		return lin_vdesEE_O, ang_vdesEE_O
		
#-------
	def CalculateDesiredJointVel(self, lin_vdesEE, ang_vdesEE, M_k, b_k, J_b_ee_k, q_k, q_dot_k, C_O_b_k, C_O_ee_k, minTorque = True, rf = 'O'):
		
		if rf == 'O':										# Checks whether the desired velocities were expressed in base or world frame
			lin_vdesEE_b = np.squeeze(np.array(C_O_b_k.inv().apply(lin_vdesEE)))
			ang_vdesEE_b = np.squeeze(np.array(C_O_b_k.inv().apply(ang_vdesEE)))
		else:
			lin_vdesEE_b = lin_vdesEE
			ang_vdesEE_b = ang_vdesEE
		
		vdesEE_b = np.concatenate((lin_vdesEE_b, ang_vdesEE_b), axis=0)
		
		G_task1, a_task1, C_task1, b_task1 = self.PrepareTask1(J_b_ee_k[:6, :], vdesEE_b[:6], M_k, b_k, q_k, q_dot_k)
		
		try:
			sol1,_,_,_,_,_ = solve_qp(G=G_task1, a=a_task1, C=C_task1, b=b_task1)
			self.q_dot_optimal_previous = sol1
			
		except Exception as e:
			
			print(5*'*'+' Error occured during first optimization! '+5*'*')
			print(e)
			sol1 = self.q_dot_optimal_previous
			
		Null_task1 = Null_proj(J_b_ee_k)
			
		if minTorque:
		
			try:
				G_task2, a_task2, C_task2, b_task2 = self.PrepareTask2(M_k, b_k, Null_task1, q_k, q_dot_k, sol1)		
		
				sol2,_,_,_,_,_ = solve_qp(G=G_task2, a=a_task2, C=C_task2, b=b_task2)
				
				q_dot_optimal = np.array(sol1) + np.squeeze(np.matmul(Null_task1, np.array(sol2)))
				self.q_dot_optimal_previous = np.copy(q_dot_optimal)
				
			except Exception as e:
			
				q_dot_optimal = np.array(sol1)
				print(5*'*'+' Error occured during second optimization! '+5*'*')
				print(e)
				
		else:
			q_dot_optimal = np.array(sol1)
			
		return q_dot_optimal
		
#-------
	def PrepareTask1(self, J_b_ee_k, vdesEE_b, M_k, b_k, q_k, q_dot_k):
	
		A = J_b_ee_k
		b = vdesEE_b
		
		G_task1 = np.matmul(np.transpose(A), A) + 0.0001**2*np.eye(len(q_k))
		a_task1 = np.matmul(np.transpose(A), b)
		
		C_task1 = np.transpose(np.concatenate((M_k, -M_k, np.eye(len(q_k)), -np.eye(len(q_k))), axis=0))
		
		ineq_1 = self.dt*(-self.torque_max[:len(q_k)]-b_k) + np.matmul(M_k, q_dot_k)
		ineq_2 = self.dt*(b_k-self.torque_max[:len(q_k)]) - np.matmul(M_k, q_dot_k)
		
		ineq_3 = np.maximum(self.q_dot_min[:len(q_k)], (1/self.dt)*(self.q_min[:len(q_k)] - q_k))
		ineq_4 = -np.minimum(self.q_dot_max[:len(q_k)], (1/self.dt)*(self.q_max[:len(q_k)] - q_k))
		
		b_task1 = np.concatenate((ineq_1, ineq_2, ineq_3, ineq_4), axis=0)
		
		return G_task1, a_task1, C_task1, b_task1
		
#-------
	def PrepareTask2(self, M_k, b_k, Null_task1, q_k, q_dot_k, sol1):
	
		A = np.matmul(M_k, Null_task1)
		
		eq_1 = np.squeeze(np.matmul(M_k, q_dot_k-sol1) - self.dt*b_k)
		
		b = eq_1
		
		G_task2 = np.matmul(np.transpose(A), A) + 0.0001**2*np.eye(len(q_k))
		a_task2 = np.matmul(np.transpose(A), b)
		
		C_task2 = np.transpose(np.concatenate((np.matmul(M_k, Null_task1), -np.matmul(M_k, Null_task1)), axis=0))
		
		ineq_1 = self.dt*(-self.torque_max[:len(q_k)]-b_k) + np.matmul(M_k, q_dot_k) - np.matmul(M_k, sol1)
		ineq_2 = self.dt*(b_k-self.torque_max[:len(q_k)]) - np.matmul(M_k, q_dot_k) + np.matmul(M_k, sol1)
				
		b_task2 = np.concatenate((ineq_1, ineq_2), axis=0)
		
		return G_task2, a_task2, C_task2, b_task2

#-------
	def CalculateDesiredJointVelAndBaseVel(self, v, lin_vdesEE_O, ang_vdesEE_O, M_k, b_k, J_b_ee_k, q_k, q_dot_k, C_O_b_k, C_O_ee_k, r_rob_ee, mode='NoSelfCollision', Rmax=0.2, alpha=1.0, convexApprox = False, minTorque=True):
		
		R = LA.norm(np.array(r_rob_ee))
		print("R: ", R)
		scale_coeff = 1.0 - np.exp(-alpha*(R - Rmax))
		
		lin_vdesEE_O_arm = np.array([scale_coeff*lin_vdesEE_O[0], scale_coeff*lin_vdesEE_O[1], lin_vdesEE_O[2]])
		lin_vdesEE_O_base = np.array([(1.0-scale_coeff)*lin_vdesEE_O[0], (1.0-scale_coeff)*lin_vdesEE_O[1], 0.0])
		
		ang_vdesEE_O_arm = np.array([ang_vdesEE_O[0], ang_vdesEE_O[1], ang_vdesEE_O[2]])
		ang_vdesEE_O_base = np.array([0.0, 0.0, 0.0])

		lin_vdesEE_b_arm = np.array(C_O_b_k.inv().apply(lin_vdesEE_O_arm))
		ang_vdesEE_b_arm = np.array(C_O_b_k.inv().apply(ang_vdesEE_O_arm))	
		
		lin_vdesBase_b = np.array(C_O_b_k.inv().apply(list(lin_vdesEE_O_base)))
		ang_vdesBase_b = np.array(C_O_b_k.inv().apply(list(ang_vdesEE_O_base)))			
		
		if mode == 'NoSelfCollisionAndMaxMobility':
		
			u_v, s_v, vh_v = LA.svd(J_b_ee_k[:3, :], full_matrices=False)
			u_w, s_w, vh_w = LA.svd(J_b_ee_k[3:, :], full_matrices=False)
				
			u1_lin = np.array(u_v[:, 0])
			u1_ang = np.array(u_w[:, 0])
				
			u1_lin = np.sign(np.dot(u1_lin, lin_vdesEE_b_arm))*u1_lin
			u1_ang = np.sign(np.dot(u1_ang, ang_vdesEE_b_arm))*u1_ang			
				
			x0_lin = [0.0, 0.0]
								
			opt = {'maxiter': 100, 'disp': False}
			
			arguments = (v, )
			cons = ({'type': 'ineq', 'fun':LVO_constraint1, 'args':arguments})
				
			sol_lin = minimize(LinearVelocityObjective, np.array(x0_lin), args = (u1_lin, lin_vdesEE_b_arm, scale_coeff), method='SLSQP', constraints=cons, options=opt)
				
			print("sol_lin_vel: ", sol_lin.x)
						
			lin_vdesEE_b_arm = lin_vdesEE_b_arm - np.array([scale_coeff*sol_lin.x[0], scale_coeff*sol_lin.x[1], 0.0])
			
			lin_vdesBase_b = lin_vdesBase_b + np.array([scale_coeff*sol_lin.x[0], scale_coeff*sol_lin.x[1], 0.0])

		q_dot_optimal = self.CalculateDesiredJointVel(lin_vdesEE_b_arm, ang_vdesEE_b_arm, M_k, b_k, J_b_ee_k, q_k, q_dot_k, C_O_b_k, C_O_ee_k, minTorque, 'b')		
		
		return q_dot_optimal, lin_vdesBase_b, ang_vdesBase_b
		
#-------
	def PerformOneStep(self, mode, v, observation_available=True, modeBase='NoSelfCollisionAndMaxMobility', modeForceFeedback='NoForceFeedback', target_name="cupboard", link_idx=3, grasp_id=0, 			kp_f=0.8, ki_f=0.1, gamma=2.0):
		
		start_time = time.time()
		model_type, parameters = self.sk_mID.PickModel()
		print(model_type)
		
		M_k, b_k, J_b_ee_k, q_k, q_dot_k, C_O_b_k, r_O_b_k, C_O_ee_k, r_O_ee_k, mtorq_k, lin_vEE_O, ang_vEE_O, lin_vBase_O, ang_vBase_O, f_wristframe, t_wristframe = 				self.GetMeasurements()	
		
		r_O_obj_k, quat_k, C_O_obj_k, nobj_O_k = self.CalculateGraspedObjectPosOriNormal(target_name, link_idx, grasp_id)		# The actual drawer information

		if observation_available:
			
			lin_vdesEE_O, ang_vdesEE_O = self.CalculateDesiredEEVelocity(model_type, parameters, v, C_O_b_k, C_O_ee_k, r_O_ee_k, r_O_obj_k, C_O_obj_k, nobj_O_k) 					
		else:
				
			return
					
		r_rob_ee = np.array(self.robot.convert_pos_to_robot_frame(r_O_ee_k))
		print(r_O_ee_k)
		if mode == 'PlanJointAcc':
		
			q_dot_optimal = self.CalculateDesiredJointVel(lin_vdesEE_O, ang_vdesEE_O, M_k, b_k, J_b_ee_k, q_k, q_dot_k, C_O_b_k, C_O_ee_k, minTorque=False)	
			
			q_dot_dot_optimal = (1/self.dt)*(q_dot_optimal - q_dot_k)
			
			tau = np.matmul(M_k, q_dot_dot_optimal) + b_k
			
			p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm+self.robot.joint_idx_fingers, p.TORQUE_CONTROL, forces=list(tau[:7])+[-25.0]*2)
			
		elif mode == 'PlanJointAccAndBaseVel':
		
			q_dot_optimal, lin_vdesBase_b, ang_vdesBase_b = self.CalculateDesiredJointVelAndBaseVel(v, lin_vdesEE_O, ang_vdesEE_O, M_k, b_k, J_b_ee_k, q_k, q_dot_k, C_O_b_k, C_O_ee_k, 				r_rob_ee, mode=modeBase, Rmax=0.2, alpha=2.0, convexApprox = False, minTorque=False
			)
			
			q_dot_dot_optimal = (1/self.dt)*(q_dot_optimal - q_dot_k)
			
			tau = np.matmul(M_k, q_dot_dot_optimal) + b_k
			
			self.robot.update_velocity(lin_vdesBase_b, ang_vdesBase_b[2])						 	
			self.robot.velocity_setter()
			
			p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm+self.robot.joint_idx_fingers, p.TORQUE_CONTROL, forces=list(tau[:7])+[-25.0]*2)
		
		self.UpdateBufferForModelEstimation(observation_available, "cupboard", 3, r_O_ee_k)
		stop_time = time.time()
		
		#----- Log data for plotting -----

		self.log_actual_dir.append(nobj_O_k)
		self.log_manipulability_meas.append((LA.det(np.matmul(J_b_ee_k, np.transpose(J_b_ee_k))))**0.5)
		
		self.log_lin_vEE_O_meas.append(lin_vEE_O)
		self.log_ang_vEE_O_meas.append(ang_vEE_O)
		
		self.log_lin_vBase_O_meas.append(lin_vBase_O)
		self.log_ang_vBase_O_meas.append(ang_vBase_O)
				
		self.log_q.append(q_k)
		self.log_q_dot.append(q_dot_k)
		self.log_tau.append(tau)
		self.log_exec_time.append(stop_time - start_time)
		
		self.log_q_dot_optimal.append(q_dot_optimal)
		self.log_relative_v.append(list(C_O_b_k.apply(list(np.matmul(J_b_ee_k, q_dot_k))[:3])) + list(C_O_b_k.apply(list(np.matmul(J_b_ee_k, q_dot_k))[3:])))
		
		self.log_f_wristframe.append(f_wristframe)
		self.log_t_wristframe.append(t_wristframe)
			
		#---------------------------------
		
		self.robot._world.step_one()
		self.robot._world.sleep(self.robot._world.T_s)
		
#-------
	def UpdateBufferForModelEstimation(self, observation_available, target_name="cupboard", link_idx=3, r_O_ee_k=None):
				
		obj_info = self.scene.objects[target_name]
		target_id = obj_info.model.uid
		link_id = obj_info.grasp_links[link_idx]
		
		if link_id == -1:
			temp = p.getBasePositionAndOrientation(target_id)
			actual_pos = np.array(temp[0]).reshape((-1, 1))
			
		else:
			temp = p.getLinkState(target_id, link_id, computeForwardKinematics = True)
			actual_pos = np.array(temp[4]).reshape((-1, 1))
				
		if observation_available:
			
			new_buffer_estimate = actual_pos
					
		else:
		
			new_buffer_estimate = np.array(r_O_ee_k).reshape((-1,1))	
		
		if np.linalg.norm( self.sk_mID.obj_pose_buffer[len(self.sk_mID.obj_pose_buffer)-1]- new_buffer_estimate)>EPS:
			self.sk_mID.obj_pose_buffer.append(new_buffer_estimate)
		else:
			print('Buffer update skipped!')
			
		print("Pose of the drawer: ", np.squeeze(actual_pos))
		
		#----- Log Actual Drawer Pos -----
		self.log_actual_drawer_pos.append(actual_pos)				
		
#-------
	def FreezeRobotConfiguration(self):
		
		joint_positions, joint_velocities, joint_torques = getJointStates(self.robot.model.uid)		
		base_pos, base_ori = p.getBasePositionAndOrientation(self.robot.model.uid)		
		lin_base_vel, ang_base_vel = p.getBaseVelocity(self.robot.model.uid)
		
		return joint_positions, joint_velocities, base_pos, base_ori, lin_base_vel, ang_base_vel
		
#-------
	def ApplyRobotConfiguration(self, joint_positions, joint_velocities, base_pos, base_ori, lin_base_vel, ang_base_vel):

		p.resetBasePositionAndOrientation(self.robot.model.uid, base_pos, base_ori)
		
		self.robot.update_velocity(lin_base_vel, ang_base_vel[2])						 	
		self.robot.velocity_setter()
				
		for i in range(len(joint_positions)):
			p.resetJointState(self.robot.model.uid, i, joint_positions[i], joint_velocities[i])
					
#-------
	def ObjectConfiguration(self, mode, target_name="cupboard", joint_positions=None, joint_velocities=None):
		
		obj_info = self.scene.objects[target_name]
		target_id = obj_info.model.uid
		
		num_joints = p.getNumJoints(target_id)
		
		if mode == 'memorize':
		
			joint_states = p.getJointStates(target_id, range(num_joints))
			joint_positions = [state[0] for state in joint_states]
			joint_velocities = [state[1] for state in joint_states]
			
			return joint_positions, joint_velocities
			
		else:
			
			for i in range(len(joint_positions)):
				p.resetJointState(target_id, i, joint_positions[i], joint_velocities[i])
					
							
if __name__ == "__main__":
    main()
