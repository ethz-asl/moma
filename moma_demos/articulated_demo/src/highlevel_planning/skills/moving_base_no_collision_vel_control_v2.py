import pybullet as p
import numpy as np
from numpy import linalg as LA

from math import log

from scipy.spatial.transform import Rotation as R
from highlevel_planning.tools.util import homogenous_trafo, invert_hom_trafo
from matplotlib import pyplot as plt
from scipy.linalg import lstsq
from sklearn.metrics import mean_squared_error

from highlevel_planning.tools.door_opening_util import *

from highlevel_planning.tools.util import IKError
from collections import deque

from quadprog import solve_qp

from scipy.optimize import minimize
from scipy.optimize import minimize_scalar

EPS = 1e-6
DEBUG = True

class Controller:

	def __init__(self, scene, robot, time_step, maxMobility, splitAng=False):
	
		self.scene = scene
		self.robot = robot
		self.dt = time_step
		
		
		self.maxMobility = maxMobility
		self.splitAng = splitAng
		
		if self.maxMobility:
			self.mode_name = 'moving_base_no_collision_max_mob_control_v2'
		else:
			self.mode_name = 'moving_base_no_collision_control_v2'
		
		#----- Constraints -----

		self.torque_max = []
		self.q_max = []
		self.q_min = []
		self.q_dot_max = []
		self.q_dot_min = []
		self.q_mean = []
		
		#----- Init Values -----
		
		self.q_dot_optimal = None
		self.vLinBase_b = None
		self.vAngBase_b = None
		
		self.SetupController()
						
#-------
	def SetupController(self):
	
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
	def ResetDampings(self):
	
		num_joints = p.getNumJoints(self.robot.model.uid)
		p.changeDynamics(self.robot.model.uid, -1, linearDamping=0, angularDamping=0, jointDamping=0)
		
		for i in self.robot.joint_idx_arm:
			p.changeDynamics(self.robot.model.uid, i, linearDamping=0, angularDamping=0, jointDamping=0)
		
		p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm, p.VELOCITY_CONTROL, forces = [0.0]*len(self.robot.joint_idx_arm))
		
		for i in self.robot.joint_idx_fingers:
			p.changeDynamics(self.robot.model.uid, i, linearDamping=0, angularDamping=0, jointDamping=0)
		
		p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_fingers, p.VELOCITY_CONTROL, forces = [0.0]*len(self.robot.joint_idx_fingers))		
#-------
	def PrepareTask1(self, J_b_ee, vdesEE_b, M, b, q, q_dot):
	
		A = J_b_ee
		y = vdesEE_b
		
		G1 = np.matmul(np.transpose(A), A) + 0.0001**2*np.eye(len(q))
		a1 = np.matmul(np.transpose(A), y)		
		
		C1 = np.transpose(np.concatenate((M, -M, np.eye(len(q)), -np.eye(len(q))), axis=0))
		
		ineq1 = self.dt * (-self.torque_max[:len(q)] - b) + np.matmul(M, q_dot)
		ineq2 = self.dt * (b - self.torque_max[:len(q)]) - np.matmul(M, q_dot)
		ineq3 = np.maximum(self.q_dot_min[:len(q)], (1/self.dt)*(self.q_min[:len(q)] - q))
		ineq4 = -np.minimum(self.q_dot_max[:len(q)], (1/self.dt)*(self.q_max[:len(q)] - q))
		
		b1 = np.concatenate((ineq1, ineq2, ineq3, ineq4), axis=0)
		
		return G1, a1, C1, b1
		
#-------
	def PrepareTask2(self, M, b, q, q_dot, sol1, Null1):
	
		A = np.matmul(M, Null1)
		y = np.matmul(M, q_dot - sol1) - self.dt * b
		
		G2 = np.matmul(np.transpose(A), A) + 0.0001**2*np.eye(len(q))
		a2 = np.matmul(np.transpose(A), y)
		
		C2 = np.transpose(np.concatenate((np.matmul(M, Null1), -np.matmul(M, Null1)), axis=0))
		
		ineq1 = self.dt * (-self.torque_max[:len(q)] - b) - np.matmul(M, sol1 - q_dot)
		ineq2 = self.dt * (b - self.torque_max[:len(q)]) + np.matmul(M, sol1 - q_dot)
		
		b2 = np.concatenate((ineq1, ineq2), axis=0)
		
		return G2, a2, C2, b2
		
#-------
	def CalculateDesiredJointVel(self, veldesEE_ee, J_b_ee, M, b, q, q_dot, C_O_b, C_O_ee, minTorque):
	
		vLindesEE_ee = np.squeeze(veldesEE_ee[:3])
		vAngdesEE_ee = np.squeeze(veldesEE_ee[3:])
		
		vLindesEE_b = np.squeeze(np.array((C_O_b.inv() * C_O_ee).apply(vLindesEE_ee)))
		vAngdesEE_b = np.squeeze(np.array((C_O_b.inv() * C_O_ee).apply(vAngdesEE_ee)))
		
		vdesEE_b = np.concatenate((vLindesEE_b, vAngdesEE_b), axis=0)
		
		G1, a1, C1, b1 = self.PrepareTask1(J_b_ee, vdesEE_b[:J_b_ee.shape[0]], M, b, q, q_dot)
		
		sol1,_,_,_,_,_ = solve_qp(G1, a1, C1, b1)
		
		q_dot_optimal = np.array(sol1)
		
		if minTorque:
		
			try:
				Null1 = NullProjection(J_b_ee)
				G2, a2, C2, b2 = self.PrepareTask2(M, b, q, q_dot, sol1, Null1)
				
				sol2,_,_,_,_,_ = solve_qp(G2, a2, C2, b2)
				q_dot_optimal += np.squeeze(np.matmul(Null1, np.array(sol2))) 
			
			except Exception as e:
				
				print("FAILED TORQUE MINIMIZATION")
				print(e)
			
		return np.squeeze(q_dot_optimal)
		
#-------
	def SplitVelocity(self, veldesEE_ee, J_b_ee, C_O_b, C_O_ee, r_O_ee, v, q, Rlim=0.1, alpha=2.0):

		vLindesEE_O = np.array(C_O_ee.apply(veldesEE_ee[:3]))
		vAngdesEE_O = np.array(C_O_ee.apply(veldesEE_ee[3:]))
		
		vLindesEE_b = C_O_b.inv().apply(vLindesEE_O)
		vAngdesEE_b = C_O_b.inv().apply(vAngdesEE_O)		
		
		r_rob_ee = np.array(self.robot.convert_pos_to_robot_frame(r_O_ee))
		R = LA.norm(r_rob_ee[:2]) #
		
		scaleFactor = np.sign(R - Rlim)*(1.0 - np.exp(-alpha*np.abs(R - Rlim)))
		print("R: ", R)
		
		q_dot_des = []
		gamma = 0.1
		
		for i in range(len(self.q_mean)-2):
			
			q_k = q[i]
			q_mean = self.q_mean[i]
			sign = np.sign(q_mean - q_k)
			
			if sign<0:
				
				q_dot_abs = min([abs(self.q_dot_min[i]), gamma*(1/self.dt)*abs(q_mean - q_k)])
				
			else:
				
				q_dot_abs = min([abs(self.q_dot_max[i]), gamma*(1/self.dt)*abs(q_mean - q_k)])

				
			q_dot_des.append(sign*q_dot_abs)
			
		q_dot_des = np.array(q_dot_des)
		
		vdes = np.matmul(J_b_ee[:2, :7], q_dot_des)
		
		if abs(scaleFactor)>0:
			
			s = abs(scaleFactor)
			vdes = 1/scaleFactor * vdes
			
			A = np.eye(2)
			y = np.copy(vdes)
			
			G = np.eye(2)
			a = y
			
			C = np.transpose(np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]]))
			b = np.array([-2*v, -2*v, -2*v, -2*v])
			
			sol,_,_,_,_,_ = solve_qp(G, a, C, b)
			print("Sol: ", sol)
			
			vLinEE_b = np.array([scaleFactor*sol[0], scaleFactor*sol[1], vLindesEE_b[2]])

			vLinEE_ee = np.array((C_O_ee.inv() * C_O_b).apply(vLinEE_b))
			vAngEE_ee = np.array((C_O_ee.inv() * C_O_b).apply(vAngdesEE_b)) 			
			vEE_ee = np.concatenate((vLinEE_ee, vAngEE_ee), axis=0)
					
			vLinBase_b = vLindesEE_b - vLinEE_b			
			vAngBase_b = np.array([0.0]*3)
						
		else:
			vLinEE_b = np.array([0.0, 0.0, vLindesEE_b[2]])
			
			vLinEE_ee = np.array((C_O_ee.inv() * C_O_b).apply(vLinEE_b))
			vAngEE_ee = np.array((C_O_ee.inv() * C_O_b).apply(vAngdesEE_b)) 			
			vEE_ee = np.concatenate((vLinEE_ee, vAngEE_ee), axis=0)
						
			vLinBase_b = vLindesEE_b - vLinEE_b			
			vAngBase_b = np.array([0.0]*3)
				
		return vLinBase_b, vAngBase_b, vEE_ee				
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
		r_O_ee = infoTuple[7]
		v = infoTuple[8]
		
		vLinBase_b, vAngBase_b, vEE_ee = self.SplitVelocity(veldesEE_ee, J_b_ee, C_O_b, C_O_ee, r_O_ee, v, q)
		
		self.q_dot_optimal = self.CalculateDesiredJointVel(vEE_ee, J_b_ee, M, b, q, q_dot, C_O_b, C_O_ee, False)
		
		p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_fingers, p.TORQUE_CONTROL, forces=[-25.0]*2)
		
		q_dot_dot_optimal = (1/self.dt)*(self.q_dot_optimal - q_dot)
		
		tau = np.matmul(M, q_dot_dot_optimal) + b 
		
		#p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm, p.TORQUE_CONTROL, forces=list(tau[:7]))
		
		p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm, p.VELOCITY_CONTROL, targetVelocities=list(self.q_dot_optimal[:7]))
		self.robot.update_velocity(vLinBase_b, vAngBase_b[2])						 	
		self.robot.velocity_setter()		
		
		self.robot._world.step_one()
		self.robot._world.sleep(self.robot._world.T_s)
		
				
		
		
	
		
		
		
		
		 
		
		
		
		
		
		
		
						 
						 
				
		
