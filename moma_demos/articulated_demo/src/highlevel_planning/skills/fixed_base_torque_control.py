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

EPS = 1e-6
DEBUG = True

class Controller:

	def __init__(self, scene, robot, time_step):
	
		self.scene = scene
		self.robot = robot
		self.dt = time_step
		
		self.mode_name = 'fixed_base_torque_control'
		
		#----- Constraints -----

		self.torque_max = []
		self.q_max = []
		self.q_min = []
		self.q_dot_max = []
		self.q_dot_min = []
		self.q_mean = []
		
		#----- Init Values -----
		
		self.q_dot_optimal = None
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
		
		#for i in self.robot.joint_idx_arm:
			#p.changeDynamics(self.robot.model.uid, i, linearDamping=0, angularDamping=0, jointDamping=0)
		
		#p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm, p.VELOCITY_CONTROL, forces = [0.0]*len(self.robot.joint_idx_arm))
		
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
		
		self.q_dot_optimal = self.CalculateDesiredJointVel(veldesEE_ee, J_b_ee, M, b, q, q_dot, C_O_b, C_O_ee, False)
		
		p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_fingers, p.TORQUE_CONTROL, forces=[-25.0]*2)
		
		q_dot_dot_optimal = (1/self.dt)*(self.q_dot_optimal - q_dot)
		
		tau = np.matmul(M, q_dot_dot_optimal) + b 
		print(tau)
		#p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm, p.TORQUE_CONTROL, forces=list(tau[:7]))
		
		p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm, p.VELOCITY_CONTROL, targetVelocities=list(self.q_dot_optimal[:7]))
		
		self.robot._world.step_one()
		self.robot._world.sleep(self.robot._world.T_s)
		
				
		
		
	
		
		
		
		
		 
		
		
		
		
		
		
		
						 
						 
				
		
