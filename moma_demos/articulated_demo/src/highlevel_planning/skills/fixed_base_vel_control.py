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

EPS = 1e-6
DEBUG = True

class Controller:

	def __init__(self, scene, robot, time_step):
	
		self.scene = scene
		self.robot = robot
		self.dt = time_step
		
		self.mode_name = 'fixed_base_velocity_control'
		
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
		
		for i in self.robot.joint_idx_fingers:
			p.changeDynamics(self.robot.model.uid, i, linearDamping=0, angularDamping=0, jointDamping=0)
		
		p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_fingers, p.VELOCITY_CONTROL, forces = [0.0]*len(self.robot.joint_idx_fingers))
		
#-------
	def GetCurrOptSol(self):
	
		return self.q_dot_optimal		
		
#-------
	def PerformOneStep(self, veldesEE_ee, infoTuple=None):
		
		self.q_dot_optimal = infoTuple[4]
		
		p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_fingers, p.TORQUE_CONTROL, forces=[-25.0]*2)
		
		vLindesEE_ee = np.squeeze(veldesEE_ee[:3])
		vAngdesEE_ee = np.squeeze(veldesEE_ee[3:])
		
		self.robot.task_space_velocity_control(vLindesEE_ee, vAngdesEE_ee, 1)
		

		
		
	
		
		
		
		
		 
		
		
		
		
		
		
		
						 
						 
				
		
