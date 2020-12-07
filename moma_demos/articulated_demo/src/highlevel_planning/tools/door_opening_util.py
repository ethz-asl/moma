import argparse
import os
import pybullet as p
import pickle
import numpy as np
from datetime import datetime
from numpy import linalg as LA

EPS = 1e-6

def OrthoProjection(direction):
	
	assert np.abs(np.linalg.norm(direction) - 1.0) < EPS
	projection = np.matmul(direction, direction.T)
	
	return np.eye(3) - projection
	
def NullProjection(A):
	
	return np.eye(A.shape[1])-np.matmul(LA.pinv(A), A) 

def GetJointStates(robot):

	joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
	joint_positions = [state[0] for state in joint_states]
	joint_velocities = [state[1] for state in joint_states]
	joint_torques = [state[3] for state in joint_states]
	
	return joint_positions, joint_velocities, joint_torques


def GetMotorJointStates(robot):

	joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
	joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
	joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
	joint_positions = [state[0] for state in joint_states]
	joint_velocities = [state[1] for state in joint_states]
	joint_torques = [state[3] for state in joint_states]
	
	return joint_positions, joint_velocities, joint_torques
	
#----- Optimization objectives and constraints -----

def LinearVelocityObjective(x, *args):

	u1_lin = args[0]
	lin_vdesEE_b_arm = args[1]
	scale_coeff = args[2]
	
	temp = np.array([scale_coeff*(lin_vdesEE_b_arm[0] - x[0]), scale_coeff*(lin_vdesEE_b_arm[1] - x[1]), lin_vdesEE_b_arm[2]])
	temp = temp/LA.norm(temp)
	
	u1_lin = u1_lin/LA.norm(u1_lin)
	
	return -np.dot(temp, u1_lin)
	
def LVconstraint1(x, *args):
	
	v = args[0]
	
	return v**2 - x[0]**2 - x[1]**2
	
def AngularVelocityObjective(x, *args):

	u1_ang = args[0]
	ang_vdesEE_b_arm = args[1]
	
	temp = np.array([ang_vdesEE_b_arm[0], ang_vdesEE_b_arm[1], ang_vdesEE_b_arm[2] - x])
	temp = temp/LA.norm(temp)
	
	return -np.dot(temp, u1_ang)
	
def O1(x, *args):

	vdesEE = args[0]
	vmean = args[1]
	s = args[2]
	
	return LA.norm(vdesEE - s*x) + LA.norm(vmean - x)
	
def C1(x, *args):

	v = args[0]
	
	return v**2 - x[0]**2 - x[1]**2
