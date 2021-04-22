import os
import pybullet as p

import numpy as np
from datetime import datetime
from numpy import linalg as LA

EPS = 1e-6

#----- General utils -----

def prepare_global_dir():

	date_and_time = datetime.now()
	curr = date_and_time.strftime("%d_%m_%Y_%H_%M_%S")
	
	results_folder = os.getcwd()+'/moma/moma_demos/articulated_demo/runs'
	
	if not os.path.isdir(results_folder):
		os.makedirs(results_folder)
              	 		
	if not os.path.isdir(results_folder + '/' + curr):
		os.makedirs(results_folder + '/' + curr)
	
	global_folder = results_folder + '/' + curr
                	
	return global_folder
	
#-------
def prepare_dir(global_folder, postfix=None):
        
	if postfix is not None:
        
		if not os.path.isdir(global_folder + '/' + postfix):
			os.makedirs(global_folder + '/' + postfix)
		curr_folder = global_folder + '/' + postfix 
	
	else:
	       	 		
		if not os.path.isdir(global_folder):
			os.makedirs(global_folder)
		curr_folder = global_folder
                	
	return curr_folder

#----- Robot and planning related util

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
	
def Objective1(x, *args):

	vdesEE = args[0]
	vmean = args[1]
	s = args[2]
	
	return LA.norm(vdesEE - s*x) + LA.norm(vmean - x)
	
def Constraint1(x, *args):

	v = args[0]
	
	return v**2 - x[0]**2 - x[1]**2
