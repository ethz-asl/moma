import pybullet as p
import numpy as np
import time

from numpy import linalg as LA

from math import log
import math

from collections import deque



EPS = 1e-10
DEBUG = True

def ortho_projection(direction):

	assert np.abs(np.linalg.norm(direction) - 1.0) < EPS
	projection = np.matmul(direction, direction.T)
	
	return np.eye(3) - projection

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

def objective_velocity_planning(x, *args):
   
	u1 = args[0]
	u1 = u1/LA.norm(u1)
	x_dot_des = args[1]
	c = args[2]
  
	A = np.diag([x[0], x[1], 1, 1, 1, x[2]])
	v = np.matmul(A, x_dot_des)
	v = v/LA.norm(v)
		
	return -np.abs(np.dot(v, u1))-c*LA.norm(x)

def calculate_objective_A_b_t1(M, J_O_ee, drift, F_meas_O):

	A = np.concatenate((M, -np.eye(7)), axis=1)
	b = -drift+np.matmul(-J_O_ee.transpose(), F_meas_O)
		
	return A, b	
	
def objective_t1(x, *args):
	
	A = args[0]
	b = args[1]

	v = np.matmul(A, x)
	
	return (LA.norm(v-b))**2   
	
def calculate_constraint_A_b_t1(n_joints, tau_max, tau_min, dt, joint_position_max, joint_position_min, joint_velocity_max, joint_velocity_min, curr_joint_position, curr_joint_velocity):

	A1 = np.concatenate((np.zeros([7,7]), np.eye(7)), axis=1)
	A2 = np.concatenate((np.zeros([7,7]), -np.eye(7)), axis=1)
	A3 = np.concatenate((np.eye(n_joints), np.zeros([n_joints,7])), axis=1)
	A4 = np.copy(A3)
	A5 = np.concatenate((-np.eye(n_joints), np.zeros([n_joints,7])), axis=1)
	A6 = np.copy(A5)
	
	A_constraint = np.concatenate((A1,A2,A3,A4,A5,A6), axis=0)
	
	b1 = tau_max
	b2 = -tau_min
	b3 = (1.0/dt)*(joint_velocity_max-curr_joint_velocity)
	b4 = (2.0/dt/dt)*(joint_position_max-curr_joint_position-dt*curr_joint_velocity)
	b5 = (-1.0/dt)*(joint_velocity_min-curr_joint_velocity)
	b6 = (-2.0/dt/dt)*(joint_position_min-curr_joint_position-dt*curr_joint_velocity)
	
	b_constraint = np.concatenate((b1,b2,b3,b4,b5,b6), axis=0)
	
	return A_constraint, b_constraint
 
def ineq_constraint_t1(x, *args):
	
	A_constraint = args[0]
	b_constraint = args[1]
	v = np.matmul(A_constraint, x)
		
	return b_constraint-v
	
def calculate_objective_A_b_t2(J_curr_O_ee, w_des_O_ee, w_curr_O_ee, dJdt_O_ee, curr_joint_velocity, dt):

	A = np.concatenate((dt*np.array(J_curr_O_ee), np.zeros([6,7])), axis=1)
	b = w_des_O_ee-w_curr_O_ee-np.matmul(dt*np.array(dJdt_O_ee), curr_joint_velocity)
	
	return A, b
		
def objective_t2(x, *args):
	
	A = args[0]
	b = args[1]
	
	task_1_sol = args[2]
	task_1_null_space = args[3]
	
	v = np.matmul(A, task_1_sol + np.matmul(task_1_null_space, x))
	
	return (LA.norm(v-b))**2
	
def ineq_constraint_t2(x, *args):

	A_constraint_task_1 = args[0]
	b_constraint_task_1 = args[1]
	task_1_sol = args[2]
	task_1_null_space = args[3]
	
	v = np.matmul(A_constraint_task_1, task_1_sol + np.matmul(task_1_null_space, x))
	return b_constraint_task_1 - v
	
def calculate_objective_A_b_t3(q_mean, dt, curr_joint_position, curr_joint_velocity, mode):

	A1 = np.concatenate((np.eye(7), np.zeros([7,7])), axis=1)
	A2 = np.concatenate((np.zeros([7,7]), np.eye(7)), axis=1)
	A = np.concatenate((A1, A2), axis=0)
	
	if mode == 'min_acc':	
		b1 = np.zeros(7)	
	else:
		b1 =  (2.0/dt/dt)*(q_mean-curr_joint_position-dt*curr_joint_velocity)
	
	b2 = np.zeros(7)	
	
	b = np.concatenate((b1, b2), axis=0)
	
	return A, b
	
	
def objective_t3(x, *args):

	A = args[0]
	b = args[1]
	
	task_1_sol = args[2]
	task_1_null_space = args[3]
	task_2_sol = args[4]
	task_1_2_null_space = args[5]
		
	v = np.matmul(A, task_1_sol + np.matmul(task_1_null_space, task_2_sol) + np.matmul(task_1_2_null_space, x))
	
	return (LA.norm(v-b))**2
	
def ineq_constraint_t3(x, *args):

	A_constraint_task_1 = args[0]
	b_constraint_task_1 = args[1]
	
	task_1_sol = args[2]
	task_2_sol = args[3]
	task_1_null_space = args[4]
	task_1_2_null_space = args[5]
	
	A = np.copy(A_constraint_task_1)
	b = np.copy(b_constraint_task_1)
	
	v = np.matmul(A, task_1_sol + np.matmul(task_1_null_space, task_2_sol) + np.matmul(task_1_2_null_space, x))
	
	return b - v
	
def Null_proj(A):
	
	return np.eye(A.shape[1])-np.matmul(LA.pinv(A), A) 
