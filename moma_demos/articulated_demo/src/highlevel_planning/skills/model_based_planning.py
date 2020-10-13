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
	
def calculate_objective_A_b_t2(task_1_sol, task_1_null_space, J_curr_O_ee, w_des_O_ee, w_curr_O_ee, dJdt_O_ee, curr_joint_velocity, dt):

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
	
def objective_t3(x, *args):

	task_1_sol = args[0]
	task_1_null_space = args[1]
	task_2_sol = args[2]
	task_1_2_null_space = args[3]
	
	q_mean = args[4]
	
	A1 = np.concatenate((np.eye(7), np.zeros([7,7])), axis=1)
	A2 = np.concatenate((np.zeros([7,7]), np.eye(7)), axis=1)
	A = np.concatenate((A1, A2), axis=0)
	
	b = np.concatenate((q_mean, np.zeros(7)), axis=0)
	
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

class SkillTrajectoryPlanning:

	def __init__(self, scene_, robot_, sk_mID_, time_step):
	
		self.scene = scene_
		self.robot = robot_
		self.dt = time_step
		self.sk_mID = sk_mID_
		
		num_joints = p.getNumJoints(self.robot.model.uid)
		
		self.torque_max = []
		self.q_max = []
		self.q_min = []
		self.q_dot_max = []
		self.q_mean_value = []
		
		print("***** Robot joint information *****")
		
		for i in range(num_joints):
		
			info = p.getJointInfo(self.robot.model.uid, i)
			joint_name = info[1] if type(info[1]) is str else info[1].decode("utf-8")
			print("i: ", i, " | name: ", joint_name, " | type: ", info[2], " | parentIndex: ", info[16], " | linkName: ", info[12], " | min and max: ", info[8], info[9], info[10], info[11])
			if i in self.robot.joint_idx_arm or i in self.robot.joint_idx_fingers:
					
				self.torque_max.append(info[10])
				self.q_max.append(info[9])
				self.q_min.append(info[8])
				self.q_dot_max.append(info[11])
					
		self.torque_max = np.array(self.torque_max)
		self.q_max = np.array(self.q_max)
		self.q_min = np.array(self.q_min)
		self.q_dot_max = np.array(self.q_dot_max)
		self.q_dot_min = -np.copy(self.q_dot_max)
		self.q_mean = 0.5*(self.q_max+self.q_min)	
		
		self.pinocchio_model = pin.buildModelFromUrdf(self.robot.urdf_path)
		self.pinocchio_data = self.pinocchio_model.createData()
		
		self.previous_J_O_ee = None
		
		
		self.log_theta = []			# Angle between z axis of the ee and object normal
		self.log_x_dot_des_rob = []		# Velocity component from model ID
		self.log_x_dot_compl_rob = []		# Velocity component from force reading
		self.log_x_dot_rob = []		# Total ee velocity
		
		self.log_optimal_a = []	
		
		self.log_x_dot_arm_rob = []		# Part of the velocity to be achieved by robot arm
		self.log_x_dot_base_rob = []		# Part to be achieved by robot base
		self.log_q = []			# Arm joint positions 
		self.log_q_dot = []			# Arm joint velocities

		self.log_v_O_ee = []			# EE trans vel in world frame commanded to the arm 
		self.log_w_O_ee = []			# EE rot vel in world frame commanded to the arm 
		self.log_v_meas_O_ee = []		# Measured EE velocity in world frame
		self.log_w_meas_O_ee = []		# Measured EE rot vel in world frame
		
		self.log_exec_time = []		# Execution time of each planing+control step
		self.log_tau = []
		
				
		#curr_q = []
		#for i in self.robot.joint_idx_arm+self.robot.joint_idx_fingers:
		#	joint_pos,_,_,_ = p.getJointState(self.robot.model.uid, i)
		#	curr_q.append(joint_pos)				
		#self.examine_robot_in_pinnochio(curr_q)		
		#M_p = p.calculateMassMatrix(self.robot.model.uid, curr_q)
		#M_p = np.array(M_p)
		#print("M_p: ", M_p[6:15, 6:15])

		#pos, vel, torq = getJointStates(self.robot.model.uid)
		#mpos, mvel, mtorq = getMotorJointStates(self.robot.model.uid)
		#print(mpos)
		#print(mvel)
		#print(mtorq)
		#zero_vec = [0.1]*len(mpos)
						
		#list_of_forces = p.calculateInverseDynamics(self.robot.model.uid, list(mpos), list(mvel), list(zero_vec))
		#print("h+g: ", list_of_forces)		
				
	def get_grasped_obj_pos_and_ori(self, target_name=None, link_idx=None, grasp_id=0):
		
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
			ori_obj = R.from_quat(np.array(temp[1]))
		else:
			temp = p.getLinkState(target_id, link_id, computeForwardKinematics = True)
			pos_obj = np.array(temp[4]).reshape((-1, 1))
			ori_obj = R.from_quat(np.array(temp[5]))
			
		return pos_obj, ori_obj	
		
	def calc_grasped_obj_plane_normal(self, target_name=None, link_idx=None, grasp_id=0):
		
		pos_obj, ori_obj = self.get_grasped_obj_pos_and_ori(target_name, link_idx, grasp_id)	
		C_O_obj = ori_obj.as_matrix()
		
		return np.array(C_O_obj[:,0])		#This is true if the object's x axis is aligned with the object's plane normal!		
		
	def calculte_desired_velocity(self, v, observation_available=True, target_name=None, link_idx=None, B=np.diag([0.0,0.0,0.0, 0.0,0.0,0.0]), grasp_id=0):
		
		model_type, parameters = self.sk_mID.PickModel()
		
		if observation_available:
			
			pos_obj, ori_obj = self.get_grasped_obj_pos_and_ori(target_name, link_idx, grasp_id)
			
		else:
		
			pos_obj = None
			ori_obj = None		# To be updated		
		
		if model_type == 'prismatic':
			
			e = parameters[0]
			C = parameters[1]
		else:
		
			n = parameters[0]
			C = parameters[1]
			r = parameters[2]
			
			e = np.cross(np.squeeze(pos_obj)-C, n)
			e = e/LA.norm(e)
			
		v_des = v*e 
		r_des = pos_obj + v_des*self.dt
		
		link_poses = p.getLinkStates(self.robot.model.uid, linkIndices=[self.robot.arm_base_link_idx, self.robot.link_name_to_index["panda_hand"]], computeForwardKinematics = True)
		
		C_O_rob = R.from_quat(link_poses[0][5])
		C_O_ee = R.from_quat(link_poses[1][5])
		
		C_O_ee_mat = C_O_ee.as_matrix()
		
		w = 0.2
		theta_des = 0
		
		n_obj_O = self.calc_grasped_obj_plane_normal(target_name, link_idx, grasp_id)
		n_w_des_O = np.cross(np.squeeze(C_O_ee_mat[:,2]), -n_obj_O)
		
		theta = np.arccos(np.dot(np.squeeze(C_O_ee_mat[:,2]), -n_obj_O))
		
		n_w_des_O = n_w_des_O/LA.norm(n_w_des_O)
		w_des_O = 0.2*(theta-theta_des)*n_w_des_O
		
		v_des_rob = C_O_rob.inv().apply(v_des) #v_des in the robot body frame
		v_des_rob = np.array(v_des_rob)
		w_des_rob = C_O_rob.inv().apply(np.array(w_des_O ))
		w_des_rob = np.array(w_des_rob)
		x_dot_des_rob = np.concatenate((v_des_rob, w_des_rob), axis=0)
		
		force, torque = self.robot.get_wrist_force_torque()
		force = force/LA.norm(force)
		torque = torque/LA.norm(torque)
		F_ee = np.concatenate((force, torque), axis=0)
		x_dot_compl_ee = np.matmul(B, F_ee)
		
		x_dot_compl_rob = np.concatenate((C_O_rob.inv().apply(C_O_ee.apply(x_dot_compl_ee[0:3])), C_O_rob.inv().apply(C_O_ee.apply(x_dot_compl_ee[3:6]))), axis=0)
		x_dot_compl_rob = np.array(x_dot_compl_rob)
		
		x_dot_rob = x_dot_des_rob+x_dot_compl_rob
		
		self.log_theta.append(theta)
		self.log_x_dot_des_rob.append(x_dot_des_rob)
		self.log_x_dot_compl_rob.append(x_dot_compl_rob)
		self.log_x_dot_rob.append(x_dot_rob)
		
		return x_dot_rob 
			
	def one_step_optimize_velocity(self, a0, x_dot_des, c, u1, a_min, a_max):
		
		b = (a_min, a_max)
		bnds = (b,b,b)
		sol = minimize(objective_velocity_planning, np.array(a0), args = (np.array(u1), np.array(x_dot_des), c), method = 'SLSQP', bounds=bnds)
		
		a = sol.x
		print("optimal a: ", a)
		A_arm = np.diag([a[0], a[1], 1, 1, 1, a[2]])
		A_base = np.eye(6)-A_arm
		x_dot_arm_rob = np.matmul(A_arm, x_dot_des)
		x_dot_base_rob = np.matmul(A_base, x_dot_des)
		
		self.log_optimal_a.append(a)
		
		return x_dot_arm_rob, x_dot_base_rob
		
	def split_arm_and_base_velocity(self, v, observation_available_= True, target_name_="cupboard", link_idx_=3):
		
		pos, vel, torq = getJointStates(self.robot.model.uid)
		mpos, mvel, mtorq = getMotorJointStates(self.robot.model.uid)
		zero_vec = [0.0]*len(mpos)
		
		lin, ang = p.calculateJacobian(self.robot.model.uid, self.robot.arm_ee_link_idx, [0.0, 0.0, 0.0], mpos, zero_vec, zero_vec)
		lin = np.array(lin)
		ang = np.array(ang)
	
		link_poses = p.getLinkStates(self.robot.model.uid, linkIndices=[self.robot.arm_base_link_idx, self.robot.link_name_to_index["panda_hand"]], computeForwardKinematics = True)		
		C_O_rob = R.from_quat(link_poses[0][5])
		C_O_ee = R.from_quat(link_poses[1][5])
		
		C_rob_O = C_O_rob.inv()
		C_rob_O = C_rob_O.as_matrix()
				
		lin_rob = np.matmul(C_rob_O, lin)
		ang_rob = np.matmul(C_rob_O, ang)
		
		J_rob = np.concatenate((lin_rob, ang_rob), axis=0)
		J_rob = J_rob[:, 6:13]
		
		u, s, vh = LA.svd(J_rob,  full_matrices=False)
		
		a_min = 0
		a_max = 1
		c = 0.01
		
		a0 = [1, 1, 1]
		u1 = u[:, 0]

		x_dot_des = self.calculte_desired_velocity(v, observation_available_, target_name_, link_idx_, np.diag([0.001,0.001,0.001,0.0,0.0,0.0]), 0)
		x_dot_arm_rob, x_dot_base_rob = self.one_step_optimize_velocity(a0, x_dot_des, c, u1, a_min, a_max)

		self.log_x_dot_arm_rob.append(x_dot_arm_rob)
		self.log_x_dot_base_rob.append(x_dot_base_rob)
		self.log_q.append(mpos)
		self.log_q_dot.append(mvel)
				
		return x_dot_arm_rob, x_dot_base_rob
		
	def perform_one_step_velocity_control(self, v, observation_available_= True, target_name_="cupboard", link_idx_=3):
		
		link_poses = p.getLinkStates(self.robot.model.uid, linkIndices=[self.robot.arm_base_link_idx, self.robot.link_name_to_index["panda_hand"]], computeForwardKinematics = True, computeLinkVelocity=1)		
		C_O_rob = R.from_quat(link_poses[0][5])
		C_O_ee = R.from_quat(link_poses[1][5])
		
		C_rob_O = C_O_rob.inv()
		C_rob_O = C_rob_O.as_matrix()
		
		#------------ Whole body control ----------------------------------
		
		start_time = time.time()
		
		x_dot_arm_rob, x_dot_base_rob = self.split_arm_and_base_velocity(v, observation_available_, target_name_, link_idx_)
		self.robot.update_velocity(np.squeeze(x_dot_base_rob[0:3]), x_dot_base_rob[5])		
		self.robot.velocity_setter()					
		
		#------------- Fixed base arm control ----------------------------------
		
		#x_dot_arm_rob = np.concatenate((C_O_rob.apply(x_dot_des[:3]), C_O_rob.apply(x_dot_des[3:6])), axis=0)

		#----------------------------------------------	
			
		velocity_translation_rob = x_dot_arm_rob[:3]
		velocity_rotation_rob = x_dot_arm_rob[3:6]
		
		velocity_translation_O = C_O_rob.apply(velocity_translation_rob)
		velocity_rotation_O = C_O_rob.apply(velocity_rotation_rob)
		velocity_translation_O = np.array(velocity_translation_O)
		velocity_rotation_O = np.array(velocity_rotation_O)
		
		self.log_v_O_ee.append(velocity_translation_O)
		self.log_w_O_ee.append(velocity_rotation_O)
		self.log_v_meas_O_ee.append(link_poses[1][6])
		self.log_w_meas_O_ee.append(link_poses[1][7])
		
		self.robot.task_space_velocity_control(np.squeeze(velocity_translation_O), np.squeeze(velocity_rotation_O), 1)
		
		stop_time = time.time()
		self.log_exec_time.append(stop_time-start_time)
		
		obj_info = self.scene.objects["cupboard"]
		target_id = obj_info.model.uid
		link_id = obj_info.grasp_links[3]
		
		if link_id == -1:
			temp = p.getBasePositionAndOrientation(target_id)
			target_pos = np.array(temp[0]).reshape((-1, 1))
			target_ori = R.from_quat(np.array(temp[1]))
		else:
			temp = p.getLinkState(target_id, link_id, computeForwardKinematics = True)
			target_pos = np.array(temp[4]).reshape((-1, 1))
			target_ori = R.from_quat(np.array(temp[5]))
		if np.linalg.norm( self.sk_mID.obj_pose_buffer[len(self.sk_mID.obj_pose_buffer)-1]- target_pos)>EPS:
			self.sk_mID.obj_pose_buffer.append(target_pos)
			
		print("Pose of the drawer: ", np.squeeze(target_pos))
		
	
	def get_arm_mass_matrix_jacobian_drift(self, q, q_dot, q_dot_dot):
		
		q = np.array(q)
		q_dot = np.array(q_dot)
		q_dot_dot = np.array(q_dot_dot)
		
		zero_vec = [0.0]*9
		
		M = pin.crba(self.pinocchio_model, self.pinocchio_data, q)
		b = pin.rnea(self.pinocchio_model, self.pinocchio_data, q, q_dot, q_dot_dot)
		
		M = np.array(M)
		b = np.array(b)
		
		lin, ang = p.calculateJacobian(self.robot.model.uid, self.robot.arm_ee_link_idx, [0.0, 0.0, 0.0], list(q), zero_vec, zero_vec)
		lin = np.array(lin)
		ang = np.array(ang)

		J_O_ee = np.concatenate((lin, ang), axis=0)
		J_O_ee = J_O_ee[:, 6:13]
		
		return M[0:7, 0:7], J_O_ee, b[:7]
		
	def one_step_optimize_torques(self, v, x_dot_arm_rob, x_dot_base_rob, observation_available_, target_name_, link_idx_):
		
		mpos, mvel, mtorq = getMotorJointStates(self.robot.model.uid)
		zero_vec = [0.0]*len(mpos)
		
		M, J_O_ee, drift = self.get_arm_mass_matrix_jacobian_drift(mpos, mvel, zero_vec)
		
		link_poses_and_vel = p.getLinkStates(self.robot.model.uid, linkIndices=[self.robot.arm_base_link_idx, self.robot.link_name_to_index["panda_hand"]], computeLinkVelocity=1, computeForwardKinematics = True)
				
		C_O_rob = R.from_quat(link_poses_and_vel[0][5])
		C_O_ee = R.from_quat(link_poses_and_vel[1][5])
		
		C_rob_O = C_O_rob.inv()
		C_rob_O = C_rob_O.as_matrix()
		
		C_O_ee = C_O_ee.as_matrix()
		
		#x_dot_arm_rob, x_dot_base_rob = self.split_arm_and_base_velocity(v, observation_available_, target_name_, link_idx_)

		velocity_translation_rob = x_dot_arm_rob[:3]
		velocity_rotation_rob = x_dot_arm_rob[3:6]
		
		velocity_translation_O = C_O_rob.apply(velocity_translation_rob)
		velocity_rotation_O = C_O_rob.apply(velocity_rotation_rob)
		velocity_translation_O = np.array(velocity_translation_O)
		velocity_rotation_O = np.array(velocity_rotation_O)

		self.log_v_O_ee.append(velocity_translation_O)
		self.log_w_O_ee.append(velocity_rotation_O)
		self.log_v_meas_O_ee.append(link_poses_and_vel[1][6])
		self.log_w_meas_O_ee.append(link_poses_and_vel[1][7])
				
		w_des_O_ee = np.concatenate((velocity_translation_O, velocity_rotation_O), axis=0)
		
		curr_v_O_ee = link_poses_and_vel[1][6]
		curr_w_O_ee = link_poses_and_vel[1][7]
		
		w_curr_O_ee = np.concatenate((np.array(curr_v_O_ee), np.array(curr_w_O_ee)), axis=0)		
		dt = self.robot._world.T_s
		
		dJdt_O_ee = (1/dt)*(J_O_ee - self.previous_J_O_ee)
		self.previous_J_O_ee = np.copy(J_O_ee)	
		
		force, torque = self.robot.get_wrist_force_torque()
		force_O = np.matmul(C_O_ee, force)
		torque_O = np.matmul(C_O_ee, torque)
		
		F_meas_O = np.concatenate((force_O, torque_O), axis=0)
		
		A_t1, b_t1 = calculate_objective_A_b_t1(M, J_O_ee, drift, F_meas_O)
		A_constraint_t1, b_constraint_t1 = calculate_constraint_A_b_t1(7, self.torque_max[:7], -self.torque_max[:7], dt, self.q_max[:7], self.q_min[:7], self.q_dot_max[:7], self.q_dot_min[:7], np.array(mpos[:7]), np.array(mvel[:7]))
		
		con_t1_1 = {'type': 'ineq', 'fun': ineq_constraint_t1, 'args': (A_constraint_t1, b_constraint_t1)}
		cons_t1 = [con_t1_1]
		
		sol_t1 = minimize(objective_t1, x0=np.array([0.0]*14), args=(A_t1, b_t1), method='SLSQP', constraints=cons_t1)
		
		null_t1 = Null_proj(A_t1)
		
		A_t2, b_t2 = calculate_objective_A_b_t2(sol_t1, null_t1, J_O_ee, w_des_O_ee, w_curr_O_ee, dJdt_O_ee, np.array(mvel[:7]), dt)
		
		con_t2_1 = {'type': 'ineq', 'fun': ineq_constraint_t2, 'args': (A_constraint_t1, b_constraint_t1, np.array(sol_t1.x), null_t1)}
		cons_t2 = [con_t2_1]
		
		sol_t2 = minimize(objective_t2, x0=np.array([0.0]*14), args=(A_t2, b_t2, np.array(sol_t1.x), null_t1), method='SLSQP', constraints=cons_t2)
		
		null_t1_t2 = Null_proj(np.concatenate((A_t1, A_t2), axis=0))
		
		con_t3_1 = {'type': 'ineq', 'fun': ineq_constraint_t3, 'args': (A_constraint_t1, b_constraint_t1, np.array(sol_t1.x), np.array(sol_t2.x), null_t1, null_t1_t2)}
		cons_t3 = [con_t3_1]
		
		sol_t3 = minimize(objective_t3, x0=np.array([0.0]*14), args=(np.array(sol_t1.x), null_t1, np.array(sol_t2.x), null_t1_t2, self.q_mean[:7]), method='SLSQP', constraints=cons_t3)
		
		optimal_variable = sol_t1.x + np.matmul(null_t1, np.array(sol_t2.x)) + np.matmul(null_t1_t2, np.array(sol_t3.x))
		
		print("Optmal :", optimal_variable)
		
		return optimal_variable
		
	def perform_one_step_torque_control(self, v, observation_available_= True, target_name_="cupboard", link_idx_=3):
		
		link_poses = p.getLinkStates(self.robot.model.uid, linkIndices=[self.robot.arm_base_link_idx, self.robot.link_name_to_index["panda_hand"]], computeForwardKinematics = True)		
		C_O_rob = R.from_quat(link_poses[0][5])
		C_O_ee = R.from_quat(link_poses[1][5])
		
		C_rob_O = C_O_rob.inv()
		C_rob_O = C_rob_O.as_matrix()

		mpos, mvel, mtorq = getMotorJointStates(self.robot.model.uid)
				
		#------------ Whole body control ----------------------------------

		start_time = time.time()	
		x_dot_arm_rob, x_dot_base_rob = self.split_arm_and_base_velocity(v, observation_available_, target_name_, link_idx_)
						
		self.robot.update_velocity(np.squeeze(x_dot_base_rob[0:3]), x_dot_base_rob[5])		
		self.robot.velocity_setter()	
		
		optimal_variable = self.one_step_optimize_torques(v, x_dot_arm_rob, x_dot_base_rob, observation_available_, target_name_, link_idx_)
		
		q_des_dot_dot = optimal_variable[:7]
		F_des = optimal_variable[7:]
						
		tau = optimal_variable[7:]
		
		p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm, p.TORQUE_CONTROL, forces=list(tau))
		
		stop_time = time.time()
		
		self.log_exec_time.append(stop_time-start_time)
		self.log_tau.append(tau)
		
		self.robot._world.step_one()
		self.robot._world.sleep(self.robot._world.T_s)

		obj_info = self.scene.objects["cupboard"]
		target_id = obj_info.model.uid
		link_id = obj_info.grasp_links[3]
		
		if link_id == -1:
			temp = p.getBasePositionAndOrientation(target_id)
			target_pos = np.array(temp[0]).reshape((-1, 1))
			target_ori = R.from_quat(np.array(temp[1]))
		else:
			temp = p.getLinkState(target_id, link_id, computeForwardKinematics = True)
			target_pos = np.array(temp[4]).reshape((-1, 1))
			target_ori = R.from_quat(np.array(temp[5]))
		if np.linalg.norm( self.sk_mID.obj_pose_buffer[len(self.sk_mID.obj_pose_buffer)-1]- target_pos)>EPS:
			self.sk_mID.obj_pose_buffer.append(target_pos)
			
		print("Pose of the drawer: ", np.squeeze(target_pos))
		
		#print(np.array(self.log_q).shape[0], np.array(self.log_q).shape[1])
		
	def plot_data(self, mode='velocity_control'):
		
		N = len(self.log_q)
		
		
		fig, (ax1, ax2, ax3, ax4, ax5, ax6, ax7) = plt.subplots(7,1)
		fig.suptitle('Arm joint position')
		
		ax = [ax1, ax2, ax3, ax4, ax5, ax6, ax7]
		t = np.arange(1,N+1)
		
		q = np.array(self.log_q)
		q_dot = np.array(self.log_q_dot)
		
		for i in range(7):
			ax[i].axhline(y=self.q_max[i], color='r', linestyle='-.')
			ax[i].axhline(y=self.q_min[i], color='r', linestyle='-.')
			ax[i].plot(t, q[:,i], '-')
			ax[i].set_ylabel('q'+str(i))
			ax[i].grid('both', 'both')
			ax[i].set_xlim(t[0], t[-1])
			if not i == 6:
				ax[i].axes.xaxis.set_ticklabels([])
			
			
		plt.xlabel('Number of steps') 
		#plt.tight_layout()		
		plt.show()
				
	def examine_robot_in_pinnochio(self, q_curr):
		
		model = pin.buildModelFromUrdf(self.robot.urdf_path)	
		data = model.createData()
		q = pin.randomConfiguration(model)
		pin.forwardKinematics(model,data,q)	

		for name, oMi in zip(model.names, data.oMi):
    			print(("{:<24} : {: .2f} {: .2f} {: .2f}".format( name, *oMi.translation.T.flat )))				
		print("***** number of q *****: ", model.nq)
		print("***** number of v *****: ", model.nv) 					#this is the number of instantaneous degrees of freedom
		
		NQ, NV = model.nq, model.nv
		q = rand(NQ)
		
		pin.updateFramePlacements(model, data)
		IDX_TOOL = self.robot.arm_ee_link_idx
		IDX_BASIS = self.robot.arm_base_link_idx
		
		#print("IDX_TOOL: ", IDX_TOOL)
		
		Mtool = data.oMf[IDX_TOOL]
		Mbasis = data.oMf[IDX_BASIS]
		
		pin.forwardKinematics(model, data, q)
		pin.updateFramePlacements(model, data) 
		Mtool = data.oMf[IDX_TOOL]
		#print(Mtool)
		
		q = np.array(q_curr)
		
		vq = np.array([0.1] * model.nv)
		aq0 = zero(model.nv)

		pin.forwardKinematics(model, data, q)
		pin.updateFramePlacements(model, data) 		
		b = pin.rnea(model, data, q, vq, aq0)
		M = pin.crba(model, data, q)
		
		#print("M: ", M)
		#print("b: ", b)
		
		pin.computeJointJacobians(model, data, q)
		pin.forwardKinematics(model, data, q)
		pin.updateFramePlacements(model, data)
		pin.computeJointJacobians(model, data, q)
		#print("J: ", data.J)
		J1 = pin.computeFrameJacobian(model, data, q, IDX_TOOL, pin.ReferenceFrame.WORLD) #tool jacobian in the tool frame
		#print("J1 :", J1)
		J2 = pin.computeFrameJacobian(model, data, q, IDX_TOOL, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  
		#print("J2: ", J2)	
		
	def examine_robot_in_bullet(self):
	
		num_joints = p.getNumJoints(self.robot.model.uid)

		setJointPosition(self.robot.model.uid, [0.5] * num_joints)
		p.stepSimulation()
		
		for i in range(num_joints):
            		info = p.getJointInfo(self.robot.model.uid, i)
            		joint_name = info[1] if type(info[1]) is str else info[1].decode("utf-8")
            		print("i: ", i, " | name: ", joint_name, " | type: ", info[2], " | parentIndex: ", info[16], " | linkName: ", info[12], " | max and min: ", info[8], info[9])
            		
		print("***** For this robot: *****")
		print("joint idx arm: ", self.robot.joint_idx_arm)
		print("joint_idx_fingers: ", self.robot.joint_idx_fingers)
		print("joint_idx_hand: ", self.robot.joint_idx_hand)
		print("arm base link idx: ", self.robot.arm_base_link_idx)
		print("arm_ee_link_idx: ", self.robot.arm_ee_link_idx)
		
		

				
		
