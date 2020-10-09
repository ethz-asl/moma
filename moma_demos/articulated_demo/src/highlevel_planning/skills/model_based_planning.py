import pybullet as p
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from scipy.optimize import minimize
from pinocchio.utils import *

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

def getJointStates(robot):
  joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques


def getMotorJointStates(robot):
  joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
  joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
#  for j, i in zip(joint_states, joint_infos):
  
#  	print(i)
  joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques


def setJointPosition(robot, position, kp=1.0, kv=0.3):
  num_joints = p.getNumJoints(robot)
  zero_vec = [0.0] * num_joints
  if len(position) == num_joints:
    p.setJointMotorControlArray(robot,
                                range(num_joints),
                                p.POSITION_CONTROL,
                                targetPositions=position,
                                targetVelocities=zero_vec,
                                positionGains=[kp] * num_joints,
                                velocityGains=[kv] * num_joints)
  else:
    print("Not setting torque. "
          "Expected torque vector of "
          "length {}, got {}".format(num_joints, len(torque)))

def objective(x, *args):
   
	u1 = args[0]
	u1 = u1/LA.norm(u1)
	x_dot_des = args[1]
	c = args[2]
  
	A = np.diag([x[0], x[1], 1, 1, 1, x[2]])
	v = np.matmul(A, x_dot_des)
	v = v/LA.norm(v)
		
	return -np.abs(np.dot(v, u1))-c*LA.norm(x)    


class SkillTrajectoryPlanning:
	def __init__(self, scene_, robot_, sk_mID_, time_step):
	
		self.scene = scene_
		self.robot = robot_
		self.dt = time_step
		self.sk_mID = sk_mID_
		
	def calculte_desired_velocity(self, v, observation_available=True, target_name=None, link_idx=None, B=np.diag([0,0,0,0,0,0]), grasp_id=0):
	
		model_type, parameters = self.sk_mID.PickModel()
		
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
			
			if link_id == -1:
			
				temp = p.getBasePositionAndOrientation(target_id)
				pos_obj  = np.array(temp[0]).reshape((-1, 1))
				ori_obj = R.from_quat(np.array(temp[1]))
			else:
				temp = p.getLinkState(target_id, link_id)
				pos_obj = np.array(temp[4]).reshape((-1, 1))
				ori_obj = R.from_quat(np.array(temp[5]))
				
					
		
		if model_type == 'prismatic':
			
			e = parameters[0]
			C = parameters[1]
		else:
		
			n = parameters[0]
			C = parameters[1]
			r = parameters[2]
			
			e = np.cross(pos_obj-C, n)
			e = e/LA.norm(e)
			
		v_des = v*e 
		r_des = pos_obj + v_des*self.dt
		print("************ vdes: ", v_des)
		
		link_poses = p.getLinkStates(self.robot.model.uid, linkIndices=[self.robot.arm_base_link_idx, self.robot.link_name_to_index["panda_hand"]])
		
		C_O_rob = R.from_quat(link_poses[0][5])
		C_O_ee = R.from_quat(link_poses[1][5])
		
		v_des_rob = C_O_rob.inv().apply(v_des) #v_des in the robot body frame
		v_des_rob = np.array(v_des_rob)
		w_des_rob = C_O_rob.inv().apply(np.array([0,0,0]))
		w_des_rob = np.array(w_des_rob)
		x_dot_des_rob = np.concatenate((v_des_rob, w_des_rob), axis=0)
		
		force, torque = self.robot.get_wrist_force_torque()
		F_ee = np.concatenate((force, torque), axis=0)
		x_dot_compl_ee = np.matmul(B, F_ee)
		
		x_dot_compl_rob = np.concatenate((C_O_rob.inv().apply(C_O_ee.apply(x_dot_compl_ee[0:3])), C_O_rob.inv().apply(C_O_ee.apply(x_dot_compl_ee[3:6]))), axis=0)
		x_dot_compl_rob = np.array(x_dot_compl_rob)
		
		x_dot_rob = x_dot_des_rob+x_dot_compl_rob
		
		return x_dot_rob 
			
	def one_step_optimize(self, a0, x_dot_des, c, u1, a_min, a_max):
	
		b = (a_min, a_max)
		bnds = (b,b,b)
		sol = minimize(objective, np.array(a0), args = (np.array(u1), np.array(x_dot_des), c), method = 'SLSQP', bounds=bnds)
		
		a = sol.x
		print("optimal a: ", a)
		A_arm = np.diag([a[0], a[1], 1, 1, 1, a[2]])
		A_base = np.eye(6)-A_arm
		x_dot_arm_rob = np.matmul(A_arm, x_dot_des)
		x_dot_base_rob = np.matmul(A_base, x_dot_des)
		
		return x_dot_arm_rob, x_dot_base_rob
		
	def perform_one_step(self, v, observation_available_= True, target_name_="cupboard", link_idx_=3):
	
		pos, vel, torq = getJointStates(self.robot.model.uid)
		mpos, mvel, mtorq = getMotorJointStates(self.robot.model.uid)
		zero_vec = [0.0]*len(mpos)
		
		lin, ang = p.calculateJacobian(self.robot.model.uid, self.robot.arm_ee_link_idx, [0.0, 0.0, 0.0], mpos, zero_vec, zero_vec)
		lin = np.array(lin)
		ang = np.array(ang)
	
		link_poses = p.getLinkStates(self.robot.model.uid, linkIndices=[self.robot.arm_base_link_idx, self.robot.link_name_to_index["panda_hand"]])		
		C_O_rob = R.from_quat(link_poses[0][5])
		C_O_ee = R.from_quat(link_poses[1][5])
		
		C_rob_O = C_O_rob.inv()
		C_rob_O = C_rob_O.as_matrix()
				
		lin_rob = np.matmul(C_rob_O, lin)
		ang_rob = np.matmul(C_rob_O, ang)
		
		J_rob = np.concatenate((lin_rob, ang_rob), axis=0)
		J_rob = J_rob[:, 6:13]
		print(J_rob)
		u, s, vh = LA.svd(J_rob,  full_matrices=False)
		
		a_min = 0
		a_max = 1
		c = 0.01
		
		a0 = [1, 1, 1]
		u1 = u[:, 0]

		x_dot_des = self.calculte_desired_velocity(v, observation_available_, target_name_, link_idx_, np.diag([0,0,0,0,0,0]), 0)

		x_dot_arm_rob, x_dot_base_rob = self.one_step_optimize(a0, x_dot_des, c, u1, a_min, a_max)

		#self.robot.update_velocity(np.squeeze(x_dot_base_rob[0:3]), np.squeeze(x_dot_base_rob[3:6]))		
		#self.robot.velocity_setter()		#this will update the base speed which will be later used in simulation.step inside self.robot.task_space_velocity_control
		
		p.resetBaseVelocity(self.robot.model.uid, np.squeeze(x_dot_base_rob[0:3]), np.squeeze(x_dot_base_rob[3:6]))
		
		print("x_dot_des: ", x_dot_des)
		print("x_dot_arm: ", x_dot_arm_rob)
		
		velocity_translation_rob = x_dot_arm_rob[:3]
		velocity_rotation_rob = x_dot_arm_rob[3:6]
		
		print("GOVNO: ", velocity_translation_rob, velocity_rotation_rob)
		
		velocity_translation_O = C_O_rob.apply(velocity_translation_rob)
		velocity_rotation_O = C_O_rob.apply(velocity_rotation_rob)
		velocity_translation_O = np.array(velocity_translation_O)
		velocity_rotation_O = np.array(velocity_rotation_O)
		
		print("GOVNO2: ", velocity_translation_O)
		
		self.robot.task_space_velocity_control(np.squeeze(velocity_translation_O), np.squeeze(velocity_rotation_O), 1)
		
		obj_info = self.scene.objects["cupboard"]
		target_id = obj_info.model.uid
		link_id = obj_info.grasp_links[3]
		if link_id == -1:
			temp = p.getBasePositionAndOrientation(target_id)
			target_pos = np.array(temp[0]).reshape((-1, 1))
			target_ori = R.from_quat(np.array(temp[1]))
		else:
			temp = p.getLinkState(target_id, link_id)
			target_pos = np.array(temp[4]).reshape((-1, 1))
			target_ori = R.from_quat(np.array(temp[5]))
		if np.linalg.norm( self.sk_mID.obj_pose_buffer[len(self.sk_mID.obj_pose_buffer)-1]- target_pos)>EPS:
			self.sk_mID.obj_pose_buffer.append(target_pos)
		print("Pose of the drawer: ", target_pos)		
	def examination_pinn(self):
		
		model = pin.buildModelFromUrdf(self.robot.urdf_path)	
		data = model.createData()
		q = pin.randomConfiguration(model)
		pin.forwardKinematics(model,data,q)	

		for name, oMi in zip(model.names, data.oMi):
    			print(("{:<24} : {: .2f} {: .2f} {: .2f}".format( name, *oMi.translation.T.flat )))				
		print("***** number of q: ", model.nq)
		print("***** number of v: ", model.nv) 					#this is the number of instantaneous degrees of freedom
		
		NQ, NV = model.nq, model.nv
		print(q)
		q = rand(NQ)
		
		pin.updateFramePlacements(model, data)
		IDX_TOOL = self.robot.arm_ee_link_idx
		IDX_BASIS = self.robot.arm_base_link_idx
		
		print("IDX_TOOL: ", IDX_TOOL)
		
		Mtool = data.oMf[IDX_TOOL]
		Mbasis = data.oMf[IDX_BASIS]
		
		pin.forwardKinematics(model, data, q)
		pin.updateFramePlacements(model, data) 
		Mtool = data.oMf[IDX_TOOL]
		print(Mtool)
		
		#q = 2*rand(model.nq)
		vq = rand(model.nv)
		aq0 = zero(model.nv)

		pin.forwardKinematics(model, data, q)
		pin.updateFramePlacements(model, data) 		
		b = pin.rnea(model, data, q, vq, aq0)
		M = pin.crba(model, data, q)
		
		print("M: ", M)
		print("b: ", b)
		
		pin.computeJointJacobians(model, data, q)
		pin.forwardKinematics(model, data, q)
		pin.updateFramePlacements(model, data)
		pin.computeJointJacobians(model, data, q)
		print("J: ", data.J)
		J1 = pin.computeFrameJacobian(model, data, q, IDX_TOOL, pin.ReferenceFrame.WORLD) #tool jacobian in the tool frame
		print("J1 :", J1)
		J2 = pin.computeFrameJacobian(model, data, q, IDX_TOOL, pin.ReferenceFrame.LOCAL)  
		print("J2: ", J2)	
		
	def examination_bullet(self):
	
		num_joints = p.getNumJoints(self.robot.model.uid)

		setJointPosition(self.robot.model.uid, [0.5] * num_joints)
		p.stepSimulation()
		
		for i in range(num_joints):
            		info = p.getJointInfo(self.robot.model.uid, i)
            		joint_name = info[1] if type(info[1]) is str else info[1].decode("utf-8")
            		print("i: ", i, " | name: ", joint_name, " | type: ", info[2], " | parentIndex: ", info[16], " | linkName: ", info[12], " | max and min: ", info[8], info[9])
            		
		print("***** For this robot: ")
		print("joint idx arm: ", self.robot.joint_idx_arm)
		print("joint_idx_fingers: ", self.robot.joint_idx_fingers)
		print("joint_idx_hand: ", self.robot.joint_idx_hand)
		print("arm base link idx: ", self.robot.arm_base_link_idx)
		print("arm_ee_link_idx: ", self.robot.arm_ee_link_idx)
		
		
		pos, vel, torq = getJointStates(self.robot.model.uid)
		mpos, mvel, mtorq = getMotorJointStates(self.robot.model.uid)
		zero_vec = [0.0]*len(mpos)
		
		lin, ang = p.calculateJacobian(self.robot.model.uid, self.robot.joint_idx_arm[3], [0.1, 0.01, 0.05], mpos, zero_vec, zero_vec)
		lin = np.array(lin)
		ang = np.array(ang)
		print("Lin jac: ", lin)
		print("Ang jac: ", ang.shape)				 
						 
				
		
