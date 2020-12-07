from highlevel_planning.sim.scene_move_skill import SceneMoveSkill
from highlevel_planning.skills.navigate import SkillNavigate
from highlevel_planning.skills.grasping import SkillGrasping
from highlevel_planning.skills.move import SkillMove
from highlevel_planning.tools.config import ConfigYaml
from highlevel_planning.tools import run_util

from highlevel_planning.skills.model_fitting import SkillModelIdentification


from collections import deque

import pybullet as p
import numpy as np
import os

import pinocchio as pin
from numpy import linalg as LA

EPS = 1e-6
BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def getMotorJointStates(robot):

	joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
	joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
	joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
	joint_positions = [state[0] for state in joint_states]
	joint_velocities = [state[1] for state in joint_states]
	joint_torques = [state[3] for state in joint_states]
	
	return joint_positions, joint_velocities, joint_torques
    
def simple_PID(robot, scene):
	
	
	num_joints = p.getNumJoints(robot.model.uid)

	#----------------- Reset robot joint dampings -----------------------------------------	
			
	p.changeDynamics(robot.model.uid, -1, linearDamping=0, angularDamping=0, jointDamping=0)
	for i in robot.joint_idx_arm:
			
		p.changeDynamics(robot.model.uid, i, linearDamping=0, angularDamping=0, jointDamping=0)
				
	#p.setJointMotorControlArray(robot.model.uid, range(num_joints), p.VELOCITY_CONTROL, forces = [0.0]*num_joints)
			
	N_steps = 10000
	dt = robot._world.T_s
	p.setTimeStep(dt)
	
	r_O_object = []
	v_O_object = []
	
	r_O_object_y = np.linspace(0.7, 0.5, N_steps)

	pinocchio_model = pin.buildModelFromUrdf(robot.urdf_path)
	pinocchio_data = pinocchio_model.createData()
		
	for i in range(N_steps):
		r_O_object.append([0.7, r_O_object_y[i]*0, 0.6])
		v_O_object.append([-0.1, 0.0, 0.002])

	tau_previous = np.array([0.0]*7)		
	previous_q_dot = np.array([0.0]*7)
	
	for i in range(N_steps):

		info = p.getLinkStates(robot.model.uid, linkIndices=[robot.arm_base_link_idx, robot.link_name_to_index["panda_hand"]])
		
		pos = info[1][4]
		ori = info[1][5]
		
		base_pos = info[0][4]
		base_ori = info[0][5]
				
		q, q_dot, mtorq = getMotorJointStates(robot.model.uid)
		zero_vec = [0.0]*len(q)
		
		q = np.array(q)
		q_dot = np.array(q_dot)
		q_dot_dot = np.array(zero_vec)
		
		M = pin.crba(pinocchio_model, pinocchio_data, q)
		b = pin.rnea(pinocchio_model, pinocchio_data, q, q_dot, q_dot_dot)
		
		M = np.array(M)
		b = np.array(b)
		
		lin, ang = p.calculateJacobian(robot.model.uid, robot.arm_ee_link_idx, [0.0, 0.0, 0.0], list(q), zero_vec, zero_vec)
		lin = np.array(lin)
		ang = np.array(ang)
			
		J_O_ee = np.concatenate((lin, ang), axis=0)
		J_O_ee = J_O_ee[:, :7]
		
		q_dot_des = np.matmul(LA.pinv(J_O_ee[:3, :]), np.array(v_O_object[i]))
		q_des = np.array(robot.ik(r_O_object[i], ori))
		
		print("Iteration: ", i)
		
		if i == 0:
			previous_q = np.copy(q)
			previous_q_dot = np.copy(q_dot)
			previous_torque = np.copy(mtorq)
			previous_b = np.copy(b)
			previous_M = np.copy(M)
			
		else:
			
			q_dot_dot_measured = (1/dt)*(q_dot - previous_q_dot)			
			previous_tau = np.matmul(previous_M, q_dot_dot_measured)+previous_b
			tau = pin.rnea(pinocchio_model, pinocchio_data, previous_q, previous_q_dot, q_dot_dot_measured)
			
			b_p = np.array(p.calculateInverseDynamics(robot.model.uid, list(previous_q), list(previous_q_dot), [0.0]*9))
			M_p = np.array(p.calculateMassMatrix(robot.model.uid, list(previous_q)))
			tau_p = np.matmul(M_p[:7,:7], q_dot_dot_measured[:7])+b_p[:7]
			tau = np.array(p.calculateInverseDynamics(robot.model.uid, list(previous_q), list(previous_q_dot), list(q_dot_dot_measured)))
						
			print("Actual torque: ", previous_torque)
			print("Calculated using pybullet: ", tau)
			print("Using matrix manipulation: ", tau_p)
			
			f_wristframe, t_wristframe = robot.get_wrist_force_torque()
			print("Force: ", f_wristframe)
			previous_q = np.copy(q)
			previous_q_dot = np.copy(q_dot)
			previous_torque = np.copy(mtorq)
			previous_b = np.copy(b)
			previous_M = np.copy(M)			
			
		print('*'*20)
		p.resetBaseVelocity(robot.model.uid, [-1, 0.0, 0.0], [0.0, 0.0, 0.0])			
		p.setJointMotorControlArray(robot.model.uid, robot.joint_idx_arm, p.VELOCITY_CONTROL, targetVelocities=list(q_dot_des))	
		#p.setJointMotorControlArray(robot.model.uid, robot.joint_idx_arm, p.TORQUE_CONTROL, forces=list(tau))	

		robot._world.step_one()
		robot._world.sleep(robot._world.T_s)
		
	robot._world.step_seconds(50)
		
		
		
		
def main():
    # Command line arguments
    args = run_util.parse_arguments()

    # Load existing simulation data if desired
    savedir = os.path.join(BASEDIR, "data", "sim")
    objects, robot_mdl = run_util.restore_pybullet_sim(savedir, args)

    # Load config file
    cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

    # Create world
    robot, scene = run_util.setup_pybullet_world(
        SceneMoveSkill, BASEDIR, savedir, objects, args, cfg, robot_mdl
    )

    # Set up skills
    sk_grasp = SkillGrasping(scene, robot, cfg)
    sk_nav = SkillNavigate(scene, robot)
    sk_move = SkillMove(scene, robot, 0.02, robot._world.T_s)
    
    # ---------- Run examples -----------
    
    simple_PID(robot, scene)
     
    # -----------------------------------

    robot._world.step_seconds(50)


if __name__ == "__main__":
    main()
