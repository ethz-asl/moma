from highlevel_planning.sim.scene_move_skill import SceneMoveSkill

from highlevel_planning.skills.navigate_with_offset import SkillNavigate					#THIS IS CHANGED!

from highlevel_planning.skills.direction_estimation import SkillUnconstrainedDirectionEstimation
from highlevel_planning.skills.grasping import SkillGrasping
from highlevel_planning.skills.move import SkillMove
from highlevel_planning.tools.config import ConfigYaml
from highlevel_planning.tools import run_util

from highlevel_planning.skills.fixed_base_vel_control import Controller as controller1
from highlevel_planning.skills.fixed_base_torque_control import Controller as controller2
from highlevel_planning.skills.moving_base_no_collision_vel_control import Controller as controller3

from highlevel_planning.skills.New_model_based_control_with_object_est import SkillTrajectoryPlanning

from highlevel_planning.skills.model_fitting import SkillModelIdentification
from highlevel_planning.skills.New_model_based_control_with_object_est import getMotorJointStates

from scipy.spatial.transform import Rotation as R

from highlevel_planning.tools.door_opening_util import *
from highlevel_planning.tools.plot_util import *

from highlevel_planning.tools.util import IKError
from collections import deque

import pybullet as p
import numpy as np
import os

EPS = 1e-6
BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def trajectory_generation_example(num_steps, observation_available, SceneMoveSkill, BASEDIR, savedir, objects, args, cfg, robot_mdl, offset_pos, offset_angle, list_of_modes):
			
	curr_folder = prepare_dir()
	
	init_joint_positions = None
	init_joint_velocities = None
	init_base_pos = None 
	init_base_ori = None 
	init_lin_base_vel = None 
	init_ang_base_vel = None	
	
	init_obj_joint_pos = None
	init_obj_joint_vel = None
	
	robot, scene = run_util.setup_pybullet_world(
		SceneMoveSkill, BASEDIR, savedir, objects, args, cfg, robot_mdl
	)	
				
	for mode in list_of_modes:
	 
		sk_grasp = SkillGrasping(scene, robot, cfg)	
		sk_nav = SkillNavigate(scene, robot, offset_pos, offset_angle)
		sk_mID = SkillModelIdentification(scene, robot, 100, robot._world.T_s)
		sk_traj = SkillTrajectoryPlanning(scene, robot, sk_mID, robot._world.T_s)
		
		#try:
		
		if mode == list_of_modes[0]:
		
			init_obj_joint_pos, init_obj_joint_vel = sk_traj.ObjectConfiguration(mode='memorize', target_name="cupboard")	# Log starting configuration of the drawers
			sk_nav.move_to_object("cupboard", nav_min_dist=1.0)
			res = sk_grasp.grasp_object(target_name="cupboard", link_idx=3)	
			if not res:
				print('Initial grasping failed')
				break	
			init_joint_positions, init_joint_velocities, init_base_pos, init_base_ori, init_lin_base_vel, init_ang_base_vel = sk_traj.FreezeRobotConfiguration()
			
		else:
			robot.reset()
			robot.to_start()
			sk_traj.ObjectConfiguration(mode='reset', target_name="cupboard", joint_positions=init_obj_joint_pos, joint_velocities=init_obj_joint_vel)
			sk_traj.ApplyRobotConfiguration(init_joint_positions, init_joint_velocities, init_base_pos, init_base_ori, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
			robot.close_gripper()
					
		sk_traj.sk_mID.init_trajectory(desired_velocity=0.02, sk_grasp=sk_grasp, sk_nav=sk_nav, observation_available=observation_available, target_name="cupboard", link_idx=3)		
		sk_traj.getInitialObjectOri(target_name="cupboard", link_idx=3, grasp_id=0)
		sk_traj.ResetArmAndBaseJointDampings()
			
		for i in range(num_steps):
		
			print(30*'*')
			print("Iteration i: ", i)
						
			if mode == 'PlanJointAcc':
					
				sk_traj.PerformOneStep(mode="PlanJointAcc", v=0.1, observation_available=observation_available)
			
			if mode == 'PlanJointAccAndBaseVel':
														
				sk_traj.PerformOneStep(mode="PlanJointAccAndBaseVel", v=0.1, observation_available=observation_available, modeBase='NoSelfCollision')
								
			if mode == 'PlanJointAccAndBaseVelMaxMob':
							
				sk_traj.PerformOneStep(mode="PlanJointAccAndBaseVel", v=0.1, observation_available=observation_available, modeBase='NoSelfCollisionAndMaxMobility') 
					
		save_run(sk_traj, curr_folder, mode)
			
		#except Exception as e:
		
			#print(e)
		#save_run(sk_traj, curr_folder, mode)
		
	plot_runs(sk_traj, curr_folder, list_of_modes)
	robot._world.step_seconds(50)
	
def main():

	# Command line arguments
	args = run_util.parse_arguments()

	# Load existing simulation data if desired
	savedir = os.path.join(BASEDIR, "data", "sim")
	objects, robot_mdl = run_util.restore_pybullet_sim(savedir, args)

	# Load config file
	cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))
    
	# ---------- Run examples -----------
	
	offset_pos = [0.0, 0.0, 0.0]
	offset_angle = 0.0
	observation_available = True
	
	list_of_modes = ['PlanJointAcc', 'PlanJointAccAndBaseVelMaxMob']		

	trajectory_generation_example(700, observation_available, SceneMoveSkill, BASEDIR, savedir, objects, args, cfg, robot_mdl, offset_pos, offset_angle, list_of_modes)

if __name__ == "__main__":
	main()
