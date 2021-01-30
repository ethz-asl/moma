#----- Standard Skills -----                                    

from highlevel_planning.sim.scene_move_skill_configurable import SceneMoveSkill                    
from highlevel_planning.skills.navigate_configurable import SkillNavigate                    

from highlevel_planning.skills.grasping import SkillGrasping
from highlevel_planning.tools.config import ConfigYaml
from highlevel_planning.tools import run_util_configurable                                

#----- Controllers -----

from highlevel_planning.sim.controllers.fixed_base_torque_control import Controller as controller1
from highlevel_planning.sim.controllers.fixed_base_cartesian_vel_control import Controller as controller2
from highlevel_planning.sim.controllers.moving_base_QCQP_vel_control import Controller as controller3
from highlevel_planning.sim.controllers.moving_base_LCQP_vel_control import Controller as controller4
from highlevel_planning.sim.controllers.moving_base_SOCP_vel_control import Controller as controller5


#----- Direction Estimators -----

#from highlevel_planning.sim.direction_estimators.direction_estimation_no_filter import Estimator 
from highlevel_planning.sim.direction_estimators.direction_estimation_with_filter import Estimator       

#----- Additional skills -----

from highlevel_planning.skills.door_opening import SkillTrajectoryPlanning

#----- Utils -----

from highlevel_planning.tools.door_opening_util import *

#----- Other -----

import numpy as np
import os
import pybullet as p

#-----------------

EPS = 1e-6
BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#-------
def main(name_of_the_object = "cupboard"):
    
    offsetPos = [0.0, 0.0, 0.0]
    offsetAng = 0.0
    offsetGr = -45.0
    initLen = 100
    vRegular = 0.1
    vInit = vRegular/4
    
    N_steps = 1000
    

    if name_of_the_object == "cupboard":
        
        link_index = 3
        init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
        
    elif name_of_the_object == "roomdoor":
        
        link_index = 0
        init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
        
    elif name_of_the_object == "slidingdoor":

        link_index = 0
        init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
        
    elif name_of_the_object == "slidinglid":

        link_index = 0
        init_direction = np.array([0.0, -1.0, 0.0]).reshape(3,1)
        N_steps = 700

    elif name_of_the_object == "removablelid":

        link_index = 0
        init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
        
    elif name_of_the_object == "dishwasher":
        
        link_index = 0
        init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
        N_steps =600
        
    # Command line arguments
    
    args = run_util_configurable.parse_arguments()

    # Load existing simulation data if desired
    
    savedir = os.path.join(BASEDIR, "data", "sim")
    objects, robot_mdl = run_util_configurable.restore_pybullet_sim(savedir, args)

    # Load config file
    
    cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))
        
    robot, scene = run_util_configurable.setup_pybullet_world(
        SceneMoveSkill, BASEDIR, savedir, objects, args, cfg, robot_mdl, offsetGr, name_of_the_object
    )
           
    if name_of_the_object == 'dishwasher':
        
        target_id = scene.objects[name_of_the_object].model.uid
        nj = p.getNumJoints(target_id)          
        for counter in range(nj):
            p.resetJointState(target_id, counter, -0.01, 0.0) 
                
    list_of_controllers = []    
    fd1 = 1/(50*robot._world.T_s)
       
    #----- Object definitions -----
                  
    sk_grasp = SkillGrasping(scene, robot, cfg)
    sk_nav = SkillNavigate(scene, robot, offsetPos, offsetAng)
    sk_dir = Estimator(scene, robot, robot._world.T_s, 100, init_direction, initLen, fd1)    

    c1 = controller1(scene, robot, robot._world.T_s)
    c2 = controller2(scene, robot, robot._world.T_s)
    c3 = controller3(scene, robot, robot._world.T_s, noCollision=True)
    c4 = controller4(scene, robot, robot._world.T_s, noCollision=True, Npolygon=256)
    c5 = controller5(scene, robot, robot._world.T_s, noCollision=True)
    
    cinit = controller2(scene, robot, robot._world.T_s)
    
    #list_of_controllers.append(c1)
    list_of_controllers.append(c2)
    list_of_controllers.append(c3)
    #list_of_controllers.append(c4)
    list_of_controllers.append(c5)
    
    sk_traj = SkillTrajectoryPlanning(scene, robot, cfg, list_of_controllers, cinit, robot._world.T_s, initLen, vInit, vRegular)
    
    #----- Run -----
    
    globalFolder = prepare_global_dir()
    
    sk_traj.Run(num_steps=N_steps, sk_grasp=sk_grasp, sk_dir=sk_dir, sk_nav=sk_nav, target_name=name_of_the_object, link_idx=link_index, grasp_id=0, global_folder=globalFolder, postfix=None)

    robot._world.step_seconds(50)        

if __name__ == "__main__":
    
#========== SETUP ==========
    
# 1) Import the appropriate direction estimator
# 2) Import all desired controllers
# 3) Set desired object name: "cupboard/roomdoor/slidingdoor/slidinglid/dishwasher"
    
    main(name_of_the_object = "dishwasher")
