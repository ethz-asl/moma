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

from highlevel_planning.sim.direction_estimators.direction_estimation_with_filter_and_abs_force_estimation import Estimator       

#----- Additional skills -----

from highlevel_planning.skills.door_opening import SkillTrajectoryPlanning

#----- Utils -----

from highlevel_planning.tools.door_opening_util import *

#----- Other -----

import pybullet as p
import numpy as np
import os
import math

#-----------------

EPS = 1e-6
BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#----- Description -----

# This script is to be run if multiple controllers are to be tested for 
# different initial configurations. The complete procedure is restarted automatically
# for each starting configuration

#-----------------------

def main(offsetX, offsetY, offsetAngle, offsetGrasp, name_of_the_object = "cupboard"):
    
    Nx =len(offsetX)                                                           # Number of different x axis starting coordinates
    Ny = len(offsetY)                                                          # Number of different y axis starting coordinates
    Nang = len(offsetAngle)                                                    # Number of different orientations of the base
    Noff = len(offsetGrasp)                                                    # Number of different grasping orientations
    
    globalFolder = prepare_global_dir()
    N_steps = 1200

    if name_of_the_object == "cupboard":
        
        link_index = 3
        init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
        
    elif name_of_the_object == "roomdoor":
        
        link_index = 0
        init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
        offsetPos = [0.0, 0.0, 0.0]
        
    elif name_of_the_object == "slidingdoor":

        link_index = 0
        init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
        offsetPos = [0.0, 0.0, 0.0]
        
    elif name_of_the_object == "slidinglid":

        link_index = 0
        init_direction = np.array([0.0, -1.0/2**0.5, -1.0/2**0.5]).reshape(3,1)
        offsetPos = [0.0, 0.0, 0.0]
        N_steps = 600

    elif name_of_the_object == "removablelid":

        link_index = 0
        init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
        offsetPos = [0.0, 0.0, 0.0]
        
    elif name_of_the_object == "dishwasher":
        
        link_index = 0
        init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
        offsetPos = [0.0, 0.0, 0.0]
        N_steps = 600
        
    initLen = 100
    vRegular = 0.1
    vInit = vRegular/4
    
    list_of_failed_names = []
        
    for i in range(Nx):
        for j in range(Ny):
            for k in range(Nang):
                for m in range(Noff):
                
                    scenName = str(i)+'_'+str(j)+'_'+str(k)+'_'+str(m)
                    
                    print(50*'=')
                    print('Executing combination: ', scenName)
                    print(50*'=')
                
                    offsetPos = [offsetX[i], offsetY[j], 0.0]
                    offsetAng = offsetAngle[k]
                    offsetGr = offsetGrasp[m]
                
                    try:
                        # Command line arguments
                        args = run_util_configurable.parse_arguments()

                        # Load existing simulation data if desired
                        savedir = os.path.join(BASEDIR, "data", "sim")
                        #objects, robot_mdl = run_util_configurable.restore_pybullet_sim(savedir, args)
                        
                        objects = None
                        robot_mdl = None
                        
                        # Load config file
                        cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

                        # ---------- Run examples -----------

                        robot, scene = run_util_configurable.setup_pybullet_world(
                                SceneMoveSkill, BASEDIR, savedir, objects, args, cfg, robot_mdl, offsetGr, name_of_the_object
                                )
                        
                        #----- Special preparation is required for the dishwasher model 
                        # as it tends to open on its own otherwise -----
                        
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
                        
                        #----- Controller for the initial direction estimation is always fixed based -----
                        
                        cinit = controller2(scene, robot, robot._world.T_s)
                        
                        #----- Defining all the controllers that we want to test -----
                        
                        c2 = controller1(scene, robot, robot._world.T_s)
                        c1 = controller2(scene, robot, robot._world.T_s)
                        c3 = controller3(scene, robot, robot._world.T_s, noCollision=True)
                        c4 = controller4(scene, robot, robot._world.T_s, noCollision=True, Npolygon=32)
                        c5 = controller5(scene, robot, robot._world.T_s, noCollision=True)
                        
                        list_of_controllers.append(c1)
                        list_of_controllers.append(c2)
                        list_of_controllers.append(c3)
                        list_of_controllers.append(c4)
                        list_of_controllers.append(c5)

                        #---------------
                        
                        sk_traj = SkillTrajectoryPlanning(scene, robot, cfg, list_of_controllers, cinit, robot._world.T_s, initLen, vInit, vRegular)
                    
                        #----- Run -----
                    
                        sk_traj.Run(num_steps=N_steps, sk_grasp=sk_grasp, sk_dir=sk_dir, sk_nav=sk_nav, target_name=name_of_the_object, link_idx=link_index, grasp_id=0, global_folder=globalFolder, postfix=scenName)
                        robot._world.step_seconds(10) 
                        p.disconnect()
                    
                    except:
                        
                        list_of_failed_names.append(scenName)
                        p.disconnect()
                        continue
                    
    print(30*'='+" List of failed experiments "+30*'=')
    for name in list_of_failed_names:
        print(name)
                
if __name__ == "__main__":
    
    # Set desired object name: "cupboard/roomdoor/slidingdoor/slidinglid/dishwasher"
    
    rangePosX = 0.1
    rangePosY = 0.0
    rangeAngle = math.pi/12
    rangeOffset = 45
    
    Nx =5
    Ny = 1
    Nang = 5
    Noff = 1
    
    offsetX = np.linspace(-rangePosX, rangePosX, num=Nx)
    offsetY = np.linspace(-rangePosY, rangePosY, num=Ny)
    offsetAngle = np.linspace(-rangeAngle, rangeAngle, num=Nang)
    offsetGrasp = np.linspace(-rangeOffset, rangeOffset, num=Noff)
    
    main(offsetX, offsetY, offsetAngle, offsetGrasp, name_of_the_object = "roomdoor")
