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

#----- Direction Estimators -----

#from highlevel_planning.sim.direction_estimators.direction_estimation_no_filter import Estimator 
from highlevel_planning.sim.direction_estimators.direction_estimation_with_filter import Estimator       

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

def main(offsetX, offsetY, offsetAngle, offsetGrasp, name_of_the_object = "cupboard"):
    
    Nx =len(offsetX)
    Ny = len(offsetY)
    Nang = len(offsetAngle)
    Noff = len(offsetGrasp)
    
    globalFolder = prepare_global_dir()

    if name_of_the_object == "cupboard":
        
        link_index = 3
        
    elif name_of_the_object == "roomdoor":
        
        link_index = 0
        offsetPos = [0.0, 0.0, 0.0]
        
    initLen = 100
    vRegular = 0.1
    vInit = vRegular/4
        
    for i in range(Nx):
        for j in range(Ny):
            for k in range(Nang):
                for m in range(Noff):
                
                    scenName = str(i)+'_'+str(j)+'_'+str(k)+'_'+str(m)
                    
                    print(50*'=')
                    print('Executing combination: ', scenName)
                    print(50*'=')
                    n_failed = 0
                
                    offsetPos = [offsetX[i], offsetY[j], 0.0]
                    offsetAng = offsetAngle[k]
                    offsetGr = offsetGrasp[m]
                
                    while(1):        
                            
                        try:
                            # Command line arguments
                            args = run_util_configurable.parse_arguments()

                            # Load existing simulation data if desired
                            savedir = os.path.join(BASEDIR, "data", "sim")
                            objects, robot_mdl = run_util_configurable.restore_pybullet_sim(savedir, args)

                            # Load config file
                            cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

                            # ---------- Run examples -----------

                            robot, scene = run_util_configurable.setup_pybullet_world(
                                SceneMoveSkill, BASEDIR, savedir, objects, args, cfg, robot_mdl, offsetGr, name_of_the_object
                            )
                            list_of_controllers = []
                            fd1 = 1/(50*robot._world.T_s)
                            
                            #----- Object definitions -----
                
                            sk_grasp = SkillGrasping(scene, robot, cfg)
                            sk_nav = SkillNavigate(scene, robot, offsetPos, offsetAng)
                            sk_dir = Estimator(scene, robot, robot._world.T_s, 100, np.array([0.0, 0.0, -1.0]).reshape(3,1), initLen, fd1)

                            c1 = controller1(scene, robot, robot._world.T_s)
                            c2 = controller2(scene, robot, robot._world.T_s)
                            c3 = controller3(scene, robot, robot._world.T_s, noCollision=True)
                            c4 = controller4(scene, robot, robot._world.T_s, noCollision=True, Npolygon=256)
    
                            #list_of_controllers.append(c1)
                            list_of_controllers.append(c2)
                            #list_of_controllers.append(c3)
                            list_of_controllers.append(c4)
    
                            sk_traj = SkillTrajectoryPlanning(scene, robot, cfg, list_of_controllers, robot._world.T_s, initLen, vInit, vRegular)
                            
                            #----- Run -----
                            
                            sk_traj.Run(num_steps=1200, sk_grasp=sk_grasp, sk_dir=sk_dir, sk_nav=sk_nav, target_name=name_of_the_object, link_idx=link_index, grasp_id=0, global_folder=globalFolder, postfix=None)
                
                            p.disconnect()
                        
                            break
                        
                        except:
                            
                            n_failed +=1
                            if n_failed>10:
                                break         

if __name__ == "__main__":
    
    # Do not forget to change SkillMove 
    # Set appropriate direction vector estimator
    
    rangePosX = 0.1
    rangePosY = 0.1
    rangeAngle = math.pi/24
    rangeOffset = 45
    
    Nx =5
    Ny = 1
    Nang = 5
    Noff = 1
    
    offsetX = np.linspace(-rangePosX, rangePosX, num=Nx)
    offsetY = np.linspace(-rangePosY, rangePosY, num=Ny)
    offsetAngle = np.linspace(-rangeAngle, rangeAngle, num=Nang)
    offsetGrasp = np.linspace(-rangeOffset, rangeOffset, num=Noff)
    
    main(offsetX, offsetY, offsetAngle, offsetGrasp, name_of_the_object = "cupboard")
