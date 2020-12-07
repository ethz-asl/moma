#----- Standard Skills -----

#from highlevel_planning.sim.scene_move_skill import SceneMoveSkill                                        

from highlevel_planning.sim.scene_move_skill_test import SceneMoveSkill                    #THIS IS CHANGED!
from highlevel_planning.skills.navigate_with_offset import SkillNavigate                    #THIS IS CHANGED!

from highlevel_planning.skills.grasping import SkillGrasping
from highlevel_planning.skills.move import SkillMove
from highlevel_planning.tools.config import ConfigYaml
from highlevel_planning.tools import run_util_test                                #THIS IS CHANGED!
from highlevel_planning.tools import run_util

#----- Controllers -----

from highlevel_planning.skills.fixed_base_vel_control import Controller as controller1
from highlevel_planning.skills.fixed_base_torque_control import Controller as controller2
#from highlevel_planning.skills.moving_base_no_collision_vel_control import Controller as controller3
from highlevel_planning.skills.moving_base_no_collision_vel_control_v3 import Controller as controller3
from highlevel_planning.skills.moving_base_no_collision_vel_control_v4 import Controller as controller4

#----- New Skills -----

#from highlevel_planning.skills.direction_estimation import SkillUnconstrainedDirectionEstimation
from highlevel_planning.skills.direction_estimation_v3 import SkillUnconstrainedDirectionEstimation        #THIS IS CHANGED!

from highlevel_planning.skills.door_opening import SkillTrajectoryPlanning
from highlevel_planning.skills.model_fitting import SkillModelIdentification

#----- Utils -----

from highlevel_planning.tools.door_opening_util import *
from highlevel_planning.tools.plot_util import *

#----- Other -----

from scipy.spatial.transform import Rotation as R
from highlevel_planning.tools.util import IKError
from collections import deque

import pybullet as p
import numpy as np
import os
import math

#-----------------

EPS = 1e-6
BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def test():
    
    rangePosX = 0.1
    rangePosY = 0.1
    rangeAngle = math.pi/24
    rangeOffset = 45
    
    Nx =5
    Ny = 3
    Nang = 5
    Noff = 3
    
    offsetX = np.linspace(-rangePosX, rangePosX, num=Nx)
    offsetY = np.linspace(-rangePosY, rangePosY, num=Ny)
    offsetAngle = np.linspace(-rangeAngle, rangeAngle, num=Nang)
    offsetGrasp = np.linspace(-rangeOffset, rangeOffset, num=Noff)
    
    globalFolder = prepare_global_dir()
    
    
    for i in range(Nx):
        #for j in range(Ny):
            for k in range(Nang):
                #for m in range(Noff):
                
                    scenName = str(i)+'_'+str(0)+'_'+str(k)+'_'+str(0)
                    print(50*'=')
                    print('Executing combination: ', scenName)
                    print(50*'=')
                    n_failed = 0
                
                    offsetPos = [offsetX[i], 0.0, 0.0]
                    offsetAng = offsetAngle[k]
                    offsetGr = offsetGrasp[0]
                
                    while(1):        
                            
                        try:
                            # Command line arguments
                            args = run_util_test.parse_arguments()

                            # Load existing simulation data if desired
                            savedir = os.path.join(BASEDIR, "data", "sim")
                            objects, robot_mdl = run_util_test.restore_pybullet_sim(savedir, args)

                            # Load config file
                            cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

                            # ---------- Run examples -----------

                            initLen = 100
                            vRegular = 0.1
                            vInit = vRegular/5
                
                            robot, scene = run_util_test.setup_pybullet_world(
                                SceneMoveSkill, BASEDIR, savedir, objects, args, cfg, robot_mdl, offsetGr
                            )
                
                            sk_grasp = SkillGrasping(scene, robot, cfg)
                            sk_dir = SkillUnconstrainedDirectionEstimation(scene, robot, robot._world.T_s, 50, np.array([0.0, 0.0, -1.0]).reshape(3,1), initLen)    

                            list_of_controllers = []        
                            c1 = controller1(scene, robot, robot._world.T_s)
                            c2 = controller2(scene, robot, robot._world.T_s)
                            c3 = controller3(scene, robot, robot._world.T_s, maxMobility=True, splitAng=False)
                            c4 = controller4(scene, robot, robot._world.T_s, maxMobility=True, splitAng=False, Npolygon=16)
    
                            list_of_controllers.append(c1)
                            #list_of_controllers.append(c2)
                            list_of_controllers.append(c3)
                            list_of_controllers.append(c4)
    
                            sk_traj = SkillTrajectoryPlanning(scene, robot, cfg, list_of_controllers, robot._world.T_s, initLen, vInit, vRegular)
                                
                            sk_nav = SkillNavigate(scene, robot, offsetPos, offsetAng)
                            sk_traj.Run(num_steps=1200, sk_grasp=sk_grasp, sk_dir=sk_dir, sk_nav=sk_nav, target_name="cupboard", link_idx=3, grasp_id=0, global_folder=globalFolder, postfix=scenName)
                
                            p.disconnect()
                        
                            break
                        
                        except:
                            n_failed +=1
                            if n_failed>10:
                                break    
#-------
def main():
    
    offsetPos = [0.0, 0.0, 0.0]
    offsetAng = 0.0
    initLen = 100
    vRegular = 0.1
    vInit = vRegular/4 
    
    name_of_the_object = "roomdoor"
    
    if name_of_the_object == "cupboard":
        
        link_index = 3
        
    elif name_of_the_object == "roomdoor":
        
        link_index = 0
        offsetPos = [0.0, 0.0, 0.0]
        
    # Command line arguments
    args = run_util_test.parse_arguments()

    # Load existing simulation data if desired
    savedir = os.path.join(BASEDIR, "data", "sim")
    objects, robot_mdl = run_util_test.restore_pybullet_sim(savedir, args)

    # Load config file
    cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))
   
    # ---------- Run examples -----------
        
    robot, scene = run_util_test.setup_pybullet_world(
        SceneMoveSkill, BASEDIR, savedir, objects, args, cfg, robot_mdl, -45.0, name_of_the_object
    )
    

    
    list_of_controllers = []    
    fd1 = 1/(50*robot._world.T_s)
                
    sk_grasp = SkillGrasping(scene, robot, cfg)
    sk_nav = SkillNavigate(scene, robot, offsetPos, offsetAng)
    sk_dir = SkillUnconstrainedDirectionEstimation(scene, robot, robot._world.T_s, 100, np.array([0.0, 0.0, -1.0]).reshape(3,1), initLen, fd1)    

    c1 = controller1(scene, robot, robot._world.T_s)
    c2 = controller2(scene, robot, robot._world.T_s)
    c3 = controller3(scene, robot, robot._world.T_s, maxMobility=True, splitAng=False)
    c4 = controller4(scene, robot, robot._world.T_s, maxMobility=True, splitAng=False, Npolygon=256)
    
    list_of_controllers.append(c1)
    list_of_controllers.append(c2)
    list_of_controllers.append(c3)
    list_of_controllers.append(c4)
    
    sk_traj = SkillTrajectoryPlanning(scene, robot, cfg, list_of_controllers, robot._world.T_s, initLen, vInit, vRegular)
    
    globalFolder = prepare_global_dir()
    
    sk_traj.Run(num_steps=1200, sk_grasp=sk_grasp, sk_dir=sk_dir, sk_nav=sk_nav, target_name=name_of_the_object, link_idx=link_index, grasp_id=0, global_folder=globalFolder, postfix=None)

    robot._world.step_seconds(50)        

if __name__ == "__main__":
    # Do not forget to change SkillMove 
    # Set appropriate direction vector estimator
    main()
