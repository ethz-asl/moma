import pybullet as p
import numpy as np

from highlevel_planning.tools.door_opening_util import *

from highlevel_planning.sim.controllers.controller import ControllerTemplate

EPS = 1e-6
DEBUG = True

#----- Description -----

# This controller uses the built in procedure to perform the EE movement by 
# commanding a desired velocity expressed in the Cartesian frame. Here, no 
# hardware imposed constraints are taken into account. This is basically a dummy
# version of the algorithm used to test if everything is working properly in the 
# rest of the code.

#----------------------- 

class Controller(ControllerTemplate):

    def __init__(self, scene, robot, time_step):
        
        super(Controller, self).__init__(scene, robot, time_step)
        
        self.mode_name = "fixed_base_cartesian_velocity_control"
        
#-------
    def PerformOneStep(self, veldesEE_ee, infoTuple=None):
        
        self.q_dot_optimal = infoTuple[4]
        
        p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_fingers, p.TORQUE_CONTROL, forces=[-25.0]*2)
        
        vLindesEE_ee = np.squeeze(veldesEE_ee[:3])
        vAngdesEE_ee = np.squeeze(veldesEE_ee[3:])
        
        self.robot.task_space_velocity_control(vLindesEE_ee, vAngdesEE_ee, 1)
        

        
        
    
        
        
        
        
         
        
        
        
        
        
        
        
                         
                         
                
        