#!/usr/bin/env python

import rospy

#----- Skills -----

from ROS_optimizer1 import Controller as controller1
from ROS_optimizer2 import Controller as controller2
from ROS_optimizer3 import Controller as controller3

from ROS_direction_estimation import SkillUnconstrainedDirectionEstimation

#----- ROS Msgs ------

from panda_control_v2.msg import Command
from panda_control_v2.msg import FullState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

#----- Other -----

from scipy.spatial.transform import Rotation as R

import numpy as np
import os
import math
import time

class RobotPlanner:

    def __init__(self, controller, direction_estimatior, time_step):
        
        self.dt = time_step
        self.controller = controller
        self.direction_estimator = direction_estimatior
        
        self.processing = False
        
        self.armState_msg = None
	self.baseState_msg = None
        
        #----- Robot state -----
        
        self.M = None
        self.b = None
        self.q = None
        self.qdot = None
        self.tau = None
        self.J_b_ee = None
        self.T_b_ee = None
        self.force = None
        self.torque = None
        
        self.linVelBase = None
        self.angVelBase = None
        self.T_O_b = None
        
        #----- Subsrcriber and publishers -----
        
        self.subscriber_arm_state = rospy.Subscriber('full_arm_state', FullState, self.armState_cb)
	self.subscriber_base_state = rospy.Subscriber('odom', Odometry , self.baseState_cb)
	
        self.publisher_arm_velocity = rospy.Publisher('set_command', Command, queue_size=10)
	self.publisher_base_velocity = rospy.Publisher('cmd_vel', Twist, queue_size=10)
       
    def armState_cb(self, msg):
        
        if not self.processing:
            
            self.armState_msg = msg

    def baseState_cb(self, msg):

	if not self.processing:

	    self.baseState_msg = msg
            
    def publishArmAndBaseVelocities(self, q_dot_des, linVelBase, angVelBase):
        
        armAndBase_msg = arm_and_base_msg()
        
        armAndBase_msg.q_dot = q_dot_des
        armAndBase_msg.linVelBase = linVelBase
        armAndBase_msg.angVelBase = angVelBase
        
        self.publisher_arm_and_base_velocity.publish(armAndBase_msg)
        
    def VelocityProfile(self, t, vInit, vFinal, alphaInit, alphaFinal, t0, tConv):
    
        if t<t0:
        
            v = vInit * 2.0/math.pi*np.arctan(alphaInit*t)
        
        else:
        
            a1 = (vFinal - vInit*2.0/math.pi*np.arctan(alphaInit*t0))/(1.0 - 2.0/math.pi*np.arctan(alphaFinal*(t0 - tConv)))    
            a2 = vFinal - a1
        
            v = a1 * 2.0/math.pi*np.arctan(alphaFinal*(t - tConv)) + a2
        
        return v
        
    def run_once(self, t, vInit, vFinal, alphaInit, alphaFinal, t0, tConv, alpha=0.1, smooth=False, mixCoeff=0.1):
        
        #----- Copying -----
        
        curr_arm_msg = FullState()
	curr_base_msg = 
        
        self.processing = True
        curr_msg = self.armState_msg
        self.processing = False
        
        #----- Arm info -----
            
        self.M = np.array(curr_msg.M).reshape(7,7)
        self.b = np.array(curr_msg.b)
        self.q = np.array(curr_msg.q)
        self.qdot = np.array(curr_msg.qdot)
        self.tau = np.array(curr_msg.tau)
        self.J_b_ee = np.array(curr_msg.J_b_ee).reshape(6,7)
        self.T_b_ee = np.array(curr_msg.T_b_ee).reshape(4,4)
        self.force = curr_msg.f
        self.torque = curr_msg.t
            
        #----- Base info -----
            
        self.linVelBase = np.array(curr_msg.linVelBase)
        self.angVelBase = np.array(curr_msg.anggVelBase)
        self.T_O_b = np.array(curr_msg.T_O_b).reshape(4,4)
        
        #----- Calculate Info -----
        
        T_O_ee = np.matmul(self.T_O_b, self.T_b_ee)  
        
        C_O_ee = R.from_matrix(T_O_ee[:3, :3])
        C_O_b = R.from_matrix(self.T_O_b[:3, :3])
        C_b_ee = R.from_matrix(self.T_b_ee[:3, :3])
        
        r_O_ee = np.squeeze(np.copy(T_O_ee[:3, 3]))
            
        #----- Update Buffers in direction_estimator class -----
                     
        self.direction_estimator.UpdateBuffers(self.force, r_O_ee)
        
        #----- Processing -----
        
        self.direction_estimator.UpdateEstimate(self.force, alpha, C_O_ee, smooth, mixCoeff)
        
        velProfile = self.VelocityProfile(t, vInit, vFinal, alphaInit, alphaFinal, t0, tConv)
        
        veldesEE_ee = self.direction_estimator.GetPlannedVelocities(v=velProfile, calcAng=False, kAng=1)
        
        infoTuple = (self.M, self.b, self.J_b_ee, self.q, self.q_dot, C_O_b, C_O_ee, C_b_ee, self.tau)
        
        _ = self.controller.PerformOneStep(veldesEE_ee, infoTuple)
        
        self.publishArmAndBaseVelocities(self.controller.GetCurrOptSol(), linVelBase = self.controller.vLinBase_b, angVelBase = self.controller.vLinBase_b[2])
        
def main():
    
    #----- Parameters for direction estimation -----
    time_step = 0.08
    buffer_length = 100
    init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
    initN = 100
    fd1 = 1/(50*time_step)
    
    direction_estimator = SkillUnconstrainedDirectionEstimation(time_step, buffer_length, init_direction, initN, fd1)
    
    #----- Parameters for the controller -----
    
    N_steps = 1200   
    controller = controller1(time_step)
    
    #----- Planner object -----
    
    vFinal = 0.1
    vInit = vFinal/4
    alphaInit = 0.5
    alphaFinal = 0.5
    
    tConv = initN
    t0 = np.ceil(tConv/3)
    robot = RobotPlanner(controller, direction_estimator, time_step)
    
    #----- Init node -----
    
    rospy.init_node('ROS_planner_node')
    
    counter = 0
    
    try:
        while counter<=N_steps and not rospy.is_shutdown():
            
            t = counter
            robot.run_once(t, vInit, vFinal, alphaInit, alphaFinal, t0, tConv, alpha=0.1, smooth=False, mixCoeff=0.1)         
            counter += 1
            
    except rospy.ROSInterruptException:  pass

if __name__ == "__main__":
    main()
