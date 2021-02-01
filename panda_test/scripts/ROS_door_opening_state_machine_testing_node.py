#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 25 13:11:26 2021

@author: marko
"""

import rospy
from matplotlib import pyplot as plt

#----- Skills -----

from panda_test.ROS_optimizer1 import Controller as controller1
#from panda_test.ROS_optimizer2 import Controller as controller2

from panda_test.ROS_direction_estimation import SkillUnconstrainedDirectionEstimation

from panda_test.ROS_planner import RobotPlanner

from panda_test.msg import *

from panda_test.srv import *

from franka_msgs.srv import *

#----- Other -----

import numpy as np

class State(object):

    def __init__(self, direction_estimator=None, controller=None, robot=None):

        print(50*'-')
        print('Processing current state: ', str(self))

        self.direction_estimator = direction_estimator
        self.controller = controller
        self.robot = robot

    def run(self):

        pass

    def next_state(self, event):

        pass

    def __repr__(self):

        return self.__str__()

    def __str__(self):

        return self.__class__.__name__

#----- States -----

class StartState(State):

    def run(self):

        time_step = 0.05
        buffer_length = 100
        init_direction = np.array([0.0, 0.0, -1.0]).reshape(3,1)
        initN = 100
        fd1 = 1/(50*time_step)

        self.direction_estimator = SkillUnconstrainedDirectionEstimation(time_step, buffer_length, init_direction, initN, fd1)

        self.controller = controller1(time_step)

        self.robot = RobotPlanner(self.controller, self.direction_estimator)

        #----- Waiting for all the topics to start publishing -----

        print("")
        print("Waiting for all the subscribed topics and services to be initiated...")
        waiting = True
        rospy.sleep(1.0)
        
        
        rospy.wait_for_service('/franka_control/set_EE_frame')
        print("SET_EE_FRAME initiated")
        rospy.wait_for_service('/franka_control/set_K_frame')
        print("SET_K_FRAME inititated")
        rospy.wait_for_service('/panda_state_srv')
        print("PANDA_STATE_SRV initiated")
        rospy.wait_for_service('/robot_gripper_srv')
        print("PANDA_GRIPPER_SRV initiated")
        
        print(10*'*'+" All services started! "+10*'*')

        while(waiting):

            if self.robot.base_state_topic_initiated:
                waiting = False

            if waiting:

                print("- /odom : "              + str(self.robot.base_state_topic_initiated))

                temp1 = input('Retry? [y/n]: ')

                if temp1 in ['n', 'N']:

                    return 'stop'

        #----- State machine can now start working -----

        print("Setting EE and K referance frames...")

        F_T_EE = [1.0, 0.0, 0.0 ,0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.15, 1.0]
        EE_T_K  = [1.0, 0.0, 0.0 ,0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        succ = self.robot.set_frames(F_T_EE, EE_T_K)
        #succ = True
        
        if not succ:

            print("")
            print("Failed to set the EE and Stiffness frames!")
            temp1 = input("Abort? [y/n]: ")
            if temp1 in ['Y', 'y']:

                return 'stop'

            else:

                return 'hold'

        temp1 = input('Close the gripper? [y/n]: ')

        if temp1 in ['y', 'Y']:
            
            grasping_width = 0.03
            grasping_vel = 0.1
            grasping_force = 2            

            succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force)
            
            if not succ:
                
                print("")
                print("Failed to grasp!")
                temp2 = input("Abort? [y/n]: ")
                if temp2 in ['Y', 'y']:
                    
                    return 'stop'
                
                else:
                
                    return 'hold'            
            
            return 'gripper_closed'

        else:

            temp2 = input("Abort? [y/n]: ")
            if temp2 in ['Y', 'y']:
                return 'stop'
            else:
                return 'hold'

    def transition(self, event):

        if event == 'hold':
            return self

        if event == 'gripper_closed':
            return ObjectGraspedState(self.direction_estimator, self.controller, self.robot)

        if event == 'stop':
            return StopState(self.direction_estimator, self.controller, self.robot)

#----- State after the object has been grasped -----

class ObjectGraspedState(State):

    def run(self):

        temp1 = input("Inititate door opening procedure? [y/n]: ")
        if temp1 in ['Y','y']:

            return 'start_control'

        else:

            temp2 = input('Abort? [y/n]: ')
            if temp2 in ['Y', 'y']:
                return 'stop'
            else:
                return 'hold'

    def transition(self, event):

        if event == 'hold':
            return self

        if event == 'start_control':
            return RunningState(self.direction_estimator, self.controller, self.robot)

        if event == 'stop':
            return StopState(self.direction_estimator, self.controller, self.robot)
        
#----- Simplified Running State -----
            
class RunningState(State):
    
    def test1(self):
        
        counter = 0
        N_steps = 1000
        freq = rospy.Rate(2) 
        
        while counter <= N_steps and not rospy.is_shutdown():
        
            print("Iteration: " + str(counter) )
            
            req = PandaStateSrvRequest()
            panda_model = self.robot.panda_model_state_srv(req)
            
            print("q: ", panda_model.q)
            print("dq: ", panda_model.dq)
            print("q_d: ", panda_model.q_d)
            print("dq_d: ", panda_model.dq_d)
            print("ddq_d: ", panda_model.ddq_d)
            print("tau: ", panda_model.tau)
            print("tau_ext: ", panda_model.tau_ext)
            print("tau_d_no_gravity: ", panda_model.tau_d_no_gravity)
            print("coriolis: ", panda_model.coriolis)
            print("gravity: ", panda_model.gravity)
            #print("jacobian: ", panda_model.jacobian)
            #print("mass_matrix: ", panda_model.mass_matrix)
            print("K_F_ext_hat_K: ", panda_model.K_F_ext_hat_K)
            print("EE_T_K: ", panda_model.EE_T_K)
            print("O_T_EE: ", panda_model.O_T_EE)
            print(100*"=")
            
            counter += 1
        
    def test2(self):
        
        counter = 0
        N_steps = 1000
        freq = rospy.Rate(2) 
        
        while counter <= N_steps and not rospy.is_shutdown():
        
            print("Iteration: " + str(counter) )
            
            req = PandaStateSrvRequest()
            panda_model = self.robot.panda_model_state_srv(req)        
            self.robot.publishArmAndBaseVelocityControl([0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1], 3*[0.0], 3*[0.0])
            print("RECEIVED: "+str(panda_model.K_F_ext_hat_K))
            
            counter += 1
        
    def test3(self):

        vFinal = 0.01
        vInit = vFinal/4
        alphaInit = 0.5
        alphaFinal = 0.5

        initN = 100

        tConv = initN
        t0 = np.ceil(tConv/3)

        counter = 0
        N_steps = 1000
        freq = rospy.Rate(2)         
        
        while counter <= N_steps and not rospy.is_shutdown():
        
            print("Iteration: " + str(counter) )
        
            self.robot.run_once(counter, vInit, vFinal, alphaInit, alphaFinal, t0, tConv, alpha=0.1, smooth=False, mixCoeff=0.1)
            
            counter += 1
        
    def test4(self):

        counter = 0
        N_steps = 1000
        freq = rospy.Rate(2) 
        
        force_x = []
        force_y = []
        force_z = []
        
        while counter < N_steps and not rospy.is_shutdown():
        
            print("Iteration: " + str(counter) )
            
            req = PandaStateSrvRequest()
            panda_model = self.robot.panda_model_state_srv(req)        
            
            ext_wrench = np.array(panda_model.K_F_ext_hat_K)
            force = ext_wrench[:3]

            force_x.append(force[0])
            force_y.append(force[1])
            force_z.append(force[2])
            
            counter += 1
            
        N = N_steps
        t = np.arange(1, N+1)
        
        fig1, (ax1_1, ax1_2, ax1_3) = plt.subplots(3,1,figsize=(10,10))
        
        ax1_1.plot(t, force_x)
        ax1_1.set_ylabel('x force')
        ax1_1.grid('on')
        ax1_1.set_xlim(t[0], t[-1])
        
        ax1_2.plot(t, force_y)
        ax1_2.set_ylabel('y force')
        ax1_2.grid('on')
        ax1_2.set_xlim(t[0], t[-1])
        
        ax1_3.plot(t, force_z)
        ax1_3.set_ylabel('z force')
        ax1_3.grid('on')
        ax1_3.set_xlim(t[0], t[-1])
        
        plt.show()

       
    def run(self):
                
        try:

            self.test4()
                
        except rospy.ROSInterruptException:  pass

        return 'stop'
    
    def transition(self, event):

        if event == 'stop':
            return StopState(self.direction_estimator, self.controller, self.robot)        
        
class StopState(State):

    def run(self):

        self.robot.prepare_for_stop()

        temp1 = input("Restart? [y/n]: ")

        if temp1 in ['Y','y']:

            return 'restart'

        else:

            return 'hold'

    def transition(self, event):

        if event == 'hold':
            return self

        if event == 'restart':
            return StartState()

#----- State Machine -----

class DoorOpenningStateMachine:

    def __init__(self):

        rospy.init_node("ROS_door_openning_state_machine_node")

        self.state = StartState()
        self.event = 'hold'

    def run_machine(self):

        while(1):

            self.event = self.state.run()
            self.state = self.state.transition(self.event)

#----- main -----

def main():

    machine = DoorOpenningStateMachine()

    machine.run_machine()

if __name__ == "__main__":
    main()



