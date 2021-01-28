#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  2 13:42:10 2021

@author: marko
"""
import rospy

#----- Skills -----

from panda_test.ROS_optimizer1 import Controller as controller1
from panda_test.ROS_optimizer2 import Controller as controller2

from panda_test.ROS_direction_estimation import SkillUnconstrainedDirectionEstimation

from panda_test.ROS_planner import RobotPlanner

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

        rospy.wait_for_service('/get_panda_state_srv')
        rospy.wait_for_service('/panda_init_srv')
        rospy.wait_for_service('/robot_gripper_srv')

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

        NE_T_EE = [1.0, 0.0, 0.0 ,0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.15, 1.0]
        EE_T_K  = [1.0, 0.0, 0.0 ,0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        succ = self.robot.set_frames(NE_T_EE, EE_T_K)

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
            
            grasping_width = 0.04
            grasping_vel = 0.1
            grasping_force = 60            

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

#----- State when the control loop is running -----

class RunningState(State):

    def run(self):

        time_step = 0.05

        freq = rospy.Rate(1/time_step)

        vFinal = 0.1
        vInit = vFinal/4
        alphaInit = 0.5
        alphaFinal = 0.5

        initN = 100

        tConv = initN
        t0 = np.ceil(tConv/3)

        counter = 0
        N_steps = 1200

        print("")
        try:

            while counter <= N_steps and not rospy.is_shutdown():
                print("Iteration: " + str(counter) )
                self.robot.run_once(counter, vInit, vFinal, alphaInit, alphaFinal, t0, tConv, alpha=0.1, smooth=False, mixCoeff=0.1)
                #freq.sleep()
                counter += 1

        except rospy.ROSInterruptException:  pass

        return 'stop'

    def transition(self, event):

        if event == 'stop':
            return StopState(self.direction_estimator, self.controller, self.robot)

#----- Finishing state -----

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









