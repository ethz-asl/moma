#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  2 13:42:10 2021

@author: marko
"""
import rospy

#----- Skills -----

from moma_gazebo.ROS_optimizer1 import Controller as controller1
from moma_gazebo.ROS_optimizer2 import Controller as controller2

from moma_gazebo.ROS_direction_estimation import SkillUnconstrainedDirectionEstimation

from moma_gazebo.ROS_planner import RobotPlanner

#----- Other -----

import numpy as np
import os
from datetime import datetime
import time
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

class State(object):
    
    def __init__(self, direction_estimator=None, controller=None, robot=None, folder_name=None):
        
        print(50*'-')
        print('Processing current state: ', str(self))
        
        self.direction_estimator = direction_estimator
        self.controller = controller
        self.robot = robot
        
        self.folder_name = folder_name
        
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

        self.robot = RobotPlanner(self.controller, self.direction_estimator, time_step)
        
        #----- Waiting for all the topics to start publishing -----
        
        print("")
        print("Waiting for all the subscribed topics and services to be initiated...")
        waiting = True 
        rospy.sleep(1.0)
        
        rospy.wait_for_service('/get_panda_state_srv')
        rospy.wait_for_service('/panda_init_srv')
        rospy.wait_for_service('/robot_gripper_srv')
        
        while(waiting):
            
            if self.robot.arm_state_topic_initiated and self.robot.base_state_topic_initiated and self.robot.ee_force_topic_initiated and self.robot.ee_state_topic_initiated:
                waiting = False
            
            if waiting:
                
                print(50*"*")
                print(" Topics published so far: ")
                print("- /joint_states : "      + str(self.robot.arm_state_topic_initiated)) 
                print("- /odom : "              + str(self.robot.base_state_topic_initiated))
                print("- /eeforce : "           + str(self.robot.ee_force_topic_initiated))            
                print("- /gazebo/link_state : " + str(self.robot.ee_state_topic_initiated))
            
                temp1 = input('Retry? [y/n]: ')
                
                if temp1 in ['n', 'N']:
                    
                    return 'stop'
        
        #----- State machine can now start working -----
        
        print("Setting EE and K referance frames...")
        
        NE_T_EE = [1.0, 0.0, 0.0 ,0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.015, 1.0]
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
        
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        urdf_filename = os.path.join(base_dir, "data/models/box_panda_hand_pb.urdf")
        
        robot_base = [0.0, 0.0, 0.0]
        robot_orientation = [0., 0., 0., 1.]
            
        id_simulator, id_robot, joint_idx_arm, joint_idx_fingers, joint_idx_hand, arm_base_link_idx, arm_ee_link_idx, link_name_to_index = self.robot.InitURDF(time_step, urdf_filename, robot_base, robot_orientation)                
        
        counter = 0
        N_steps = 400
        
        robot_dq = []
        robot_v = []
        
        iteration_times = []
        
        folder_name = os.getcwd()
        
        date_and_time = datetime.now()
        curr = date_and_time.strftime("%d_%m_%Y_%H_%M_%S")
        
        folder_name = folder_name + '/runs'
        
        if not os.path.isdir(folder_name):
            os.makedirs(folder_name)
            
        if not os.path.isdir(folder_name + '/' + curr):
            os.makedirs(folder_name + '/' + curr)
        
        folder_name = folder_name + '/' + curr + '/' + self.robot.controller.mode_name
        
        if not os.path.isdir(folder_name):
            os.makedirs(folder_name) 
            
        self.folder_name = folder_name
        
        print("")
        try:
            
            while counter <= N_steps and not rospy.is_shutdown():
                print("Iteration: " + str(counter) )
                
                startTime = time.time()
                
                self.robot.run_once(counter, vInit, vFinal, alphaInit, alphaFinal, t0, tConv, id_robot, arm_ee_link_idx, arm_base_link_idx, link_name_to_index, joint_idx_arm, alpha=0.1, smooth=False, mixCoeff=0.1)  
            
                stopTime = time.time()
                interval = stopTime - startTime
                
                iteration_times.append(interval)
                robot_dq.append(self.robot.q_dot)

                v_total = list(np.squeeze(np.array(self.robot.linVelBase) + np.squeeze(np.matmul(self.robot.J_b_ee[:3,:7], self.robot.q_dot))))
                C_O_b = R.from_dcm(self.robot.T_O_b[:3, :3])
                robot_v.append(C_O_b.apply(v_total[:3]))
                
                counter += 1

            np.save(self.folder_name +'/robot_dq.npy', np.array(robot_dq))
            np.save(self.folder_name +'/iteration_times.npy', np.array(iteration_times))
            np.save(self.folder_name +'/robot_v.npy', np.array(robot_v))
                
        except rospy.ROSInterruptException:  pass
        
        return 'stop'

    def transition(self, event):
        
        if event == 'stop':            
            return StopState(self.direction_estimator, self.controller, self.robot, self.folder_name) 
            
#----- Finishing state -----
            
class StopState(State):
    
    def plot_run(self):
        
        folder_name = self.folder_name
        
        iteration_times = np.load(self.folder_name +'/iteration_times.npy')           
        robot_dq = np.load(self.folder_name +'/robot_dq.npy')
        robot_v = np.load(self.folder_name +'/robot_v.npy')

        N = len(iteration_times)
        t = np.arange(1, N+1)
        
        fig1, (ax1_1, ax1_2, ax1_3) = plt.subplots(3,1,figsize=(10,10))
        
        v_x = robot_v[:, 0]
        v_y = robot_v[:, 1]
        v_z = robot_v[:, 2]
        
        ax1_1.plot(t, v_x, color='b')
        ax1_1.set_ylabel('x velocity')
        ax1_1.grid('on')
        ax1_1.set_xlim(t[0], t[-1])
        
        ax1_2.plot(t, v_y, color='b')
        ax1_2.set_ylabel('y velocity')
        ax1_2.grid('on')
        ax1_2.set_xlim(t[0], t[-1])
        
        ax1_3.plot(t, v_z, color='b')
        ax1_3.set_ylabel('z velocity')
        ax1_3.grid('on')
        ax1_3.set_xlim(t[0], t[-1])
        
        fig2 = plt.figure(2)
        
        plt.plot(t, iteration_times, color='b')
        plt.ylabel('iteration time')
        plt.grid('on')
        plt.xlim(t[0], t[-1])
        plt.ylim(0.0, 0.03)
        
        fig1.savefig(self.folder_name + '//' + 'velocities.jpg')
        fig2.savefig(self.folder_name + '//' + 'iteration_times.jpg')
        
        plt.close('all')      

    def run(self):
        
        self.robot.prepare_for_stop()
        
        self.plot_run()
        
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

                   
        
            
            
            
            
            
            
        