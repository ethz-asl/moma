#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from matplotlib import pyplot as plt
import time 
from datetime import datetime

#----- Skills -----

from panda_test.ROS_velocity_planner1 import Controller as controller1
from panda_test.ROS_velocity_planner2 import Controller as controller2

from panda_test.ROS_direction_estimation import SkillUnconstrainedDirectionEstimation

from panda_test.ROS_planner import RobotPlanner

from panda_test.msg import *

from panda_test.srv import *

from franka_msgs.srv import *

import os
import numpy.linalg as LA

#----- Other -----

import numpy as np

#----- Description -----

# This is the state machine implementation. It resembles the one provided for the 
# Gazebo simulation. We provide a class for each of the visisted states
# and the state machine class that performs the transitions. Finally, we provide a
# function that analyses the performance of the closed loop system.

#-----------------------

class State(object):

    def __init__(self, direction_estimator=None, controller=None, controller_init=None, robot=None, folder_name=None):

        print(50*'-')
        print('Processing current state: ', str(self))

        self.direction_estimator = direction_estimator
        self.controller = controller
        self.robot = robot
        self.cinit = controller_init
        
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
        buffer_length = 50
        init_direction = np.array([0.0, 0.0, -1.0])
        init_direction = init_direction/LA.norm(init_direction)
        init_direction = init_direction.reshape(3,1)
        initN = 50
        fd1 = 1/(50*time_step)

        self.direction_estimator = SkillUnconstrainedDirectionEstimation(time_step, buffer_length, init_direction, initN, fd1)

        self.controller = controller2(time_step)
        self.cinit = controller1(time_step)

        self.robot = RobotPlanner(self.controller, self.direction_estimator, self.cinit)

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
        rospy.wait_for_service('/franka_control/set_force_torque_collision_behavior')
        print("PANDA_SET_FORCE_COLLISION initiated")
        
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
        
        if not succ:

            print("")
            print("Failed to set the EE and Stiffness frames!")
            temp1 = input("Abort? [y/n]: ")
            if temp1 in ['Y', 'y']:

                return 'stop'

            else:

                return 'hold'
            
        temp1 = input('Homing the gripper? [y/n]: ')

        if temp1 in ['y', 'Y']:
            
            grasping_width = 0.01
            grasping_vel = 0.01
            grasping_force = 20  
            grasping_homing = True
            grasping_close = False
            grasping_move = False

            succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force, grasping_homing, grasping_close, grasping_move)
            
            if not succ:
                
                print("")
                print("Failed to grasp!")
                temp2 = input("Abort? [y/n]: ")
                if temp2 in ['Y', 'y']:
                    
                    return 'stop'
                
                else:
                
                    return 'hold'
                
            temp2 = input("Record the true init direction? [y/n]: ")
            if temp2 in ['Y', 'y']:
                
                grasping_width = 0.007
                grasping_vel = 0.01
                grasping_force = 50
                grasping_homing = False
                grasping_close = True
                grasping_move = False
    
                succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force, grasping_homing, grasping_close, grasping_move)                
                
                self.robot.RecordTrueInitDirection()
            
                grasping_width = 0.01
                grasping_vel = 0.01
                grasping_force = 20  
                grasping_homing = True
                grasping_close = False
                grasping_move = False
    
                succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force, grasping_homing, grasping_close, grasping_move)
                
                succ = input("Move the gripper for the second homming [f]: ")
                
                grasping_width = 0.01
                grasping_vel = 0.01
                grasping_force = 20  
                grasping_homing = True
                grasping_close = False
                grasping_move = False
    
                succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force, grasping_homing, grasping_close, grasping_move) 
            
            return 'gripper_closed'

        else:

            temp2 = input("Abort? [y/n]: ")
            if temp2 in ['Y', 'y']:
                return 'stop'
            else:
                return 'hold'

#-------
    def transition(self, event):

        if event == 'hold':
            return self

        if event == 'gripper_closed':
            return ObjectGraspedState(self.direction_estimator, self.controller, self.cinit, self.robot)

        if event == 'stop':
            return StopState(self.direction_estimator, self.controller, self.cinit, self.robot)

#----- State after the object has been grasped -----

class ObjectGraspedState(State):

    def run(self):

        temp1 = input("Inititate door opening procedure? [y/n]: ")
        if temp1 in ['Y','y']:
                
            grasping_width = 0.007
            grasping_vel = 0.01
            grasping_force = 50
            grasping_homing = False
            grasping_close = True
            grasping_move = False
    
            succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force, grasping_homing, grasping_close, grasping_move)

            return 'start_control'

        else:

            condition = True
            
            while (condition):
                
                aux = input("Manually move the robot base? [y/n]: ")
                if aux in ['y', 'Y']:
                    
                    vx = float(input("vx: "))
                    vy = float(input("vy: "))
                    N_steps = int(input("N: "))
                    
                    counter = 0
                    
                    while counter <= N_steps and not rospy.is_shutdown():
                        print("Iteration: "+str(counter))
                        req = PandaStateSrvRequest()
                        panda_model = self.robot.panda_model_state_srv(req)
                        
                        base_state_msg = self.robot.baseState_msg
                        print("Base pos: "+str([base_state_msg.pose.pose.position.x, base_state_msg.pose.pose.position.y, base_state_msg.pose.pose.position.z]))
                        
                        self.robot.publishArmAndBaseVelocityControl([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [vx, vy, 0.0], 0.0)            
                        counter += 1
                        
                else:
                        
                    condition = False

            temp2 = input('Abort? [y/n]: ')
            if temp2 in ['Y', 'y']:
                return 'stop'
            else:
                return 'hold'

#-------
    def transition(self, event):

        if event == 'hold':
            return self

        if event == 'start_control':
            return RunningState(self.direction_estimator, self.controller, self.cinit, self.robot)

        if event == 'stop':
            return StopState(self.direction_estimator, self.controller, self.cinit, self.robot)
        
#----- Various Running States -----
            
class RunningState(State):
    
    def test1(self):
        
        #----- This function is used to test the communication -----
        
        counter = 0
        N_steps = 1000
        
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
            print("K_F_ext_hat_K: ", panda_model.K_F_ext_hat_K)
            print("EE_T_K: ", panda_model.EE_T_K)
            print("O_T_EE: ", panda_model.O_T_EE)
            print(100*"=")
            
            counter += 1

#-------        
    def test2(self):
        
        #----- This function is used to test simple movement commands -----
        
        # It is used activate individual joints or the mobile base
        
        counter = 0
        N_steps = 50
        
        while counter <= N_steps and not rospy.is_shutdown():
        
            print("Iteration: " + str(counter) )
            
            req = PandaStateSrvRequest()
            panda_model = self.robot.panda_model_state_srv(req)        
            self.robot.publishArmAndBaseVelocityControl([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.05, 0.0, 0.0], 0.0)
            print("RECEIVED: "+str(panda_model.K_F_ext_hat_K))
            
            counter += 1

#-------        
    def test3(self):
        
        #----- This is simple closed loop test without data logging -----

        vFinal = 0.02
        vInit = vFinal/4
        alphaInit = 0.5
        alphaFinal = 0.5

        initN = 100

        tConv = initN
        t0 = np.ceil(tConv/3)

        counter = 0
        N_steps = 150  
        
        while counter <= N_steps and not rospy.is_shutdown():
        
            print("Iteration: " + str(counter) )
        
            self.robot.run_once(counter, vInit, vFinal, alphaInit, alphaFinal, t0, tConv, alpha=0.1, smooth=False, mixCoeff=0.1)
            
            counter += 1

#-------        
    def test4(self):
        
        #----- This function is used to test the force reading at the EE -----

        counter = 0
        N_steps = 1000
        
        force_x = []
        force_y = []
        force_z = []
        
        while counter < N_steps and not rospy.is_shutdown():
        
            print("Iteration: " + str(counter) )
            
            req = PandaStateSrvRequest()
            panda_model = self.robot.panda_model_state_srv(req)        
            
            ext_wrench = np.array(panda_model.K_F_ext_hat_K)
            
            print("EE_T_K: "+str(panda_model.EE_T_K))
            print("O_T_EE: "+str(panda_model.O_T_EE))
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
        
#-------        
    def test5(self):
        
        #===== THIS IS THE ACTUAL RUNNING STATE =====
        
        # It is used to perform the closed loop simulation for both door models.
        # All the data required to generate the plots presented in the thesis are 
        # logged within this state.

        vFinal = 0.01
        vInit = vFinal/2
        alphaInit = 0.5
        alphaFinal = 0.5

        initN = 50

        tConv = initN
        t0 = np.ceil(tConv/3)

        counter = 0
        N_steps = 550        
        
        robot_force = []
        
        iteration_times = []
        
        robot_q = []
        robot_q_d = []
        robot_dq = []
        robot_dq_d = []
        robot_tau_d_no_g = []
        robot_g = []
        robot_b = []
        robot_manip = []
        robot_EE_T_K = []
        robot_O_T_EE = []
        
        robot_Jacobian = []
        robot_base_vel = []
        
        robot_dir_estimate = []
        
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
        
        #----- Perform Init estimation -----
        
        self.robot.InitProgram()
               
        while counter < N_steps and not rospy.is_shutdown():
            
            print(50*'*')
            print("Iteration: " + str(counter) )
            
            startTime = time.time()
        
            self.robot.run_once(counter, vInit, vFinal, alphaInit, alphaFinal, t0, tConv, alpha=0.05, smooth=False, mixCoeff=0.1)
            
            req = PandaStateSrvRequest()
            panda_model = self.robot.panda_model_state_srv(req)
            
            stopTime = time.time()
            interval = stopTime - startTime
            
            ext_wrench = np.array(panda_model.K_F_ext_hat_K)
            
            force = list(np.squeeze(ext_wrench[:3]))

            robot_force.append(force)
            iteration_times.append(interval)
            
            robot_q.append(panda_model.q)
            robot_q_d.append(panda_model.q_d)
            robot_dq.append(panda_model.dq)
            robot_dq_d.append(panda_model.dq_d)
            robot_g.append(panda_model.gravity)
            robot_b.append(panda_model.coriolis)

            J_b_ee = np.array(panda_model.jacobian)                     # It is saved as column major but python does everyhing row major
            robot_Jacobian.append(J_b_ee)
            J_b_ee = np.transpose(J_b_ee.reshape(7, 6))
            manipulabilityMeasure = LA.det(np.matmul(J_b_ee, np.transpose(J_b_ee)))**0.5
            
            robot_manip.append(manipulabilityMeasure)
            
            robot_tau_d_no_g.append(panda_model.tau_d_no_gravity)
            robot_EE_T_K.append(panda_model.EE_T_K)
            robot_O_T_EE.append(panda_model.O_T_EE)
            
            robot_base_vel.append([self.robot.baseState_msg.twist.twist.linear.x, self.robot.baseState_msg.twist.twist.linear.y, 0.0])
            
            direction = list(np.squeeze(np.array(self.robot.direction_estimator.GetCurrEstimate())))
            robot_dir_estimate.append(direction)
            
            counter += 1 
            
        np.save(self.folder_name +'/force.npy', np.array(robot_force))
        np.save(self.folder_name +'/iteration_times.npy', np.array(iteration_times))           
        np.save(self.folder_name +'/robot_q.npy', np.array(robot_q))  
        np.save(self.folder_name +'/robot_q_d.npy', np.array(robot_q_d))
        np.save(self.folder_name +'/robot_dq.npy', np.array(robot_dq))
        np.save(self.folder_name +'/robot_dq_d.npy', np.array(robot_dq_d))
        np.save(self.folder_name +'/robot_g.npy', np.array(robot_g))
        np.save(self.folder_name +'/robot_b.npy', np.array(robot_b))
        np.save(self.folder_name +'/robot_manip.npy', np.array(robot_manip))
        np.save(self.folder_name +'/robot_tau_d_no_g.npy', np.array(robot_tau_d_no_g))
        np.save(self.folder_name +'/robot_EE_T_K.npy', np.array(robot_EE_T_K))
        np.save(self.folder_name +'/robot_O_T_EE.npy', np.array(robot_O_T_EE))
        np.save(self.folder_name +'/robot_dir_estimate.npy', np.array(robot_dir_estimate)) 
        np.save(self.folder_name + '/robot_jacobian.npy', np.array(robot_Jacobian))
        np.save(self.folder_name + '/robot_base_vel.npy', np.array(robot_base_vel))
        np.save(self.folder_name + '/robot_true_init_dir', np.array(self.robot.true_init_dir))
        np.save(self.folder_name + '/robot_ee_world_pos', np.array(self.robot.world_ee_pos))

#-------
    def test7(self):
        
        #----- This function is used to test the initial direction estimation module -----
        
        self.robot.InitProgram()

#-------        
    def test8(self):
        
        self.robot.AlignZAxis()

#-------                             
    def run(self):
                
        try:
            
            waiting = input('START!')
            
            self.test5()
            
            grasping_width = 0.02
            grasping_vel = 0.01
            grasping_force = 2  
            grasping_homing = True 
            grasping_close = False
            grasping_move = False

            succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force, grasping_homing, grasping_close, grasping_move)

            #----- Moving the base back to its original place -----
            
            # This part of the code is used when the external PS4 controller used for
            # the manual movement of the robot base is not available
            
            condition = True
            
            while (condition):
                
                aux = input("Manually move robot base? [y/n]: ")
                if aux in ['y', 'Y']:
                    
                    vx = float(input("vx: "))
                    vy = float(input("vy: "))
                    N_steps = int(input("N: "))
                    
                    counter = 0
                    
                    while counter <= N_steps and not rospy.is_shutdown():
                        print("Iteration: "+str(counter))
                        req = PandaStateSrvRequest()
                        panda_model = self.robot.panda_model_state_srv(req)
                        
                        base_state_msg = self.robot.baseState_msg
                        print("Base pos: "+str([base_state_msg.pose.pose.position.x, base_state_msg.pose.pose.position.y, base_state_msg.pose.pose.position.z]))
                        
                        self.robot.publishArmAndBaseVelocityControl([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [vx, vy, 0.0], 0.0)            
                        counter += 1
                        
                else:
                        
                    condition = False
                
        except rospy.ROSInterruptException:  pass

        return 'stop'

#-------    
    def transition(self, event):

        if event == 'stop':
            return StopState(self.direction_estimator, self.controller, self.cinit, self.robot, self.folder_name)        

#----- Stopping state -----
        
class StopState(State):
    
    def plot_run(self):
        
        folder_name = self.folder_name
        
        force = np.load(self.folder_name +'/force.npy')
        iteration_times = np.load(self.folder_name +'/iteration_times.npy')           
        robot_q = np.load(self.folder_name +'/robot_q.npy')  
        robot_q_d = np.load(self.folder_name +'/robot_q_d.npy')
        robot_dq = np.load(self.folder_name +'/robot_dq.npy')
        robot_dq_d = np.load(self.folder_name +'/robot_dq_d.npy')
        robot_g = np.load(self.folder_name +'/robot_g.npy')
        robot_b = np.load(self.folder_name +'/robot_b.npy')
        robot_manip = np.load(self.folder_name +'/robot_manip.npy')
        robot_tau_d_no_g = np.load(self.folder_name +'/robot_tau_d_no_g.npy')
        robot_EE_T_K = np.load(self.folder_name +'/robot_EE_T_K.npy')
        robot_O_T_EE = np.load(self.folder_name +'/robot_O_T_EE.npy')
        robot_dir_estimate = np.load(self.folder_name +'/robot_dir_estimate.npy')         

        N = len(force)
        t = np.arange(1, N+1)
        
        fig1, (ax1_1, ax1_2, ax1_3) = plt.subplots(3,1,figsize=(10,10))
        
        force_x = force[:, 0]
        force_y = force[:, 1]
        force_z = force[:, 2]
        
        ax1_1.plot(t, force_x, color='b')
        ax1_1.set_ylabel('x force')
        ax1_1.grid('on')
        ax1_1.set_xlim(t[0], t[-1])
        
        ax1_2.plot(t, force_y, color='b')
        ax1_2.set_ylabel('y force')
        ax1_2.grid('on')
        ax1_2.set_xlim(t[0], t[-1])
        
        ax1_3.plot(t, force_z, color='b')
        ax1_3.set_ylabel('z force')
        ax1_3.grid('on')
        ax1_3.set_xlim(t[0], t[-1])
        
        fig2, (ax2_1, ax2_2, ax2_3) = plt.subplots(3,1,figsize=(10,10))
        
        ax2_1.plot(t, iteration_times, color='b')
        ax2_1.set_ylabel('planning interval')
        ax2_1.grid('on')
        ax2_1.set_xlim(t[0], t[-1])
        
        ax2_2.plot(t, robot_manip, color='b')
        ax2_2.set_ylabel('manipulability index')
        ax2_2.grid('on')
        ax2_2.set_xlim(t[0], t[-1])
        
        robot_dot = []
        
        for i in range(N):
            
            EE_T_K = robot_EE_T_K[i, :]
            EE_T_K = np.transpose(EE_T_K.reshape(4, 4))
            
            O_T_EE = robot_O_T_EE[i, :]
            O_T_EE = np.transpose(O_T_EE.reshape(4, 4))
            
            O_T_K = np.matmul(O_T_EE, EE_T_K) 
            
            O_C_K = O_T_K[:3, :3]
            K_C_O = np.transpose(O_C_K)
            
            true_dir = np.squeeze(K_C_O[:, 0])
            estimated_dir = np.squeeze(robot_dir_estimate[i, :])
            robot_dot.append(np.dot(-true_dir, estimated_dir))
            
        ax2_3.plot(t, robot_dot, color='b')
        ax2_3.set_ylabel('dot product')
        ax2_3.grid('on')
        ax2_3.set_xlim(t[0], t[-1])        

        fig1.savefig(self.folder_name + '//' + 'forces.jpg')
        fig2.savefig(self.folder_name + '//' + 'metrics.jpg')
        
        plt.close('all')

#-------        
    def run(self):

        self.robot.prepare_for_stop()
        
        grasping_width = 0.05
        grasping_vel = 0.01
        grasping_force = 2  
        grasping_homing = False 
        grasping_close = True
        grasping_move = False
        
        try:
            succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force, grasping_homing, grasping_close, grasping_move)
            
        except:
            
            self.plot_run()
           
            
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



