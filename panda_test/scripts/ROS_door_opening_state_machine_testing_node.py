#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 25 13:11:26 2021

@author: marko
"""

import rospy
from matplotlib import pyplot as plt
import time 
from datetime import datetime

#----- Skills -----

from panda_test.ROS_optimizer1 import Controller as controller1
#from panda_test.ROS_optimizer2 import Controller as controller2

from panda_test.ROS_direction_estimation import SkillUnconstrainedDirectionEstimation

from panda_test.ROS_planner import RobotPlanner

from panda_test.msg import *

from panda_test.srv import *

from franka_msgs.srv import *

import os
import numpy.linalg as LA

#----- Other -----

import numpy as np

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
        
#        lower_torque = []
#        higher_torque = []
#        lower_force = []
#        higher_force = []
#        
#        succ2 = self.set_force_torque_collision( lower_torque, higher_torque, lower_force, higher_force)
        
#        if not succ2:
#
#            print("")
#            print("Failed to set collision behaviour!")
#            temp1 = input("Abort? [y/n]: ")
#            if temp1 in ['Y', 'y']:
#
#                return 'stop'
#
#            else:
#
#                return 'hold'
            
        temp1 = input('Homing the gripper? [y/n]: ')

        if temp1 in ['y', 'Y']:
            
            grasping_width = 0.01
            grasping_vel = 0.01
            grasping_force = 10  
            grasping_homing = True
            grasping_close = False
            grasping_move = False

            succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force, grasping_homing, grasping_close, grasping_move)
            #succ = True
            
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
                
            grasping_width = 0.008
            grasping_vel = 0.01
            grasping_force = 40
            grasping_homing = False
            grasping_close = True
            grasping_move = False
    
            succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force, grasping_homing, grasping_close, grasping_move)

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
            self.robot.publishArmAndBaseVelocityControl([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1], 3*[0.0], 0.0)
            print("RECEIVED: "+str(panda_model.K_F_ext_hat_K))
            
            counter += 1
        
    def test3(self):

        vFinal = 0.02
        vInit = vFinal/4
        alphaInit = 0.5
        alphaFinal = 0.5

        initN = 100

        tConv = initN
        t0 = np.ceil(tConv/3)

        counter = 0
        N_steps = 150
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
        
    def test5(self):

        vFinal = 0.01
        vInit = vFinal
        alphaInit = 0.5
        alphaFinal = 0.5

        initN = 30

        tConv = initN
        t0 = np.ceil(tConv/3)

        counter = 0
        N_steps = 400        
        
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
        
        robot_dir_estimate = []
        
        folder_name = os.getcwd()
        
        date_and_time = datetime.now()
        curr = date_and_time.strftime("%d_%m_%Y_%H_%M_%S")
        
        folder_name = folder_name + '/runs'
        
        if not os.path.isdir(folder_name):
            os.makedirs(folder_name)
            
        if not os.path.isdir(folder_name + '/' + curr):
            os.makedirs(folder_name + '/' + curr)
            
        self.folder_name = folder_name + '/' + curr
               
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
            
            force = ext_wrench[:3]

            robot_force.append(force)
            iteration_times.append(interval)
            
            robot_q.append(panda_model.q)
            robot_q_d.append(panda_model.q_d)
            robot_dq.append(panda_model.dq)
            robot_dq_d.append(panda_model.dq_d)
            robot_g.append(panda_model.gravity)
            robot_b.append(panda_model.coriolis)

            J_b_ee = np.array(panda_model.jacobian)                     # It is saved as column major but python does everyhing row major
            J_b_ee = np.transpose(J_b_ee.reshape(7, 6))
            manipulabilityMeasure = LA.det(np.matmul(J_b_ee, np.transpose(J_b_ee)))**0.5
            
            robot_manip.append(manipulabilityMeasure)
            
            robot_tau_d_no_g.append(panda_model.tau_d_no_gravity)
            robot_EE_T_K.append(panda_model.EE_T_K)
            robot_O_T_EE.append(panda_model.O_T_EE)
            
            direction = list(np.squeeze(np.array(self.robot.direction_estimator.GetCurrEstimate())))
            robot_dir_estimate.append(direction)
            
            counter += 1 
            
        np.save(self.folder_name +'/force.npy', np.array(force))
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
        
        
    def test6(self):
        
        counter = 0
        N_steps = 30
        freq = rospy.Rate(2) 
        
        force_x = []
        force_y = []
        force_z = []
        
        torque_x = []
        torque_y = []
        torque_z = []
        
        list_of_C = []
        
        list_of_C_mat = []
        
        while counter < N_steps and not rospy.is_shutdown():
        
            print("Iteration: " + str(counter) )
            
            temp1 = input("Press enter to read the force value: ")
            
            req = PandaStateSrvRequest()
            panda_model = self.robot.panda_model_state_srv(req)        
            
            ext_wrench = np.array(panda_model.K_F_ext_hat_K)
            
            force = ext_wrench[:3]
            torque = ext_wrench[3:]
            print('Force: '+str(force))

            force_x.append(force[0])
            force_y.append(force[1])
            force_z.append(force[2])
            
            torque_x.append(torque[0])
            torque_y.append(torque[1])
            torque_z.append(torque[2])
            
            
            T_b_ee = np.array(panda_model.O_T_EE)
            T_b_ee = np.transpose(T_b_ee.reshape(4, 4))

            T_ee_k = np.array(panda_model.EE_T_K)
            T_ee_k = np.transpose(T_ee_k.reshape(4, 4))

            T_b_ee = np.matmul(T_b_ee, T_ee_k) 
            
            C_b_ee = T_b_ee[:3, :3]
            
            list_of_C_mat.append(C_b_ee)
            
            list_of_C.append(list(np.squeeze(C_b_ee.reshape(-1, 1))))
                       
            counter += 1
            
        folder_name = os.getcwd()
        
        np.save(folder_name +'/forcex.npy', np.array(force_x))
        np.save(folder_name +'/forcey.npy', np.array(force_y))
        np.save(folder_name +'/forcez.npy', np.array(force_z))
        
        np.save(folder_name +'/torquex.npy', np.array(torque_x))
        np.save(folder_name +'/torquey.npy', np.array(torque_y))
        np.save(folder_name +'/torquez.npy', np.array(torque_z))
        
        np.save(folder_name +'/C_b_ee.npy', np.array(list_of_C))
        
        b_force_est1, b_torque_est1 = self.calibrate1(force_x, force_y, force_z, torque_x, torque_y, torque_z, list_of_C_mat, inv_sigma = (1/0.035**2)*np.eye(3))
        
        b_force_est2, b_torque_est2 = self.calibrate2(force_x, force_y, force_z, torque_x, torque_y, torque_z)
        
        print('b_force1: ' + str(b_force_est1))
        print('b_torque1: ' + str(b_torque_est1))
        
        print('b_force2: ',b_force_est2)
        print('b_torque2: ',b_torque_est2)
        
        np.save(folder_name +'/b_force1.npy', b_force_est1)
        np.save(folder_name +'/b_torque1.npy', b_torque_est1)

        np.save(folder_name +'/b_force2.npy', b_force_est2)
        np.save(folder_name +'/b_torque2.npy', b_torque_est2)
        
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
        
    def calibrate1(self, force_x, force_y, force_z, torque_x, torque_y, torque_z, list_of_C_mat, inv_sigma = np.eye(3)):
        
        A = np.zeros((3,3))
        B = np.zeros((3,1))
        C = np.zeros((3,1))
        
        N = len(force_x)
        
        for i in range(N):
            
            Ci = list_of_C_mat[i]
            Fi = np.array([force_x[i], force_y[i], force_z[i]]).reshape(-1, 1)
            Ti = np.array([torque_x[i], torque_y[i], torque_z[i]]).reshape(-1, 1)
            
            A = A + np.matmul(np.matmul(np.transpose(Ci), inv_sigma), Ci)
            B = B + np.matmul(np.matmul(np.transpose(Ci), inv_sigma), Fi)
            C = C + np.matmul(np.matmul(np.transpose(Ci), inv_sigma), Ti)
            
        b_force = np.matmul(LA.pinv(A), B)
        b_torque = np.matmul(LA.pinv(A), C)
        
        return np.squeeze(b_force), np.squeeze(b_torque)
    
    def calibrate2(self, force_x, force_y, force_z, torque_x, torque_y, torque_z):
        
        N = len(force_x)
        
        b_force = np.array([0.0, 0.0, 0.0])
        b_torque = np.array([0.0, 0.0, 0.0])
        
        for i in range(N):
            
            Fi = np.array([force_x[i], force_y[i], force_z[i]])
            Ti = np.array([torque_x[i], torque_y[i], torque_z[i]])
            print("Fi ",Fi)
            print("Ti ", Ti)
            
            b_force = b_force + Fi
            
            b_torque = b_torque + Ti
        print("bforce ",b_force)   
        b_force = (1/20.0)*b_force
        b_torque = (1/20.0)*b_torque
        print("bforce ",b_force)
        return b_force, b_torque
     
                         
    def run(self):
                
        try:

            self.test5()
            
            grasping_width = 0.05
            grasping_vel = 0.01
            grasping_force = 2  
            grasping_homing = False 
            grasping_close = False
            grasping_move = True

            succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force, grasping_homing, grasping_close, grasping_move)
                
        except rospy.ROSInterruptException:  pass

        return 'stop'
    
    def transition(self, event):

        if event == 'stop':
            return StopState(self.direction_estimator, self.controller, self.robot, self.folder_name)        
        
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
            robot_dot.append(np.dot(true_dir, estimated_dir))
            
        ax2_3.plot(t, robot_dot, color='b')
        ax2_3.set_ylabel('dot product')
        ax2_3.grid('on')
        ax2_3.set_xlim(t[0], t[-1])        

        fig1.savefig(self.folder_name + '//' + 'forces.jpg')
        fig2.savefig(self.folder_name + '//' + 'metrics.jpg')
        
        plt.close('all')
        
    def run(self):

        self.robot.prepare_for_stop()
        
        grasping_width = 0.05
        grasping_vel = 0.01
        grasping_force = 2  
        grasping_homing = False 
        grasping_close = True
        grasping_move = False

        succ = self.robot.close_gripper(grasping_width, grasping_vel, grasping_force, grasping_homing, grasping_close, grasping_move)

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



