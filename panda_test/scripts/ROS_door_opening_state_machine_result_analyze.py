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
from panda_test.ROS_optimizer2 import Controller as controller2

from panda_test.ROS_direction_estimation import SkillUnconstrainedDirectionEstimation

from panda_test.ROS_planner import RobotPlanner

from panda_test.msg import *

from panda_test.srv import *

from franka_msgs.srv import *

import os
import numpy.linalg as LA

#----- Other -----

import numpy as np

def GenerateImages(door_type):
    
    folder_name = os.getcwd()
    
    folder_name = folder_name + '/GOOD RUNS'
    
    if door_type == 'rotational_door':
        
        folder_sufix = '/Rotational door'
        list_of_positions = ['/Pos_up', '/Pos_middle', '/Pos_down']
        title_pos = [': up', ': middle', ': down']
        
    else:
        
        folder_sufix = '/Drawer door'
        list_of_positions = ['/Pos_left', '/Pos_middle', '/Pos_right']
        title_pos = [': left', ': middle', ': right']
        
    folder_name = folder_name + folder_sufix
    
    list_of_alg = ['/Fixed_Base', '/Mobile_Base']
    
    list_of_config = ['/Init1', '/Init2']
    
    N = 500
    initL = 50
    N_runs = 5
    
    list_of_iteration_times = np.zeros((2,3,2,N_runs,500))
    list_of_manipulability = np.zeros((2,3,2,N_runs,500))
    list_of_EE_T_K = np.zeros((2,3,2,N_runs,500))
    list_of_O_T_EE = np.zeros((2,3,2,N_runs,500))
    list_of_dot = np.zeros((2,3,2,N_runs,500))
    
    for a in range(2):  #Fixed or Mobile Base
        
        alg = list_of_alg[a]
        curr_folder1 = folder_name + alg
        
        if a == 1:
            
            mode = '/moving_base_no_collision_max_mob_control_QCQP'
            
        else:
            
            mode = '/fixed_base_torque_control'
                
        for p in range(3):  #Pos left, middle ,right
            
            pos = list_of_positions[p]
            
            curr_folder2 = curr_folder1 + pos
            
            for c in range(2):  #Init1 or Init2
                
                config = list_of_config[c]
                
                curr_folder3 = curr_folder2 + config 
            
                for r in range(N_runs): #Exp 1,2,3,4,5
                    
                    curr_folder4 = curr_folder3 + '/run_' + str(r+1) + mode
                    
                    force = np.load(curr_folder4 +'/force.npy')
                    iteration_times = np.load(curr_folder4 +'/iteration_times.npy')           
                    robot_q = np.load(curr_folder4 +'/robot_q.npy')  
                    robot_q_d = np.load(curr_folder4 +'/robot_q_d.npy')
                    robot_dq = np.load(curr_folder4 +'/robot_dq.npy')
                    robot_dq_d = np.load(curr_folder4 +'/robot_dq_d.npy')
                    robot_g = np.load(curr_folder4 +'/robot_g.npy')
                    robot_b = np.load(curr_folder4 +'/robot_b.npy')
                    robot_manip = np.load(curr_folder4 +'/robot_manip.npy')
                    robot_tau_d_no_g = np.load(curr_folder4 +'/robot_tau_d_no_g.npy')
                    robot_EE_T_K = np.load(curr_folder4 +'/robot_EE_T_K.npy')
                    robot_O_T_EE = np.load(curr_folder4 +'/robot_O_T_EE.npy')
                    robot_dir_estimate = np.load(curr_folder4 +'/robot_dir_estimate.npy')
                    
                    #robot_dot = []
                    
                    for i in range(N): # 500 sampling intervals
                        
                        EE_T_K = robot_EE_T_K[i, :]
                        EE_T_K = np.transpose(EE_T_K.reshape(4, 4))
                        
                        O_T_EE = robot_O_T_EE[i, :]
                        O_T_EE = np.transpose(O_T_EE.reshape(4, 4))
                        
                        O_T_K = np.matmul(O_T_EE, EE_T_K) 
                        
                        O_C_K = O_T_K[:3, :3]
                        K_C_O = np.transpose(O_C_K)
                        
                        true_dir = np.squeeze(K_C_O[:, 0])
                        estimated_dir = np.squeeze(robot_dir_estimate[i, :])
                        #robot_dot.append(np.dot(-true_dir, estimated_dir))
                        
                        list_of_iteration_times[a, p, c, r, i] = iteration_times[i]
                        list_of_manipulability[a, p, c, r, i] = robot_manip[i]
                        list_of_dot[a, p, c, r, i] = np.dot(-true_dir, estimated_dir)
        
        for i in range(3):
                        
            fig1, (ax1_1, ax1_2, ax1_3) = plt.subplots(3,1,figsize=(15,15))
            fig1.suptitle('Metrics for position ' + title_pos[i], fontsize=15)
            
            Init1_iteration_times = np.squeeze(list_of_iteration_times[a, i, 0, :, :])
            
            Init1_iteration_times_mean = np.mean(Init1_iteration_times, axis=0)
            Init1_iteration_times_std = np.std(Init1_iteration_times, axis=0)
            Init1_iteration_times_upper_limit = Init1_iteration_times_mean + 2*Init1_iteration_times_std
            Init1_iteration_times_lower_limit = Init1_iteration_times_mean - 2*Init1_iteration_times_std
                       
            Init1_manipulability = np.squeeze(list_of_manipulability[a, i, 0, :, :])
            
            Init1_manipulability_mean = np.mean(Init1_manipulability, axis=0)
            Init1_manipulability_std = np.std(Init1_manipulability, axis=0)
            Init1_manipulability_upper_limit = Init1_manipulability_mean + 2*Init1_manipulability_std
            Init1_manipulability_lower_limit = Init1_manipulability_mean - 2*Init1_manipulability_std            
            
            Init1_dot = np.squeeze(list_of_dot[a, i, 0, :, :])
            
            Init1_dot_mean = np.mean(Init1_dot, axis=0) 
            Init1_dot_std = np.std(Init1_dot, axis=0)
            Init1_dot_upper_limit = Init1_dot_mean + 2*Init1_dot_std
            Init1_dot_lower_limit = Init1_dot_mean - 2*Init1_dot_std 
            
            for elem in range(len(Init1_dot_upper_limit)):
                if Init1_dot_upper_limit[elem]>1:
                    Init1_dot_upper_limit[elem] = 1
                                     
            Init2_iteration_times = np.squeeze(list_of_iteration_times[a, i, 1, :, :])
            
            Init2_iteration_times_mean = np.mean(Init2_iteration_times, axis=0)
            Init2_iteration_times_std = np.std(Init2_iteration_times, axis=0)
            Init2_iteration_times_upper_limit = Init2_iteration_times_mean + 2*Init2_iteration_times_std
            Init2_iteration_times_lower_limit = Init2_iteration_times_mean - 2*Init2_iteration_times_std
            
            
            Init2_manipulability = np.squeeze(list_of_manipulability[a, i, 1, :, :])
            
            Init2_manipulability_mean = np.mean(Init2_manipulability, axis=0)
            Init2_manipulability_std = np.std(Init2_manipulability, axis=0)
            Init2_manipulability_upper_limit = Init2_manipulability_mean + 2*Init2_manipulability_std
            Init2_manipulability_lower_limit = Init2_manipulability_mean - 2*Init2_manipulability_std            
            
            Init2_dot = np.squeeze(list_of_dot[a, i, 1, :, :])
            
            Init2_dot_mean = np.mean(Init2_dot, axis=0) 
            Init2_dot_std = np.std(Init2_dot, axis=0)
            Init2_dot_upper_limit = Init2_dot_mean + 2*Init2_dot_std
            Init2_dot_lower_limit = Init2_dot_mean - 2*Init2_dot_std 
            
            for elem in range(len(Init2_dot_upper_limit)):
                if Init2_dot_upper_limit[elem]>1:
                    Init2_dot_upper_limit[elem] = 1

            t = np.arange(1, N+1)
            
            ax1_1.plot(t, Init1_iteration_times_mean, label = 'Configuration 1', linewidth=2.5)
            ax1_1.fill_between(t, Init1_iteration_times_lower_limit, Init1_iteration_times_upper_limit, alpha=0.2)
            ax1_1.plot(t, Init2_iteration_times_mean, label = 'Configuration 2', linewidth=2.5)
            ax1_1.fill_between(t, Init2_iteration_times_lower_limit, Init2_iteration_times_upper_limit, alpha=0.2)

            ax1_1.axvline(x=initL, linewidth=3, color='k', ls='--')
            ax1_1.grid('on')
            ax1_1.set_ylabel(r"$\Delta t_{plann}$", fontsize=15)
            ax1_1.set_ylim(0.02, 0.06)
            ax1_1.set_xlim(t[0], t[-1])
            ax1_1.legend(loc="lower right", fontsize=15)
            
            ax1_2.plot(t, Init1_manipulability_mean, label = 'Configuration 1', linewidth=2.5)
            ax1_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.2)
            ax1_2.plot(t, Init2_manipulability_mean, label = 'Configuration 2', linewidth=2.5)
            ax1_2.fill_between(t, Init2_manipulability_lower_limit, Init2_manipulability_upper_limit, alpha=0.2)

            ax1_2.axvline(x=initL, linewidth=3, color='k', ls='--')
            ax1_2.grid('on')
            ax1_2.set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=15)
            ax1_2.set_xlim(t[0], t[-1])
            ax1_2.legend(loc="lower right", fontsize=15)            

            ax1_3.plot(t, Init1_dot_mean, label = 'Configuration 1', linewidth=2.5)
            ax1_3.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.2)
            ax1_3.plot(t, Init2_dot_mean, label = 'Configuration 2', linewidth=2.5)
            ax1_3.fill_between(t, Init2_dot_lower_limit, Init2_dot_upper_limit, alpha=0.2)

            ax1_3.axvline(x=initL, linewidth=3, color='k', ls='--')
            ax1_3.grid('on')
            ax1_3.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=15)
            ax1_3.set_ylim(0.7, 1.0)
            ax1_3.set_xlim(t[0], t[-1])
            ax1_3.legend(loc="lower right", fontsize=15)   
            
            saving_folder = curr_folder1 + list_of_positions[i]
            
            fig1.savefig(saving_folder + '//' + 'Metrics.png' )  
            plt.close('all')

            f1 = open(saving_folder + '/Table_1', "w")
            f2 = open(saving_folder + '/Table_2', "w")
            N_last =20
            
            start_average_dot1_mat = np.squeeze(list_of_dot[a, i, 0, :, :initL])
            start_average_dot1_mean = np.mean(start_average_dot1_mat, axis=0)
            start_average_dot1 = np.mean(np.squeeze(start_average_dot1_mean))
            
            whole_average_dot1_mat = np.squeeze(list_of_dot[a, i, 0, :, :])
            whole_average_dot1_mean = np.mean(whole_average_dot1_mat, axis=0)
            whole_average_dot1 = np.mean(np.squeeze(whole_average_dot1_mean))
            
            whole_average_manip1_mat = np.squeeze(list_of_manipulability[a, i, 0, :, :])
            whole_average_manip1_mean = np.mean(whole_average_manip1_mat, axis=0)
            whole_average_manip1 = np.mean(np.squeeze(whole_average_manip1_mean))
            
            end_average_manip1_mat = np.squeeze(list_of_manipulability[a, i, 0, :, 500-N_last:])
            end_average_manip1_mean = np.mean(end_average_manip1_mat, axis=0)
            end_average_manip1 = np.mean(np.squeeze(end_average_manip1_mean)) 
            
            total_planning_time1_mat = np.squeeze(list_of_iteration_times[a, i, 0, :, :])
            total_planning_time1 = np.sum(np.mean(total_planning_time1_mat, axis=0))
            
            average_planning_time1 = np.mean(np.mean(total_planning_time1_mat, axis=0))
            
            
            start_average_dot2_mat = np.squeeze(list_of_dot[a, i, 1, :, :initL])
            start_average_dot2_mean = np.mean(start_average_dot2_mat, axis=0)
            start_average_dot2 = np.mean(np.squeeze(start_average_dot2_mean))
                        
            whole_average_dot2_mat = np.squeeze(list_of_dot[a, i, 1, :, :])
            whole_average_dot2_mean = np.mean(whole_average_dot2_mat, axis=0)
            whole_average_dot2 = np.mean(np.squeeze(whole_average_dot2_mean))

            whole_average_manip2_mat = np.squeeze(list_of_manipulability[a, i, 1, :, :])
            whole_average_manip2_mean = np.mean(whole_average_manip2_mat, axis=0)
            whole_average_manip2 = np.mean(np.squeeze(whole_average_manip2_mean))
            
            end_average_manip2_mat = np.squeeze(list_of_manipulability[a, i, 1, :, 500-N_last:])
            end_average_manip2_mean = np.mean(end_average_manip2_mat, axis=0)
            end_average_manip2 = np.mean(np.squeeze(end_average_manip2_mean)) 
            
            total_planning_time2_mat = np.squeeze(list_of_iteration_times[a, i, 1, :, :])
            total_planning_time2 = np.sum(np.mean(total_planning_time2_mat, axis=0))
            
            average_planning_time2 = np.mean(np.mean(total_planning_time2_mat, axis=0))
            
            f1.write("Table 1 for position "+title_pos[i]+" of the "+list_of_alg[a]+"\n")
            f1.write("\n")
            f1.write("Average dot product during init period       : "+str(start_average_dot1)+"\n")
            f1.write("Average dot product during whole procedure   : "+str(whole_average_dot1)+"\n")
            f1.write("Average manipulability during whole procedure: "+str(whole_average_manip1)+"\n")
            f1.write("Average manipulability during end period     : "+str(end_average_manip1)+"\n")
            f1.write("Total planning time                          : "+str(total_planning_time1)+"\n")
            f1.write("Average planning  per iteration              : "+str(average_planning_time1)+"\n")
            
            f2.write("Table 2 for position "+title_pos[i]+" of the "+list_of_alg[a]+"\n")
            f2.write("\n")
            f2.write("Average dot product during init period       : "+str(start_average_dot2)+"\n")
            f2.write("Average dot product during whole procedure   : "+str(whole_average_dot2)+"\n")
            f2.write("Average manipulability during whole procedure: "+str(whole_average_manip2)+"\n")
            f2.write("Average manipulability during end period     : "+str(end_average_manip2)+"\n")
            f2.write("Total planning time                          : "+str(total_planning_time2)+"\n")
            f2.write("Average planning  per iteration              : "+str(average_planning_time2)+"\n")
            
            f1.close()
            f2.close()


#----- main -----

def main():
    
    door_type = 'drawer_door'
    GenerateImages(door_type)
    print("PLOTS CPMPLETED")



if __name__ == "__main__":
    main()



