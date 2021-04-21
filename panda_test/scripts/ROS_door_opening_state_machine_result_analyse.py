#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from matplotlib import pyplot as plt
import math

#----- Skills -----

from panda_test.msg import *

from panda_test.srv import *

from franka_msgs.srv import *

import os

#----- Other -----

import numpy as np

#----- Description -----

# This script is used to analyse and plot all the results used in the report

#-----------------------

def GenerateImages(door_type, plot_vel=False):
    
    folder_name = os.getcwd()
    
    folder_name = folder_name + '/GOOD RUNS'
    
    if door_type == 'rotational_door':
        
        folder_sufix = '/Rotational door'
        list_of_positions_init = ['/Pos']
        title_pos = [': middle', ': middle', ': middle']
        list_of_config_init = ['/Init1']
        N_runs_init = 10
        
    else:
        
        folder_sufix = '/Drawer door'
        list_of_positions_init = ['/Pos_left', '/Pos_middle', '/Pos_right']
        title_pos = [': left', ': middle', ': right', ': middle', ': middle']
        list_of_config_init = ['/Init1', '/Init2']
        N_runs_init = 5
        
    folder_name = folder_name + folder_sufix
    
    list_of_alg = ['/Fixed_Base', '/Mobile_Base']
        
    N = 500
    initL = 50
    
    N_runs_vel = 5
    
    list_of_abs_vel = []
    
    tConv = 50
    t0 = np.ceil(tConv/3)
    vInit = 0.005
    alphaInit = 0.5
    alphaFinal= 0.5
    vFinal = 0.01
    
    
    for i in range(1,N+1):
        t = i 
        if t<t0:

            vd = vInit * 2.0/math.pi*np.arctan(alphaInit*t)

        else:

            a1 = (vFinal - vInit*2.0/math.pi*np.arctan(alphaInit*t0))/(1.0 - 2.0/math.pi*np.arctan(alphaFinal*(t0 - tConv)))
            a2 = vFinal - a1

            vd = a1 * 2.0/math.pi*np.arctan(alphaFinal*(t - tConv)) + a2
            
        list_of_abs_vel.append(vd)
       
    list_of_iteration_times = np.zeros((2,5,2,N_runs_init,500))
    list_of_manipulability = np.zeros((2,5,2,N_runs_init,500))
    list_of_EE_T_K = np.zeros((2,5,2,N_runs_init,500))
    list_of_O_T_EE = np.zeros((2,5,2,N_runs_init,500))
    list_of_dot = np.zeros((2,5,2,N_runs_init,500))
    
    v_mat = np.zeros((2, N_runs_vel, 500, 9))
    
    for a in range(len(list_of_alg)):  #Fixed or Mobile Base
        
        alg = list_of_alg[a]
        curr_folder1 = folder_name + alg
        
        if a == 1:
            
            mode = '/moving_base_no_collision_max_mob_control_QCCO'
            list_of_positions.append('/Runs complete algorithm')
            list_of_positions.append('/Runs for velocity plot')
            
        else:
            
            mode = '/fixed_base_torque_control'
            list_of_positions = list_of_positions_init
            
                
        for p in range(len(list_of_positions)):  #Pos left, middle ,right
            pos = list_of_positions[p]

            curr_folder2 = curr_folder1 + pos

            if list_of_positions[p] in ['/Runs complete algorithm', '/Runs for velocity plot']:
                
                list_of_config = ['/Init1']
                N_runs = 5
                
            else:
                
                list_of_config = list_of_config_init
                N_runs = N_runs_init

            for c in range(len(list_of_config)):  #Init1 or Init2
                
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
                    
                    if list_of_positions[p] in ['/Runs complete algorithm', '/Runs for velocity plot']:
                        
                        robot_base_vel = np.load(curr_folder4 + '/robot_base_vel.npy')
                        robot_jacobian = np.load(curr_folder4 + '/robot_jacobian.npy')
                    
                    if door_type == 'rotational_door':
                        
                        robot_ee_world_pos = np.load(curr_folder4 + '/robot_ee_world_pos.npy')
                        array_x = np.squeeze(np.copy(robot_ee_world_pos[:, 0]))
                        array_y = np.squeeze(np.copy(robot_ee_world_pos[:, 1]))
                        
                        A = []
                        b = []
                        
                        for i in range(N):
                            
                            A.append([1.0, -2*array_x[i], -2*array_y[i]])
                            b.append(-array_x[i]**2-array_y[i]**2)
                            
                        A = np.array(A)
                        b = np.array(b)
                        opt_circle = np.squeeze(np.matmul(np.linalg.pinv(A), b))
                        
                        R = (opt_circle[1]**2 + opt_circle[2]**2 - opt_circle[0])**0.5 
                        xc = opt_circle[1]
                        yc = opt_circle[2]

                    for i in range(N): # 500 sampling intervals
                        
                        EE_T_K = robot_EE_T_K[i, :]
                        EE_T_K = np.transpose(EE_T_K.reshape(4, 4))
                        
                        O_T_EE = robot_O_T_EE[i, :]
                        O_T_EE = np.transpose(O_T_EE.reshape(4, 4))
                        
                        O_T_K = np.matmul(O_T_EE, EE_T_K) 
                        
                        O_C_K = O_T_K[:3, :3]
                        K_C_O = np.transpose(O_C_K)
                        
                        if door_type == 'rotational_door':
                            
                            fi = math.atan2(robot_ee_world_pos[i, 1] - yc, robot_ee_world_pos[i, 0] - xc)
                            true_dir_O = np.array([-np.sin(fi), np.cos(fi), 0.0])
                            true_dir = np.squeeze(np.matmul(K_C_O, true_dir_O))
                            #print("True: ", true_dir)

                            
                        else:
                            
                            true_dir = np.squeeze(K_C_O[:, 0])
                        
                        estimated_dir = np.squeeze(robot_dir_estimate[i, :])
                        #print("Est: ", estimated_dir)
                        
                        if list_of_positions[p] in ['/Runs complete algorithm', '/Runs for velocity plot']:
                            
                            if list_of_positions[p] == '/Runs complete algorithm':
                                idx = 0
                            else:
                                idx = 1
                                
                            dq = robot_dq[i, :]
                            v = robot_base_vel[i, :]
                            J = robot_jacobian[i, :]
                            J = np.transpose(J.reshape(7, 6))
                        
                            varmr = np.squeeze(np.matmul(J, dq))
                            
                            varm = varmr[:3]
                            vang = varmr[3:]
                        
                            varm = varm + np.array([v[0], v[1], 0.0])
                        
                            v_mat[idx,r,i,0] = varm[0]
                            v_mat[idx,r,i,1] = varm[1]
                            v_mat[idx,r,i,2] = varm[2]
                            v_mat[idx,r,i,3] = (varm[0]**2+varm[1]**2+varm[2]**2)**0.5
                            
                            v_mat[idx,r,i,4] = v[0]
                            v_mat[idx,r,i,5] = v[1]

                            v_mat[idx,r,i,6] = varmr[0]
                            v_mat[idx,r,i,7] = varmr[1]
                            v_mat[idx,r,i,8] = varmr[2]
                        
                        list_of_iteration_times[a, p, c, r, i] = iteration_times[i]
                        list_of_manipulability[a, p, c, r, i] = robot_manip[i]
                        list_of_dot[a, p, c, r, i] = np.dot(-true_dir, estimated_dir)
        
        for i in range(len(list_of_positions)):
            
            if list_of_positions[i] in ['/Runs complete algorithm', '/Runs for velocity plot']:
                
                list_of_config = ['/Init1']
                N_runs = 5
                
            else:
                
                list_of_config = list_of_config_init
                N_runs = N_runs_init
                        
            fig1, (ax1_1, ax1_2, ax1_3) = plt.subplots(3,1,figsize=(15,15))
#            if door_type == 'drawer_door': 
#                fig1.suptitle('Metrics for position ' + title_pos[i], fontsize=18)
#            else:
#                fig1.suptitle('Metrics', fontsize=18)

            Init1_iteration_times = np.squeeze(list_of_iteration_times[a, i, 0, :N_runs, :])
            
            Init1_iteration_times_mean = np.mean(Init1_iteration_times, axis=0)
            Init1_iteration_times_std = np.std(Init1_iteration_times, axis=0)
            Init1_iteration_times_upper_limit = Init1_iteration_times_mean + 2*Init1_iteration_times_std
            Init1_iteration_times_lower_limit = Init1_iteration_times_mean - 2*Init1_iteration_times_std
                       
            Init1_manipulability = np.squeeze(list_of_manipulability[a, i, 0, :N_runs, :])
            
            Init1_manipulability_mean = np.mean(Init1_manipulability, axis=0)
            Init1_manipulability_std = np.std(Init1_manipulability, axis=0)
            Init1_manipulability_upper_limit = Init1_manipulability_mean + 2*Init1_manipulability_std
            Init1_manipulability_lower_limit = Init1_manipulability_mean - 2*Init1_manipulability_std            
            
            Init1_dot = np.squeeze(list_of_dot[a, i, 0, :N_runs, :])
            Init1_ang = np.squeeze(np.arccos(list_of_dot[a, i, 0, :N_runs, :]))*180.0/3.14
            
            Init1_dot_mean = np.mean(Init1_dot, axis=0) 
            Init1_dot_std = np.std(Init1_dot, axis=0)
            Init1_dot_upper_limit = Init1_dot_mean + 2*Init1_dot_std
            Init1_dot_lower_limit = Init1_dot_mean - 2*Init1_dot_std 
            
            Init1_ang_mean = np.mean(Init1_ang, axis=0)
            Init1_ang_std = np.std(Init1_ang, axis=0)
            Init1_ang_upper_limit = Init1_ang_mean + 2*Init1_ang_std
            Init1_ang_lower_limit = Init1_ang_mean - 2*Init1_ang_std
            
            for elem in range(len(Init1_dot_upper_limit)):
                if Init1_dot_upper_limit[elem]>1:
                    Init1_dot_upper_limit[elem] = 1

            if len(list_of_config) == 2: 
                        
                Init2_iteration_times = np.squeeze(list_of_iteration_times[a, i, 1, :N_runs, :])
                
                Init2_iteration_times_mean = np.mean(Init2_iteration_times, axis=0)
                Init2_iteration_times_std = np.std(Init2_iteration_times, axis=0)
                Init2_iteration_times_upper_limit = Init2_iteration_times_mean + 2*Init2_iteration_times_std
                Init2_iteration_times_lower_limit = Init2_iteration_times_mean - 2*Init2_iteration_times_std
                
                
                Init2_manipulability = np.squeeze(list_of_manipulability[a, i, 1, :N_runs, :])
                
                Init2_manipulability_mean = np.mean(Init2_manipulability, axis=0)
                Init2_manipulability_std = np.std(Init2_manipulability, axis=0)
                Init2_manipulability_upper_limit = Init2_manipulability_mean + 2*Init2_manipulability_std
                Init2_manipulability_lower_limit = Init2_manipulability_mean - 2*Init2_manipulability_std            
                
                Init2_dot = np.squeeze(list_of_dot[a, i, 1, :N_runs, :])
                Init2_ang = np.squeeze(np.arccos(list_of_dot[a, i, 1, :N_runs, :]))*180.0/3.14
                
                Init2_dot_mean = np.mean(Init2_dot, axis=0) 
                Init2_dot_std = np.std(Init2_dot, axis=0)
                Init2_dot_upper_limit = Init2_dot_mean + 2*Init2_dot_std
                Init2_dot_lower_limit = Init2_dot_mean - 2*Init2_dot_std 
                
                Init2_ang_mean = np.mean(Init2_ang, axis=0)
                Init2_ang_std = np.std(Init2_ang, axis=0)
                Init2_ang_upper_limit = Init2_ang_mean + 2*Init2_ang_std
                Init2_ang_lower_limit = Init2_ang_mean - 2*Init2_ang_std
                
                for elem in range(len(Init2_dot_upper_limit)):
                    if Init2_dot_upper_limit[elem]>1:
                        Init2_dot_upper_limit[elem] = 1

            t = np.arange(1, N+1)
            
            ax1_1.plot(t, Init1_iteration_times_mean, label = 'Configuration 1', linewidth=2.5)
            ax1_1.fill_between(t, Init1_iteration_times_lower_limit, Init1_iteration_times_upper_limit, alpha=0.2)
            if len(list_of_config) == 2:
                ax1_1.plot(t, Init2_iteration_times_mean, label = 'Configuration 2', linewidth=2.5)
                ax1_1.fill_between(t, Init2_iteration_times_lower_limit, Init2_iteration_times_upper_limit, alpha=0.2)

            ax1_1.axvline(x=initL, linewidth=3, color='k', ls='--')
            ax1_1.grid('on')
            ax1_1.set_ylabel(r"$\Delta t_{plann}$", fontsize=18)
            ax1_1.set_ylim(0.02, 0.06)
            ax1_1.set_xlim(t[0], t[-1])
            ax1_1.tick_params(axis='both', which='major', labelsize=15)
            if len(list_of_config) == 2:
                ax1_1.legend(loc="lower right", fontsize=15)
            
            ax1_2.plot(t, Init1_manipulability_mean, label = 'Configuration 1', linewidth=2.5)
            ax1_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.2)
            if len(list_of_config) == 2:
                ax1_2.plot(t, Init2_manipulability_mean, label = 'Configuration 2', linewidth=2.5)
                ax1_2.fill_between(t, Init2_manipulability_lower_limit, Init2_manipulability_upper_limit, alpha=0.2)

            ax1_2.axvline(x=initL, linewidth=3, color='k', ls='--')
            ax1_2.grid('on')
            ax1_2.set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=18)
            ax1_2.set_xlim(t[0], t[-1])
            ax1_2.tick_params(axis='both', which='major', labelsize=15)
            if len(list_of_config) == 2:
                ax1_2.legend(loc="lower right", fontsize=15)            

            ax1_3.plot(t, Init1_dot_mean, label = 'Configuration 1', linewidth=2.5)
            ax1_3.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.2)
            if len(list_of_config) == 2:
                ax1_3.plot(t, Init2_dot_mean, label = 'Configuration 2', linewidth=2.5)
                ax1_3.fill_between(t, Init2_dot_lower_limit, Init2_dot_upper_limit, alpha=0.2)
            
            ax1_3_2 = ax1_3.twinx()
            ax1_3_2.plot(t, Init1_ang_mean, 'r--', label = 'Configuration 1, angle', linewidth=2.5)
            ax1_3_2.fill_between(t, Init1_ang_lower_limit, Init1_ang_upper_limit, color='r', alpha=0.2)
                
            if len(list_of_config) == 2:
                ax1_3_2.plot(t, Init2_ang_mean, 'g--', label = 'Configuration 2, angle', linewidth=2.5)
                ax1_3_2.fill_between(t, Init2_ang_lower_limit, Init2_ang_upper_limit, color='g' ,alpha=0.2)

            ax1_3.axvline(x=initL, linewidth=3, color='k', ls='--')
            ax1_3.grid('on')
            
            ax1_3.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=18)
            ax1_3.set_ylim(0.5, 1.0)
            ax1_3.tick_params(axis='both', which='major', labelsize=15)
            
            ax1_3_2.set_ylabel(r"$\sphericalangle\left(\overrightarrow{n_{est}}, \overrightarrow{n_{true}}\right) [\degree]$", fontsize=18)
            ax1_3_2.set_ylim(min(Init1_ang_lower_limit), math.pi/2*180.0/3.14)
            ax1_3_2.tick_params(axis='both', which='major', labelsize=15)
            
            ax1_3.set_xlim(t[0], t[-1])
            ax1_3.set_xlabel("k-th iteration", fontsize=18)
            
            ax1_3.legend(fontsize=15)  
            ax1_3_2.legend(loc="center right", fontsize=15)
            
            
            saving_folder = curr_folder1 + list_of_positions[i]
            
            fig1.savefig(saving_folder + '//' + 'Metrics.png' )  
            

            f1 = open(saving_folder + '/Table_1', "w")
            if door_type == 'drawer_door':
                f2 = open(saving_folder + '/Table_2', "w")
            N_last =20
            
            start_average_dot1_mat = np.squeeze(list_of_dot[a, i, 0, :N_runs, :initL])
            
            start_average_dot1_mat = np.ones((start_average_dot1_mat.shape[0], start_average_dot1_mat.shape[1]))-start_average_dot1_mat
            start_average_dot1_mat = np.multiply(start_average_dot1_mat, start_average_dot1_mat)
                
            start_average_dot1_mean = np.mean(start_average_dot1_mat, axis=1)
            start_average_dot1_mean = np.sqrt(start_average_dot1_mean)
            
            start_average_dot1 = np.mean(np.squeeze(start_average_dot1_mean))
            
            whole_average_dot1_mat = np.squeeze(list_of_dot[a, i, 0, :N_runs, :])
            whole_average_dot1_mean = np.mean(whole_average_dot1_mat, axis=0)
            whole_average_dot1 = np.mean(np.squeeze(whole_average_dot1_mean))
            
            whole_average_manip1_mat = np.squeeze(list_of_manipulability[a, i, 0, :N_runs, :])
            whole_average_manip1_mean = np.mean(whole_average_manip1_mat, axis=0)
            whole_average_manip1 = np.mean(np.squeeze(whole_average_manip1_mean))
            
            end_average_manip1_mat = np.squeeze(list_of_manipulability[a, i, 0, :N_runs, 500-N_last:])
            end_average_manip1_mean = np.mean(end_average_manip1_mat, axis=0)
            end_average_manip1 = np.mean(np.squeeze(end_average_manip1_mean)) 
            
            total_planning_time1_mat = np.squeeze(list_of_iteration_times[a, i, 0, :N_runs, :])
            total_planning_time1 = np.sum(np.mean(total_planning_time1_mat, axis=0))
            
            average_planning_time1 = np.mean(np.mean(total_planning_time1_mat, axis=0))
            
            if len(list_of_config) == 2:
                start_average_dot2_mat = np.squeeze(list_of_dot[a, i, 1, :N_runs, :initL])
#                start_average_dot2_mean = np.mean(start_average_dot2_mat, axis=0)
#                start_average_dot2 = np.mean(np.squeeze(start_average_dot2_mean))
                
                start_average_dot2_mat = np.ones((start_average_dot2_mat.shape[0], start_average_dot2_mat.shape[1]))-start_average_dot2_mat
                start_average_dot2_mat = np.multiply(start_average_dot2_mat, start_average_dot2_mat)
                    
                start_average_dot2_mean = np.mean(start_average_dot2_mat, axis=1)
                start_average_dot2_mean = np.sqrt(start_average_dot2_mean)
                
                start_average_dot2 = np.mean(np.squeeze(start_average_dot2_mean))
                            
                whole_average_dot2_mat = np.squeeze(list_of_dot[a, i, 1, :N_runs, :])
                whole_average_dot2_mean = np.mean(whole_average_dot2_mat, axis=0)
                whole_average_dot2 = np.mean(np.squeeze(whole_average_dot2_mean))
    
                whole_average_manip2_mat = np.squeeze(list_of_manipulability[a, i, 1, :N_runs, :])
                whole_average_manip2_mean = np.mean(whole_average_manip2_mat, axis=0)
                whole_average_manip2 = np.mean(np.squeeze(whole_average_manip2_mean))
                
                end_average_manip2_mat = np.squeeze(list_of_manipulability[a, i, 1, :N_runs, 500-N_last:])
                end_average_manip2_mean = np.mean(end_average_manip2_mat, axis=0)
                end_average_manip2 = np.mean(np.squeeze(end_average_manip2_mean)) 
                
                total_planning_time2_mat = np.squeeze(list_of_iteration_times[a, i, 1, :N_runs, :])
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
            
            if door_type == 'drawer_door':
                f2.write("Table 2 for position "+title_pos[i]+" of the "+list_of_alg[a]+"\n")
                f2.write("\n")
                f2.write("Average dot product during init period       : "+str(start_average_dot2)+"\n")
                f2.write("Average dot product during whole procedure   : "+str(whole_average_dot2)+"\n")
                f2.write("Average manipulability during whole procedure: "+str(whole_average_manip2)+"\n")
                f2.write("Average manipulability during end period     : "+str(end_average_manip2)+"\n")
                f2.write("Total planning time                          : "+str(total_planning_time2)+"\n")
                f2.write("Average planning  per iteration              : "+str(average_planning_time2)+"\n")
            
            f1.close()
            if door_type == 'drawer_door':
                f2.close()
                
            if list_of_positions[i] in ['/Runs complete algorithm', '/Runs for velocity plot']:
                
                if list_of_positions[i] == '/Runs complete algorithm':
                    idx = 0
                else:
                    idx = 1
                                
                fig2, (ax2_1, ax2_2, ax2_3, ax2_4) = plt.subplots(4,1,figsize=(15,15))
                #fig2.suptitle('Velocity plots', fontsize=20)
                
                vx_mat = np.squeeze(v_mat[idx, :N_runs_vel, :, 0])
                vy_mat = np.squeeze(v_mat[idx, :N_runs_vel, :, 1])
                vz_mat = np.squeeze(v_mat[idx, :N_runs_vel, :, 2])
                vabs_mat = np.squeeze(v_mat[idx, :N_runs_vel, :, 3])
                
                vx_base_mat = np.squeeze(v_mat[idx, :N_runs_vel, :, 4])
                vy_base_mat = np.squeeze(v_mat[idx, :N_runs_vel, :, 5])
                
                vx_rel_mat = np.squeeze(v_mat[idx, :N_runs_vel, :, 6])
                vy_rel_mat = np.squeeze(v_mat[idx, :N_runs_vel, :, 7])
                vz_rel_mat = np.squeeze(v_mat[idx, :N_runs_vel, :, 8])
            
                vx_mat_mean = np.mean(vx_mat, axis=0)
                vx_mat_std = np.std(vx_mat, axis=0)
                vx_mat_upper_limit = vx_mat_mean + 2*vx_mat_std
                vx_mat_lower_limit = vx_mat_mean - 2*vx_mat_std
                
                vy_mat_mean = np.mean(vy_mat, axis=0)
                vy_mat_std = np.std(vy_mat, axis=0)
                vy_mat_upper_limit = vy_mat_mean + 2*vy_mat_std
                vy_mat_lower_limit = vy_mat_mean - 2*vy_mat_std                

                vz_mat_mean = np.mean(vz_mat, axis=0)
                vz_mat_std = np.std(vz_mat, axis=0)
                vz_mat_upper_limit = vz_mat_mean + 2*vz_mat_std
                vz_mat_lower_limit = vz_mat_mean - 2*vz_mat_std 
                
                vabs_mat_mean = np.mean(vabs_mat, axis=0)
                vabs_mat_std = np.std(vabs_mat, axis=0)
                vabs_mat_upper_limit = vabs_mat_mean + 2*vabs_mat_std
                vabs_mat_lower_limit = vabs_mat_mean - 2*vabs_mat_std

                vx_base_mat_mean = np.mean(vx_base_mat, axis=0)
                vx_base_mat_std = np.std(vx_base_mat, axis=0)
                vx_base_mat_upper_limit = vx_base_mat_mean + 2*vx_base_mat_std
                vx_base_mat_lower_limit = vx_base_mat_mean - 2*vx_base_mat_std
                
                vy_base_mat_mean = np.mean(vy_base_mat, axis=0)
                vy_base_mat_std = np.std(vy_base_mat, axis=0)
                vy_base_mat_upper_limit = vy_base_mat_mean + 2*vy_base_mat_std
                vy_base_mat_lower_limit = vy_base_mat_mean - 2*vy_base_mat_std                

                vx_rel_mat_mean = np.mean(vx_rel_mat, axis=0)
                vx_rel_mat_std = np.std(vx_rel_mat, axis=0)
                vx_rel_mat_upper_limit = vx_rel_mat_mean + 2*vx_rel_mat_std
                vx_rel_mat_lower_limit = vx_rel_mat_mean - 2*vx_rel_mat_std
                
                vy_rel_mat_mean = np.mean(vy_rel_mat, axis=0)
                vy_rel_mat_std = np.std(vy_rel_mat, axis=0)
                vy_rel_mat_upper_limit = vy_rel_mat_mean + 2*vy_rel_mat_std
                vy_rel_mat_lower_limit = vy_rel_mat_mean - 2*vy_rel_mat_std
                
                vz_rel_mat_mean = np.mean(vz_rel_mat, axis=0)
                vz_rel_mat_std = np.std(vz_rel_mat, axis=0)
                vz_rel_mat_upper_limit = vz_rel_mat_mean + 2*vz_rel_mat_std
                vz_rel_mat_lower_limit = vz_rel_mat_mean - 2*vz_rel_mat_std

                ax2_1.plot(t, vx_mat_mean, label = 'Total velocity', linewidth=2.5)
                ax2_1.fill_between(t, vx_mat_lower_limit, vx_mat_upper_limit, alpha=0.2)
                ax2_1.plot(t, vx_base_mat_mean,'--',label = '$v_{b}^{Base}$', linewidth=2.5)
                ax2_1.fill_between(t, vx_base_mat_lower_limit, vx_base_mat_upper_limit, alpha=0.2)
                ax2_1.plot(t, vx_rel_mat_mean,'--', label = '$v_{b}^{Arm}$', linewidth=2.5)
                ax2_1.fill_between(t, vx_rel_mat_lower_limit, vx_rel_mat_upper_limit, alpha=0.2)
                
                ax2_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                ax2_1.grid('on')
                ax2_1.set_ylabel(r"$v_{x}$", fontsize=18)
                ax2_1.set_ylim(-0.015, 0.005)
                ax2_1.set_xlim(t[0], t[-1])
                ax2_1.tick_params(axis='both', which='major', labelsize=15)
                ax2_1.legend(loc="lower left",fontsize=12)
                              
                ax2_2.plot(t, vy_mat_mean, label = 'Total velocity', linewidth=2.5)
                ax2_2.fill_between(t, vy_mat_lower_limit, vy_mat_upper_limit, alpha=0.2)
                ax2_2.plot(t, vy_base_mat_mean,'--', label = '$v_{b}^{Base}$', linewidth=2.5)
                ax2_2.fill_between(t, vy_base_mat_lower_limit, vy_base_mat_upper_limit, alpha=0.2)
                ax2_2.plot(t, vy_rel_mat_mean,'--', label = '$v_{b}^{Arm}$', linewidth=2.5)
                ax2_2.fill_between(t, vy_rel_mat_lower_limit, vy_rel_mat_upper_limit, alpha=0.2)
                
                ax2_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                ax2_2.grid('on')
                ax2_2.set_ylabel(r"$v_{y}$", fontsize=18)
                ax2_2.set_ylim(-0.015, 0.015)
                ax2_2.set_xlim(t[0], t[-1])
                ax2_2.tick_params(axis='both', which='major', labelsize=15)
                ax2_2.legend(loc="lower right",fontsize=12)                

                ax2_3.plot(t, vz_mat_mean, label = 'Configuration 1', linewidth=2.5)
                ax2_3.fill_between(t, vz_mat_lower_limit, vz_mat_upper_limit, alpha=0.2)

                ax2_3.axvline(x=initL, linewidth=3, color='k', ls='--')
                ax2_3.grid('on')
                ax2_3.set_ylabel(r"$v_{z}$", fontsize=18)
                ax2_3.set_ylim(-0.005, 0.005)
                ax2_3.set_xlim(t[0], t[-1])
                ax2_3.tick_params(axis='both', which='major', labelsize=15)                

                ax2_4.plot(t, vabs_mat_mean, label = 'Measured value', linewidth=2.5)
                ax2_4.fill_between(t, vabs_mat_lower_limit, vabs_mat_upper_limit, alpha=0.2)
                ax2_4.plot(t, list_of_abs_vel,'r--', label = 'Velocity profile', linewidth=4)

                ax2_4.axvline(x=initL, linewidth=3, color='k', ls='--')
                ax2_4.grid('on')
                ax2_4.set_ylabel(r"$|v|$", fontsize=18)
                ax2_4.set_ylim(0.0, 0.015)
                ax2_4.set_xlim(t[0], t[-1])
                ax2_4.tick_params(axis='both', which='major', labelsize=15) 
                ax2_4.set_xlabel("k-th iteration", fontsize=18)
                ax2_4.legend(loc="lower right",fontsize=12)
                
                fig2.savefig(saving_folder + '//' + 'Velocities.png' )
            plt.tight_layout()    
            plt.close('all')
                
def PlotCircle(door_type):
    
    folder_name = os.getcwd()
    
    folder_name = folder_name + '/GOOD RUNS'
    
    if door_type == 'rotational_door':
        
        folder_sufix = '/Rotational door'
        list_of_positions = ['/Pos']
        title_pos = [': middle']
        list_of_config = ['/Init1']
        N_runs = 10
        
    else:
        
        folder_sufix = '/Drawer door'
        list_of_positions = ['/Pos_left', '/Pos_middle', '/Pos_right']
        title_pos = [': left', ': middle', ': right']
        list_of_config = ['/Init1', '/Init2']
        N_runs = 5
        
    folder_name = folder_name + folder_sufix
    
    list_of_alg = ['/Fixed_Base', '/Mobile_Base']
        
    N = 500
    initL = 50
       
    list_of_iteration_times = np.zeros((2,3,2,N_runs,500))
    list_of_manipulability = np.zeros((2,3,2,N_runs,500))
    list_of_EE_T_K = np.zeros((2,3,2,N_runs,500))
    list_of_O_T_EE = np.zeros((2,3,2,N_runs,500))
    list_of_dot = np.zeros((2,3,2,N_runs,500))
    
    for a in range(1):  #Fixed or Mobile Base
        
        alg = list_of_alg[a]
        curr_folder1 = folder_name + alg
        
        if a == 1:
            
            mode = '/moving_base_no_collision_max_mob_control_QCCO'
            
        else:
            
            mode = '/fixed_base_torque_control'
                
        for p in range(len(list_of_positions)):  #Pos left, middle ,right
            
            pos = list_of_positions[p]
            
            curr_folder2 = curr_folder1 + pos
            
            for c in range(len(list_of_config)):  #Init1 or Init2
                
                config = list_of_config[c]
                
                curr_folder3 = curr_folder2 + config 
            
                for r in range(1): #Exp 1,2,3,4,5
                    
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
                    
                    if door_type == 'rotational_door':
                        
                        robot_ee_world_pos = np.load(curr_folder4 + '/robot_ee_world_pos.npy')
                        array_x = np.squeeze(np.copy(robot_ee_world_pos[:, 0]))
                        array_y = np.squeeze(np.copy(robot_ee_world_pos[:, 1]))
                        
                        A = []
                        b = []
                        
                        for i in range(N):
                            
                            A.append([1.0, -2*array_x[i], -2*array_y[i]])
                            b.append(-array_x[i]**2-array_y[i]**2)
                            
                        A = np.array(A)
                        b = np.array(b)
                        opt_circle = np.squeeze(np.matmul(np.linalg.pinv(A), b))
                        print(opt_circle[0], opt_circle[1], opt_circle[2])
                        
                        R = (opt_circle[1]**2 + opt_circle[2]**2 - opt_circle[0])**0.5 
                        xc = opt_circle[1]
                        yc = opt_circle[2]
                        
                        print("Opt circle: " + str([xc, yc, R])) 
                        
                        xopt = []
                        yopt = []
                        
                        Nopt = 1000
                        
                        fiopt = np.linspace(0, 2*math.pi, Nopt)
                        for fi in fiopt:
                            
                            xopt.append(R*np.cos(fi)+xc)
                            yopt.append(R*np.sin(fi)+yc)
                        
                        plt.plot(array_x, array_y)
                        plt.plot(xopt, yopt)
                        plt.grid('on')
                        plt.show()
                        
    
#----- main -----

def main():
    
    #door_type = 'rotational_door'
    door_type = 'drawer_door'
    
    GenerateImages(door_type)
    print("PLOTS CPMPLETED")



if __name__ == "__main__":
    main()



