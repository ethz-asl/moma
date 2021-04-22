#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import matplotlib.pyplot as plt
import math

#----- Description -----

# This script is used to analyse and plot all the results used in the report

#-----------------------

def AnalyseResultsPybullet():
    print("Collect the data...")
    
    list_of_names = ['Dishwasher','Drawer','Room Door','Sliding door', 'Sliding lid']
    list_of_criteria2 = ['Beta','c1c2','f','m', 'Nominal model', 'Complete alg']
    list_of_criteria1 = ['Beta','c1c2','f','m', 'Nominal model']
    
    list_of_beta = ['beta_1','beta_2']
    list_of_c1c2 = ['ratio_1','ratio_2']
    list_of_m = ['m_1','m_2']
    list_of_f = ['f_1','f_2']
    
    list_of_controllers1 = ['fixed_base_torque_control', 'moving_base_no_collision_max_mob_control_LCQP_32', 'moving_base_no_collision_max_mob_control_QCCO', 'moving_base_no_collision_max_mob_control_SOCP']
    list_of_controllers2 = ['fixed_base_cartesian_velocity_control', 'moving_base_no_collision_max_mob_control_LCQP_32', 'moving_base_no_collision_max_mob_control_QCCO', 'moving_base_no_collision_max_mob_control_SOCP']
    list_of_controllers3 = ['fixed_base_torque_control', 'moving_base_no_collision_max_mob_control_QCCO']
    
    folder = os.getcwd()+'/moma/moma_demos/articulated_demo/RESULTS SECTION RUNS'
    
    big_matrix = np.zeros((5,6,2,25,4,12,1201))    
    
    for i in range(len(list_of_names)):
        
        name = list_of_names[i]
        folder2 = folder + '/' + name

        n2 = 5
        
        if i==0:           
            n1 = 2
            list_of_criteria = list_of_criteria1
        if i==1:
            n1 = 4
            list_of_criteria = list_of_criteria2
        if i==2:
            n1 = 5
            list_of_criteria = list_of_criteria2
        if i==3:
            n1 = 5
            list_of_criteria = list_of_criteria1
        if i==4:
            n1 = 5
            list_of_criteria = list_of_criteria1
            
        for j in range(len(list_of_criteria)):
            
            criteria = list_of_criteria[j]
            folder3 = folder2 + '/' +criteria
            
            if j==0:                
                list_of_val = list_of_beta
                list_of_controllers = list_of_controllers2                
            if j==1:                
                list_of_val = list_of_c1c2
                list_of_controllers = list_of_controllers2                
            if j==2:               
                list_of_val = list_of_f
                list_of_controllers = list_of_controllers2                
            if j==3:               
                list_of_val = list_of_m
                list_of_controllers = list_of_controllers2                
            if j==4:
                list_of_val = ['dummy']
                list_of_controllers = list_of_controllers1
            if j==5:
                list_of_val = ['dummy']
                list_of_controllers = list_of_controllers3

                
            for k in range(len(list_of_val)):
            
                val_name = list_of_val[k]
                
                if 'dummy' in list_of_val:
                    folder4 = folder3
                else:
                    folder4 = folder3 + '/' + val_name
                
                subfolder = [fol1 for fol1 in os.listdir(folder4)]
                subfolder = subfolder[0]
                
                folder5 = folder4 + '/' + subfolder
                
                list_of_removed = []
                
                for b1 in range(n1):
                    
                    for b2 in range(n2):
                        
                        current_run = int(b1*n2+b2)
                        
                        config_name = str(b1) + '_0_'+str(b2)+'_0'
                        
                        folder6 = folder5 + '/' + config_name
                        
                        for c in range(len(list_of_controllers)):
                            
                            contr = list_of_controllers[c]
                            folder7 = folder6 + '/' + contr
                            
                            actual_dir = np.load(folder7+'//actual_dir.npy')
                            estimated_dir = np.load(folder7+'//estimated_dir.npy')
                            manipulability = np.load(folder7+'//manipulability_meas.npy')
                            exec_time = np.load(folder7+'//exec_time.npy')
                            
                            vel_base = np.load(folder7+'//lin_vBase_O_meas.npy')
                            vel_ee_des = np.load(folder7+'//lin_vdesEE_O.npy')
                            vel_ee_meas = np.load(folder7+'//lin_vEE_O_meas.npy')
                            
                            actual_pose = np.load(folder7+'//actual_drawer_pos.npy')
                            
                            for idx1 in range(estimated_dir.shape[0]):
                                                                
                                big_matrix[i,j,k,current_run,c,0,idx1]=np.dot(np.squeeze(estimated_dir[idx1, :]), np.squeeze(actual_dir[idx1, :]))
                                big_matrix[i,j,k,current_run,c,1,idx1]=manipulability[idx1]
                                big_matrix[i,j,k,current_run,c,2,idx1]=exec_time[idx1]
                                
                                big_matrix[i,j,k,current_run,c,3,idx1]=vel_ee_meas[idx1,0]
                                big_matrix[i,j,k,current_run,c,4,idx1]=vel_ee_meas[idx1,1]
                                big_matrix[i,j,k,current_run,c,5,idx1]=vel_ee_meas[idx1,2]
                                
                                big_matrix[i,j,k,current_run,c,6,idx1]=vel_ee_des[idx1,0]
                                big_matrix[i,j,k,current_run,c,7,idx1]=vel_ee_des[idx1,1]
                                big_matrix[i,j,k,current_run,c,8,idx1]=vel_ee_des[idx1,2]
                                
                                big_matrix[i,j,k,current_run,c,9,idx1]=actual_pose[idx1,0]
                                big_matrix[i,j,k,current_run,c,10,idx1]=actual_pose[idx1,1]
                                big_matrix[i,j,k,current_run,c,11,idx1]=actual_pose[idx1,2]
                                
                            big_matrix[i,j,k,current_run,c,0,1200] = estimated_dir.shape[0]
                                
    np.save(folder+'//big_matrix.npy', big_matrix)                           
    return big_matrix

def task1(big_matrix):
    
    print("Analysing...")

    list_of_names = ['Dishwasher','Drawer','Room Door','Sliding door', 'Sliding lid']
    list_of_criteria = ['Beta','c1c2','f','m', 'Nominal model']
    
    list_of_beta = ['beta_1','beta_2']
    list_of_c1c2 = ['ratio_1','ratio_2']
    list_of_m = ['m_1','m_2']
    list_of_f = ['f_1','f_2']
    
    list_of_controllers = ['fixed_base_cartesian_velocity_control', 'moving_base_no_collision_max_mob_control_LCQP_32', 'moving_base_no_collision_max_mob_control_QCCO', 'moving_base_no_collision_max_mob_control_SOCP']
    
    
    initL=100
    
    folder = os.getcwd()+'/moma/moma_demos/articulated_demo/RESULTS SECTION RUNS'
    
    matrix1 = np.zeros((4, 7, 6))
    
    matrix1b = np.zeros((2,7,2,3))
    matrix1ratio = np.zeros((2,7,2,3))
    matrix1f = np.zeros((2,7,2,3))
    matrix1m = np.zeros((2,7,2,3))
    
    matrix1complete=np.zeros((2,7,2))
    
    fig1, (ax1_1, ax1_2) = plt.subplots(2,1,figsize=(10,10), constrained_layout=True)
    
    fig21, (ax21_1, ax21_2, ax21_3) = plt.subplots(3,1,figsize=(15,15), constrained_layout=True)
    fig22, (ax22_1, ax22_2, ax22_3) = plt.subplots(3,1,figsize=(15,15), constrained_layout=True)
       
    fig31, (ax31_1, ax31_2, ax31_3) = plt.subplots(3,1,figsize=(15,15), constrained_layout=True)
    fig32, (ax32_1, ax32_2, ax32_3) = plt.subplots(3,1,figsize=(15,15), constrained_layout=True)
    
#    fig21, (ax21_1, ax21_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)
#    fig22, (ax22_1, ax22_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)
#       
#    fig31, (ax31_1, ax31_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)
#    fig32, (ax32_1, ax32_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)
 
    ax31_1_2 = ax31_1.twinx()
    ax21_1_2 = ax21_1.twinx()
    ax22_1_2 = ax22_1.twinx()
    ax32_1_2 = ax32_1.twinx()  
        
    fig4, (ax4_1, ax4_2, ax4_3, ax4_4) = plt.subplots(4,1,figsize=(15,15), constrained_layout=True)         #velocity mobile drawer
    fig5, (ax5_1, ax5_2, ax5_3, ax5_4) = plt.subplots(4,1,figsize=(15,15), constrained_layout=True)         #velocity mobile room door 
    
    fig6, (ax6_1, ax6_2, ax6_3) = plt.subplots(3,1,figsize=(15,15), constrained_layout=True)                #position all doors
    fig7, (ax7_1, ax7_2, ax7_3) = plt.subplots(3,1,figsize=(15,15), constrained_layout=True)                #position all doors
    
    fig8, (ax8_1, ax8_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)                       #plots for m drawer
    fig9, (ax9_1, ax9_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)                       #plots for m door
    
    fig10, (ax10_1, ax10_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)                       #plots for b drawer
    fig11, (ax11_1, ax11_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)                       #plots for b door    

    fig12, (ax12_1, ax12_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)                       #plots for f drawer
    fig13, (ax13_1, ax13_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)                       #plots for f door

    fig14, (ax14_1, ax14_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)                       #plots for c1c12 drawer
    fig15, (ax15_1, ax15_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True)                       #plots for c1c2 door
    
    fig16, (ax16_1, ax16_2, ax16_3) = plt.subplots(3,1,figsize=(15,15), constrained_layout=True) 
    #fig16, (ax16_1, ax16_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True) 
    ax16_1_2 = ax16_1.twinx()

    fig17, (ax17_1, ax17_2, ax17_3) = plt.subplots(3,1,figsize=(15,15), constrained_layout=True)                       #complete mobile   
    #fig17, (ax17_1, ax17_2) = plt.subplots(2,1,figsize=(15,15), constrained_layout=True) 
    ax17_1_2 = ax17_1.twinx()
    
    list_of_abs_vel = []
    
    tConv = 100
    t0 = np.ceil(tConv/3)
    vInit = 0.025
    alphaInit = 0.5
    alphaFinal= 0.5
    vFinal = 0.1
    
    for i in range(1,1201):
        t = i 
        if t<t0:

            vd = vInit * 2.0/math.pi*np.arctan(alphaInit*t)

        else:

            a1 = (vFinal - vInit*2.0/math.pi*np.arctan(alphaInit*t0))/(1.0 - 2.0/math.pi*np.arctan(alphaFinal*(t0 - tConv)))
            a2 = vFinal - a1

            vd = a1 * 2.0/math.pi*np.arctan(alphaFinal*(t - tConv)) + a2
            
        list_of_abs_vel.append(vd)
    
    for c in range(4):
        
        if c==0:            
            legend_name = 'Fixed base'
        if c==1:
            legend_name = 'LCQP, N=32'
        if c==2:
            legend_name = 'QCCO'
        if c==3:
            legend_name = 'SOCP'
                
        for i in range(len(list_of_names)):
            
            name = list_of_names[i]
            n2 = 5
            
            if i==0:           
                n1 = 2
                actual_t = 600
                min_t = 500
            if i==1:
                n1 = 4
                actual_t = 1200
                min_t = 1100
            if i==2:
                n1 = 5
                actual_t = 1200
                min_t = 1100
            if i==3:
                n1 = 5
                actual_t = 1200
                min_t = 1100
            if i==4:
                n1 = 5  
                actual_t = 600
                min_t = 500

                           
            list_of_good_runs = []
            n_bad_runs = 0
            
            if c==0 or c==2: 
                list_of_good_runsb1 = []
                n_bad_runsb1 = 0
    
                list_of_good_runsb2 = []
                n_bad_runsb2 = 0 
    
                list_of_good_runsm1 = []
                n_bad_runsm1 = 0
    
                list_of_good_runsm2 = []
                n_bad_runsm2 = 0
    
                list_of_good_runsf1 = []
                n_bad_runsf1 = 0
    
                list_of_good_runsf2 = []
                n_bad_runsf2 = 0 
    
                list_of_good_runsratio1 = []
                n_bad_runsratio1 = 0
    
                list_of_good_runsratio2 = []
                n_bad_runsratio2 = 0
                
                if i==1 or i==2:
                    list_of_good_runscomplete = []
                    n_bad_runscomplete = 0                
              
            for br in range(int(n1*n2)):
                
                current_run = np.squeeze(big_matrix[i, 4, 0, br, c, :, :])
                
                if big_matrix[i, 4, 0, br, c, 0, 1200]<min_t:
                    
                    n_bad_runs+=1
                    
                else:
                    
                    list_of_good_runs.append(current_run)
                    
                if c==0 or c==2:
                    #-----    
                    current_runb1 = np.squeeze(big_matrix[i, 0, 0, br, c, :, :])                
                    if big_matrix[i, 0, 0, br, c, 0, 1200]<min_t:                    
                        n_bad_runsb1+=1                    
                    else:                    
                        list_of_good_runsb1.append(current_runb1)               
    
                    current_runb2 = np.squeeze(big_matrix[i, 0,1, br, c, :, :])                
                    if big_matrix[i, 0, 1, br, c, 0, 1200]<min_t:                    
                        n_bad_runsb2+=1                    
                    else:                    
                        list_of_good_runsb2.append(current_runb2)
                    #-----
                    current_runratio1 = np.squeeze(big_matrix[i, 1, 0, br, c, :, :])                
                    if big_matrix[i, 1, 0, br, c, 0, 1200]<min_t:                    
                        n_bad_runsratio1+=1                    
                    else:                    
                        list_of_good_runsratio1.append(current_runratio1)
    
                    current_runratio2 = np.squeeze(big_matrix[i, 1, 1, br, c, :, :])                
                    if big_matrix[i, 1, 1, br, c, 0, 1200]<min_t:                    
                        n_bad_runsratio2+=1                    
                    else:                    
                        list_of_good_runsratio2.append(current_runratio2)
                    #-----
                    current_runf1 = np.squeeze(big_matrix[i, 2, 0, br, c, :, :])                
                    if big_matrix[i, 2, 0, br, c, 0, 1200]<min_t:                    
                        n_bad_runsf1+=1                    
                    else:                    
                        list_of_good_runsf1.append(current_runf1)               
    
                    current_runf2 = np.squeeze(big_matrix[i, 2,1, br, c, :, :])                
                    if big_matrix[i, 2, 1, br, c, 0, 1200]<min_t:                    
                        n_bad_runsf2+=1                    
                    else:                    
                        list_of_good_runsf2.append(current_runf2) 
                    #-----
                    current_runm1 = np.squeeze(big_matrix[i, 3, 0, br, c, :, :])                
                    if big_matrix[i, 3, 0, br, c, 0, 1200]<min_t:                    
                        n_bad_runsm1+=1                    
                    else:                    
                        list_of_good_runsm1.append(current_runm1)               
    
                    current_runm2 = np.squeeze(big_matrix[i, 3,1, br, c, :, :])                
                    if big_matrix[i, 3, 1, br, c, 0, 1200]<min_t:                    
                        n_bad_runsm2+=1                    
                    else:                    
                        list_of_good_runsm2.append(current_runm2) 
                    #-----
                    if i==1 or i==2:
                        
                        if c==0:
                            cpom=0
                        else:
                            cpom=1
                        
                        current_runcomplete = np.squeeze(big_matrix[i, 5, 0, br, cpom, :, :]) 
                        
                        if big_matrix[i, 5, 0, br, cpom, 0, 1200]<min_t or big_matrix[i, 5, 0, br, cpom, 0, 600]<0.5:                    
                            n_bad_runscomplete+=1                    
                        else:                    
                            list_of_good_runscomplete.append(current_runcomplete) 
                        
            n_good = len(list_of_good_runs)
            
            if c==0 or c==2:
                n_goodb1 = len(list_of_good_runsb1)
                n_goodb2 = len(list_of_good_runsb2)
                
                n_goodratio1 = len(list_of_good_runsratio1)
                n_goodratio2 = len(list_of_good_runsratio2)
                
                n_goodf1 = len(list_of_good_runsf1)
                n_goodf2 = len(list_of_good_runsf2)            
    
                n_goodm1 = len(list_of_good_runsm1)
                n_goodm2 = len(list_of_good_runsm2)
                
                if i==1 or i==2:
                    n_goodcomplete = len(list_of_good_runscomplete)
                
                dot_productb1_mat = []
                manipulabilityb1_mat = []
                planning_timeb1_mat = []            
                dot_productb2_mat = []
                manipulabilityb2_mat = []
                planning_timeb2_mat = []             
                
                dot_productratio1_mat = []
                manipulabilityratio1_mat = []
                planning_timeratio1_mat = []            
                dot_productratio2_mat = []
                manipulabilityratio2_mat = []
                planning_timeratio2_mat = [] 
    
                dot_productm1_mat = []
                manipulabilitym1_mat = []
                planning_timem1_mat = []            
                dot_productm2_mat = []
                manipulabilitym2_mat = []
                planning_timem2_mat = [] 
    
                dot_productf1_mat = []
                manipulabilityf1_mat = []
                planning_timef1_mat = []            
                dot_productf2_mat = []
                manipulabilityf2_mat = []
                planning_timef2_mat = [] 
                
                if i==1 or i==2:
                    dot_productcomplete_mat = []
                    manipulabilitycomplete_mat = []
                    planning_timecomplete_mat = [] 
                
            dot_product_mat = []
            manipulability_mat = []
            planning_time_mat = []
            
            vel_x_meas_mat = []
            vel_y_meas_mat = []
            vel_z_meas_mat = []

            vel_x_des_mat = []
            vel_y_des_mat = []
            vel_z_des_mat = []
            
            rx_mat = []
            ry_mat = []
            rz_mat = []
            
            for j in range(n_good):
                
                current_run = list_of_good_runs[j]
                
                dot_product_mat.append(np.squeeze(current_run[0,:min_t]))
                manipulability_mat.append(np.squeeze(current_run[1,:min_t]))
                planning_time_mat.append(np.squeeze(current_run[2,:min_t]))
                
                vel_x_meas_mat.append(np.squeeze(current_run[3,:min_t]))
                vel_y_meas_mat.append(np.squeeze(current_run[4,:min_t]))
                vel_z_meas_mat.append(np.squeeze(current_run[5,:min_t]))

                vel_x_des_mat.append(np.squeeze(current_run[6,:min_t]))
                vel_y_des_mat.append(np.squeeze(current_run[7,:min_t]))
                vel_z_des_mat.append(np.squeeze(current_run[8,:min_t]))
                
                rx_mat.append(np.squeeze(current_run[9,:min_t]))
                ry_mat.append(np.squeeze(current_run[10,:min_t]))
                rz_mat.append(np.squeeze(current_run[11,:min_t]))
            
            if c==0 or c==2:
                for j in range(n_goodb1):
                    
                    current_runb1 = list_of_good_runsb1[j]
                    
                    dot_productb1_mat.append(np.squeeze(current_runb1[0,:min_t]))
                    manipulabilityb1_mat.append(np.squeeze(current_runb1[1,:min_t]))
                    planning_timeb1_mat.append(np.squeeze(current_runb1[2,:min_t]))
                for j in range(n_goodb2):
                    
                    current_runb2 = list_of_good_runsb2[j]
                    
                    dot_productb2_mat.append(np.squeeze(current_runb2[0,:min_t]))
                    manipulabilityb2_mat.append(np.squeeze(current_runb2[1,:min_t]))
                    planning_timeb2_mat.append(np.squeeze(current_runb2[2,:min_t]))
                #-----
                for j in range(n_goodratio1):
                    
                    current_runratio1 = list_of_good_runsratio1[j]
                    
                    dot_productratio1_mat.append(np.squeeze(current_runratio1[0,:min_t]))
                    manipulabilityratio1_mat.append(np.squeeze(current_runratio1[1,:min_t]))
                    planning_timeratio1_mat.append(np.squeeze(current_runratio1[2,:min_t]))
                for j in range(n_goodratio2):
                    
                    current_runratio2 = list_of_good_runsratio2[j]
                    
                    dot_productratio2_mat.append(np.squeeze(current_runratio2[0,:min_t]))
                    manipulabilityratio2_mat.append(np.squeeze(current_runratio2[1,:min_t]))
                    planning_timeratio2_mat.append(np.squeeze(current_runratio2[2,:min_t]))
                #-----
                for j in range(n_goodm1):
                    
                    current_runm1 = list_of_good_runsm1[j]
                    
                    dot_productm1_mat.append(np.squeeze(current_runm1[0,:min_t]))
                    manipulabilitym1_mat.append(np.squeeze(current_runm1[1,:min_t]))
                    planning_timem1_mat.append(np.squeeze(current_runm1[2,:min_t]))
                for j in range(n_goodm2):
                    
                    current_runm2 = list_of_good_runsm2[j]
                    
                    dot_productm2_mat.append(np.squeeze(current_runm2[0,:min_t]))
                    manipulabilitym2_mat.append(np.squeeze(current_runm2[1,:min_t]))
                    planning_timem2_mat.append(np.squeeze(current_runm2[2,:min_t]))
                #-----
                for j in range(n_goodf1):
                    
                    current_runf1 = list_of_good_runsf1[j]
                    
                    dot_productf1_mat.append(np.squeeze(current_runf1[0,:min_t]))
                    manipulabilityf1_mat.append(np.squeeze(current_runf1[1,:min_t]))
                    planning_timef1_mat.append(np.squeeze(current_runf1[2,:min_t]))
                for j in range(n_goodf2):
                    
                    current_runf2 = list_of_good_runsf2[j]
                    
                    dot_productf2_mat.append(np.squeeze(current_runf2[0,:min_t]))
                    manipulabilityf2_mat.append(np.squeeze(current_runf2[1,:min_t]))
                    planning_timef2_mat.append(np.squeeze(current_runf2[2,:min_t]))
                #-----
                if i==1 or i==2:
                    for j in range(n_goodcomplete):
                        
                        current_runcomplete = list_of_good_runscomplete[j]
                        dot_productcomplete_mat.append(np.squeeze(current_runcomplete[0,:min_t]))
                        manipulabilitycomplete_mat.append(np.squeeze(current_runcomplete[1,:min_t]))
                        planning_timecomplete_mat.append(np.squeeze(current_runcomplete[2,:min_t]))

            #-----
            start_average_dot1_mat = np.array(dot_product_mat)
            start_average_dot1_mat = start_average_dot1_mat[:,:initL]
                
            start_average_dot1_mat = np.ones((start_average_dot1_mat.shape[0], start_average_dot1_mat.shape[1]))-start_average_dot1_mat
            start_average_dot1_mat = np.multiply(start_average_dot1_mat, start_average_dot1_mat)
                
            start_average_dot1_mean = np.mean(start_average_dot1_mat, axis=1)
            start_average_dot1_mean = np.sqrt(start_average_dot1_mean)
            
            start_average_dot1 = np.mean(np.squeeze(start_average_dot1_mean))
            
            whole_average_dot1_mat = np.array(dot_product_mat)

            whole_average_dot1_mean = np.mean(whole_average_dot1_mat, axis=0)
            whole_average_dot1 = np.mean(np.squeeze(whole_average_dot1_mean))
            
            whole_average_manip1_mat = np.array(manipulability_mat)
            
            whole_average_manip1_mean = np.mean(whole_average_manip1_mat, axis=0)
            whole_average_manip1 = np.mean(np.squeeze(whole_average_manip1_mean))
            
            end_average_manip1_mat = np.array(manipulability_mat)
            end_average_manip1_mat = end_average_manip1_mat[:, min_t-20:]
            
            end_average_manip1_mean = np.mean(end_average_manip1_mat, axis=0)
            end_average_manip1 = np.mean(np.squeeze(end_average_manip1_mean)) 
            
            total_planning_time1_mat = np.array(planning_time_mat)
            total_planning_time1 = np.sum(np.mean(total_planning_time1_mat, axis=0))
            
            average_planning_time1 = np.mean(np.mean(total_planning_time1_mat, axis=0))
            
            if c==0 or c==2:
                
                if i==1 or i==2:
                    
                    if i==1:
                        maux=0
                    else:
                        maux=1

                    #-----
                    start_average_dot1b1_mat = np.array(dot_productb1_mat)
                    start_average_dot1b1_mat = start_average_dot1b1_mat[:,:initL]                   
                    start_average_dot1b1_mat = np.ones((start_average_dot1b1_mat.shape[0], start_average_dot1b1_mat.shape[1]))-start_average_dot1b1_mat
                    start_average_dot1b1_mat = np.multiply(start_average_dot1b1_mat, start_average_dot1b1_mat)                    
                    start_average_dot1b1_mean = np.mean(start_average_dot1b1_mat, axis=1)
                    start_average_dot1b1_mean = np.sqrt(start_average_dot1b1_mean)                
                    start_average_dot1b1 = np.mean(np.squeeze(start_average_dot1b1_mean))                
                    whole_average_dot1b1_mat = np.array(dot_productb1_mat)    
                    whole_average_dot1b1_mean = np.mean(whole_average_dot1b1_mat, axis=0)
                    whole_average_dot1b1 = np.mean(np.squeeze(whole_average_dot1b1_mean))                
                    whole_average_manip1b1_mat = np.array(manipulabilityb1_mat)                
                    whole_average_manip1b1_mean = np.mean(whole_average_manip1b1_mat, axis=0)
                    whole_average_manip1b1 = np.mean(np.squeeze(whole_average_manip1b1_mean))                
                    end_average_manip1b1_mat = np.array(manipulabilityb1_mat)
                    end_average_manip1b1_mat = end_average_manip1b1_mat[:, min_t-20:]                
                    end_average_manip1b1_mean = np.mean(end_average_manip1b1_mat, axis=0)
                    end_average_manip1b1 = np.mean(np.squeeze(end_average_manip1b1_mean))                
                    total_planning_time1b1_mat = np.array(planning_timeb1_mat)
                    total_planning_time1b1 = np.sum(np.mean(total_planning_time1b1_mat, axis=0))                
                    average_planning_time1b1 = np.mean(np.mean(total_planning_time1b1_mat, axis=0))
    
                    #-----
                    start_average_dot1b2_mat = np.array(dot_productb2_mat)
                    start_average_dot1b2_mat = start_average_dot1b2_mat[:,:initL]                   
                    start_average_dot1b2_mat = np.ones((start_average_dot1b2_mat.shape[0], start_average_dot1b2_mat.shape[1]))-start_average_dot1b2_mat
                    start_average_dot1b2_mat = np.multiply(start_average_dot1b2_mat, start_average_dot1b2_mat)                    
                    start_average_dot1b2_mean = np.mean(start_average_dot1b2_mat, axis=1)
                    start_average_dot1b2_mean = np.sqrt(start_average_dot1b2_mean)                
                    start_average_dot1b2 = np.mean(np.squeeze(start_average_dot1b2_mean))                
                    whole_average_dot1b2_mat = np.array(dot_productb2_mat)    
                    whole_average_dot1b2_mean = np.mean(whole_average_dot1b2_mat, axis=0)
                    whole_average_dot1b2 = np.mean(np.squeeze(whole_average_dot1b2_mean))                
                    whole_average_manip1b2_mat = np.array(manipulabilityb2_mat)                
                    whole_average_manip1b2_mean = np.mean(whole_average_manip1b2_mat, axis=0)
                    whole_average_manip1b2 = np.mean(np.squeeze(whole_average_manip1b2_mean))                
                    end_average_manip1b2_mat = np.array(manipulabilityb2_mat)
                    end_average_manip1b2_mat = end_average_manip1b2_mat[:, min_t-20:]                
                    end_average_manip1b2_mean = np.mean(end_average_manip1b2_mat, axis=0)
                    end_average_manip1b2 = np.mean(np.squeeze(end_average_manip1b2_mean))                
                    total_planning_time1b2_mat = np.array(planning_timeb2_mat)
                    total_planning_time1b2 = np.sum(np.mean(total_planning_time1b2_mat, axis=0))                
                    average_planning_time1b2 = np.mean(np.mean(total_planning_time1b2_mat, axis=0))
    
                    #-----
                    start_average_dot1ratio1_mat = np.array(dot_productratio1_mat)
                    start_average_dot1ratio1_mat = start_average_dot1ratio1_mat[:,:initL]                   
                    start_average_dot1ratio1_mat = np.ones((start_average_dot1ratio1_mat.shape[0], start_average_dot1ratio1_mat.shape[1]))-start_average_dot1ratio1_mat
                    start_average_dot1ratio1_mat = np.multiply(start_average_dot1ratio1_mat, start_average_dot1ratio1_mat)                    
                    start_average_dot1ratio1_mean = np.mean(start_average_dot1ratio1_mat, axis=1)
                    start_average_dot1ratio1_mean = np.sqrt(start_average_dot1ratio1_mean)                
                    start_average_dot1ratio1 = np.mean(np.squeeze(start_average_dot1ratio1_mean))                
                    whole_average_dot1ratio1_mat = np.array(dot_productratio1_mat)    
                    whole_average_dot1ratio1_mean = np.mean(whole_average_dot1ratio1_mat, axis=0)
                    whole_average_dot1ratio1 = np.mean(np.squeeze(whole_average_dot1ratio1_mean))                
                    whole_average_manip1ratio1_mat = np.array(manipulabilityratio1_mat)                
                    whole_average_manip1ratio1_mean = np.mean(whole_average_manip1ratio1_mat, axis=0)
                    whole_average_manip1ratio1 = np.mean(np.squeeze(whole_average_manip1ratio1_mean))                
                    end_average_manip1ratio1_mat = np.array(manipulabilityratio1_mat)
                    end_average_manip1ratio1_mat = end_average_manip1ratio1_mat[:, min_t-20:]                
                    end_average_manip1ratio1_mean = np.mean(end_average_manip1ratio1_mat, axis=0)
                    end_average_manip1ratio1 = np.mean(np.squeeze(end_average_manip1ratio1_mean))                
                    total_planning_time1ratio1_mat = np.array(planning_timeratio1_mat)
                    total_planning_time1ratio1 = np.sum(np.mean(total_planning_time1ratio1_mat, axis=0))                
                    average_planning_time1ratio1 = np.mean(np.mean(total_planning_time1ratio1_mat, axis=0))
    
                    #-----
                    start_average_dot1ratio2_mat = np.array(dot_productratio2_mat)
                    start_average_dot1ratio2_mat = start_average_dot1ratio2_mat[:,:initL]                   
                    start_average_dot1ratio2_mat = np.ones((start_average_dot1ratio2_mat.shape[0], start_average_dot1ratio2_mat.shape[1]))-start_average_dot1ratio2_mat
                    start_average_dot1ratio2_mat = np.multiply(start_average_dot1ratio2_mat, start_average_dot1ratio2_mat)                    
                    start_average_dot1ratio2_mean = np.mean(start_average_dot1ratio2_mat, axis=1)
                    start_average_dot1ratio2_mean = np.sqrt(start_average_dot1ratio2_mean)                
                    start_average_dot1ratio2 = np.mean(np.squeeze(start_average_dot1ratio2_mean))                
                    whole_average_dot1ratio2_mat = np.array(dot_productratio2_mat)    
                    whole_average_dot1ratio2_mean = np.mean(whole_average_dot1ratio2_mat, axis=0)
                    whole_average_dot1ratio2 = np.mean(np.squeeze(whole_average_dot1ratio2_mean))                
                    whole_average_manip1ratio2_mat = np.array(manipulabilityratio2_mat)                
                    whole_average_manip1ratio2_mean = np.mean(whole_average_manip1ratio2_mat, axis=0)
                    whole_average_manip1ratio2 = np.mean(np.squeeze(whole_average_manip1ratio2_mean))                
                    end_average_manip1ratio2_mat = np.array(manipulabilityratio2_mat)
                    end_average_manip1ratio2_mat = end_average_manip1ratio2_mat[:, min_t-20:]                
                    end_average_manip1ratio2_mean = np.mean(end_average_manip1ratio2_mat, axis=0)
                    end_average_manip1ratio2 = np.mean(np.squeeze(end_average_manip1ratio2_mean))                
                    total_planning_time1ratio2_mat = np.array(planning_timeratio2_mat)
                    total_planning_time1ratio2 = np.sum(np.mean(total_planning_time1ratio2_mat, axis=0))                
                    average_planning_time1ratio2 = np.mean(np.mean(total_planning_time1ratio2_mat, axis=0))                
    
                    #-----
                    start_average_dot1m1_mat = np.array(dot_productm1_mat)
                    start_average_dot1m1_mat = start_average_dot1m1_mat[:,:initL]                   
                    start_average_dot1m1_mat = np.ones((start_average_dot1m1_mat.shape[0], start_average_dot1m1_mat.shape[1]))-start_average_dot1m1_mat
                    start_average_dot1m1_mat = np.multiply(start_average_dot1m1_mat, start_average_dot1m1_mat)                    
                    start_average_dot1m1_mean = np.mean(start_average_dot1m1_mat, axis=1)
                    start_average_dot1m1_mean = np.sqrt(start_average_dot1m1_mean)                
                    start_average_dot1m1 = np.mean(np.squeeze(start_average_dot1m1_mean))                
                    whole_average_dot1m1_mat = np.array(dot_productm1_mat)    
                    whole_average_dot1m1_mean = np.mean(whole_average_dot1m1_mat, axis=0)
                    whole_average_dot1m1 = np.mean(np.squeeze(whole_average_dot1m1_mean))                
                    whole_average_manip1m1_mat = np.array(manipulabilitym1_mat)                
                    whole_average_manip1m1_mean = np.mean(whole_average_manip1m1_mat, axis=0)
                    whole_average_manip1m1 = np.mean(np.squeeze(whole_average_manip1m1_mean))                
                    end_average_manip1m1_mat = np.array(manipulabilitym1_mat)
                    end_average_manip1m1_mat = end_average_manip1m1_mat[:, min_t-20:]                
                    end_average_manip1m1_mean = np.mean(end_average_manip1m1_mat, axis=0)
                    end_average_manip1m1 = np.mean(np.squeeze(end_average_manip1m1_mean))                
                    total_planning_time1m1_mat = np.array(planning_timem1_mat)
                    total_planning_time1m1 = np.sum(np.mean(total_planning_time1m1_mat, axis=0))                
                    average_planning_time1m1 = np.mean(np.mean(total_planning_time1m1_mat, axis=0))
                    
                    #-----
                    start_average_dot1m2_mat = np.array(dot_productm2_mat)
                    start_average_dot1m2_mat = start_average_dot1m2_mat[:,:initL]                   
                    start_average_dot1m2_mat = np.ones((start_average_dot1m2_mat.shape[0], start_average_dot1m2_mat.shape[1]))-start_average_dot1m2_mat
                    start_average_dot1m2_mat = np.multiply(start_average_dot1m2_mat, start_average_dot1m2_mat)                    
                    start_average_dot1m2_mean = np.mean(start_average_dot1m2_mat, axis=1)
                    start_average_dot1m2_mean = np.sqrt(start_average_dot1m2_mean)                
                    start_average_dot1m2 = np.mean(np.squeeze(start_average_dot1m2_mean))                
                    whole_average_dot1m2_mat = np.array(dot_productm2_mat)    
                    whole_average_dot1m2_mean = np.mean(whole_average_dot1m2_mat, axis=0)
                    whole_average_dot1m2 = np.mean(np.squeeze(whole_average_dot1m2_mean))                
                    whole_average_manip1m2_mat = np.array(manipulabilitym2_mat)                
                    whole_average_manip1m2_mean = np.mean(whole_average_manip1m2_mat, axis=0)
                    whole_average_manip1m2 = np.mean(np.squeeze(whole_average_manip1m2_mean))                
                    end_average_manip1m2_mat = np.array(manipulabilitym2_mat)
                    end_average_manip1m2_mat = end_average_manip1m2_mat[:, min_t-20:]                
                    end_average_manip1m2_mean = np.mean(end_average_manip1m2_mat, axis=0)
                    end_average_manip1m2 = np.mean(np.squeeze(end_average_manip1m2_mean))                
                    total_planning_time1m2_mat = np.array(planning_timem2_mat)
                    total_planning_time1m2 = np.sum(np.mean(total_planning_time1m2_mat, axis=0))                
                    average_planning_time1m2 = np.mean(np.mean(total_planning_time1m2_mat, axis=0))
                    
                    #-----
                    start_average_dot1f1_mat = np.array(dot_productf1_mat)
                    start_average_dot1f1_mat = start_average_dot1f1_mat[:,:initL]                   
                    start_average_dot1f1_mat = np.ones((start_average_dot1f1_mat.shape[0], start_average_dot1f1_mat.shape[1]))-start_average_dot1f1_mat
                    start_average_dot1f1_mat = np.multiply(start_average_dot1f1_mat, start_average_dot1f1_mat)                    
                    start_average_dot1f1_mean = np.mean(start_average_dot1f1_mat, axis=1)
                    start_average_dot1f1_mean = np.sqrt(start_average_dot1f1_mean)                
                    start_average_dot1f1 = np.mean(np.squeeze(start_average_dot1f1_mean))                
                    whole_average_dot1f1_mat = np.array(dot_productf1_mat)    
                    whole_average_dot1f1_mean = np.mean(whole_average_dot1f1_mat, axis=0)
                    whole_average_dot1f1 = np.mean(np.squeeze(whole_average_dot1f1_mean))                
                    whole_average_manip1f1_mat = np.array(manipulabilityf1_mat)                
                    whole_average_manip1f1_mean = np.mean(whole_average_manip1f1_mat, axis=0)
                    whole_average_manip1f1 = np.mean(np.squeeze(whole_average_manip1f1_mean))                
                    end_average_manip1f1_mat = np.array(manipulabilityf1_mat)
                    end_average_manip1f1_mat = end_average_manip1f1_mat[:, min_t-20:]                
                    end_average_manip1f1_mean = np.mean(end_average_manip1f1_mat, axis=0)
                    end_average_manip1f1 = np.mean(np.squeeze(end_average_manip1f1_mean))                
                    total_planning_time1f1_mat = np.array(planning_timef1_mat)
                    total_planning_time1f1 = np.sum(np.mean(total_planning_time1f1_mat, axis=0))                
                    average_planning_time1f1 = np.mean(np.mean(total_planning_time1f1_mat, axis=0))
                    
                    #-----
                    start_average_dot1f2_mat = np.array(dot_productf2_mat)
                    start_average_dot1f2_mat = start_average_dot1f2_mat[:,:initL]                   
                    start_average_dot1f2_mat = np.ones((start_average_dot1f2_mat.shape[0], start_average_dot1f2_mat.shape[1]))-start_average_dot1f2_mat
                    start_average_dot1f2_mat = np.multiply(start_average_dot1f2_mat, start_average_dot1f2_mat)                    
                    start_average_dot1f2_mean = np.mean(start_average_dot1f2_mat, axis=1)
                    start_average_dot1f2_mean = np.sqrt(start_average_dot1f2_mean)                
                    start_average_dot1f2 = np.mean(np.squeeze(start_average_dot1f2_mean))                
                    whole_average_dot1f2_mat = np.array(dot_productf2_mat)    
                    whole_average_dot1f2_mean = np.mean(whole_average_dot1f2_mat, axis=0)
                    whole_average_dot1f2 = np.mean(np.squeeze(whole_average_dot1f2_mean))                
                    whole_average_manip1f2_mat = np.array(manipulabilityf2_mat)                
                    whole_average_manip1f2_mean = np.mean(whole_average_manip1f2_mat, axis=0)
                    whole_average_manip1f2 = np.mean(np.squeeze(whole_average_manip1f2_mean))                
                    end_average_manip1f2_mat = np.array(manipulabilityf2_mat)
                    end_average_manip1f2_mat = end_average_manip1f2_mat[:, min_t-20:]                
                    end_average_manip1f2_mean = np.mean(end_average_manip1f2_mat, axis=0)
                    end_average_manip1f2 = np.mean(np.squeeze(end_average_manip1f2_mean))                
                    total_planning_time1f2_mat = np.array(planning_timef2_mat)
                    total_planning_time1f2 = np.sum(np.mean(total_planning_time1f2_mat, axis=0))                
                    average_planning_time1f2 = np.mean(np.mean(total_planning_time1f2_mat, axis=0))

                    #-----
                    start_average_dot1complete_mat = np.array(dot_productcomplete_mat)
                    start_average_dot1complete_mat = start_average_dot1complete_mat[:,:initL]                   
                    start_average_dot1complete_mat = np.ones((start_average_dot1complete_mat.shape[0], start_average_dot1complete_mat.shape[1]))-start_average_dot1complete_mat
                    start_average_dot1complete_mat = np.multiply(start_average_dot1complete_mat, start_average_dot1complete_mat)                    
                    start_average_dot1complete_mean = np.mean(start_average_dot1complete_mat, axis=1)
                    start_average_dot1complete_mean = np.sqrt(start_average_dot1complete_mean)                
                    start_average_dot1complete = np.mean(np.squeeze(start_average_dot1complete_mean))                
                    whole_average_dot1complete_mat = np.array(dot_productcomplete_mat)    
                    whole_average_dot1complete_mean = np.mean(whole_average_dot1complete_mat, axis=0)
                    whole_average_dot1complete = np.mean(np.squeeze(whole_average_dot1complete_mean))                
                    whole_average_manip1complete_mat = np.array(manipulabilitycomplete_mat)                
                    whole_average_manip1complete_mean = np.mean(whole_average_manip1complete_mat, axis=0)
                    whole_average_manip1complete = np.mean(np.squeeze(whole_average_manip1complete_mean))                
                    end_average_manip1complete_mat = np.array(manipulabilitycomplete_mat)
                    end_average_manip1complete_mat = end_average_manip1complete_mat[:, min_t-20:]                
                    end_average_manip1complete_mean = np.mean(end_average_manip1complete_mat, axis=0)
                    end_average_manip1complete = np.mean(np.squeeze(end_average_manip1complete_mean))                
                    total_planning_time1complete_mat = np.array(planning_timecomplete_mat)
                    total_planning_time1complete = np.sum(np.mean(total_planning_time1complete_mat, axis=0))                
                    average_planning_time1complete = np.mean(np.mean(total_planning_time1complete_mat, axis=0))
                    
                    if c==0:
                        caux=0
                    else:
                        caux=1

                    matrix1complete[caux, 0, maux] = start_average_dot1complete
                    matrix1complete[caux, 1, maux] = whole_average_dot1complete
                    matrix1complete[caux, 2, maux] = whole_average_manip1complete
                    matrix1complete[caux, 3, maux] = end_average_manip1complete
                    matrix1complete[caux, 4, maux] = total_planning_time1complete
                    matrix1complete[caux, 5, maux] = average_planning_time1complete
                    matrix1complete[caux, 6, maux] = n_bad_runscomplete
                        
                    matrix1b[caux,0,maux,0]=start_average_dot1b1
                    matrix1b[caux,0,maux,1]=start_average_dot1
                    matrix1b[caux,0,maux,2]=start_average_dot1b2
                    matrix1b[caux,1,maux,0]=whole_average_dot1b1
                    matrix1b[caux,1,maux,1]=whole_average_dot1
                    matrix1b[caux,1,maux,2]=whole_average_dot1b2                
                    matrix1b[caux,2,maux,0]=whole_average_manip1b1
                    matrix1b[caux,2,maux,1]=whole_average_manip1
                    matrix1b[caux,2,maux,2]=whole_average_manip1b2                 
                    matrix1b[caux,3,maux,0]=end_average_manip1b1
                    matrix1b[caux,3,maux,1]=end_average_manip1
                    matrix1b[caux,3,maux,2]=end_average_manip1b2                 
                    matrix1b[caux,4,maux,0]=total_planning_time1b1
                    matrix1b[caux,4,maux,1]=total_planning_time1
                    matrix1b[caux,4,maux,2]=total_planning_time1b2                 
                    matrix1b[caux,5,maux,0]=average_planning_time1b1
                    matrix1b[caux,5,maux,1]=average_planning_time1
                    matrix1b[caux,5,maux,2]=average_planning_time1b2                 
                    matrix1b[caux,6,maux,0]=n_bad_runsb1
                    matrix1b[caux,6,maux,1]=n_bad_runs
                    matrix1b[caux,6,maux,2]=n_bad_runsb2                
                    
                    matrix1ratio[caux,0,maux,0]=start_average_dot1ratio1
                    matrix1ratio[caux,0,maux,1]=start_average_dot1
                    matrix1ratio[caux,0,maux,2]=start_average_dot1ratio2
                    matrix1ratio[caux,1,maux,0]=whole_average_dot1ratio1
                    matrix1ratio[caux,1,maux,1]=whole_average_dot1
                    matrix1ratio[caux,1,maux,2]=whole_average_dot1ratio2                
                    matrix1ratio[caux,2,maux,0]=whole_average_manip1ratio1
                    matrix1ratio[caux,2,maux,1]=whole_average_manip1
                    matrix1ratio[caux,2,maux,2]=whole_average_manip1ratio2                 
                    matrix1ratio[caux,3,maux,0]=end_average_manip1ratio1
                    matrix1ratio[caux,3,maux,1]=end_average_manip1
                    matrix1ratio[caux,3,maux,2]=end_average_manip1ratio2                 
                    matrix1ratio[caux,4,maux,0]=total_planning_time1ratio1
                    matrix1ratio[caux,4,maux,1]=total_planning_time1
                    matrix1ratio[caux,4,maux,2]=total_planning_time1ratio2                 
                    matrix1ratio[caux,5,maux,0]=average_planning_time1ratio1
                    matrix1ratio[caux,5,maux,1]=average_planning_time1
                    matrix1ratio[caux,5,maux,2]=average_planning_time1ratio2                 
                    matrix1ratio[caux,6,maux,0]=n_bad_runsratio1
                    matrix1ratio[caux,6,maux,1]=n_bad_runs
                    matrix1ratio[caux,6,maux,2]=n_bad_runsratio2                 
    
                    matrix1m[caux,0,maux,0]=start_average_dot1m1
                    matrix1m[caux,0,maux,1]=start_average_dot1
                    matrix1m[caux,0,maux,2]=start_average_dot1m2
                    matrix1m[caux,1,maux,0]=whole_average_dot1m1
                    matrix1m[caux,1,maux,1]=whole_average_dot1
                    matrix1m[caux,1,maux,2]=whole_average_dot1m2                
                    matrix1m[caux,2,maux,0]=whole_average_manip1m1
                    matrix1m[caux,2,maux,1]=whole_average_manip1
                    matrix1m[caux,2,maux,2]=whole_average_manip1m2                 
                    matrix1m[caux,3,maux,0]=end_average_manip1m1
                    matrix1m[caux,3,maux,1]=end_average_manip1
                    matrix1m[caux,3,maux,2]=end_average_manip1m2                 
                    matrix1m[caux,4,maux,0]=total_planning_time1m1
                    matrix1m[caux,4,maux,1]=total_planning_time1
                    matrix1m[caux,4,maux,2]=total_planning_time1m2                 
                    matrix1m[caux,5,maux,0]=average_planning_time1m1
                    matrix1m[caux,5,maux,1]=average_planning_time1
                    matrix1m[caux,5,maux,2]=average_planning_time1m2                 
                    matrix1m[caux,6,maux,0]=n_bad_runsm1
                    matrix1m[caux,6,maux,1]=n_bad_runs
                    matrix1m[caux,6,maux,2]=n_bad_runsm2  
    
                    matrix1f[caux,0,maux,0]=start_average_dot1f1
                    matrix1f[caux,0,maux,1]=start_average_dot1
                    matrix1f[caux,0,maux,2]=start_average_dot1f2
                    matrix1f[caux,1,maux,0]=whole_average_dot1f1
                    matrix1f[caux,1,maux,1]=whole_average_dot1
                    matrix1f[caux,1,maux,2]=whole_average_dot1f2                
                    matrix1f[caux,2,maux,0]=whole_average_manip1f1
                    matrix1f[caux,2,maux,1]=whole_average_manip1
                    matrix1f[caux,2,maux,2]=whole_average_manip1f2                 
                    matrix1f[caux,3,maux,0]=end_average_manip1f1
                    matrix1f[caux,3,maux,1]=end_average_manip1
                    matrix1f[caux,3,maux,2]=end_average_manip1f2                 
                    matrix1f[caux,4,maux,0]=total_planning_time1f1
                    matrix1f[caux,4,maux,1]=total_planning_time1
                    matrix1f[caux,4,maux,2]=total_planning_time1f2                 
                    matrix1f[caux,5,maux,0]=average_planning_time1f1
                    matrix1f[caux,5,maux,1]=average_planning_time1
                    matrix1f[caux,5,maux,2]=average_planning_time1f2                 
                    matrix1f[caux,6,maux,0]=n_bad_runsf1
                    matrix1f[caux,6,maux,1]=n_bad_runs
                    matrix1f[caux,6,maux,2]=n_bad_runsf2  
                
            matrix1[c, 0, i] = start_average_dot1
            matrix1[c, 1, i] = whole_average_dot1
            matrix1[c, 2, i] = whole_average_manip1
            matrix1[c, 3, i] = end_average_manip1
            matrix1[c, 4, i] = total_planning_time1
            matrix1[c, 5, i] = average_planning_time1
            matrix1[c, 6, i] = n_bad_runs
                                
            if i==1 or i==2:
                
                ts1 = np.arange(1, min_t+1)
                if i==1:
                    ax1_1.plot(ts1, np.squeeze(np.mean(total_planning_time1_mat, axis=0)),'--', label = legend_name, linewidth=2.5)
                    ax1_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax1_1.grid('on')
                    ax1_1.set_ylabel(r"$\Delta t_{plan}$", fontsize=22)
                    ax1_1.set_ylim(0.00, 0.01)
                    ax1_1.set_xlim(ts1[0], ts1[-1])
                    ax1_1.tick_params(axis='both', which='major', labelsize=15)
                    ax1_1.legend(loc="lower right", fontsize=20)
                    ax1_1.tick_params(axis='both', which='major', labelsize=15)
                    ax1_1.set_title("Drawer mechanism", fontsize=22)
                if i==2:
                    ax1_2.plot(ts1, np.squeeze(np.mean(total_planning_time1_mat, axis=0)),'--', label = legend_name, linewidth=2.5)
                    ax1_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax1_2.grid('on')
                    ax1_2.set_ylabel(r"$\Delta t_{plan}$", fontsize=22)
                    ax1_2.set_ylim(0.00, 0.01)
                    ax1_2.set_xlim(ts1[0], ts1[-1])
                    ax1_2.tick_params(axis='both', which='major', labelsize=15)  
                    ax1_2.legend(loc="lower right", fontsize=20) 
                    ax1_2.set_xlabel("k-th iteration", fontsize=22)
                    ax1_2.tick_params(axis='both', which='major', labelsize=15)
                    ax1_2.set_title(" Room door mechanism", fontsize=22)
                    
            vx_mat = np.array(vel_x_meas_mat)
            
            vx_mat_mean = np.mean(vx_mat, axis=0)
            vx_mat_std = np.std(vx_mat, axis=0)
            vx_mat_upper_limit = vx_mat_mean + 2*vx_mat_std
            vx_mat_lower_limit = vx_mat_mean - 2*vx_mat_std 

            vy_mat = np.array(vel_y_meas_mat)
            
            vy_mat_mean = np.mean(vy_mat, axis=0)
            vy_mat_std = np.std(vy_mat, axis=0)
            vy_mat_upper_limit = vy_mat_mean + 2*vy_mat_std
            vy_mat_lower_limit = vy_mat_mean - 2*vy_mat_std

            vz_mat = np.array(vel_z_meas_mat)
            
            vz_mat_mean = np.mean(vz_mat, axis=0)
            vz_mat_std = np.std(vz_mat, axis=0)
            vz_mat_upper_limit = vz_mat_mean + 2*vz_mat_std
            vz_mat_lower_limit = vz_mat_mean - 2*vz_mat_std

            vabs_mat = np.sqrt(np.multiply(vx_mat,vx_mat)+np.multiply(vy_mat,vy_mat)+np.multiply(vz_mat,vz_mat))            

            vabs_mat_mean = np.mean(vabs_mat, axis=0)
            vabs_mat_std = np.std(vabs_mat, axis=0)
            vabs_mat_upper_limit = vabs_mat_mean + 2*vabs_mat_std
            vabs_mat_lower_limit = vabs_mat_mean - 2*vabs_mat_std

            vxdes_mat = np.array(vel_x_des_mat)
            
            vxdes_mat_mean = np.mean(vxdes_mat, axis=0)
            vxdes_mat_std = np.std(vxdes_mat, axis=0)
            vxdes_mat_upper_limit = vxdes_mat_mean + 2*vxdes_mat_std
            vxdes_mat_lower_limit = vxdes_mat_mean - 2*vxdes_mat_std 

            vydes_mat = np.array(vel_y_des_mat)
            
            vydes_mat_mean = np.mean(vydes_mat, axis=0)
            vydes_mat_std = np.std(vydes_mat, axis=0)
            vydes_mat_upper_limit = vydes_mat_mean + 2*vydes_mat_std
            vydes_mat_lower_limit = vydes_mat_mean - 2*vydes_mat_std

            vzdes_mat = np.array(vel_z_des_mat)
            
            vzdes_mat_mean = np.mean(vzdes_mat, axis=0)
            vzdes_mat_std = np.std(vzdes_mat, axis=0)
            vzdes_mat_upper_limit = vzdes_mat_mean + 2*vzdes_mat_std
            vzdes_mat_lower_limit = vzdes_mat_mean - 2*vzdes_mat_std

            rex_mat = np.array(rx_mat)
            
            rex_mat_mean = np.mean(rex_mat, axis=0)
            rex_mat_std = np.std(rex_mat, axis=0)
            rex_mat_upper_limit = rex_mat_mean + 2*rex_mat_std
            rex_mat_lower_limit = rex_mat_mean - 2*rex_mat_std 

            rey_mat = np.array(ry_mat)
            
            rey_mat_mean = np.mean(rey_mat, axis=0)
            rey_mat_std = np.std(rey_mat, axis=0)
            rey_mat_upper_limit = rey_mat_mean + 2*rey_mat_std
            rey_mat_lower_limit = rey_mat_mean - 2*rey_mat_std 

            rez_mat = np.array(rz_mat)
            
            rez_mat_mean = np.mean(rez_mat, axis=0)
            rez_mat_std = np.std(rez_mat, axis=0)
            rez_mat_upper_limit = rez_mat_mean + 2*rez_mat_std
            rez_mat_lower_limit = rez_mat_mean - 2*rez_mat_std 
            
                           
            if c==0 or c==2:
                
                if c == 0:
                    
                    col = 'g'
                else:
                    col = 'r'
            
                t = np.arange(1, min_t+1)
                
                if c==2 and (i==0 or i==4):
                    ax6_1.plot(t, rex_mat_mean, label = list_of_names[i], linewidth=2.5)
                    ax6_1.fill_between(t, rex_mat_lower_limit, rex_mat_upper_limit, alpha=0.2)
                    ax6_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax6_1.grid('on')
                    ax6_1.set_ylabel(r"$x_{I}$", fontsize=22)
                    ax6_1.set_xlim(t[0], t[-1])
                    ax6_1.tick_params(axis='both', which='major', labelsize=15)
                    ax6_1.legend(loc="lower left",fontsize=20)                    

                    ax6_2.plot(t, rey_mat_mean, label = list_of_names[i], linewidth=2.5)
                    ax6_2.fill_between(t, rey_mat_lower_limit, rey_mat_upper_limit, alpha=0.2)
                    ax6_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax6_2.grid('on')
                    ax6_2.set_ylabel(r"$y_{I}$", fontsize=22)
                    ax6_2.set_xlim(t[0], t[-1])
                    ax6_2.tick_params(axis='both', which='major', labelsize=15)
                    ax6_2.legend(loc="lower left",fontsize=20)                    

                    ax6_3.plot(t, rez_mat_mean, label = list_of_names[i], linewidth=2.5)
                    ax6_3.fill_between(t, rez_mat_lower_limit, rez_mat_upper_limit, alpha=0.2)
                    ax6_3.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax6_3.grid('on')
                    ax6_3.set_ylabel(r"$z_{I}$", fontsize=22)
                    ax6_3.set_xlim(t[0], t[-1])
                    ax6_3.tick_params(axis='both', which='major', labelsize=15)
                    ax6_3.legend(loc="lower left",fontsize=20) 

                if c==2 and (i==1 or i==2 or i==3):
                    ax7_1.plot(t, rex_mat_mean, label = list_of_names[i], linewidth=2.5)
                    ax7_1.fill_between(t, rex_mat_lower_limit, rex_mat_upper_limit, alpha=0.2)
                    ax7_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax7_1.grid('on')
                    ax7_1.set_ylabel(r"$x_{I}$", fontsize=22)
                    ax7_1.set_xlim(t[0], t[-1])
                    ax7_1.tick_params(axis='both', which='major', labelsize=15)
                    ax7_1.legend(loc="lower left",fontsize=20)                    

                    ax7_2.plot(t, rey_mat_mean, label = list_of_names[i], linewidth=2.5)
                    ax7_2.fill_between(t, rey_mat_lower_limit, rey_mat_upper_limit, alpha=0.2)
                    ax7_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax7_2.grid('on')
                    ax7_2.set_ylabel(r"$y_{I}$", fontsize=22)
                    ax7_2.set_xlim(t[0], t[-1])
                    ax7_2.tick_params(axis='both', which='major', labelsize=15)
                    ax7_2.legend(loc="lower left",fontsize=20)                    

                    ax7_3.plot(t, rez_mat_mean, label = list_of_names[i], linewidth=2.5)
                    ax7_3.fill_between(t, rez_mat_lower_limit, rez_mat_upper_limit, alpha=0.2)
                    ax7_3.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax7_3.grid('on')
                    ax7_3.set_ylabel(r"$z_{I}$", fontsize=22)
                    ax7_3.set_xlim(t[0], t[-1])
                    ax7_3.tick_params(axis='both', which='major', labelsize=15)
                    ax7_3.legend(loc="lower left",fontsize=20) 
                    
                if i==1:
                    ax4_1.plot(t, vx_mat_mean, label = 'Measured'+', '+legend_name, linewidth=2.5)
                    ax4_1.fill_between(t, vx_mat_lower_limit, vx_mat_upper_limit, alpha=0.2)
                    ax4_1.plot(t, vxdes_mat_mean,'--',color = col,label = '$v_{b}^{Arm}$'+', '+legend_name, linewidth=2.5)
                    ax4_1.fill_between(t, vxdes_mat_lower_limit, vxdes_mat_upper_limit, alpha=0.1)
                    ax4_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax4_1.grid('on')
                    ax4_1.set_ylabel(r"$v_{x}$", fontsize=22)
                    ax4_1.set_ylim(-0.02, 0.015)
                    ax4_1.set_xlim(t[0], t[-1])
                    ax4_1.tick_params(axis='both', which='major', labelsize=15)
                    ax4_1.legend(loc="lower right",fontsize=20)
                
                    ax4_2.plot(t, vy_mat_mean, label = 'Measured'+', '+legend_name, linewidth=2.5)
                    ax4_2.fill_between(t, vy_mat_lower_limit, vy_mat_upper_limit, alpha=0.2)
                    ax4_2.plot(t, vydes_mat_mean,'--',color = col,label = '$v_{b}^{Arm}$'+', '+legend_name, linewidth=2.5)
                    #ax4_2.fill_between(t, vydes_mat_lower_limit, vydes_mat_upper_limit, alpha=0.1)
                    ax4_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax4_2.grid('on')
                    ax4_2.set_ylabel(r"$v_{y}$", fontsize=22)
                    ax4_2.set_ylim(-0.12, 0.015)
                    ax4_2.set_xlim(t[0], t[-1])
                    ax4_2.tick_params(axis='both', which='major', labelsize=15)
                    ax4_2.legend(loc="upper right",fontsize=20) 

                    ax4_3.plot(t, vz_mat_mean, label = 'Measured'+', '+legend_name, linewidth=2.5)
                    ax4_3.fill_between(t, vz_mat_lower_limit, vz_mat_upper_limit, alpha=0.2)
                    ax4_3.plot(t, vzdes_mat_mean,'--',color = col,label = '$v_{b}^{Arm}$'+', '+legend_name, linewidth=2.5)
                    #ax4_3.fill_between(t, vzdes_mat_lower_limit, vzdes_mat_upper_limit, alpha=0.1)
                    ax4_3.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax4_3.grid('on')
                    ax4_3.set_ylabel(r"$v_{z}$", fontsize=22)
                    ax4_3.set_ylim(-0.01, 0.005)
                    ax4_3.set_xlim(t[0], t[-1])
                    ax4_3.tick_params(axis='both', which='major', labelsize=15)
                    ax4_3.legend(loc="lower right",fontsize=20) 

                    ax4_4.plot(t, vabs_mat_mean, label = 'Measured'+', '+legend_name, linewidth=2.5)
                    ax4_4.fill_between(t, vabs_mat_lower_limit, vabs_mat_upper_limit, alpha=0.2)
                    if c==0:
                        ax4_4.plot(t, list_of_abs_vel[:len(vabs_mat_mean)],'r--', label = 'Velocity profile', linewidth=4)

                    ax4_4.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax4_4.grid('on')
                    ax4_4.set_ylabel(r"$|v|$", fontsize=22)
                    ax4_4.set_ylim(0.0, 0.12)
                    ax4_4.set_xlim(t[0], t[-1])
                    ax4_4.tick_params(axis='both', which='major', labelsize=15) 
                    ax4_4.set_xlabel("k-th iteration", fontsize=22)
                    ax4_4.legend(loc="lower right",fontsize=20)
                   
                if i==2:
                    ax5_1.plot(t, vx_mat_mean, label = 'Measured'+', '+legend_name, linewidth=2.5)
                    ax5_1.fill_between(t, vx_mat_lower_limit, vx_mat_upper_limit, alpha=0.2)
                    ax5_1.plot(t, vxdes_mat_mean,'--',color = col,label = '$v_{b}^{Arm}$'+', '+legend_name, linewidth=2.5)
                    #ax5_1.fill_between(t, vxdes_mat_lower_limit, vxdes_mat_upper_limit, alpha=0.1)
                    ax5_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax5_1.grid('on')
                    ax5_1.set_ylabel(r"$v_{x}$", fontsize=22)
                    ax5_1.set_ylim(-0.12, 0.005)
                    ax5_1.set_xlim(t[0], t[-1])
                    ax5_1.tick_params(axis='both', which='major', labelsize=15)
                    ax5_1.legend(loc="lower left",fontsize=20)
                    
                    ax5_2.plot(t, vy_mat_mean, label = 'Measured'+', '+legend_name, linewidth=2.5)
                    ax5_2.fill_between(t, vy_mat_lower_limit, vy_mat_upper_limit, alpha=0.2)
                    ax5_2.plot(t, vydes_mat_mean,'--',color = col,label = '$v_{b}^{Arm}$'+', '+legend_name, linewidth=2.5)
                    #ax5_2.fill_between(t, vydes_mat_lower_limit, vydes_mat_upper_limit, alpha=0.1)
                    ax5_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax5_2.grid('on')
                    ax5_2.set_ylabel(r"$v_{y}$", fontsize=22)
                    ax5_2.set_ylim(-0.12, 0.015)
                    ax5_2.set_xlim(t[0], t[-1])
                    ax5_2.tick_params(axis='both', which='major', labelsize=15)
                    ax5_2.legend(loc="upper right",fontsize=20) 
                    
                    ax5_3.plot(t, vz_mat_mean, label = 'Measured'+', '+legend_name, linewidth=2.5)
                    ax5_3.fill_between(t, vz_mat_lower_limit, vz_mat_upper_limit, alpha=0.2)
                    ax5_3.plot(t, vzdes_mat_mean,'--',color = col,label = '$v_{b}^{Arm}$'+', '+legend_name, linewidth=2.5)
                    #ax5_3.fill_between(t, vzdes_mat_lower_limit, vzdes_mat_upper_limit, alpha=0.1) 
                    ax5_3.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax5_3.grid('on')
                    ax5_3.set_ylabel(r"$v_{z}$", fontsize=22)
                    ax5_3.set_ylim(-0.01, 0.005)
                    ax5_3.set_xlim(t[0], t[-1])
                    ax5_3.tick_params(axis='both', which='major', labelsize=15) 
                    ax5_3.legend(loc="lower right",fontsize=20)

                    ax5_4.plot(t, vabs_mat_mean, label = 'Measured'+', '+legend_name, linewidth=2.5)
                    ax5_4.fill_between(t, vabs_mat_lower_limit, vabs_mat_upper_limit, alpha=0.2)
                    if c==0:
                        ax5_4.plot(t, list_of_abs_vel[:len(vabs_mat_mean)],'r--', label = 'Velocity profile', linewidth=4)

                    ax5_4.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax5_4.grid('on')
                    ax5_4.set_ylabel(r"$|v|$", fontsize=22)
                    ax5_4.set_ylim(0.0, 0.12)
                    ax5_4.set_xlim(t[0], t[-1])
                    ax5_4.tick_params(axis='both', which='major', labelsize=15) 
                    ax5_4.set_xlabel("k-th iteration", fontsize=22)
                    ax5_4.legend(loc="lower right",fontsize=20)
                    
            t = np.arange(1, min_t+1)

            Init1_iteration_times = np.array(planning_time_mat)
            
            Init1_iteration_times_mean = np.mean(Init1_iteration_times, axis=0)
            Init1_iteration_times_std = np.std(Init1_iteration_times, axis=0)
            Init1_iteration_times_upper_limit = Init1_iteration_times_mean + 2*Init1_iteration_times_std
            Init1_iteration_times_lower_limit = Init1_iteration_times_mean - 2*Init1_iteration_times_std           
                    
            Init1_manipulability = np.array(manipulability_mat)
            
            Init1_manipulability_mean = np.mean(Init1_manipulability, axis=0)
            Init1_manipulability_std = np.std(Init1_manipulability, axis=0)
            Init1_manipulability_upper_limit = Init1_manipulability_mean + 2*Init1_manipulability_std
            Init1_manipulability_lower_limit = Init1_manipulability_mean - 2*Init1_manipulability_std            
                    
            Init1_dot = np.array(dot_product_mat)
            Init1_ang = np.squeeze(np.arccos(np.array(dot_product_mat)))*180.0/3.14
            
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
                    
            if c==2:

                Init1_manipulabilitym1 = np.array(manipulabilitym1_mat)
            
                Init1_manipulabilitym1_mean = np.mean(Init1_manipulabilitym1, axis=0)
                Init1_manipulabilitym1_std = np.std(Init1_manipulabilitym1, axis=0)
                Init1_manipulabilitym1_upper_limit = Init1_manipulabilitym1_mean + 2*Init1_manipulabilitym1_std
                Init1_manipulabilitym1_lower_limit = Init1_manipulabilitym1_mean - 2*Init1_manipulabilitym1_std    
                Init1_manipulabilitym1 = np.array(manipulabilitym1_mat)

                Init1_manipulabilitym2 = np.array(manipulabilitym2_mat)
            
                Init1_manipulabilitym2_mean = np.mean(Init1_manipulabilitym2, axis=0)
                Init1_manipulabilitym2_std = np.std(Init1_manipulabilitym2, axis=0)
                Init1_manipulabilitym2_upper_limit = Init1_manipulabilitym2_mean + 2*Init1_manipulabilitym2_std
                Init1_manipulabilitym2_lower_limit = Init1_manipulabilitym2_mean - 2*Init1_manipulabilitym2_std
                
                Init1_manipulabilityb1 = np.array(manipulabilityb1_mat)
            
                Init1_manipulabilityb1_mean = np.mean(Init1_manipulabilityb1, axis=0)
                Init1_manipulabilityb1_std = np.std(Init1_manipulabilityb1, axis=0)
                Init1_manipulabilityb1_upper_limit = Init1_manipulabilityb1_mean + 2*Init1_manipulabilityb1_std
                Init1_manipulabilityb1_lower_limit = Init1_manipulabilityb1_mean - 2*Init1_manipulabilityb1_std
                
                Init1_manipulabilityb2 = np.array(manipulabilityb2_mat)
            
                Init1_manipulabilityb2_mean = np.mean(Init1_manipulabilityb2, axis=0)
                Init1_manipulabilityb2_std = np.std(Init1_manipulabilityb2, axis=0)
                Init1_manipulabilityb2_upper_limit = Init1_manipulabilityb2_mean + 2*Init1_manipulabilityb2_std
                Init1_manipulabilityb2_lower_limit = Init1_manipulabilityb2_mean - 2*Init1_manipulabilityb2_std
                
                Init1_manipulabilityratio1 = np.array(manipulabilityratio1_mat)
            
                Init1_manipulabilityratio1_mean = np.mean(Init1_manipulabilityratio1, axis=0)
                Init1_manipulabilityratio1_std = np.std(Init1_manipulabilityratio1, axis=0)
                Init1_manipulabilityratio1_upper_limit = Init1_manipulabilityratio1_mean + 2*Init1_manipulabilityratio1_std
                Init1_manipulabilityratio1_lower_limit = Init1_manipulabilityratio1_mean - 2*Init1_manipulabilityratio1_std

                Init1_manipulabilityratio2 = np.array(manipulabilityratio2_mat)
            
                Init1_manipulabilityratio2_mean = np.mean(Init1_manipulabilityratio2, axis=0)
                Init1_manipulabilityratio2_std = np.std(Init1_manipulabilityratio2, axis=0)
                Init1_manipulabilityratio2_upper_limit = Init1_manipulabilityratio2_mean + 2*Init1_manipulabilityratio2_std
                Init1_manipulabilityratio2_lower_limit = Init1_manipulabilityratio2_mean - 2*Init1_manipulabilityratio2_std

                Init1_manipulabilityf1 = np.array(manipulabilityf1_mat)
            
                Init1_manipulabilityf1_mean = np.mean(Init1_manipulabilityf1, axis=0)
                Init1_manipulabilityf1_std = np.std(Init1_manipulabilityf1, axis=0)
                Init1_manipulabilityf1_upper_limit = Init1_manipulabilityf1_mean + 2*Init1_manipulabilityf1_std
                Init1_manipulabilityf1_lower_limit = Init1_manipulabilityf1_mean - 2*Init1_manipulabilityf1_std

                Init1_manipulabilityf2 = np.array(manipulabilityf2_mat)
            
                Init1_manipulabilityf2_mean = np.mean(Init1_manipulabilityf2, axis=0)
                Init1_manipulabilityf2_std = np.std(Init1_manipulabilityf2, axis=0)
                Init1_manipulabilityf2_upper_limit = Init1_manipulabilityf2_mean + 2*Init1_manipulabilityf2_std
                Init1_manipulabilityf2_lower_limit = Init1_manipulabilityf2_mean - 2*Init1_manipulabilityf2_std
                
                #-----

                Init1_dot_productm1 = np.array(dot_productm1_mat)
            
                Init1_dot_productm1_mean = np.mean(Init1_dot_productm1, axis=0)
                Init1_dot_productm1_std = np.std(Init1_dot_productm1, axis=0)
                Init1_dot_productm1_upper_limit = Init1_dot_productm1_mean + 2*Init1_dot_productm1_std
                Init1_dot_productm1_lower_limit = Init1_dot_productm1_mean - 2*Init1_dot_productm1_std 
                
                for elem in range(len(Init1_dot_productm1_upper_limit)):
                    if Init1_dot_productm1_upper_limit[elem]>1:
                        Init1_dot_productm1_upper_limit[elem] = 1 
                        
                Init1_dot_productm2 = np.array(dot_productm2_mat)
            
                Init1_dot_productm2_mean = np.mean(Init1_dot_productm2, axis=0)
                Init1_dot_productm2_std = np.std(Init1_dot_productm2, axis=0)
                Init1_dot_productm2_upper_limit = Init1_dot_productm2_mean + 2*Init1_dot_productm2_std
                Init1_dot_productm2_lower_limit = Init1_dot_productm2_mean - 2*Init1_dot_productm2_std
                
                for elem in range(len(Init1_dot_productm2_upper_limit)):
                    if Init1_dot_productm2_upper_limit[elem]>1:
                        Init1_dot_productm2_upper_limit[elem] = 1 
                
                Init1_dot_productb1 = np.array(dot_productb1_mat)
            
                Init1_dot_productb1_mean = np.mean(Init1_dot_productb1, axis=0)
                Init1_dot_productb1_std = np.std(Init1_dot_productb1, axis=0)
                Init1_dot_productb1_upper_limit = Init1_dot_productb1_mean + 2*Init1_dot_productb1_std
                Init1_dot_productb1_lower_limit = Init1_dot_productb1_mean - 2*Init1_dot_productb1_std

                for elem in range(len(Init1_dot_productb1_upper_limit)):
                    if Init1_dot_productb1_upper_limit[elem]>1:
                        Init1_dot_productb1_upper_limit[elem] = 1
                        
                Init1_dot_productb2 = np.array(dot_productb2_mat)
            
                Init1_dot_productb2_mean = np.mean(Init1_dot_productb2, axis=0)
                Init1_dot_productb2_std = np.std(Init1_dot_productb2, axis=0)
                Init1_dot_productb2_upper_limit = Init1_dot_productb2_mean + 2*Init1_dot_productb2_std
                Init1_dot_productb2_lower_limit = Init1_dot_productb2_mean - 2*Init1_dot_productb2_std

                for elem in range(len(Init1_dot_productb2_upper_limit)):
                    if Init1_dot_productb2_upper_limit[elem]>1:
                        Init1_dot_productb2_upper_limit[elem] = 1
                        
                Init1_dot_productratio1 = np.array(dot_productratio1_mat)
            
                Init1_dot_productratio1_mean = np.mean(Init1_dot_productratio1, axis=0)
                Init1_dot_productratio1_std = np.std(Init1_dot_productratio1, axis=0)
                Init1_dot_productratio1_upper_limit = Init1_dot_productratio1_mean + 2*Init1_dot_productratio1_std
                Init1_dot_productratio1_lower_limit = Init1_dot_productratio1_mean - 2*Init1_dot_productratio1_std
                
                for elem in range(len(Init1_dot_productratio1_upper_limit)):
                    if Init1_dot_productratio1_upper_limit[elem]>1:
                        Init1_dot_productratio1_upper_limit[elem] = 1

                Init1_dot_productratio2 = np.array(dot_productratio2_mat)
            
                Init1_dot_productratio2_mean = np.mean(Init1_dot_productratio2, axis=0)
                Init1_dot_productratio2_std = np.std(Init1_dot_productratio2, axis=0)
                Init1_dot_productratio2_upper_limit = Init1_dot_productratio2_mean + 2*Init1_dot_productratio2_std
                Init1_dot_productratio2_lower_limit = Init1_dot_productratio2_mean - 2*Init1_dot_productratio2_std

                for elem in range(len(Init1_dot_productratio2_upper_limit)):
                    if Init1_dot_productratio2_upper_limit[elem]>1:
                        Init1_dot_productratio2_upper_limit[elem] = 1
                        
                Init1_dot_productf1 = np.array(dot_productf1_mat)
            
                Init1_dot_productf1_mean = np.mean(Init1_dot_productf1, axis=0)
                Init1_dot_productf1_std = np.std(Init1_dot_productf1, axis=0)
                Init1_dot_productf1_upper_limit = Init1_dot_productf1_mean + 2*Init1_dot_productf1_std
                Init1_dot_productf1_lower_limit = Init1_dot_productf1_mean - 2*Init1_dot_productf1_std

                for elem in range(len(Init1_dot_productf1_upper_limit)):
                    if Init1_dot_productf1_upper_limit[elem]>1:
                        Init1_dot_productf1_upper_limit[elem] = 1
                        
                Init1_dot_productf2 = np.array(dot_productf2_mat)
            
                Init1_dot_productf2_mean = np.mean(Init1_dot_productf2, axis=0)
                Init1_dot_productf2_std = np.std(Init1_dot_productf2, axis=0)
                Init1_dot_productf2_upper_limit = Init1_dot_productf2_mean + 2*Init1_dot_productf2_std
                Init1_dot_productf2_lower_limit = Init1_dot_productf2_mean - 2*Init1_dot_productf2_std
                
                for elem in range(len(Init1_dot_productf2_upper_limit)):
                    if Init1_dot_productf2_upper_limit[elem]>1:
                        Init1_dot_productf2_upper_limit[elem] = 1
                
                if i==1:
                    
                    ts = np.arange(1, min_t+1)
                    ax8_2.plot(ts, Init1_manipulabilitym1_mean, label = r"$m=0.25$", linewidth=2.5)
                    ax8_2.fill_between(t, Init1_manipulabilitym1_lower_limit, Init1_manipulabilitym1_upper_limit, alpha=0.15)  
                    ax8_2.plot(ts, Init1_manipulability_mean, label = r"$m=0.5$", linewidth=2.5)
                    ax8_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)                    
                    ax8_2.plot(ts, Init1_manipulabilitym2_mean, label = r"$m=0.75$", linewidth=2.5)
                    ax8_2.fill_between(t, Init1_manipulabilitym2_lower_limit, Init1_manipulabilitym2_upper_limit, alpha=0.15)                    
                    
                    ax8_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax8_2.grid('on')
                    ax8_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax8_2.set_xlim(t[0], t[-1])
                    ax8_2.tick_params(axis='both', which='major', labelsize=15)
                    ax8_2.legend(loc="lower right", fontsize=20)
                    ax8_2.set_xlabel("k-th iteration", fontsize=22)

                    ax8_1.plot(ts, Init1_dot_productm1_mean, label = r"$m=0.25$", linewidth=2.5)
                    ax8_1.fill_between(t, Init1_dot_productm1_lower_limit, Init1_dot_productm1_upper_limit, alpha=0.15)  
                    ax8_1.plot(ts, Init1_dot_mean, label = r"$m=0.5$", linewidth=2.5)
                    ax8_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)                    
                    ax8_1.plot(ts, Init1_dot_productm2_mean, label = r"$m=0.75$", linewidth=2.5)
                    ax8_1.fill_between(t, Init1_dot_productm2_lower_limit, Init1_dot_productm2_upper_limit, alpha=0.15)                    
                    
                    ax8_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax8_1.grid('on')
                    ax8_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax8_1.set_xlim(t[0], t[-1])
                    ax8_1.tick_params(axis='both', which='major', labelsize=15)
                    ax8_1.legend(loc="lower right", fontsize=20)
                    
                    #-----

                    ax10_2.plot(ts, Init1_manipulabilityf1_mean, label = r"$f=\frac{1}{2}f_{0}$", linewidth=2.5)
                    ax10_2.fill_between(t, Init1_manipulabilityf1_lower_limit, Init1_manipulabilityf1_upper_limit, alpha=0.15)  
                    ax10_2.plot(ts, Init1_manipulability_mean, label = r"$f=f_{0}$", linewidth=2.5)
                    ax10_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)                    
                    ax10_2.plot(ts, Init1_manipulabilityf2_mean, label = r"$f=\frac{3}{2}f_{0}$", linewidth=2.5)
                    ax10_2.fill_between(t, Init1_manipulabilityf2_lower_limit, Init1_manipulabilityf2_upper_limit, alpha=0.15)                    
                    
                    ax10_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax10_2.grid('on')
                    ax10_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax10_2.set_xlim(t[0], t[-1])
                    ax10_2.tick_params(axis='both', which='major', labelsize=15)
                    ax10_2.legend(loc="lower right", fontsize=20)
                    ax10_2.set_xlabel("k-th iteration", fontsize=22)

                    ax10_1.plot(ts, Init1_dot_productf1_mean, label = r"$f=\frac{1}{2}f_{0}$", linewidth=2.5)
                    ax10_1.fill_between(t, Init1_dot_productf1_lower_limit, Init1_dot_productf1_upper_limit, alpha=0.15)  
                    ax10_1.plot(ts, Init1_dot_mean, label = r"$f=f_{0}$", linewidth=2.5)
                    ax10_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)                    
                    ax10_1.plot(ts, Init1_dot_productf2_mean, label = r"$f=\frac{3}{2}f_{0}$", linewidth=2.5)
                    ax10_1.fill_between(t, Init1_dot_productf2_lower_limit, Init1_dot_productf2_upper_limit, alpha=0.15)                    
                    
                    ax10_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax10_1.grid('on')
                    ax10_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax10_1.set_xlim(t[0], t[-1])
                    ax10_1.tick_params(axis='both', which='major', labelsize=15)
                    ax10_1.legend(loc="lower right", fontsize=20)
                    
                    #-----

                    ax12_2.plot(ts, Init1_manipulabilityb1_mean, label = r"$\beta=0.25$", linewidth=2.5)
                    ax12_2.fill_between(t, Init1_manipulabilityb1_lower_limit, Init1_manipulabilityb1_upper_limit, alpha=0.15)  
                    ax12_2.plot(ts, Init1_manipulability_mean, label = r"$\beta=0.5$", linewidth=2.5)
                    ax12_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)                    
                    ax12_2.plot(ts, Init1_manipulabilityb2_mean, label = r"$\beta=0.75$", linewidth=2.5)
                    ax12_2.fill_between(t, Init1_manipulabilityb2_lower_limit, Init1_manipulabilityb2_upper_limit, alpha=0.15)                    
                    
                    ax12_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax12_2.grid('on')
                    ax12_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax12_2.set_xlim(t[0], t[-1])
                    ax12_2.tick_params(axis='both', which='major', labelsize=15)
                    ax12_2.legend(loc="lower right", fontsize=20)
                    ax12_2.set_xlabel("k-th iteration", fontsize=22)

                    ax12_1.plot(ts, Init1_dot_productb1_mean, label = r"$\beta=0.25$", linewidth=2.5)
                    ax12_1.fill_between(t, Init1_dot_productb1_lower_limit, Init1_dot_productb1_upper_limit, alpha=0.15)  
                    ax12_1.plot(ts, Init1_dot_mean, label = r"$\beta=0.5$", linewidth=2.5)
                    ax12_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)                    
                    ax12_1.plot(ts, Init1_dot_productb2_mean, label = r"$\beta=0.75$", linewidth=2.5)
                    ax12_1.fill_between(t, Init1_dot_productb2_lower_limit, Init1_dot_productb2_upper_limit, alpha=0.15)                    
                    
                    ax12_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax12_1.grid('on')
                    ax12_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax12_1.set_xlim(t[0], t[-1])
                    ax12_1.tick_params(axis='both', which='major', labelsize=15)
                    ax12_1.legend(loc="lower right", fontsize=20)

                    #-----
                    
                    ax14_2.plot(ts, Init1_manipulabilityratio1_mean, label = r"$c_{1}/c_{2}=0.5$", linewidth=2.5)
                    ax14_2.fill_between(t, Init1_manipulabilityratio1_lower_limit, Init1_manipulabilityratio1_upper_limit, alpha=0.15)  
                    ax14_2.plot(ts, Init1_manipulability_mean, label = r"$c_{1}/c_{2}=1$", linewidth=2.5)
                    ax14_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)                    
                    ax14_2.plot(ts, Init1_manipulabilityratio2_mean, label = r"$c_{1}/c_{2}=2$", linewidth=2.5)
                    ax14_2.fill_between(t, Init1_manipulabilityratio2_lower_limit, Init1_manipulabilityratio2_upper_limit, alpha=0.15)                    
                    
                    ax14_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax14_2.grid('on')
                    ax14_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax14_2.set_xlim(t[0], t[-1])
                    ax14_2.tick_params(axis='both', which='major', labelsize=15)
                    ax14_2.legend(loc="lower right", fontsize=20)
                    ax14_2.set_xlabel("k-th iteration", fontsize=22)

                    ax14_1.plot(ts, Init1_dot_productratio1_mean, label = r"$c_{1}/c_{2}=0.5$", linewidth=2.5)
                    ax14_1.fill_between(t, Init1_dot_productratio1_lower_limit, Init1_dot_productratio1_upper_limit, alpha=0.15)  
                    ax14_1.plot(ts, Init1_dot_mean, label = r"$c_{1}/c_{2}=1$", linewidth=2.5)
                    ax14_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)                    
                    ax14_1.plot(ts, Init1_dot_productratio2_mean, label = r"$c_{1}/c_{2}=2$", linewidth=2.5)
                    ax14_1.fill_between(t, Init1_dot_productratio2_lower_limit, Init1_dot_productratio2_upper_limit, alpha=0.15)                    
                    
                    ax14_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax14_1.grid('on')
                    ax14_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax14_1.set_xlim(t[0], t[-1])
                    ax14_1.tick_params(axis='both', which='major', labelsize=15)
                    ax14_1.legend(loc="lower right", fontsize=20)
                    
                if i==2:
                    
                    ts = np.arange(1, min_t+1)
                    ax9_2.plot(ts, Init1_manipulabilitym1_mean, label = r"$m=0.25$", linewidth=2.5)
                    ax9_2.fill_between(t, Init1_manipulabilitym1_lower_limit, Init1_manipulabilitym1_upper_limit, alpha=0.15)  
                    ax9_2.plot(ts, Init1_manipulability_mean, label = r"$m=0.5$", linewidth=2.5)
                    ax9_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)                    
                    ax9_2.plot(ts, Init1_manipulabilitym2_mean, label = r"$m=0.75$", linewidth=2.5)
                    ax9_2.fill_between(t, Init1_manipulabilitym2_lower_limit, Init1_manipulabilitym2_upper_limit, alpha=0.15)                    
                    
                    ax9_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax9_2.grid('on')
                    ax9_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax9_2.set_xlim(t[0], t[-1])
                    ax9_2.tick_params(axis='both', which='major', labelsize=15)
                    ax9_2.legend(loc="lower right", fontsize=20) 
                    ax9_2.set_xlabel("k-th iteration", fontsize=22)
                    
                    ax9_1.plot(ts, Init1_dot_productm1_mean, label = r"$m=0.25$", linewidth=2.5)
                    ax9_1.fill_between(t, Init1_dot_productm1_lower_limit, Init1_dot_productm1_upper_limit, alpha=0.15)  
                    ax9_1.plot(ts, Init1_dot_mean, label = r"$m=0.5$", linewidth=2.5)
                    ax9_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)                    
                    ax9_1.plot(ts, Init1_dot_productm2_mean, label = r"$m=0.75$", linewidth=2.5)
                    ax9_1.fill_between(t, Init1_dot_productm2_lower_limit, Init1_dot_productm2_upper_limit, alpha=0.15)                    
                    
                    ax9_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax9_1.grid('on')
                    ax9_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax9_1.set_xlim(t[0], t[-1])
                    ax9_1.tick_params(axis='both', which='major', labelsize=15)
                    ax9_1.legend(loc="lower right", fontsize=20)
                    
                    #-----
                    
                    ax11_2.plot(ts, Init1_manipulabilityf1_mean, label = r"$f=\frac{1}{2}f_{0}$", linewidth=2.5)
                    ax11_2.fill_between(t, Init1_manipulabilityf1_lower_limit, Init1_manipulabilityf1_upper_limit, alpha=0.15)  
                    ax11_2.plot(ts, Init1_manipulability_mean, label = r"$f=f_{0}$", linewidth=2.5)
                    ax11_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)                    
                    ax11_2.plot(ts, Init1_manipulabilityf2_mean, label = r"$f=\frac{3}{2}f_{0}$", linewidth=2.5)
                    ax11_2.fill_between(t, Init1_manipulabilitym2_lower_limit, Init1_manipulabilitym2_upper_limit, alpha=0.15)                    
                    
                    ax11_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax11_2.grid('on')
                    ax11_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax11_2.set_xlim(t[0], t[-1])
                    ax11_2.tick_params(axis='both', which='major', labelsize=15)
                    ax11_2.legend(loc="lower right", fontsize=20)
                    ax11_2.set_xlabel("k-th iteration", fontsize=22)

                    ax11_1.plot(ts, Init1_dot_productf1_mean, label = r"$f=\frac{1}{2}f_{0}$", linewidth=2.5)
                    ax11_1.fill_between(t, Init1_dot_productf1_lower_limit, Init1_dot_productf1_upper_limit, alpha=0.15)  
                    ax11_1.plot(ts, Init1_dot_mean, label = r"$f=f_{0}$", linewidth=2.5)
                    ax11_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)                    
                    ax11_1.plot(ts, Init1_dot_productf2_mean, label = r"$f=\frac{3}{2}f_{0}$", linewidth=2.5)
                    ax11_1.fill_between(t, Init1_dot_productf2_lower_limit, Init1_dot_productf2_upper_limit, alpha=0.15)                    
                    
                    ax11_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax11_1.grid('on')
                    ax11_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax11_1.set_xlim(t[0], t[-1])
                    ax11_1.tick_params(axis='both', which='major', labelsize=15)
                    ax11_1.legend(loc="lower right", fontsize=20)
                    
                    #-----
                    
                    ax13_2.plot(ts, Init1_manipulabilityb1_mean, label = r"$\beta=0.25$", linewidth=2.5)
                    ax13_2.fill_between(t, Init1_manipulabilityb1_lower_limit, Init1_manipulabilityb1_upper_limit, alpha=0.15)  
                    ax13_2.plot(ts, Init1_manipulability_mean, label = r"$\beta=0.5$", linewidth=2.5)
                    ax13_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)                    
                    ax13_2.plot(ts, Init1_manipulabilityb2_mean, label = r"$\beta=0.75$", linewidth=2.5)
                    ax13_2.fill_between(t, Init1_manipulabilityb2_lower_limit, Init1_manipulabilityb2_upper_limit, alpha=0.15)                    
                    
                    ax13_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax13_2.grid('on')
                    ax13_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax13_2.set_xlim(t[0], t[-1])
                    ax13_2.tick_params(axis='both', which='major', labelsize=15)
                    ax13_2.legend(loc="lower right", fontsize=20)
                    ax13_2.set_xlabel("k-th iteration", fontsize=22)

                    ax13_1.plot(ts, Init1_dot_productb1_mean, label = r"$\beta=0.25$", linewidth=2.5)
                    ax13_1.fill_between(t, Init1_dot_productb1_lower_limit, Init1_dot_productb1_upper_limit, alpha=0.15)  
                    ax13_1.plot(ts, Init1_dot_mean, label = r"$\beta=0.5$", linewidth=2.5)
                    ax13_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)                    
                    ax13_1.plot(ts, Init1_dot_productb2_mean, label = r"$\beta=0.75$", linewidth=2.5)
                    ax13_1.fill_between(t, Init1_dot_productb2_lower_limit, Init1_dot_productb2_upper_limit, alpha=0.15)                    
                    
                    ax13_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax13_1.grid('on')
                    ax13_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax13_1.set_xlim(t[0], t[-1])
                    ax13_1.tick_params(axis='both', which='major', labelsize=15)
                    ax13_1.legend(loc="lower right", fontsize=20)

                    #-----
                    
                    ax15_2.plot(ts, Init1_manipulabilityratio1_mean, label = r"$c_{1}/c_{2}=0.5$", linewidth=2.5)
                    ax15_2.fill_between(t, Init1_manipulabilityratio1_lower_limit, Init1_manipulabilityratio1_upper_limit, alpha=0.15)  
                    ax15_2.plot(ts, Init1_manipulability_mean, label = r"$c_{1}/c_{2}=1$", linewidth=2.5)
                    ax15_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)                    
                    ax15_2.plot(ts, Init1_manipulabilityratio2_mean, label = r"$c_{1}/c_{2}=2$", linewidth=2.5)
                    ax15_2.fill_between(t, Init1_manipulabilityratio2_lower_limit, Init1_manipulabilityratio2_upper_limit, alpha=0.15)                    
                    
                    ax15_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax15_2.grid('on')
                    ax15_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax15_2.set_xlim(t[0], t[-1])
                    ax15_2.tick_params(axis='both', which='major', labelsize=15)
                    ax15_2.legend(loc="lower right", fontsize=20)
                    ax15_2.set_xlabel("k-th iteration", fontsize=22)

                    ax15_1.plot(ts, Init1_dot_productratio1_mean, label = r"$c_{1}/c_{2}=0.5$", linewidth=2.5)
                    ax15_1.fill_between(t, Init1_dot_productratio1_lower_limit, Init1_dot_productratio1_upper_limit, alpha=0.15)  
                    ax15_1.plot(ts, Init1_dot_mean, label = r"$c_{1}/c_{2}=1$", linewidth=2.5)
                    ax15_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)                    
                    ax15_1.plot(ts, Init1_dot_productratio2_mean, label = r"$c_{1}/c_{2}=2$", linewidth=2.5)
                    ax15_1.fill_between(t, Init1_dot_productratio2_lower_limit, Init1_dot_productratio2_upper_limit, alpha=0.15)                    
                    
                    ax15_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax15_1.grid('on')
                    ax15_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax15_1.set_xlim(t[0], t[-1])
                    ax15_1.tick_params(axis='both', which='major', labelsize=15)
                    ax15_1.legend(loc="lower right", fontsize=20)
                    
            if i==1 or i==2:

                if c==0 or c==2:
                    t = np.arange(1, min_t+1)
        
                    Init1_iteration_timescomplete = np.array(planning_timecomplete_mat)
                    
                    Init1_iteration_timescomplete_mean = np.mean(Init1_iteration_timescomplete, axis=0)
                    Init1_iteration_timescomplete_std = np.std(Init1_iteration_timescomplete, axis=0)
                    Init1_iteration_timescomplete_upper_limit = Init1_iteration_timescomplete_mean + 2*Init1_iteration_timescomplete_std
                    Init1_iteration_timescomplete_lower_limit = Init1_iteration_timescomplete_mean - 2*Init1_iteration_timescomplete_std           
                            
                    Init1_manipulabilitycomplete = np.array(manipulabilitycomplete_mat)
                    
                    Init1_manipulabilitycomplete_mean = np.mean(Init1_manipulabilitycomplete, axis=0)
                    Init1_manipulabilitycomplete_std = np.std(Init1_manipulabilitycomplete, axis=0)
                    Init1_manipulabilitycomplete_upper_limit = Init1_manipulabilitycomplete_mean + 2*Init1_manipulabilitycomplete_std
                    Init1_manipulabilitycomplete_lower_limit = Init1_manipulabilitycomplete_mean - 2*Init1_manipulabilitycomplete_std            
                            
                    Init1_dotcomplete = np.array(dot_productcomplete_mat)
                    Init1_angcomplete = np.squeeze(np.arccos(np.array(dot_productcomplete_mat)))*180.0/3.14
                    
                    Init1_dotcomplete_mean = np.mean(Init1_dotcomplete, axis=0) 
                    Init1_dotcomplete_std = np.std(Init1_dotcomplete, axis=0)
                    Init1_dotcomplete_upper_limit = Init1_dotcomplete_mean + 2*Init1_dotcomplete_std
                    Init1_dotcomplete_lower_limit = Init1_dotcomplete_mean - 2*Init1_dotcomplete_std 
                    
                    Init1_angcomplete_mean = np.mean(Init1_angcomplete, axis=0)
                    Init1_angcomplete_std = np.std(Init1_angcomplete, axis=0)
                    Init1_angcomplete_upper_limit = Init1_angcomplete_mean + 2*Init1_angcomplete_std
                    Init1_angcomplete_lower_limit = Init1_angcomplete_mean - 2*Init1_angcomplete_std
                    
                    for elem in range(len(Init1_dotcomplete_upper_limit)):
                        if Init1_dotcomplete_upper_limit[elem]>1:
                            Init1_dotcomplete_upper_limit[elem] = 1 
                            
                    if c==0:                                  
                        ax16_3.plot(t, Init1_iteration_timescomplete_mean, label = list_of_names[i], linewidth=2.5)
                        ax16_3.fill_between(t, Init1_iteration_timescomplete_lower_limit, Init1_iteration_timescomplete_upper_limit, alpha=0.15)
                        ax16_3.axvline(x=initL, linewidth=3, color='k', ls='--')
                        ax16_3.grid('on')
                        ax16_3.set_ylabel(r"$\Delta t_{plan}$", fontsize=22)
                        ax16_3.set_ylim(0.00, 0.0025)
                        ax16_3.set_xlim(t[0], t[-1])
                        ax16_3.tick_params(axis='both', which='major', labelsize=15)
                        ax16_3.legend(loc="lower right", fontsize=20)
                        ax16_3.set_xlabel("k-th iteration", fontsize=22)
                        
                        ax16_2.plot(t, Init1_manipulabilitycomplete_mean, label = list_of_names[i], linewidth=2.5)
                        ax16_2.fill_between(t, Init1_manipulabilitycomplete_lower_limit, Init1_manipulabilitycomplete_upper_limit, alpha=0.15)                    
                        ax16_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                        ax16_2.grid('on')
                        ax16_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                        ax16_2.set_xlim(t[0], t[-1])
                        ax16_2.tick_params(axis='both', which='major', labelsize=15)
                        ax16_2.legend(loc="lower right", fontsize=20)
                        #ax16_2.set_xlabel("k-th iteration", fontsize=22)
    
                        ax16_1.plot(t, Init1_dotcomplete_mean, label = list_of_names[i], linewidth=2.5)
                        ax16_1.fill_between(t, Init1_dotcomplete_lower_limit, Init1_dotcomplete_upper_limit, alpha=0.15)
                                            
                        ax16_1_2.plot(t, Init1_angcomplete_mean, '--', label = list_of_names[i]+', angle', linewidth=2.5)
                        ax16_1_2.fill_between(t, Init1_angcomplete_lower_limit, Init1_angcomplete_upper_limit, alpha=0.15)
                        
                        ax16_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                        ax16_1.grid('on')
                        ax16_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                        ax16_1.set_ylim(0.00, 1.0)
                        ax16_1.set_xlim(t[0], t[-1])
                        ax16_1.tick_params(axis='both', which='major', labelsize=15)
                        ax16_1.legend(loc="upper right", fontsize=20) 
                        
                        ax16_1_2.set_ylabel("Angle "+r"$ [\degree]$", fontsize=22)    #set_ylabel("Angle "+r"$ [\degree]$", fontsize=22)    #set_ylabel(r"$\sphericalangle\left(\overrightarrow{n_{est}}, \overrightarrow{n_{true}}\right) [\degree]$", fontsize=22)
                        ax16_1_2.set_ylim(0.0, math.pi/2*180.0/3.14)
                        ax16_1_2.tick_params(axis='both', which='major', labelsize=15)                     
                        ax16_1_2.legend(loc="center right", fontsize=20)
    
                        
                    if c==2:
                        ax17_3.plot(t, Init1_iteration_timescomplete_mean, label = list_of_names[i], linewidth=2.5)
                        ax17_3.fill_between(t, Init1_iteration_timescomplete_lower_limit, Init1_iteration_timescomplete_upper_limit, alpha=0.15)            
                        ax17_3.axvline(x=initL, linewidth=3, color='k', ls='--')
                        ax17_3.grid('on')
                        ax17_3.set_ylabel(r"$\Delta t_{plan}$", fontsize=22)
                        ax17_3.set_ylim(0.00, 0.02)
                        ax17_3.set_xlim(t[0], t[-1])
                        ax17_3.tick_params(axis='both', which='major', labelsize=15)
                        ax17_3.legend(loc="lower right", fontsize=20)
                        ax17_3.set_xlabel("k-th iteration", fontsize=22)
    
                        ax17_2.plot(t, Init1_manipulabilitycomplete_mean, label = list_of_names[i], linewidth=2.5)
                        ax17_2.fill_between(t, Init1_manipulabilitycomplete_lower_limit, Init1_manipulabilitycomplete_upper_limit, alpha=0.15)                    
                        ax17_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                        ax17_2.grid('on')
                        ax17_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                        ax17_2.set_xlim(t[0], t[-1])
                        ax17_2.tick_params(axis='both', which='major', labelsize=15)
                        ax17_2.legend(loc="lower right", fontsize=20)
                        #ax17_2.set_xlabel("k-th iteration", fontsize=22)
    
                        ax17_1.plot(t, Init1_dotcomplete_mean, label = list_of_names[i], linewidth=2.5)
                        ax17_1.fill_between(t, Init1_dotcomplete_lower_limit, Init1_dotcomplete_upper_limit, alpha=0.15)
                        
                        
                        ax17_1_2.plot(t, Init1_angcomplete_mean,'--', label = list_of_names[i]+', angle', linewidth=2.5)
                        ax17_1_2.fill_between(t, Init1_angcomplete_lower_limit, Init1_angcomplete_upper_limit, alpha=0.15)
                        
                        ax17_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                        ax17_1.grid('on')
                        ax17_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                        ax17_1.set_ylim(0.00, 1.0)
                        ax17_1.set_xlim(t[0], t[-1])
                        ax17_1.tick_params(axis='both', which='major', labelsize=15)
                        ax17_1.legend(loc="upper right", fontsize=20) 
                        
                        ax17_1_2.set_ylabel("Angle "+r"$ [\degree]$", fontsize=22)    #set_ylabel(r"$\sphericalangle\left(\overrightarrow{n_{est}}, \overrightarrow{n_{true}}\right) [\degree]$", fontsize=22)
                        ax17_1_2.set_ylim(0.0, math.pi/2*180.0/3.14)
                        ax17_1_2.tick_params(axis='both', which='major', labelsize=15)                     
                        ax17_1_2.legend(loc="center right", fontsize=20)
                    
                    
                    
            if i==0 or i==4:                #ako su dishwasher ili lid
                
                if c==0:                    #ako je fixed              
                    ax21_3.plot(t, Init1_iteration_times_mean, label = list_of_names[i], linewidth=2.5)
                    ax21_3.fill_between(t, Init1_iteration_times_lower_limit, Init1_iteration_times_upper_limit, alpha=0.15)
                    ax21_3.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax21_3.grid('on')
                    ax21_3.set_ylabel(r"$\Delta t_{plan}$", fontsize=22)
                    ax21_3.set_ylim(0.00, 0.0025)
                    ax21_3.set_xlim(t[0], t[-1])
                    ax21_3.tick_params(axis='both', which='major', labelsize=15)
                    ax21_3.legend(loc="lower right", fontsize=20)
                    ax21_3.set_xlabel("k-th iteration", fontsize=22)
                    
                    ax21_2.plot(t, Init1_manipulability_mean, label = list_of_names[i], linewidth=2.5)
                    ax21_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)                    
                    ax21_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax21_2.grid('on')
                    ax21_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax21_2.set_xlim(t[0], t[-1])
                    ax21_2.tick_params(axis='both', which='major', labelsize=15)
                    ax21_2.legend(loc="lower right", fontsize=20)
                    #ax21_2.set_xlabel("k-th iteration", fontsize=22)

                    ax21_1.plot(t, Init1_dot_mean, label = list_of_names[i], linewidth=2.5)
                    ax21_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)
                                        
                    ax21_1_2.plot(t, Init1_ang_mean, '--',label = list_of_names[i]+', angle', linewidth=2.5)
                    ax21_1_2.fill_between(t, Init1_ang_lower_limit, Init1_ang_upper_limit, alpha=0.15)
                    
                    ax21_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax21_1.grid('on')
                    ax21_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax21_1.set_ylim(0.00, 1.0)
                    ax21_1.set_xlim(t[0], t[-1])
                    ax21_1.tick_params(axis='both', which='major', labelsize=15)
                    ax21_1.legend(loc="upper right", fontsize=20) 
                    
                    ax21_1_2.set_ylabel("Angle "+r"$ [\degree]$", fontsize=22)    #set_ylabel(r"$\sphericalangle\left(\overrightarrow{n_{est}}, \overrightarrow{n_{true}}\right) [\degree]$", fontsize=22)
                    ax21_1_2.set_ylim(0.0, math.pi/2*180.0/3.14)
                    ax21_1_2.tick_params(axis='both', which='major', labelsize=15)                     
                    ax21_1_2.legend(loc="center right", fontsize=20)

                    
                if c==2:
                    ax22_3.plot(t, Init1_iteration_times_mean, label = list_of_names[i], linewidth=2.5)
                    ax22_3.fill_between(t, Init1_iteration_times_lower_limit, Init1_iteration_times_upper_limit, alpha=0.15)            
                    ax22_3.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax22_3.grid('on')
                    ax22_3.set_ylabel(r"$\Delta t_{plan}$", fontsize=22)
                    ax22_3.set_ylim(0.00, 0.02)
                    ax22_3.set_xlim(t[0], t[-1])
                    ax22_3.tick_params(axis='both', which='major', labelsize=15)
                    ax22_3.legend(loc="lower right", fontsize=20)
                    ax22_3.set_xlabel("k-th iteration", fontsize=22)

                    ax22_2.plot(t, Init1_manipulability_mean, label = list_of_names[i], linewidth=2.5)
                    ax22_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)                    
                    ax22_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax22_2.grid('on')
                    ax22_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax22_2.set_xlim(t[0], t[-1])
                    ax22_2.tick_params(axis='both', which='major', labelsize=15)
                    ax22_2.legend(loc="lower right", fontsize=20)
                    #ax22_2.set_xlabel("k-th iteration", fontsize=22)

                    ax22_1.plot(t, Init1_dot_mean, label = list_of_names[i], linewidth=2.5)
                    ax22_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)
                    
                    
                    ax22_1_2.plot(t, Init1_ang_mean, '--',label = list_of_names[i]+', angle', linewidth=2.5)
                    ax22_1_2.fill_between(t, Init1_ang_lower_limit, Init1_ang_upper_limit, alpha=0.15)
                    
                    ax22_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax22_1.grid('on')
                    ax22_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax22_1.set_ylim(0.00, 1.0)
                    ax22_1.set_xlim(t[0], t[-1])
                    ax22_1.tick_params(axis='both', which='major', labelsize=15)
                    ax22_1.legend(loc="upper right", fontsize=20) 
                    
                    ax22_1_2.set_ylabel("Angle "+r"$ [\degree]$", fontsize=22)    #set_ylabel(r"$\sphericalangle\left(\overrightarrow{n_{est}}, \overrightarrow{n_{true}}\right) [\degree]$", fontsize=22)
                    ax22_1_2.set_ylim(0.0, math.pi/2*180.0/3.14)
                    ax22_1_2.tick_params(axis='both', which='major', labelsize=15)                     
                    ax22_1_2.legend(loc="center right", fontsize=20)
            else:
            
                if c==0:                    #ako je fixed              
                    ax31_3.plot(t, Init1_iteration_times_mean, label = list_of_names[i], linewidth=2.5)
                    ax31_3.fill_between(t, Init1_iteration_times_lower_limit, Init1_iteration_times_upper_limit, alpha=0.15)
                    ax31_3.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax31_3.grid('on')
                    ax31_3.set_ylabel(r"$\Delta t_{plan}$", fontsize=22)
                    ax31_3.set_ylim(0.00, 0.0025)
                    ax31_3.set_xlim(t[0], t[-1])
                    ax31_3.tick_params(axis='both', which='major', labelsize=15)
                    ax31_3.legend(loc="lower right", fontsize=20)
                    ax31_3.set_xlabel("k-th iteration", fontsize=22)
                    
                    ax31_2.plot(t, Init1_manipulability_mean, label = list_of_names[i], linewidth=2.5)
                    ax31_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)
                    ax31_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax31_2.grid('on')
                    ax31_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax31_2.set_xlim(t[0], t[-1])
                    ax31_2.tick_params(axis='both', which='major', labelsize=15)
                    ax31_2.legend(loc="lower right", fontsize=20)  
                    #ax31_2.set_xlabel("k-th iteration", fontsize=22)

                    ax31_1.plot(t, Init1_dot_mean, label = list_of_names[i], linewidth=2.5)
                    ax31_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)
                    
                    
                    ax31_1_2.plot(t, Init1_ang_mean, '--',label = list_of_names[i]+', angle', linewidth=2.5)
                    ax31_1_2.fill_between(t, Init1_ang_lower_limit, Init1_ang_upper_limit, alpha=0.15)
                    
                    ax31_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax31_1.grid('on')
                    ax31_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax31_1.set_ylim(0.00, 1.0)
                    ax31_1.set_xlim(t[0], t[-1])
                    ax31_1.tick_params(axis='both', which='major', labelsize=15)
                    ax31_1.legend(loc="upper right", fontsize=20) 
                    
                    ax31_1_2.set_ylabel("Angle "+r"$ [\degree]$", fontsize=22)    #set_ylabel(r"$\sphericalangle\left(\overrightarrow{n_{est}}, \overrightarrow{n_{true}}\right) [\degree]$", fontsize=22)
                    ax31_1_2.set_ylim(0.0, math.pi/2*180.0/3.14)
                    ax31_1_2.tick_params(axis='both', which='major', labelsize=15)                     
                    ax31_1_2.legend(loc="center right", fontsize=20)
                    
                if c==2:
                    ax32_3.plot(t, Init1_iteration_times_mean, label = list_of_names[i], linewidth=2.5)
                    ax32_3.fill_between(t, Init1_iteration_times_lower_limit, Init1_iteration_times_upper_limit, alpha=0.15)            
                    ax32_3.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax32_3.grid('on')
                    ax32_3.set_ylabel(r"$\Delta t_{plan}$", fontsize=22)
                    ax32_3.set_ylim(0.00, 0.02)
                    ax32_3.set_xlim(t[0], t[-1])
                    ax32_3.tick_params(axis='both', which='major', labelsize=15)
                    ax32_3.legend(loc="lower right", fontsize=20) 
                    ax32_3.set_xlabel("k-th iteration", fontsize=22)
            
                    ax32_2.plot(t, Init1_manipulability_mean, label = list_of_names[i], linewidth=2.5)
                    ax32_2.fill_between(t, Init1_manipulability_lower_limit, Init1_manipulability_upper_limit, alpha=0.15)            
                    ax32_2.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax32_2.grid('on')
                    ax32_2.set_ylabel("Manipulability index", fontsize=22)  #set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=22)
                    ax32_2.set_xlim(t[0], t[-1])
                    ax32_2.tick_params(axis='both', which='major', labelsize=15)
                    ax32_2.legend(loc="lower right", fontsize=20) 
                    #ax32_2.set_xlabel("k-th iteration", fontsize=22)

                    ax32_1.plot(t, Init1_dot_mean, label = list_of_names[i], linewidth=2.5)
                    ax32_1.fill_between(t, Init1_dot_lower_limit, Init1_dot_upper_limit, alpha=0.15)
                                       
                    ax32_1_2.plot(t, Init1_ang_mean, '--',label = list_of_names[i]+', angle', linewidth=2.5)
                    ax32_1_2.fill_between(t, Init1_ang_lower_limit, Init1_ang_upper_limit, alpha=0.15)
                    
                    ax32_1.axvline(x=initL, linewidth=3, color='k', ls='--')
                    ax32_1.grid('on')
                    ax32_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=22)
                    ax32_1.set_ylim(0.00, 1.0)
                    ax32_1.set_xlim(t[0], t[-1])
                    ax32_1.tick_params(axis='both', which='major', labelsize=15)
                    ax32_1.legend(loc="upper right", fontsize=20) 
                    
                    ax32_1_2.set_ylabel("Angle "+r"$ [\degree]$", fontsize=22)    #set_ylabel(r"$\sphericalangle\left(\overrightarrow{n_{est}}, \overrightarrow{n_{true}}\right) [\degree]$", fontsize=22)
                    ax32_1_2.set_ylim(0.0, math.pi/2*180.0/3.14)
                    ax32_1_2.tick_params(axis='both', which='major', labelsize=15)                     
                    ax32_1_2.legend(loc="center right", fontsize=20)                    
                    
                    
                    
        for br in range(7):
            
            matrix1[c,br,5] = np.mean(matrix1[c,br,:5])
            if br==6:
                
                matrix1[c,br,5] = matrix1[c,br,0]*10/105+matrix1[c,br,1]*20/105+matrix1[c,br,2]*25/105+matrix1[c,br,3]*25/105+matrix1[c,br,4]*25/105
                
    fig21.savefig(folder + '//Shorter_fixed_metrics.png')
    fig22.savefig(folder + '//Shorter_mobile_metrics.png')
                
    fig31.savefig(folder + '//Longer_fixed_metrics.png')
    fig32.savefig(folder + '//Longer_mobile_metrics.png')
    
    fig4.savefig(folder + '//velocities_drawer.png')
    fig5.savefig(folder + '//velocities_room_door.png')
    
    fig6.savefig(folder + '//position_mobile_base1.png')
    fig7.savefig(folder + '//position_mobile_base2.png')
    
    fig8.savefig(folder + '//m_drawer.png')
    fig10.savefig(folder + '//f_drawer.png')
    fig12.savefig(folder + '//b_drawer.png')
    fig14.savefig(folder + '//ratio_drawer.png') 
    
    fig9.savefig(folder + '//m_door.png')
    fig11.savefig(folder + '//f_door.png')
    fig13.savefig(folder + '//b_door.png')
    fig15.savefig(folder + '//ratio_door.png') 

    fig16.savefig(folder + '//Completerun_fixed.png')    
    fig17.savefig(folder + '//Completerun_mobile.png')     
    
    np.save(folder + '/Table_1_'+list_of_controllers[0]+'.npy', np.squeeze(matrix1[0,:,:]))
    np.save(folder + '/Table_1_'+list_of_controllers[1]+'.npy', np.squeeze(matrix1[1,:,:])) 
    np.save(folder + '/Table_1_'+list_of_controllers[2]+'.npy', np.squeeze(matrix1[2,:,:]))
    np.save(folder + '/Table_1_'+list_of_controllers[3]+'.npy', np.squeeze(matrix1[3,:,:]))
    
    a1 = np.transpose(np.squeeze(matrix1complete[:,:,0]))
    b1 = np.transpose(np.squeeze(matrix1complete[:,:,1]))
    np.save(folder + '/Table_complete_run_drawer_door_fb_mb.npy', np.squeeze(np.concatenate((a1,b1),axis=1)))
    
    matrix1bdrawer = np.squeeze(matrix1b[:,:,0,:])
    matrix1bdoor = np.squeeze(matrix1b[:,:,1,:])    
    matrix1bdrawer = np.concatenate((np.squeeze(matrix1bdrawer[0,:,:]), np.squeeze(matrix1bdrawer[1,:,:])),axis=1)
    matrix1bdoor = np.concatenate((np.squeeze(matrix1bdoor[0,:,:]), np.squeeze(matrix1bdoor[1,:,:])),axis=1)
    
    matrix1fdrawer = np.squeeze(matrix1f[:,:,0,:])
    matrix1fdoor = np.squeeze(matrix1f[:,:,1,:])    
    matrix1fdrawer = np.concatenate((np.squeeze(matrix1fdrawer[0,:,:]), np.squeeze(matrix1fdrawer[1,:,:])),axis=1)
    matrix1fdoor = np.concatenate((np.squeeze(matrix1fdoor[0,:,:]), np.squeeze(matrix1fdoor[1,:,:])),axis=1)

    matrix1ratiodrawer = np.squeeze(matrix1ratio[:,:,0,:])
    matrix1ratiodoor = np.squeeze(matrix1ratio[:,:,1,:])    
    matrix1ratiodrawer = np.concatenate((np.squeeze(matrix1ratiodrawer[0,:,:]), np.squeeze(matrix1ratiodrawer[1,:,:])),axis=1)
    matrix1ratiodoor = np.concatenate((np.squeeze(matrix1ratiodoor[0,:,:]), np.squeeze(matrix1ratiodoor[1,:,:])),axis=1)

    matrix1mdrawer = np.squeeze(matrix1m[:,:,0,:])
    matrix1mdoor = np.squeeze(matrix1m[:,:,1,:])    
    matrix1mdrawer = np.concatenate((np.squeeze(matrix1mdrawer[0,:,:]), np.squeeze(matrix1mdrawer[1,:,:])),axis=1)
    matrix1mdoor = np.concatenate((np.squeeze(matrix1mdoor[0,:,:]), np.squeeze(matrix1mdoor[1,:,:])),axis=1)
    
    np.save(folder + '/matrix_b_drawer.npy',matrix1bdrawer)
    np.save(folder + '/matrix_b_door.npy',matrix1bdoor)
    
    np.save(folder + '/matrix_m_drawer.npy',matrix1mdrawer)
    np.save(folder + '/matrix_m_door.npy',matrix1mdoor)   

    np.save(folder + '/matrix_f_drawer.npy',matrix1fdrawer)
    np.save(folder + '/matrix_f_door.npy',matrix1fdoor)

    np.save(folder + '/matrix_ratio_drawer.npy',matrix1ratiodrawer)
    np.save(folder + '/matrix_ratio_door.npy',matrix1ratiodoor)
    
    average_table = np.zeros((7,4))
    
    for aux1 in range(7):
        for aux2 in range(4):
            
            average_table[aux1, aux2]= matrix1[aux2,aux1,5]
            
    np.save(folder + '/Average_table.npy', average_table)
    

            


    fig1.savefig(folder + '//' + 'Metrics.png' )  
    plt.close('all')        

def main():
    
    #big = AnalyseResultsPybullet()
    
    big = np.load(os.getcwd()+'/moma/moma_demos/articulated_demo/RESULTS SECTION RUNS/big_matrix.npy')
    task1(big)
                

if __name__ == "__main__":
    main()
               
                    
    
    
    
    
    
    
    
    
    
    