import numpy as np
import os
import matplotlib.pyplot as plt

#-----------------

EPS = 1e-6
BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def analyze(global_folder, list_of_mode_names, Nx, Ny, Nang, Noff, initL, list_of_removed):

    folder = os.getcwd()+'/moma/moma_demos/articulated_demo/runs'
    folder = folder + '/' + global_folder
    
    comparison_results_folder = os.getcwd()+'/moma/moma_demos/articulated_demo/comparisons'
    
    if not os.path.isdir(comparison_results_folder):
        os.makedirs(comparison_results_folder)
                           
    if not os.path.isdir(comparison_results_folder + '/' + global_folder):
        os.makedirs(comparison_results_folder + '/' + global_folder)    
        
    curr_folder = comparison_results_folder + '/' + global_folder
        
    #----- Determine max length -----
    
    for mode_name in list_of_mode_names:
        
        save_folder = curr_folder + '/' + mode_name 

        if not os.path.isdir(save_folder):
            os.makedirs(save_folder)
        
        actual_t = 1200
        
        list_of_dot_product = []
        list_of_manpulability = []
        list_of_delta_t = []
        
        list_of_vx_meas = []
        list_of_vy_meas = []
        list_of_vz_meas = []
        
        list_of_vx_des = []
        list_of_vy_des = []
        list_of_vz_des = []
                
        list_of_drawer_pos = []
        list_of_f_wrist_frame = []    

        for i1 in range(Nx):
            for i2 in range(Ny):
                for i3 in range(Nang):
                    for i4 in range(Noff):
                        
                        scenName = str(i1)+'_'+str(i2)+'_'+str(i3)+'_'+str(i4)
                        
                        if not scenName in list_of_removed:
                                        
                            folder_name = folder + '/' + scenName + '/' + mode_name
                                    
                            actual_dir = np.load(folder_name+'//actual_dir.npy')
                        
                            curr_t = actual_dir.shape[0]
                        
                            if curr_t<500:
                                print('removed config: ',scenName)
                        
                            if curr_t>500:
                        
                                if curr_t < actual_t:
                        
                                    actual_t = curr_t
                            
                                estimated_dir = np.load(folder_name+'//estimated_dir.npy')
                            
                                list_of_model_fitting_metric = []
                            
                                for i in range(estimated_dir.shape[0]):
            
                                    list_of_model_fitting_metric.append(np.dot(np.squeeze(estimated_dir[i, :]), np.squeeze(actual_dir[i, :])))    
                            
                                list_of_dot_product.append(np.array(list_of_model_fitting_metric))
                            
                                list_of_delta_t.append(np.load(folder_name+'//exec_time.npy'))     
                            
                                lin_vdesEE_O = np.load(folder_name+'//lin_vdesEE_O.npy')
                            
                                list_of_vx_des.append(lin_vdesEE_O[:, 0])
                                list_of_vy_des.append(lin_vdesEE_O[:, 1])
                                list_of_vz_des.append(lin_vdesEE_O[:, 2])                            
                                
                                lin_vEE_O_meas = np.load(folder_name+'//lin_vEE_O_meas.npy')

                                list_of_vx_meas.append(lin_vEE_O_meas[:, 0])
                                list_of_vy_meas.append(lin_vEE_O_meas[:, 1])
                                list_of_vz_meas.append(lin_vEE_O_meas[:, 2])                            
                            
                                list_of_manpulability.append(np.load(folder_name+'//manipulability_meas.npy'))                     
                                list_of_f_wrist_frame.append(np.load(folder_name+'//f_wristframe.npy'))
                            
                                drawer = np.load(folder_name+'//actual_drawer_pos.npy')
                                drawer = drawer.reshape(-1, 3)             
                                list_of_drawer_pos.append(np.squeeze(np.copy(drawer[:, 1])))
        print("Actual t: ", actual_t)                    
        L = len(list_of_dot_product)
        
        for j in range(L):

            list_of_dot_product[j] = np.copy(list_of_dot_product[j][:actual_t])
            list_of_manpulability[j] = np.copy(list_of_manpulability[j][:actual_t])
            list_of_delta_t[j] = np.copy(list_of_delta_t[j][:actual_t])
        
            list_of_vx_meas[j] = np.copy(list_of_vx_meas[j][:actual_t])
            list_of_vy_meas[j] = np.copy(list_of_vy_meas[j][:actual_t])
            list_of_vz_meas[j] = np.copy(list_of_vz_meas[j][:actual_t])
        
            list_of_vx_des[j] = np.copy(list_of_vx_des[j][:actual_t])
            list_of_vy_des[j] = np.copy(list_of_vy_des[j][:actual_t])
            list_of_vz_des[j] = np.copy(list_of_vz_des[j][:actual_t])
                
            list_of_drawer_pos[j] = np.copy(list_of_drawer_pos[j][:actual_t])
            list_of_f_wrist_frame[j]    = np.copy(list_of_f_wrist_frame[j][:actual_t])                                

        mat_of_dot_product = np.array(list_of_dot_product)
        
        mat_of_manpulability = np.array(list_of_manpulability)
        mat_of_delta_t = np.array(list_of_delta_t)
        
        mat_of_vx_meas = np.array(list_of_vx_meas)
        mat_of_vy_meas = np.array(list_of_vy_meas)
        mat_of_vz_meas = np.array(list_of_vz_meas)
        
        mat_of_vx_des = np.array(list_of_vx_des)
        mat_of_vy_des = np.array(list_of_vy_des)
        mat_of_vz_des = np.array(list_of_vz_des)
                
        mat_of_drawer_pos = np.array(list_of_drawer_pos)
        mat_of_f_wrist_frame = np.array(list_of_f_wrist_frame)
        
        #----- Calculate mean and std -----
        
        mean_of_mat_of_dot_product = np.mean(mat_of_dot_product, axis=0)
        std_of_mat_of_dot_product = np.std(mat_of_dot_product, axis=0)
        upper_of_mat_of_dot_product = mean_of_mat_of_dot_product + 2*std_of_mat_of_dot_product
        
        for elem in range(len(upper_of_mat_of_dot_product)):
            if upper_of_mat_of_dot_product[elem]>1:
                upper_of_mat_of_dot_product[elem] = 1
        
        lower_of_mat_of_dot_product = mean_of_mat_of_dot_product - 2*std_of_mat_of_dot_product
        
        mean_of_mat_of_manpulability = np.mean(mat_of_manpulability, axis=0)
        std_of_mat_of_manpulability = np.std(mat_of_manpulability, axis=0)
        upper_of_mat_of_manpulability = mean_of_mat_of_manpulability + 2*std_of_mat_of_manpulability
        lower_of_mat_of_manpulability = mean_of_mat_of_manpulability - 2*std_of_mat_of_manpulability
                                            
        mean_of_mat_of_delta_t = np.mean(mat_of_delta_t, axis=0)
        std_of_mat_of_delta_t = np.std(mat_of_delta_t, axis=0)
        upper_of_mat_of_delta_t = mean_of_mat_of_delta_t + 2*std_of_mat_of_delta_t
        lower_of_mat_of_delta_t = mean_of_mat_of_delta_t - 2*std_of_mat_of_delta_t        

        mean_of_mat_of_vx_meas = np.mean(mat_of_vx_meas, axis=0)
        std_of_mat_of_vx_meas = np.std(mat_of_vx_meas, axis=0)
        upper_of_mat_of_vx_meas = mean_of_mat_of_vx_meas + 2*std_of_mat_of_vx_meas
        lower_of_mat_of_vx_meas = mean_of_mat_of_vx_meas - 2*std_of_mat_of_vx_meas
        
        mean_of_mat_of_vy_meas = np.mean(mat_of_vy_meas, axis=0)
        std_of_mat_of_vy_meas = np.std(mat_of_vy_meas, axis=0)
        upper_of_mat_of_vy_meas = mean_of_mat_of_vy_meas + 2*std_of_mat_of_vy_meas
        lower_of_mat_of_vy_meas = mean_of_mat_of_vy_meas - 2*std_of_mat_of_vy_meas
        
        mean_of_mat_of_vz_meas = np.mean(mat_of_vz_meas, axis=0)
        std_of_mat_of_vz_meas = np.std(mat_of_vz_meas, axis=0)
        upper_of_mat_of_vz_meas = mean_of_mat_of_vz_meas + 2*std_of_mat_of_vz_meas
        lower_of_mat_of_vz_meas = mean_of_mat_of_vz_meas - 2*std_of_mat_of_vz_meas
        
        mean_of_mat_of_vx_des = np.mean(mat_of_vx_des, axis=0)
        std_of_mat_of_vx_des = np.std(mat_of_vx_des, axis=0)
        upper_of_mat_of_vx_des = mean_of_mat_of_vx_des + 2*std_of_mat_of_vx_des
        lower_of_mat_of_vx_des = mean_of_mat_of_vx_des - 2*std_of_mat_of_vx_des
        
        mean_of_mat_of_vy_des = np.mean(mat_of_vy_des, axis=0)
        std_of_mat_of_vy_des = np.std(mat_of_vy_des, axis=0)
        upper_of_mat_of_vy_des = mean_of_mat_of_vy_des + 2*std_of_mat_of_vy_des
        lower_of_mat_of_vy_des = mean_of_mat_of_vy_des - 2*std_of_mat_of_vy_des
        
        mean_of_mat_of_vz_des = np.mean(mat_of_vz_des, axis=0)
        std_of_mat_of_vz_des = np.std(mat_of_vz_des, axis=0)
        upper_of_mat_of_vz_des = mean_of_mat_of_vz_des + 2*std_of_mat_of_vz_des
        lower_of_mat_of_vz_des = mean_of_mat_of_vz_des - 2*std_of_mat_of_vz_des
        
        mean_of_mat_of_drawer_pos = np.mean(mat_of_drawer_pos, axis=0)
        std_of_mat_of_drawer_pos = np.std(mat_of_drawer_pos, axis=0)
        upper_of_mat_of_drawer_pos = mean_of_mat_of_drawer_pos + 2*std_of_mat_of_drawer_pos
        lower_of_mat_of_drawer_pos = mean_of_mat_of_drawer_pos - 2*std_of_mat_of_drawer_pos
        
        mean_of_mat_of_f_wrist_frame = np.mean(mat_of_f_wrist_frame, axis=0)
        std_of_mat_of_f_wrist_frame = np.std(mat_of_f_wrist_frame, axis=0)
        upper_of_mat_of_f_wrist_frame = mean_of_mat_of_f_wrist_frame + 2*std_of_mat_of_f_wrist_frame
        lower_of_mat_of_f_wrist_frame = mean_of_mat_of_f_wrist_frame - 2*std_of_mat_of_f_wrist_frame
        
        #----- Plots -----
        
        t = np.arange(1, actual_t+1)

        fig1, (ax1_1, ax1_2, ax1_3) = plt.subplots(3,1,figsize=(15,15), constrained_layout=True)        
        #fig1, (ax1_1, ax1_2, ax1_3, ax1_4) = plt.subplots(4,1,figsize=(15,15), constrained_layout=True)
        fig1.suptitle('Metrics', fontsize=15)
        
        ax1_1.plot(t, mean_of_mat_of_dot_product)
        ax1_1.fill_between(t, lower_of_mat_of_dot_product, upper_of_mat_of_dot_product, color='gray', alpha=0.2)    
        ax1_1.axvline(x=initL, linewidth=4, color='b', ls='--')
            
        ax1_1.grid('both', 'both')
        ax1_1.set_ylabel(r"$\overrightarrow{n_{est}} \cdot \overrightarrow{n_{true}}$", fontsize=15)
        ax1_1.set_xlim(t[0], t[-1])
        
        ax1_2.plot(t, mean_of_mat_of_manpulability)
        ax1_2.fill_between(t, lower_of_mat_of_manpulability, upper_of_mat_of_manpulability, color='gray', alpha=0.2)    
        ax1_2.axvline(x=initL, linewidth=4, color='b', ls='--')
            
        ax1_2.grid('both', 'both')
        ax1_2.set_ylabel(r"$\sqrt{det(J_{EE}J^{T}_{EE})}$", fontsize=15)
        ax1_2.set_xlim(t[0], t[-1])            
        
        ax1_3.plot(t, mean_of_mat_of_delta_t )
        ax1_3.fill_between(t, lower_of_mat_of_delta_t , upper_of_mat_of_delta_t , color='gray', alpha=0.2)    
        ax1_3.axvline(x=initL, linewidth=4, color='b', ls='--')
            
        ax1_3.grid('both', 'both')
        ax1_3.set_ylabel(r"$\Delta t_{plann}$", fontsize=15)
        ax1_3.set_ylim(0, 0.04)
        ax1_3.set_xlim(t[0], t[-1])
        ax1_3.set_xlabel("k-th iteration", fontsize=15)
        
        #ax1_4.plot(t, mean_of_mat_of_drawer_pos )
        #ax1_4.fill_between(t, lower_of_mat_of_drawer_pos , upper_of_mat_of_drawer_pos, color='gray', alpha=0.2)    
        #ax1_4.axvline(x=initL, linewidth=4, color='b', ls='--')
            
        #ax1_4.grid('both', 'both')
        #ax1_4.set_ylabel(r"$r_{y}$")
        #ax1_4.set_xlim(t[0], t[-1])
        
        fig1.savefig(save_folder + '//' + 'Metrics.png' )
        
        #----- Measured vel -----    
        
        fig2, (ax2_1, ax2_2, ax2_3) = plt.subplots(3,1,figsize=(10,10), constrained_layout=True)
        fig2.suptitle('Measured linear velocities')
        
        ax2_1.plot(t, mean_of_mat_of_vx_meas)
        ax2_1.fill_between(t, lower_of_mat_of_vx_meas, upper_of_mat_of_vx_meas, color='gray', alpha=0.2)    
        ax2_1.axvline(x=initL, linewidth=4, color='b', ls='--')
            
        ax2_1.grid('both', 'both')
        ax2_1.set_ylabel(r"$v^{meas}_{x}$")
        ax2_1.set_xlim(t[0], t[-1])
        
        ax2_2.plot(t, mean_of_mat_of_vy_meas)
        ax2_2.fill_between(t, lower_of_mat_of_vy_meas, upper_of_mat_of_vy_meas, color='gray', alpha=0.2)    
        ax2_2.axvline(x=initL, linewidth=4, color='b', ls='--')
            
        ax2_2.grid('both', 'both')
        ax2_2.set_ylabel(r"$v^{meas}_{y}$")
        ax2_2.set_xlim(t[0], t[-1])            
        
        ax2_3.plot(t, mean_of_mat_of_vz_meas )
        ax2_3.fill_between(t, lower_of_mat_of_vz_meas , upper_of_mat_of_vz_meas , color='gray', alpha=0.2)    
        ax2_3.axvline(x=initL, linewidth=4, color='b', ls='--')
            
        ax2_3.grid('both', 'both')
        ax2_3.set_ylabel(r"$v^{meas}_{z}$")
        ax2_3.set_xlim(t[0], t[-1])
        
        fig2.savefig(save_folder + '//' + 'Measured_vel.png' )
        
        #----- Desired vel -----
        
        fig3, (ax3_1, ax3_2, ax3_3) = plt.subplots(3,1,figsize=(10,10), constrained_layout=True)
        fig3.suptitle('Desired linear velocities')
        
        ax3_1.plot(t, mean_of_mat_of_vx_des)
        ax3_1.fill_between(t, lower_of_mat_of_vx_des, upper_of_mat_of_vx_des, color='gray', alpha=0.2)    
        ax3_1.axvline(x=initL, linewidth=4, color='b', ls='--')
            
        ax3_1.grid('both', 'both')
        ax3_1.set_ylabel(r"$v^{des}_{x}$")
        ax3_1.set_xlim(t[0], t[-1])
        
        ax3_2.plot(t, mean_of_mat_of_vy_des)
        ax3_2.fill_between(t, lower_of_mat_of_vy_des, upper_of_mat_of_vy_des, color='gray', alpha=0.2)    
        ax3_2.axvline(x=initL, linewidth=4, color='b', ls='--')
            
        ax3_2.grid('both', 'both')
        ax3_2.set_ylabel(r"$v^{des}_{y}$")
        ax3_2.set_xlim(t[0], t[-1])            
        
        ax3_3.plot(t, mean_of_mat_of_vz_des )
        ax3_3.fill_between(t, lower_of_mat_of_vz_des , upper_of_mat_of_vz_des , color='gray', alpha=0.2)    
        ax3_3.axvline(x=initL, linewidth=4, color='b', ls='--')
            
        ax3_3.grid('both', 'both')
        ax3_3.set_ylabel(r"$v^{des}_{z}$")
        ax3_3.set_xlim(t[0], t[-1])
        
        fig3.savefig(save_folder + '//' + 'Desired_vel.png' )
                    
#-------
def main():

    globalName = '11_12_2020_00_07_11'
    list_of_mode_names = ['fixed_base_torque_control','moving_base_no_collision_max_mob_control_QCQP']
    
    #list_of_removed = ['0_0_1_0', '0_1_3_0', '1_0_1_0', '1_0_4_0', '2_0_2_0', '4_0_0_0', '4_0_2_0', '4_0_3_0']    
    list_of_removed = ['0_0_0_0', '0_0_2_0', '0_0_3_0']
    
    Nx = 5
    Ny = 1
    Nang = 5
    Noff = 1
    initL = 100
    analyze(globalName, list_of_mode_names, Nx, Ny, Nang, Noff, initL, list_of_removed)    


if __name__ == "__main__":
    main()
