import argparse
import os
import pybullet as p
import pickle
import numpy as np
from datetime import datetime

import matplotlib.pyplot as plt 

def prepare_global_dir():

	date_and_time = datetime.now()
	curr = date_and_time.strftime("%d_%m_%Y_%H_%M_%S")
	
	results_folder = os.getcwd()+'/moma/moma_demos/articulated_demo/runs'
	
	if not os.path.isdir(results_folder):
		os.makedirs(results_folder)
              	 		
	if not os.path.isdir(results_folder + '/' + curr):
		os.makedirs(results_folder + '/' + curr)
	
	global_folder = results_folder + '/' + curr
                	
	return global_folder
	
#-------
def prepare_dir(global_folder, postfix=None):
        
	if postfix is not None:
        
		if not os.path.isdir(global_folder + '/' + postfix):
			os.makedirs(global_folder + '/' + postfix)
		curr_folder = global_folder + '/' + postfix 
	
	else:
	       	 		
		if not os.path.isdir(global_folder):
			os.makedirs(global_folder)
		curr_folder = global_folder
                	
	return curr_folder

#-------        	
def save_run(sk_traj, curr_folder, mode_name):

	folder_name = curr_folder + '/' + mode_name
	
	if not os.path.isdir(folder_name):
		os.makedirs(folder_name)
        		  
	np.save(folder_name+'/q.npy', np.array(sk_traj.log_q))
	np.save(folder_name+'/q_dot.npy', np.array(sk_traj.log_q_dot))
	np.save(folder_name+'/tau.npy', np.array(sk_traj.log_tau))
		
	np.save(folder_name+'/estimated_dir.npy', np.array(sk_traj.log_estimated_dir))		
	np.save(folder_name+'/actual_dir.npy',  np.array(sk_traj.log_actual_dir))			
								
	np.save(folder_name+'/exec_time.npy', np.array(sk_traj.log_exec_time)) 
	np.save(folder_name+'/lin_vdesEE_O.npy', np.array(sk_traj.log_lin_vdesEE_O))
	np.save(folder_name+'/ang_vdesEE_O.npy', np.array(sk_traj.log_ang_vdesEE_O))
		
	np.save(folder_name+'/theta.npy', np.array(sk_traj.log_theta))
	np.save(folder_name+'/lin_vEE_O_meas.npy', np.array(sk_traj.log_lin_vEE_O_meas))
	np.save(folder_name+'/ang_vEE_O_meas.npy', np.array(sk_traj.log_ang_vEE_O_meas))
	np.save(folder_name+'/manipulability_meas.npy', np.array(sk_traj.log_manipulability_meas)) 
		
	np.save(folder_name+'/lin_vBase_O_meas.npy', np.array(sk_traj.log_lin_vBase_O_meas)) 
	np.save(folder_name+'/ang_vBase_O_meas.npy', np.array(sk_traj.log_ang_vBase_O_meas))
		
	np.save(folder_name+'/q_dot_optimal.npy', np.array(sk_traj.log_q_dot_optimal))
	np.save(folder_name+'/relative_v.npy', np.array(sk_traj.log_relative_v))
	
	np.save(folder_name+'/f_wristframe.npy', np.array(sk_traj.log_f_wristframe))
	np.save(folder_name+'/t_wristframe.npy', np.array(sk_traj.log_t_wristframe))
	np.save(folder_name+'/actual_drawer_pos.npy', np.array(sk_traj.log_actual_drawer_pos))
	
	print("***** Run saved! *****")
	
#-------	
def plot_runs(sk_traj, curr_folder, list_of_modes):

	print("Preparing plots...")

	for mode_name in list_of_modes:
	
		folder_name = curr_folder + '/' + mode_name
		
		plots_folder_name = folder_name + '/plots'
		
		if not os.path.isdir(plots_folder_name):
			os.makedirs(plots_folder_name)
		
		#----- LOAD -----
		
		q = np.load(folder_name+'//q.npy')
		q_dot = np.load(folder_name+'//q_dot.npy')
		tau = np.load(folder_name+'//tau.npy')
		
		estimated_dir = np.load(folder_name+'//estimated_dir.npy')		
		actual_dir = np.load(folder_name+'//actual_dir.npy')			
								
		exec_time = np.load(folder_name+'//exec_time.npy') 
		lin_vdesEE_O = np.load(folder_name+'//lin_vdesEE_O.npy')
		ang_vdesEE_O = np.load(folder_name+'//ang_vdesEE_O.npy')
		
		theta = np.load(folder_name+'//theta.npy')
		lin_vEE_O_meas = np.load(folder_name+'//lin_vEE_O_meas.npy')
		ang_vEE_O_meas = np.load(folder_name+'//ang_vEE_O_meas.npy')
		manipulability_meas = np.load(folder_name+'//manipulability_meas.npy') 
		
		lin_vBase_O_meas = np.load(folder_name+'//lin_vBase_O_meas.npy') 
		ang_vBase_O_meas = np.load(folder_name+'//ang_vBase_O_meas.npy')
		
		q_dot_optimal = np.load(folder_name+'//q_dot_optimal.npy')
		relative_v = np.load(folder_name+'//relative_v.npy')
		
		f_wristframe = np.load(folder_name+'//f_wristframe.npy')
		t_wristframe = np.load(folder_name+'//t_wristframe.npy')
		actual_drawer_pose = np.load(folder_name+'//actual_drawer_pos.npy')
		
		#----- CALCULATE -----
		
		lin_vEE_O_sum = lin_vBase_O_meas + relative_v[:, :3]
		ang_vEE_O_sum = ang_vBase_O_meas + relative_v[:, 3:]
		
		list_of_model_fitting_metrics = []
		
		for i in range(estimated_dir.shape[0]):
			
			list_of_model_fitting_metrics.append(np.dot(np.squeeze(estimated_dir[i, :]), np.squeeze(actual_dir[i, :])))
					
		#--------------------
		
		N = len(exec_time)
		t = np.arange(1, N+1)
		
		#----- ARM JOINT POSITIONS -----
		
		fig1, (ax1_1, ax1_2, ax1_3) = plt.subplots(3,1,figsize=(10,10), constrained_layout=True)
		fig1.suptitle('Arm joint position')
		
		fig2, (ax2_1, ax2_2, ax2_3, ax2_4) = plt.subplots(4,1,figsize=(10,10), constrained_layout=True)
		fig2.suptitle('Arm joint position')
		
		ax_q = [ax1_1, ax1_2, ax1_3, ax2_1, ax2_2, ax2_3, ax2_4]		
		
		#----- ARM JOINT VELOCITIES -----
		
		fig3, (ax3_1, ax3_2, ax3_3) = plt.subplots(3,1,figsize=(10,10), constrained_layout=True)
		fig3.suptitle('Arm joint velocities')
		
		fig4, (ax4_1, ax4_2, ax4_3, ax4_4) = plt.subplots(4,1,figsize=(10,10), constrained_layout=True)
		fig4.suptitle('Arm joint velocities')
		
		ax_q_dot = [ax3_1, ax3_2, ax3_3, ax4_1, ax4_2, ax4_3, ax4_4]		

		#----- ARM JOINT TORQUES -----
		
		fig5, (ax5_1, ax5_2, ax5_3) = plt.subplots(3,1,figsize=(10,10), constrained_layout=True)
		fig5.suptitle('Joint torques')
		
		fig6, (ax6_1, ax6_2, ax6_3, ax6_4) = plt.subplots(4,1,figsize=(10,10), constrained_layout=True)
		fig6.suptitle('Joint torques')
		
		ax_tau = [ax5_1, ax5_2, ax5_3, ax6_1, ax6_2, ax6_3, ax6_4]
		
		#----- EE velocity -----
		
		fig7, (ax7_1, ax7_2, ax7_3) = plt.subplots(3,1,figsize=(10,10), constrained_layout=True)
		fig7.suptitle('EE linear velocity and its components in world frame')
		
		fig8, (ax8_1, ax8_2, ax8_3) = plt.subplots(3,1,figsize=(10,10), constrained_layout=True)
		fig8.suptitle('EE angular velocity and its components in world frame')		
		
		#----- Miscelaneous -----
		
		fig9, (ax9_1, ax9_2, ax9_3, ax9_4) = plt.subplots(4,1,figsize=(10,10), constrained_layout=True)
		
		#----- Wrist Forces and Torques -----
		
		fig10, ((ax10_1, ax10_2), (ax10_3, ax10_4), (ax10_5, ax10_6)) = plt.subplots(3,2,figsize=(15,10), constrained_layout=True)
		fig10.suptitle('Wrist forces')
		
		#----- Drawer position -----
		
		fig11, (ax11_1, ax11_2, ax11_3) = plt.subplots(3,1,figsize=(10,10), constrained_layout=True)
		fig11.suptitle('Drawer position')		
		
		#----- PLOT q, q_dot, tau -----
		
		for i in range(7):
		
			ax_q[i].axhline(y=sk_traj.q_max[i], color='r')
			ax_q[i].axhline(y=sk_traj.q_min[i], color='r')
			ax_q[i].plot(t, q[:,i], linestyle='-', color='k')
			
			ax_q[i].set_ylabel(r'$q_{'+str(i+1)+'}$')
			ax_q[i].grid('both', 'both')
			ax_q[i].set_xlim(t[0], t[-1])
			
			ax_q_dot[i].axhline(y=sk_traj.q_dot_max[i], color='r')
			ax_q_dot[i].axhline(y=sk_traj.q_dot_min[i], color='r')
			ax_q_dot[i].plot(t, q_dot[:,i], linestyle='-', color='k', label="Measured")
			ax_q_dot[i].plot(t, q_dot_optimal[:,i], color='g', label="Optimal")
			
			ax_q_dot[i].set_ylabel(r'$\dot{q}_{'+str(i+1)+'}$')
			ax_q_dot[i].grid('both', 'both')
			ax_q_dot[i].set_xlim(t[0], t[-1])
			ax_q_dot[i].legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                		mode="expand", borderaxespad=0, ncol=3, fontsize='small'
                	)
                					
			ax_tau[i].axhline(y=sk_traj.torque_max[i], color='r')
			ax_tau[i].axhline(y=-sk_traj.torque_max[i], color='r')
			ax_tau[i].plot(t, tau[:,i], linestyle='-', color='k')
			
			ax_tau[i].set_ylabel(r'$\tau_{'+str(i+1)+'}$')
			ax_tau[i].grid('both', 'both')
			ax_tau[i].set_xlim(t[0], t[-1])
										
			if i==2 or i==6:
				
				ax_q[i].set_xlabel('Number of steps')
				ax_q_dot[i].set_xlabel('Number of steps')
				ax_tau[i].set_xlabel('Number of steps')

		fig1.savefig(plots_folder_name + '//' + 'joint_positions_1_3.png' )
		fig2.savefig(plots_folder_name + '//' + 'joint_positions_4_7.png' )
		fig3.savefig(plots_folder_name + '//' + 'joint_velocities_1_3.png' )
		fig4.savefig(plots_folder_name + '//' + 'joint_velocities_4_7.png' )
		fig5.savefig(plots_folder_name + '//' + 'joint_torques_1_3.png' )
		fig6.savefig(plots_folder_name + '//' + 'joint_torques_4_7.png' )
				
		#----- PLOT EE velocity and its components -----	
			
		ax7_1.plot(t, lin_vEE_O_meas[:, 0], color='k', label=r"$v^{meas}_{x}$")
		ax7_1.plot(t, lin_vdesEE_O[:, 0], color='g', label=r"$v^{des}_{x}$")
		ax7_1.plot(t, lin_vEE_O_sum[:, 0], color='b', label=r"$v^{meas}_{base, x}+v^{meas}_{arm, x}$")
		ax7_1.grid('both', 'both')
		ax7_1.set_ylabel('x component')
		ax7_1.set_xlim(t[0], t[-1])
		ax7_1. legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3, fontsize='small')			
				
		ax7_2.plot(t, lin_vEE_O_meas[:, 1], color='k', label=r"$v^{meas}_{y}$")
		ax7_2.plot(t, lin_vdesEE_O[:, 1], color='g', label=r"$v^{des}_{y}$")
		ax7_2.plot(t, lin_vEE_O_sum[:, 1], color='b', label=r"$v^{meas}_{base, x}+v^{meas}_{arm, y}$")
		ax7_2.grid('both', 'both')
		ax7_2.set_ylabel('y component')
		ax7_2.set_xlim(t[0], t[-1])
		ax7_2. legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3, fontsize='small')		
		
		ax7_3.plot(t, lin_vEE_O_meas[:, 2], color='k', label=r"$v^{meas}_{z}$")
		ax7_3.plot(t, lin_vdesEE_O[:, 2], color='g', label=r"$v^{des}_{z}$")
		ax7_3.plot(t, lin_vEE_O_sum[:, 2], color='b', label=r"$v^{meas}_{base, z}+v^{meas}_{arm, z}$")
		ax7_3.grid('both', 'both')
		ax7_3.set_ylabel('z component')
		ax7_3.set_xlim(t[0], t[-1])
		ax7_3.set_xlabel('Number of steps')
		ax7_3. legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3, fontsize='small')
				
		ax8_1.plot(t, ang_vEE_O_meas[:, 0], color='k', label=r"$\omega^{meas}_{x}$")
		ax8_1.plot(t, ang_vdesEE_O[:, 0], color='g', label=r"$\omega^{des}_{x}$")
		#ax8_1.plot(t, ang_vEE_O_sum[:, 0], color='b', label=r"$\omega^{meas}_{base, x}+\omega^{meas}_{arm, x}$")			
		ax8_1.grid('both', 'both')
		ax8_1.set_ylabel('x component')
		ax8_1.set_xlim(t[0], t[-1])
		ax8_1. legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3, fontsize='small')									
		ax8_2.plot(t, ang_vEE_O_meas[:, 1], color='k', label=r"$\omega^{meas}_{y}$")
		ax8_2.plot(t, ang_vdesEE_O[:, 1], color='g', label=r"$\omega^{des}_{y}$")
		#ax8_2.plot(t, ang_vEE_O_sum[:, 1], color='b', label=r"$\omega^{meas}_{base, x}+\omega^{meas}_{arm, y}$")		
		ax8_2.grid('both', 'both')
		ax8_2.set_ylabel('y component')
		ax8_2.set_xlim(t[0], t[-1])	
		ax8_2. legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3, fontsize='small')	
				
		ax8_3.plot(t, ang_vEE_O_meas[:, 2], color='k', label=r"$\omega^{meas}_{z}$")
		ax8_3.plot(t, ang_vdesEE_O[:, 2], color='g', label=r"$\omega^{des}_{z}$")
		#ax8_3.plot(t, ang_vEE_O_sum[:, 2], color='b', label=r"$\omega^{meas}_{base, z}+\omega^{meas}_{arm, z}$")				
		ax8_3.grid('both', 'both')
		ax8_3.set_ylabel('z component')
		ax8_3.set_xlim(t[0], t[-1])
		ax8_3.set_xlabel('Number of steps')
		ax8_3. legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3, fontsize='small')

		fig7.savefig(plots_folder_name + '//' + 'EE_lin_vel.png' )
		fig8.savefig(plots_folder_name + '//' + 'EE_ang_vel.png' )
								
		#----- PLOT remaining features -----
		
		ax9_1.plot(t, list_of_model_fitting_metrics)
		ax9_1.grid('both', 'both')
		ax9_1.set_ylabel(r"$<n_{est}, n_{true}>$")
		ax9_1.set_xlim(t[0], t[-1])
		
		ax9_2.plot(t, theta)
		ax9_2.grid('both', 'both')
		ax9_2.set_ylabel(r"$\theta$")
		ax9_2.set_xlim(t[0], t[-1])
		
		ax9_3.plot(t, exec_time)
		ax9_3.grid('both', 'both')
		ax9_3.set_ylabel(r"$\Delta t_{plann}$")
		ax9_3.set_xlim(t[0], t[-1])
		
		ax9_4.plot(t, manipulability_meas)
		ax9_4.grid('both', 'both')
		ax9_4.set_ylabel(r"$det(J_{EE}J^{T}_{EE})$")
		ax9_4.set_xlim(t[0], t[-1])
		
		fig9.savefig(plots_folder_name + '//' + 'Metrics.png' )
		
		#----- Plot forces -----
		
		ax10_1.plot(t, f_wristframe[:, 0])
		ax10_1.grid('both', 'both')
		ax10_1.set_ylabel(r"$F_{x}$")
		ax10_1.set_xlim(t[0], t[-1])	
		
		ax10_3.plot(t, f_wristframe[:, 1])
		ax10_3.grid('both', 'both')
		ax10_3.set_ylabel(r"$F_{y}$")
		ax10_3.set_xlim(t[0], t[-1])				
		
		ax10_5.plot(t, f_wristframe[:, 2])
		ax10_5.grid('both', 'both')
		ax10_5.set_ylabel(r"$F_{z}$")
		ax10_5.set_xlim(t[0], t[-1])	

		ax10_2.plot(t, t_wristframe[:, 0])
		ax10_2.grid('both', 'both')
		ax10_2.set_ylabel(r"$T_{x}$")
		ax10_2.set_xlim(t[0], t[-1])	
		
		ax10_4.plot(t, t_wristframe[:, 1])
		ax10_4.grid('both', 'both')
		ax10_4.set_ylabel(r"$T_{y}$")
		ax10_4.set_xlim(t[0], t[-1])				
		
		ax10_6.plot(t, t_wristframe[:, 2])
		ax10_6.grid('both', 'both')
		ax10_6.set_ylabel(r"$T_{z}$")
		ax10_6.set_xlim(t[0], t[-1])
				
		fig10.savefig(plots_folder_name + '//' + 'ForcesAndTorques.png' )
		
		#----- Plot the actual position of the drawer -----
		
		ax11_1.plot(t, actual_drawer_pose[:, 0])
		ax11_1.grid('both', 'both')
		ax11_1.set_ylabel(r"$r_{x}$")
		ax11_1.set_xlim(t[0], t[-1])	
		
		ax11_2.plot(t, actual_drawer_pose[:, 1])
		ax11_2.grid('both', 'both')
		ax11_2.set_ylabel(r"$r_{y}$")
		ax11_2.set_xlim(t[0], t[-1])				
		
		ax11_3.plot(t, actual_drawer_pose[:, 2])
		ax11_3.grid('both', 'both')
		ax11_3.set_ylabel(r"$r_{z}$")
		ax11_3.set_xlim(t[0], t[-1])	
		
		fig11.savefig(plots_folder_name + '//' + 'Drawer_pos.png' )	
		
		#-------	
		
		plt.close('all')

		print(5*'*' + ' Plots prepared '+ 5*'*')
		
	
