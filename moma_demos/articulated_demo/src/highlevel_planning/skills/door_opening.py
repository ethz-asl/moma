import pybullet as p
import numpy as np
from numpy import linalg as LA
import os
import time

from scipy.spatial.transform import Rotation as R

from highlevel_planning.tools.door_opening_util import *
from highlevel_planning.tools.plot_util import *

from highlevel_planning.tools.util import IKError
from collections import deque

import math
import matplotlib.pyplot as plt

EPS = 1e-6
DEBUG = True
BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))



class SkillTrajectoryPlanning:

    def __init__(self, scene, robot, cfg, list_of_controllers, time_step, initLength, vInit, vRegular):
    
        self.scene = scene
        self.robot = robot
        self.cfg = cfg
        
        self.dt = time_step
        
        self.initLength = initLength
        self.vInit = vInit
        self.vRegular = vRegular
        
        self.listOfControllers = list_of_controllers
        
        #----- Log data for plotting -----
        
        self.log_q = None
        self.log_q_dot = None
        self.log_tau = None
        
        self.log_estimated_dir = None
        self.log_actual_dir = None
        
        self.log_lin_vdesEE_O = None
        self.log_ang_vdesEE_O = None
        self.log_lin_vEE_O_meas = None
        self.log_ang_vEE_O_meas = None
        
        self.log_lin_vBase_O_meas = None
        self.log_ang_vBase_O_meas = None
        
        self.log_f_wristframe = None
        self.log_t_wristframe = None
        
        self.log_exec_time = None
        self.log_theta = None
        self.log_manipulability_meas = None
        
        self.log_q_dot_optimal = None
        self.log_actual_drawer_pos = None
        
        #----- -----
        
        #self.PrintRobotJointInfo()
        
#-------
    def reset(self):
    
        self.log_q = []
        self.log_q_dot = []
        self.log_tau = []
        
        self.log_estimated_dir = []
        self.log_actual_dir = []
        
        self.log_lin_vdesEE_O = []
        self.log_ang_vdesEE_O = []
        self.log_lin_vEE_O_meas = []
        self.log_ang_vEE_O_meas = []
        
        self.log_lin_vBase_O_meas = []
        self.log_ang_vBase_O_meas = []
        
        self.log_f_wristframe = []
        self.log_t_wristframe = []
        
        self.log_exec_time = []
        self.log_theta = []
        self.log_manipulability_meas = []
        
        self.log_q_dot_optimal = []
        self.log_actual_drawer_pos = []        
        
#-------
    def PrintRobotJointInfo(self):
    
        num_joints = p.getNumJoints(self.robot.model.uid)
        
        print("***** Robot joint information *****")
        
        for i in range(num_joints):
        
            info = p.getJointInfo(self.robot.model.uid, i)
            joint_name = info[1] if type(info[1]) is str else info[1].decode("utf-8")
            print("i: ", i, " | name: ", joint_name, " | type: ", info[2], " | parentIndex: ", info[16], " | linkName: ", info[12], " | min and max: ", 
                info[8], info[9], info[10], info[11]
            )
#-------
    def draw_arrow(self, vec_wristframe, color, arrow_id=None, length=0.2):
    
        if DEBUG:
        
            link_state = p.getLinkState(self.robot.model.uid, self.robot.link_name_to_index["panda_default_EE"])
            pos = np.array(link_state[4])
            orient = R.from_quat(link_state[5])
            vec_worldframe = orient.apply(np.squeeze(vec_wristframe))
            if arrow_id is None:
                return self.robot._world.draw_arrow(pos, vec_worldframe, color, length=length)
            else:
                return self.robot._world.draw_arrow(pos, vec_worldframe, color, length=length, replace_id=arrow_id)  
            
#-------
    def FreezeRobotConfiguration(self):
        
        joint_positions, joint_velocities, joint_torques = GetJointStates(self.robot.model.uid)        
        base_pos, base_ori = p.getBasePositionAndOrientation(self.robot.model.uid)        
        lin_base_vel, ang_base_vel = p.getBaseVelocity(self.robot.model.uid)
        
        return joint_positions, joint_velocities, base_pos, base_ori, lin_base_vel, ang_base_vel
        
#-------
    def ApplyRobotConfiguration(self, joint_positions, joint_velocities, base_pos, base_ori, lin_base_vel, ang_base_vel):

        p.resetBasePositionAndOrientation(self.robot.model.uid, base_pos, base_ori)
        
        self.robot.update_velocity(lin_base_vel, ang_base_vel[2])                             
        self.robot.velocity_setter()
                
        for i in range(len(joint_positions)):
            p.resetJointState(self.robot.model.uid, i, joint_positions[i], joint_velocities[i])
            
        
                    
#-------
    def ObjectConfiguration(self, mode, target_name, joint_positions=None, joint_velocities=None):
        
        obj_info = self.scene.objects[target_name]
        target_id = obj_info.model.uid
        
        num_joints = p.getNumJoints(target_id)
        
        if mode == 'memorize':
        
            joint_states = p.getJointStates(target_id, range(num_joints))
            joint_positions = [state[0] for state in joint_states]
            joint_velocities = [state[1] for state in joint_states]
            
            return joint_positions, joint_velocities
            
        else:
            
            for i in range(len(joint_positions)):
                p.resetJointState(target_id, i, joint_positions[i], joint_velocities[i]) 
            
#-------
    def GetGraspedObjActualInfo(self, target_name, link_idx, grasp_id):
    
        obj_info = self.scene.objects[target_name]
        target_id = obj_info.model.uid
        
        if len(obj_info.grasp_links) == 0:            
            raise SkillExecutionError("No grasps defined for this object")
        
        link_id = obj_info.grasp_links[link_idx]        
        num_grasps = len(obj_info.grasp_pos[link_id])
        
        if num_grasps == 0:
            raise SkillExecutionError("No grasps defined for this link")
        if grasp_id >= num_grasps:
            raise SkillExecutionError("Invalid grasp ID")        
        
        if link_id == -1:            
            temp = p.getBasePositionAndOrientation(target_id)
            pos_obj  = np.array(temp[0]).reshape((-1, 1))
            ori_obj = np.array(temp[1])
        else:
            temp = p.getLinkState(target_id, link_id, computeForwardKinematics = True)
            pos_obj = np.array(temp[4]).reshape((-1, 1))
            ori_obj = np.array(temp[5])    
        
        r_O_obj = np.squeeze(pos_obj)
        quat = np.squeeze(ori_obj)
        C_O_obj = R.from_quat(ori_obj)
        
        nobj_O = np.squeeze(np.array(C_O_obj.as_matrix())[:, 0])    
        
        return r_O_obj, quat, C_O_obj, nobj_O    
        
#-------
    def GetMeasurements(self):
    
        link_pos_and_vel = p.getLinkStates(self.robot.model.uid, linkIndices=[self.robot.arm_base_link_idx, self.robot.link_name_to_index["panda_default_EE"]], 
            computeLinkVelocity = 1, computeForwardKinematics = True
        )
        
        f_wristframe, t_wristframe = self.robot.get_wrist_force_torque()
        
        C_O_b = R.from_quat(link_pos_and_vel[0][5])
        r_O_b = link_pos_and_vel[0][4]
        
        C_O_ee = R.from_quat(link_pos_and_vel[1][5])
        r_O_ee = link_pos_and_vel[1][4]
        
        lin_vEE_O = link_pos_and_vel[1][6]
        ang_vEE_O = link_pos_and_vel[1][7]
        
        lin_vBase_O, ang_vBase_O = p.getBaseVelocity(self.robot.model.uid)
        
        q, q_dot, mtorq = GetMotorJointStates(self.robot.model.uid)
        
        zero_vec = [0.0]*len(q)
        
        b = np.array(p.calculateInverseDynamics(self.robot.model.uid, list(q), list(q_dot), zero_vec))
        M = np.array(p.calculateMassMatrix(self.robot.model.uid, list(q)))        
        
        lin, ang = p.calculateJacobian(self.robot.model.uid, self.robot.arm_ee_link_idx, [0.0, 0.0, 0.0], list(q), zero_vec, zero_vec)
        lin = np.array(lin)
        ang = np.array(ang)
            
        J_b_ee = np.concatenate((lin, ang), axis=0)
        
        return M, b, J_b_ee, q, q_dot, C_O_b, r_O_b, C_O_ee, r_O_ee, mtorq, lin_vEE_O, ang_vEE_O, lin_vBase_O, ang_vBase_O, f_wristframe, t_wristframe 

#-------
    def VelocityProfile1(self, t, vInit, vFinal, tConv):
        
        if t<=tConv:
            
            return vInit
            
        else:
        
            return vFinal
            
#-------
    def VelocityProfile2(self, t, vInit, vFinal, alphaInit, alphaFinal, t0, tConv):
    
        if t<t0:
        
            v = vInit * 2.0/math.pi*np.arctan(alphaInit*t)
        else:
        
            a1 = (vFinal - vInit*2.0/math.pi*np.arctan(alphaInit*t0))/(1.0 - 2.0/math.pi*np.arctan(alphaFinal*(t0 - tConv)))    
            a2 = vFinal - a1
        
            v = a1 * 2.0/math.pi*np.arctan(alphaFinal*(t - tConv)) + a2
        
        return v    
        
#-------
    def LogDataForPlotting(self, sk_dir, interval, q, q_dot, nObj_O, C_O_ee, veldesEE_ee, J_b_ee, mtorq, vLinEE_O, vAngEE_O, vLinBase_O, vAngBase_O, f_wristframe, t_wristframe, q_dot_optimal,             r_O_obj):

        #----- Calculate Data for logging -----
                
        e_ee = sk_dir.GetCurrEstimate()
        e_O = C_O_ee.apply(e_ee)
        #print(e_O)    
        vLinDesEE_O = C_O_ee.apply(veldesEE_ee[:3])
        vAngDesEE_O = C_O_ee.apply(veldesEE_ee[3:])
                
        theta = LA.norm(np.array(veldesEE_ee[3:]))
        manipulabilityMeasure = LA.det(np.matmul(J_b_ee, np.transpose(J_b_ee)))**0.5
                
        #----- Log Data -----
                                
        self.log_q.append(q)
        self.log_q_dot.append(q_dot)
        self.log_tau.append(mtorq)
        
        self.log_estimated_dir.append(e_O)
        self.log_actual_dir.append(nObj_O)
        
        self.log_lin_vdesEE_O.append(vLinDesEE_O)
        self.log_ang_vdesEE_O.append(vAngDesEE_O)
        self.log_lin_vEE_O_meas.append(vLinEE_O)
        self.log_ang_vEE_O_meas.append(vAngEE_O)
        
        self.log_lin_vBase_O_meas.append(vLinBase_O)
        self.log_ang_vBase_O_meas.append(vAngBase_O)
        
        self.log_f_wristframe.append(f_wristframe)
        self.log_t_wristframe.append(t_wristframe)
        
        self.log_exec_time.append(interval)
        self.log_theta.append(theta)
        self.log_manipulability_meas.append(manipulabilityMeasure)
        
        self.log_q_dot_optimal.append(q_dot_optimal)
        self.log_actual_drawer_pos.append(r_O_obj)    
        
#-------
    def SaveRun(self, curr_folder, mode_name):

        folder_name = curr_folder + '/' + mode_name
    
        if not os.path.isdir(folder_name):
            os.makedirs(folder_name)
                  
        np.save(folder_name+'/q.npy', np.array(self.log_q))
        np.save(folder_name+'/q_dot.npy', np.array(self.log_q_dot))
        np.save(folder_name+'/tau.npy', np.array(self.log_tau))
        
        np.save(folder_name+'/estimated_dir.npy', np.array(self.log_estimated_dir))        
        np.save(folder_name+'/actual_dir.npy',  np.array(self.log_actual_dir))            
                                    
        np.save(folder_name+'/exec_time.npy', np.array(self.log_exec_time)) 
        np.save(folder_name+'/lin_vdesEE_O.npy', np.array(self.log_lin_vdesEE_O))
        np.save(folder_name+'/ang_vdesEE_O.npy', np.array(self.log_ang_vdesEE_O))
        
        np.save(folder_name+'/theta.npy', np.array(self.log_theta))
        np.save(folder_name+'/lin_vEE_O_meas.npy', np.array(self.log_lin_vEE_O_meas))
        np.save(folder_name+'/ang_vEE_O_meas.npy', np.array(self.log_ang_vEE_O_meas))
        np.save(folder_name+'/manipulability_meas.npy', np.array(self.log_manipulability_meas)) 
        
        np.save(folder_name+'/lin_vBase_O_meas.npy', np.array(self.log_lin_vBase_O_meas)) 
        np.save(folder_name+'/ang_vBase_O_meas.npy', np.array(self.log_ang_vBase_O_meas))
        
        np.save(folder_name+'/q_dot_optimal.npy', np.array(self.log_q_dot_optimal))
    
        np.save(folder_name+'/f_wristframe.npy', np.array(self.log_f_wristframe))
        np.save(folder_name+'/t_wristframe.npy', np.array(self.log_t_wristframe))
        np.save(folder_name+'/actual_drawer_pos.npy', np.array(self.log_actual_drawer_pos))
    
        print("***** Run saved! *****")
        
#-------
    def PlotRuns(self, curr_folder):

        print("Preparing plots...")

        for c in self.listOfControllers:
        
            mode_name = c.mode_name
    
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
        
            f_wristframe = np.load(folder_name+'//f_wristframe.npy')
            t_wristframe = np.load(folder_name+'//t_wristframe.npy')
            actual_drawer_pose = np.load(folder_name+'//actual_drawer_pos.npy')
        
            #----- CALCULATE -----
        
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
        
                ax_q[i].axhline(y=c.q_max[i], color='r')
                ax_q[i].axhline(y=c.q_min[i], color='r')
                ax_q[i].plot(t, q[:,i], linestyle='-', color='k')
            
                ax_q[i].set_ylabel(r'$q_{'+str(i+1)+'}$')
                ax_q[i].grid('both', 'both')
                ax_q[i].set_xlim(t[0], t[-1])
            
                ax_q_dot[i].axhline(y=c.q_dot_max[i], color='r')
                ax_q_dot[i].axhline(y=c.q_dot_min[i], color='r')
                ax_q_dot[i].plot(t, q_dot[:,i], linestyle='-', color='k', label="Measured")
                ax_q_dot[i].plot(t, q_dot_optimal[:,i], color='g', label="Optimal")
            
                ax_q_dot[i].set_ylabel(r'$\dot{q}_{'+str(i+1)+'}$')
                ax_q_dot[i].grid('both', 'both')
                ax_q_dot[i].set_xlim(t[0], t[-1])
                ax_q_dot[i].legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                            mode="expand", borderaxespad=0, ncol=3, fontsize='small'
                        )
                                    
                ax_tau[i].axhline(y=c.torque_max[i], color='r')
                ax_tau[i].axhline(y=-c.torque_max[i], color='r')
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
        
            ax7_1.grid('both', 'both')
            ax7_1.set_ylabel('x component')
            ax7_1.set_xlim(t[0], t[-1])
            ax7_1. legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                    mode="expand", borderaxespad=0, ncol=3, fontsize='small')            
                
            ax7_2.plot(t, lin_vEE_O_meas[:, 1], color='k', label=r"$v^{meas}_{y}$")
            ax7_2.plot(t, lin_vdesEE_O[:, 1], color='g', label=r"$v^{des}_{y}$")
        
            ax7_2.grid('both', 'both')
            ax7_2.set_ylabel('y component')
            ax7_2.set_xlim(t[0], t[-1])
            ax7_2. legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                    mode="expand", borderaxespad=0, ncol=3, fontsize='small')        
        
            ax7_3.plot(t, lin_vEE_O_meas[:, 2], color='k', label=r"$v^{meas}_{z}$")
            ax7_3.plot(t, lin_vdesEE_O[:, 2], color='g', label=r"$v^{des}_{z}$")
            
            ax7_3.grid('both', 'both')
            ax7_3.set_ylabel('z component')
            ax7_3.set_xlim(t[0], t[-1])
            ax7_3.set_xlabel('Number of steps')
            ax7_3. legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                    mode="expand", borderaxespad=0, ncol=3, fontsize='small')
                
            ax8_1.plot(t, ang_vEE_O_meas[:, 0], color='k', label=r"$\omega^{meas}_{x}$")
            ax8_1.plot(t, ang_vdesEE_O[:, 0], color='g', label=r"$\omega^{des}_{x}$")
                    
            ax8_1.grid('both', 'both')
            ax8_1.set_ylabel('x component')
            ax8_1.set_xlim(t[0], t[-1])
            ax8_1. legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                    mode="expand", borderaxespad=0, ncol=3, fontsize='small')
                                                        
            ax8_2.plot(t, ang_vEE_O_meas[:, 1], color='k', label=r"$\omega^{meas}_{y}$")
            ax8_2.plot(t, ang_vdesEE_O[:, 1], color='g', label=r"$\omega^{des}_{y}$")

            ax8_2.grid('both', 'both')
            ax8_2.set_ylabel('y component')
            ax8_2.set_xlim(t[0], t[-1])    
            ax8_2. legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                    mode="expand", borderaxespad=0, ncol=3, fontsize='small')    
                
            ax8_3.plot(t, ang_vEE_O_meas[:, 2], color='k', label=r"$\omega^{meas}_{z}$")
            ax8_3.plot(t, ang_vdesEE_O[:, 2], color='g', label=r"$\omega^{des}_{z}$")

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
            
#-------
    def Run(self, num_steps, sk_grasp, sk_dir, sk_nav, target_name, link_idx, grasp_id, global_folder, postfix=None):
        
        currFolder = prepare_dir(global_folder, postfix)
        
        initJointPositions = None
        initJointVelocities = None
        initBasePos = None
        initBaseOri = None
        initLinVelBase = None
        initAngVelBase = None
                
        initObjJointPositions, initObjJointVelocities = self.ObjectConfiguration('memorize', target_name)
        
        nControllers = len(self.listOfControllers)
        approached = False
        
        for i in range(nControllers):
            
            try:
                print(10*'-'+' Controller '+str(i)+' '+10*'-')
            
                controller = self.listOfControllers[i]
                self.reset()
                sk_dir.reset()
        
                if i == 0:
                    failed = 0
                    while(1):
                        self.robot.reset()
                        self.robot.to_start()                
                        sk_nav.move_to_object(target_name, 1.0)        
                        approached = True
                        succeded = sk_dir.SetupEstimationProcedure(approached, target_name, link_idx, grasp_id, sk_grasp, sk_nav, 1.0)
                        
                        if succeded:
                            initJointPositions, initJointVelocities, initBasePos, initBaseOri, initLinVelBase, initAngVelBase = self.FreezeRobotConfiguration()
                            break
                        failed +=0
                        if failed>10:
                            break
                
                else:
                
                    self.robot.reset()
                    self.robot.to_start()
                    self.ObjectConfiguration('reset', target_name, initObjJointPositions, initObjJointVelocities)
                    self.ApplyRobotConfiguration(initJointPositions, initJointVelocities, initBasePos, initBaseOri, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                            
                controller.ResetDampings()
            
                #arrow_1_id = self.draw_arrow(sk_dir.GetCurrEstimate(), "green")
                #arrow_2_id = self.draw_arrow([0.1, 0.0, 0.0], "red")
                
                totalPlanningTime = 0.0
                
                for it in range(num_steps):
            
                    print(5*'-'+' Iteration '+str(it)+' '+5*'-')
                
                    r_O_obj,_,_,nObj_O = self.GetGraspedObjActualInfo(target_name, link_idx, grasp_id)
                
                    startTime = time.time()
                
                    M, b, J_b_ee, q, q_dot, C_O_b, r_O_b, C_O_ee, r_O_ee, mtorq, vLinEE_O, vAngEE_O, vLinBase_O, vAngBase_O, f_wristframe, t_wristframe = self.GetMeasurements()
                
                    #self.draw_arrow(f_wristframe, "red", arrow_id=None, length=LA.norm(f_wristframe)/22.0)
                
                    sk_dir.UpdateEstimate(f_wristframe, 0.1, C_O_ee, False)
                
                    #self.draw_arrow(sk_dir.GetCurrEstimate(), "green")
                
                    velProfile = self.VelocityProfile2(it, self.vInit, self.vRegular, 0.5, 0.5, np.floor(self.initLength/3), self.initLength)
                
                    veldesEE_ee = sk_dir.GetPlannedVelocities(v=velProfile, calcAng=False, kAng=1)        
                
                    infoTuple = (M, b, J_b_ee, q, q_dot, C_O_b, C_O_ee, r_O_ee, velProfile)
                                
                    controller.PerformOneStep(veldesEE_ee, infoTuple)
                
                    stopTime = time.time()
                
                    interval = stopTime - startTime
                    totalPlanningTime += interval
                    
                    sk_dir.UpdateBuffers(f_wristframe, r_O_ee)
                    
                    print("Drawer position: ", r_O_obj)
                    #----- Log Data -----
                
                    self.LogDataForPlotting(sk_dir, interval, q, q_dot, nObj_O, C_O_ee, veldesEE_ee, J_b_ee, mtorq, vLinEE_O, vAngEE_O, vLinBase_O, vAngBase_O, f_wristframe, 
                        t_wristframe, controller.GetCurrOptSol(), r_O_obj
                    )
                
                print("Total planning time for "+str(it)+" iterations: "+str(totalPlanningTime))
                self.SaveRun(currFolder, controller.mode_name)
                
            except Exception as e:
                print(50*'=')
                print(e)
                self.SaveRun(currFolder, controller.mode_name)
        
        #----- Reset Everything for the next run -----
            
        self.PlotRuns(currFolder)
        self.ObjectConfiguration('reset', target_name, initObjJointPositions, initObjJointVelocities)
        self.robot._world.step_seconds(2)
                                        
#-------                

        
        
        
         
        
        
        
        
        
        
        
                         
                         
                
        
