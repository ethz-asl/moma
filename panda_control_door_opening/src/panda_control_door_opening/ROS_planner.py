#!/usr/bin/env python

import rospy
import actionlib

#----- ROS Msgs ------

from nav_msgs.msg import Odometry                                                # used to recieve info from the base
from geometry_msgs.msg import Twist                                              # used to set comand for the base
from std_msgs.msg import Float64                                                 # used to set command to each of the joints 
from sensor_msgs.msg import JointState                                           # used to recieve joint state
from geometry_msgs.msg import WrenchStamped
from tf2_msgs.msg import TFMessage
from franka_msgs.msg import FrankaState
from control_msgs.msg import (
    GripperCommand,
    GripperCommandAction,
    GripperCommandGoal,
)

from panda_control_door_opening.msg import *

from panda_control_door_opening.srv import *
 
#----- Other -----

from scipy.spatial.transform import Rotation as R

import pybullet as p
import pybullet_data

import numpy as np
import os
import math

class RobotPlanner:

    def __init__(self, controller, direction_estimatior):
        
        self.controller = controller
        self.direction_estimator = direction_estimatior
        
        self.processing = False
        
        #----- Msg buffer -----
        
        self.baseState_msg = None                                                # Receives the Position and Velocity of the base
        
        #----- Check if required topics started publishing -----
        
        self.base_state_topic_initiated = False
        
        self.start_optimization = False
        
        #----- Robot state -----
        
        self.M = None
        self.b = None
        self.q = None
        self.q_dot = None
        self.tau = None
        self.J_b_ee = None
        self.T_b_ee = None
        self.force = None
        self.torque = None
        
        self.linVelBase = None                                                   # should be a list of 3 elements
        self.angVelBase = None                                                   # should be a list of 3 elements
        self.T_O_b = None
        
        #----- Subsrcribers and services -----
        
        self.get_panda_model_state_srv = rospy.ServiceProxy('/get_panda_state_srv', PandaStateSrv)
        self.set_frames_srv = rospy.ServiceProxy('/panda_init_srv', PandaInitSrv)
        
        self.subscriber_base_state = rospy.Subscriber('/odom', Odometry , self.baseState_cb)

        #----- Publishers -----
        
        self.publisher_joints = rospy.Publisher('/arm_joint_command', command_msg, latch=True, queue_size=10)
        self.publisher_base_velocity = rospy.Publisher('/cmd_vel', Twist,  latch=True, queue_size=10)
        
        #----- Gripper client -----
        
        self.gripper_client = None

    def baseState_cb(self, msg):

        if not self.processing:
            
            self.baseState_msg = msg
            if not self.base_state_topic_initiated:
                self.base_state_topic_initiated = True

    def publishArmAndBaseVelocityControl(self, q_dot_des, linVelBase, angVelBase):

        joints_des = command_msg()
        
        for i in range(7):
            joints_des.dq_arm[i] = q_dot_des[i]
        
        base_des = Twist()
        base_des.linear.x = linVelBase[0]
        base_des.linear.y = linVelBase[1]
        base_des.linear.z = 0.0
        
        base_des.angular.x = 0.0
        base_des.angular.y = 0.0
        base_des.angular.z = angVelBase
        
        self.publisher_base_velocity.publish(base_des)
        self.publisher_joints.publish(joints_des)
        
    def set_frames(self, NE_T_EE, EE_T_K):
        
        req = PandaInitSrv()
        
        for i in range(16):
            
            req.NE_T_EE[i] = NE_T_EE[i]
            req.EE_T_K[i] = EE_T_K[i]
        
        res = self.set_frames_srv(req)
        
        return res.success
        
    def VelocityProfile(self, t, vInit, vFinal, alphaInit, alphaFinal, t0, tConv):
    
        if t<t0:
        
            v = vInit * 2.0/math.pi*np.arctan(alphaInit*t)
        
        else:
        
            a1 = (vFinal - vInit*2.0/math.pi*np.arctan(alphaInit*t0))/(1.0 - 2.0/math.pi*np.arctan(alphaFinal*(t0 - tConv)))    
            a2 = vFinal - a1
        
            v = a1 * 2.0/math.pi*np.arctan(alphaFinal*(t - tConv)) + a2
        
        return v
    
    def InitURDF(self, time_step, urdf_filename, robot_base, robot_orientation):
        
        id_simulator = p.connect(p.DIRECT)
        p.setTimeStep(time_step)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -9.81, id_simulator)

        id_robot = p.loadURDF(urdf_filename, robot_base, robot_orientation, useFixedBase=True, physicsClientId=id_simulator)
        
        #----- Take info from URDF -----
    
        joint_idx_arm = [1, 2, 3, 4, 5, 6, 7]
        joint_idx_fingers = [0, 0]
        joint_idx_hand = 0
        arm_base_link_idx = -100
        arm_ee_link_idx = -100
        link_name_to_index = None
    
        link_name_to_index = {p.getBodyInfo(id_robot)[0]: -1}
        num_joints = p.getNumJoints(id_robot)
            
        for i in range(num_joints):
            
            info = p.getJointInfo(id_robot, i)
            joint_name = info[1] if type(info[1]) is str else info[1].decode("utf-8")
            
            if "panda_joint" in joint_name and len(joint_name) == 12:
                
                joint_num = int(joint_name.split("panda_joint")[1])
                if joint_num < 8:
                    
                    joint_idx_arm[joint_num - 1] = i
                    
                if joint_num == 1:
                    
                        arm_base_link_idx = info[16]
                        
            elif "panda_hand_joint" in joint_name:
                    
                arm_ee_link_idx = info[16]
                joint_idx_hand = i
                    
            elif "panda_finger_joint" in joint_name:
                    
                joint_num = int(joint_name.split("panda_finger_joint")[1])
                joint_idx_fingers[joint_num - 1] = i
    
            _name = info[12] if type(info[12]) is str else info[12].decode("utf-8")
            link_name_to_index[_name] = i
                
        return id_simulator, id_robot, joint_idx_arm, joint_idx_fingers, joint_idx_hand, arm_base_link_idx, arm_ee_link_idx, link_name_to_index
        
    
    def CalculateVars_usingSimulator(self, arm_state_msg, base_state_msg, force_msg, ee_state_msg, joint_idx_arm, model, arm_ee_link_idx, arm_base_link_idx, link_name_to_index):
        
        #----- INFO FROM JOINT STATE -----
        
        id_start =3
        
        q = []
        q_dot = []
        tau = []
        
        for i in range(id_start, id_start+7):
             
            q.append(arm_state_msg.position[i])
            q_dot.append(arm_state_msg.velocity[i]) 
            tau.append(arm_state_msg.effort[i])

        self.q = np.array(q)
        self.q_dot = np.array(q_dot)
        self.tau = np.array(tau)
        
        #----- Set joints in PyBuller simulation -----
        
        for i in range(len(joint_idx_arm)):
            p.resetJointState(model, joint_idx_arm[i], q[i], q_dot[i])            # Set the robot joints in simulator to appropriate values
                
        #----- GET INFO FROM SIMULATION -----
        
        zero_vec =[0.0] * 9
        lin, ang = p.calculateJacobian(model, arm_ee_link_idx, [0.0, 0.0, 0.0], list(q) + [0.0, 0.0], zero_vec, zero_vec)
        lin = np.array(lin)
        ang = np.array(ang)
        
        self.J_b_ee = np.concatenate((lin, ang), axis=0)
        
        self.M = np.array(p.calculateMassMatrix(model, list(q)+ [0.0, 0.0]))
        self.b = np.array(p.calculateInverseDynamics(model, list(q)+ [0.0, 0.0], list(q_dot)+ [0.0, 0.0], zero_vec))

        link_pos_and_vel = p.getLinkStates(model, linkIndices=[arm_base_link_idx, link_name_to_index["panda_default_EE"]], 
            computeLinkVelocity = 1, computeForwardKinematics = True
        )  
        
        #----- Values in the simulator -----
        
        C_O_b = R.from_quat(link_pos_and_vel[0][5])
        r_O_b = link_pos_and_vel[0][4]
        C_O_ee = R.from_quat(link_pos_and_vel[1][5])
        r_O_ee = link_pos_and_vel[1][4]
        
        C_b_ee = C_O_b.inv() * C_O_ee
        C_b_ee_mat = C_b_ee.as_dcm()
        
        r_delta = list(np.array(r_O_ee) - np.array(r_O_b))
        r_b_ee = C_O_b.inv().apply(r_delta)
        
        self.T_b_ee = np.array([[C_b_ee_mat[0, 0], C_b_ee_mat[0, 1], C_b_ee_mat[0, 2], r_b_ee[0]],
                                [C_b_ee_mat[1, 0], C_b_ee_mat[1, 1], C_b_ee_mat[1, 2], r_b_ee[1]],
                                [C_b_ee_mat[2, 0], C_b_ee_mat[2, 1], C_b_ee_mat[2, 2], r_b_ee[2]],
                                [0.0             , 0.0             , 0.0             , 1.0      ]])
        
        #----- INFO FROM FORCE MSG -----
        
        self.force = np.array([force_msg.wrench.force.x, force_msg.wrench.force.y, force_msg.wrench.force.z]) 
        #print("FORCE: ", self.force)    
        #----- INFO FROM BASE ODOM -----
            
        self.linVelBase = [base_state_msg.twist.twist.linear.x, base_state_msg.twist.twist.linear.y, 0.0]
        self.angVelBase = [0.0, 0.0, base_state_msg.twist.twist.angular.z]
        
        #print("linVelBase: ", self.linVelBase)
        #print("angVelBase: ", self.angVelBase)
        #print("orient: ", base_state_msg.pose.pose.orientation)
        
        C_O_b = R.from_quat([base_state_msg.pose.pose.orientation.x, base_state_msg.pose.pose.orientation.y, base_state_msg.pose.pose.orientation.z, base_state_msg.pose.pose.orientation.w])
        C_O_b_mat = C_O_b.as_dcm()
        
        self.T_O_b = np.array([[C_O_b_mat[0, 0], C_O_b_mat[0, 1], C_O_b_mat[0, 2], base_state_msg.pose.pose.position.x],
                               [C_O_b_mat[1, 0], C_O_b_mat[1, 1], C_O_b_mat[1, 2], base_state_msg.pose.pose.position.y],
                               [C_O_b_mat[2, 0], C_O_b_mat[2, 1], C_O_b_mat[2, 2], base_state_msg.pose.pose.position.z],
                               [0.0            , 0.0            , 0.0            , 1.0                                ]])

    def CalculateVars_usingPanda(self, panda_model, base_state_msg):
        
        #----- INFO FROM JOINT STATE -----
        
        q = []
        q_dot = []
        tau = []
        
        for i in range(7):
             
            q.append(panda_model.q_d[i])
            q_dot.append(panda_model.dq_d[i]) 
            tau.append(panda_model.tau[i])

        self.q = np.array(q)
        self.q_dot = np.array(q_dot)
        self.tau = np.array(tau)
        
        self.T_b_ee = np.array(panda_model.O_T_EE)
        self.T_b_ee = np.transpose(self.T_b_ee.reshape(4, 4))
        
        self.T_ee_k = np.array(panda_model.EE_T_K)
        self.T_ee_k = np.transpose(self.T_ee_k.reshape(4, 4))
        
        self.T_b_ee = np.matmul(self.T_b_ee, self.T_ee_k)                        # Stiffness frame is the actualy the EE frame we used in the simulation
                
        #----- GET INFO FROM MODEL STATE -----
        
        self.J_b_ee = np.array(panda_model.jacobian)                     # It is saved as column major but python does everyhing row major
        self.J_b_ee = np.transpose(self.J_b_ee.reshape(7, 6))
        
        self.M = np.array(panda_model.mass_matrix)
        self.M = np.transpose(self.M.reshape(7,7))
        
        self.b = np.array(panda_model.coriolis)       
        self.g = np.array(panda_model.gravity)
        
        self.b += self.g
        
        #----- GET FORCE INFO -----
        
        ext_wrench = np.array(panda_model.K_F_ext_hat_K)
        self.force = ext_wrench[:3] 
        
        #print("FORCE: ", self.force)
        
        #----- INFO FROM BASE ODOM -----
            
        self.linVelBase = [base_state_msg.twist.twist.linear.x, base_state_msg.twist.twist.linear.y, 0.0]
        self.angVelBase = [0.0, 0.0, base_state_msg.twist.twist.angular.z]
        
        #print("linVelBase: ", self.linVelBase)
        #print("angVelBase: ", self.angVelBase)
        #print("orient: ", base_state_msg.pose.pose.orientation)
        
        C_O_b = R.from_quat([base_state_msg.pose.pose.orientation.x, base_state_msg.pose.pose.orientation.y, base_state_msg.pose.pose.orientation.z, base_state_msg.pose.pose.orientation.w])
        C_O_b_mat = C_O_b.as_dcm()
        
        self.T_O_b = np.array([[C_O_b_mat[0, 0], C_O_b_mat[0, 1], C_O_b_mat[0, 2], base_state_msg.pose.pose.position.x],
                               [C_O_b_mat[1, 0], C_O_b_mat[1, 1], C_O_b_mat[1, 2], base_state_msg.pose.pose.position.y],
                               [C_O_b_mat[2, 0], C_O_b_mat[2, 1], C_O_b_mat[2, 2], base_state_msg.pose.pose.position.z],
                               [0.0            , 0.0            , 0.0            , 1.0                                ]])
    
    def init_gripper(self):

        name = "gripper_cmd" if rospy.get_param("use_sim_time") else "gripper_action"
        name = "franka_gripper/" + name
        self.gripper_client = actionlib.SimpleActionClient(name, GripperCommandAction)
        self.gripper_client.wait_for_server()
        
    def move_gripper(self, width, max_effort=10):

        command = GripperCommand(width, max_effort)
        goal = GripperCommandGoal(command)
        self.gripper_client.send_goal(goal)

        return self.gripper_client.wait_for_result(timeout=rospy.Duration(1.0))
                   
    def close_gripper(self):
        
        pass
        self.move_gripper(0.0)        
        
    def open_gripper(self):
        
        self.move_gripper(0.04)
    
    def run_once(self, t, vInit, vFinal, alphaInit, alphaFinal, t0, tConv, alpha=0.1, smooth=False, mixCoeff=0.1):
        
        #----- Copying -----
        
        try:
            base_state_msg = Odometry()                                              # Base position and velocity
            
            self.processing = True
            
            base_state_msg = self.baseState_msg
            panda_model = self.get_panda_model_state_srv()
            
            self.processing = False
            
            #----- Update values using URDF model -----
            
            self.CalculateVars_usingPanda(panda_model, base_state_msg)
            
            #----- Calculate Info -----
            
            T_O_ee = np.matmul(self.T_O_b, self.T_b_ee)  
            
            C_O_ee = R.from_dcm(T_O_ee[:3, :3])
            C_O_b = R.from_dcm(self.T_O_b[:3, :3])
            C_b_ee = R.from_dcm(self.T_b_ee[:3, :3])
            
            r_O_ee = np.squeeze(np.copy(T_O_ee[:3, 3]))
                
            #----- Update Buffers in direction_estimator class -----
                         
            self.direction_estimator.UpdateBuffers(self.force, r_O_ee)
            
            #----- Update Unconstrained direction estimate -----
            
            self.direction_estimator.UpdateEstimate(self.force, alpha, C_O_ee, smooth, mixCoeff)
            
            #----- Get linear velocity magnitude from the velocity profile -----
            
            velProfile = self.VelocityProfile(t, vInit, vFinal, alphaInit, alphaFinal, t0, tConv)
            
            veldesEE_ee = self.direction_estimator.GetPlannedVelocities(v=velProfile, calcAng=True, kAng=0.05)
            
            r_b_ee = self.T_b_ee[:3, 3]
            
            infoTuple = (self.M, self.b, self.J_b_ee, self.q, self.q_dot, C_O_b, C_O_ee, C_b_ee, r_b_ee, velProfile, self.tau)
            
            temp = self.controller.PerformOneStep(veldesEE_ee, infoTuple)
            
            self.publishArmAndBaseVelocityControl(self.controller.GetCurrOptSol(), linVelBase = self.controller.vLinBase_b, angVelBase = self.controller.vAngBase_b[2])
        
        except rospy.ServiceException as e:
            
            print("Service failed: " + str(e))
            
    def prepare_for_stop(self):

        self.publishArmAndBase([0.0]*7, linVelBase = [0.0, 0.0, 0.0], angVelBase = 0.0)

