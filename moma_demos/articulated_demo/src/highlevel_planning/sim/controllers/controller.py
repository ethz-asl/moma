import pybullet as p
import numpy as np

EPS = 1e-6
DEBUG = True

class ControllerTemplate:

    def __init__(self, scene, robot, time_step):
    
        self.scene = scene
        self.robot = robot
        self.dt = time_step
        
        self.mode_name = None
        
        #----- Constraints -----
        
        self.torque_max = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
        self.torque_dot_max = np.array([1000.0]*7)
        self.torque_dot_min = np.array([-1000.0]*7)
        
        self.q_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        self.q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        
        self.q_dot_max = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
        self.q_dot_min = np.array([-2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100])

        self.q_dot_dot_max = np.array([15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])
        self.q_dot_dot_min = -np.array([15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])
        
        self.q_mean = np.copy(0.5*(self.q_max+self.q_min))

        #----- Init Values -----
        
        self.q_dot_optimal = None
        #self.SetupController()
                        
#-------
    def SetupController(self):
    
        num_joints = p.getNumJoints(self.robot.model.uid)
        
        for i in range(num_joints):
            
            info = p.getJointInfo(self.robot.model.uid, i)
            if i in self.robot.joint_idx_arm or i in self.robot.joint_idx_fingers:
                
                self.q_dot_max.append(info[11])
                self.q_dot_min.append(-info[11])
                self.torque_max.append(info[10])
                self.q_max.append(info[9])
                self.q_min.append(info[8])
        
        self.q_dot_max = np.array(self.q_dot_max)
        self.q_dot_min = np.array(self.q_dot_min)
        self.torque_max = np.array(self.torque_max)
        self.q_max = np.array(self.q_max)
        self.q_min = np.array(self.q_min)
        
        self.q_mean = np.copy(0.5*(self.q_max+self.q_min))    
        
#-------
    def ResetDampings(self):
    
        p.changeDynamics(self.robot.model.uid, -1, linearDamping=0, angularDamping=0, jointDamping=0)
        
        for i in self.robot.joint_idx_arm:
            p.changeDynamics(self.robot.model.uid, i, linearDamping=0, angularDamping=0, jointDamping=0)
        
        p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_arm, p.VELOCITY_CONTROL, forces = [0.0]*len(self.robot.joint_idx_arm))
        
        for i in self.robot.joint_idx_fingers:
            p.changeDynamics(self.robot.model.uid, i, linearDamping=0, angularDamping=0, jointDamping=0)
        
        p.setJointMotorControlArray(self.robot.model.uid, self.robot.joint_idx_fingers, p.VELOCITY_CONTROL, forces = [0.0]*len(self.robot.joint_idx_fingers))        
        
#-------
    def GetCurrOptSol(self):
    
        return self.q_dot_optimal        
                                        
#-------            
    def PerformOneStep(self, veldesEE_ee, infoTuple=None):
    
        raise NotImplementedError
        
                
        
        
    
        
        
        
        
         
        
        
        
        
        
        
        
                         
                         
                
        
