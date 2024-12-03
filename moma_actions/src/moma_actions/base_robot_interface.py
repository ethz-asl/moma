#!/usr/bin/env python

import rospy
import numpy as np
import scipy as sc
from abc import ABC, abstractmethod
from typing import Optional

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class Move(ABC):
    def __init__(
        self,
        goal_pose: Pose = Pose(),
        rate: float = 1.0,
        tol_lin: float = 0.05,
        tol_ang: float = 5.0 * np.pi / 180.0,
        max_lin_vel: float = 0.2,
        max_ang_vel: float = 0.2,
    ) -> None:
        self.goal_pose: Pose = goal_pose
        self.current_pose: Optional[Pose] = None
        self.rate: rospy.Rate = rospy.Rate(rate)
        self.tol_lin = tol_lin
        self.tol_ang = tol_ang
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        #  self.odom_sub = rospy

    @abstractmethod
    def odom_cb(self, msg: Odometry) -> None:
        pass

    @abstractmethod
    def init_move(self) -> None:
        pass

class ComponentStatus(ABC):
    def __init__(self, component: str) -> None:
        pass

class AtPose(ABC):
    def __init__(self) -> None:
        pass
        
    @abstractmethod
    def at_pose(self, goal_pose: Pose, tol: float) -> bool:
        pass

class ParamClient(ABC):
    def __init__(self, param_name: str, check_condition: callable):
        self._param_name = param_name
        self._check_condition = check_condition

    @abstractmethod 
    def is_condition_met(self):
        pass

class TopicClient(ABC):
    def __init__(self, topic_name: str, msg_type, check_condition: callable):
        self._check_condition = check_condition

    @abstractmethod 
    def is_condition_met(self):
        pass
