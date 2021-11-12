import moveit_commander
import numpy as np

from moma_utils.ros.conversions import *
from moma_utils.spatial import Transform


class MoveItClient:
    def __init__(self, planning_group):
        self.planning_group = planning_group
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.planning_group)

    def goto(self, target, velocity_scaling=0.2, acceleration_scaling=0.2):
        plan = self.plan(target, velocity_scaling, acceleration_scaling)
        success = self.execute(plan)
        return success

    def plan(self, target, velocity_scaling=0.2, acceleration_scaling=0.2):
        self.move_group.set_max_velocity_scaling_factor(velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acceleration_scaling)

        if isinstance(target, Transform):
            self.move_group.set_pose_target(to_pose_msg(target))
        elif isinstance(target, (list, np.ndarray)):
            self.move_group.set_joint_value_target(target)
        elif isinstance(target, str):
            self.move_group.set_named_target(target)
        else:
            raise ValueError

        plan = self.move_group.plan()

        return plan

    def execute(self, plan):
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success
