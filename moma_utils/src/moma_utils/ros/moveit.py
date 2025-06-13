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
        if type(plan) is tuple:
            plan = plan[1]

        return plan

    # adapted from  # copied from https://github.com/ethz-asl/moma/blob/f341dc79d813d65095348e43d7b433277fa8561c/panda_control/src/panda_control/panda_commander.py#L68
    def gotoJoint(self, target, max_velocity_scaling=0.1, max_acceleration_scaling=0.1):
        self.move_group.set_max_velocity_scaling_factor(max_velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(max_acceleration_scaling)
        print(f"Current joint values: {self.move_group.get_current_joint_values()}")
        self.move_group.set_joint_value_target(target)
        # somehow returns (Planning success, plan)
        plan = self.move_group.plan()
        # print(f"Plan: {plan}")
        # print(f"Plan 0: {plan[0]}")
        # print(f"Plan 1: {plan[1]}")
        success = self.move_group.execute(plan[1], wait=True)
        print(f"success: {success}")
        self.move_group.stop()

        # success = self.execute(plan)
        return success

    def gotoL(self, target, velocity_scaling=0.1, acceleration_scaling=0.1, eef_step=0.01):
        plan = self.planL(target, velocity_scaling, acceleration_scaling, eef_step)
        # print(f"plan: {plan}")
        success = self.execute(plan)
        return success

    def planL(self, target, velocity_scaling=0.1, acceleration_scaling=0.1, eef_step=0.01):
        waypoints = [to_pose_msg(target)]
        self.move_group.set_max_velocity_scaling_factor(velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acceleration_scaling)
        plan, _ = self.move_group.compute_cartesian_path(waypoints, eef_step, True)
        return plan


    def execute(self, plan):
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success
