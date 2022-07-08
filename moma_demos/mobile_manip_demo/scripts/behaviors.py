"""Implement various py trees behaviors for object pick and place."""

from typing import Any, List

import mobile_manip_demo.robot_interface as skills
import py_trees as pt


# import the robot interface where ROS bindings are defined
# TODO: have a Node superclass that gets a State argument to track robot internal states


class RobotAtPose(pt.behaviour.Behaviour):
    """Check if robot is at position."""

    def __init__(self, name: str, pose_ID: str):
        super().__init__(name)
        self.target_pose = get_pose_from_obj(pose_ID)
        self.tolerance = get_navigation_tolerance()
        self.interface = skills.RobotAtPose()

    def update(self):
        if self.interface.at_pose(self.target_pose, self.tolerance):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class ObjectAtPose(pt.behaviour.Behaviour):
    """Check if object is at position."""

    def __init__(self, name: str, obj: str):
        super().__init__(name)
        self.target_object = obj
        self.target_pose = get_pose_from_obj(obj)
        self.tolerance = get_place_tolerance()
        self.interface = skills.ObjectAtPose()

    def update(self):
        if self.interface.at_pose(self.target_pose, self.tolerance):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class InHand(pt.behaviour.Behaviour):
    """Check if object is held."""

    def __init__(self, name: str, obj: str):
        super().__init__(name)
        self.target_object = obj
        self.interface = skills.InHand()

    def update(self):
        if self.interface.in_hand(self.target_object):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class Move(pt.behaviour.Behaviour):
    """Navigate to the input goal."""

    def __init__(self, name: str, goal_ID: str):
        super().__init__(name)
        self.interface = skills.Move()
        self.goal = goal_ID

    def initialise(self):
        self.interface.initialize_navigation(self.goal)

    def update(self) -> pt.common.Status:
        status = self.interface.get_navigation_status()
        if status == 0 or status == 1:
            return pt.common.Status.RUNNING
        elif status == 3:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def terminate(self, new_status: pt.common.Status):
        if new_status == pt.common.Status.INVALID:
            self.interface.cancel_goal()


class Pick(pt.behaviour.Behaviour):
    """Pick the target object."""

    def __init__(self, name: str, goal_ID: str):
        super().__init__(name)
        self.interface = skills.Pick()
        self.goal = goal_ID

    def initialise(self):
        self.interface.initialize_pick(self.goal)

    def update(self) -> pt.common.Status:
        status = self.interface.get_pick_status()
        if status == 0 or status == 1:
            return pt.common.Status.RUNNING
        elif status == 3:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def terminate(self, new_status: pt.common.Status):
        if new_status == pt.common.Status.INVALID:
            self.interface.cancel_goal()


class Place(pt.behaviour.Behaviour):
    """Place the target object."""

    def __init__(self, name: str, goal_ID: str):
        super().__init__(name)
        self.interface = skills.Place()
        self.goal = goal_ID

    def initialise(self):
        self.interface.initialize_place(self.goal)

    def update(self) -> pt.common.Status:
        status = self.interface.get_place_status()
        if status == 0 or status == 1:
            return pt.common.Status.RUNNING
        elif status == 3:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def terminate(self, new_status: pt.common.Status):
        if new_status == pt.common.Status.INVALID:
            self.interface.cancel_goal()
