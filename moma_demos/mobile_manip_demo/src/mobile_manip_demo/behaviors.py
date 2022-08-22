"""Implement various py trees behaviors for object pick and place."""

from typing import Any, List

import mobile_manip_demo.robot_interface as skills
import numpy as np
import py_trees as pt


# import the robot interface where ROS bindings are defined
# TODO: have a Node superclass that gets a State argument to track robot internal states


def behavior_mapping(name: str, condition: bool = False):
    """Return the correct behavior from its name."""
    # TODO: implement this function
    # TODO: have also parameters as argument so that the behavior can be properly created
    return None


class RobotAtPose(pt.behaviour.Behaviour):
    """Check if robot is at position."""

    def __init__(
        self,
        name: str,
        robot_name: str,
        pose: np.ndarray or int,
        tolerance: float,
        ground_truth: bool = False,
    ):
        super().__init__(name)
        self.target_pose = pose
        self.tolerance = tolerance
        self.ground_truth = ground_truth
        self.interface = skills.RobotAtPose(robot_name)

    def update(self):
        if self.interface.at_pose(self.target_pose, self.tolerance, self.ground_truth):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class ObjectAtPose(pt.behaviour.Behaviour):
    """Check if object is at position."""

    def __init__(
        self,
        name: str,
        object_id: str,
        model_type: str,
        pose: np.ndarray,
        tolerance: float,
        ground_truth: bool = False,
    ):
        super().__init__(name)
        self.target_pose = pose
        self.tolerance = tolerance
        self.ground_truth = ground_truth
        self.interface = skills.ObjectAtPose(object_id, model_type)

    def update(self):
        if self.interface.at_pose(self.target_pose, self.tolerance, self.ground_truth):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class InHand(pt.behaviour.Behaviour):
    """Check if object is held."""

    def __init__(self, name: str):
        super().__init__(name)
        self.interface = skills.InHand()

    def update(self):
        if self.interface.in_hand():
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class Found(pt.behaviour.Behaviour):
    """Check if object is held."""

    def __init__(self, name: str, n_IDs: int = 3):
        super().__init__(name)
        self.n_items = n_IDs
        self.interface = skills.Found()

    def update(self):
        if self.interface.found(self.n_items):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class Move(pt.behaviour.Behaviour):
    """Navigate to the input goal."""

    def __init__(
        self,
        name: str,
        goal_ID: int = None,
        ref_frame: str = "map",
        goal_pose: np.ndarray = None,
    ):
        super().__init__(name)
        self.interface = skills.Move()
        self.goal_ID = goal_ID
        self.goal_pose = goal_pose
        self.ref_frame = ref_frame

    def initialise(self):
        self.interface.initialize_navigation(
            self.goal_pose, self.ref_frame, self.goal_ID
        )

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
