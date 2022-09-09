"""Implement various py trees behaviors mobile manipulation."""

from typing import Any, List

import mobile_manip_demo.robot_interface as skills
import numpy as np
import py_trees as pt


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
    ):
        super().__init__(name)
        self.target_pose = pose
        self.tolerance = tolerance
        self.interface = skills.ObjectAtPose(object_id, model_type)

    def update(self):
        if self.interface.at_pose(self.target_pose, self.tolerance):
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


class BatteryLv(pt.behaviour.Behaviour):
    """Check if battery is at threshold value."""

    def __init__(self, name: str, relation: str, value: float):
        super().__init__(name)
        self.interface = skills.BatteryLv()
        self.relation = relation
        self.value = value

    def update(self):
        if self.interface.battery_lv(self.relation, self.value):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class Found(pt.behaviour.Behaviour):
    """Check if object is held."""

    def __init__(self, name: str, IDs: List[int] = [2]):
        super().__init__(name)
        self.IDs = IDs
        self.interface = skills.Found()

    def update(self):
        if self.interface.found(self.IDs):
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


class Recharge(pt.behaviour.Behaviour):
    """Move the robot to recharge station and charge the robot."""

    def __init__(self, name: str):
        super().__init__(name)
        self.interface = skills.Recharge()

    def initialise(self):
        self.interface.initialize_recharge()

    def update(self) -> pt.common.Status:
        status = self.interface.get_recharge_status()
        if status == 0 or status == 1:
            return pt.common.Status.RUNNING
        elif status == 3:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def terminate(self, new_status: pt.common.Status):
        if new_status == pt.common.Status.INVALID:
            self.interface.cancel_goal()


class Dock(pt.behaviour.Behaviour):
    """Move the robot to the docking station."""

    def __init__(self, name: str):
        super().__init__(name)
        self.interface = skills.Dock()

    def initialise(self):
        self.interface.initialize_docking()

    def update(self) -> pt.common.Status:
        status = self.interface.get_docking_status()
        if status == 0 or status == 1:
            return pt.common.Status.RUNNING
        elif status == 3:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def terminate(self, new_status: pt.common.Status):
        if new_status == pt.common.Status.INVALID:
            self.interface.cancel_goal()


class Search(pt.behaviour.Behaviour):
    """Move the robot to recharge station and charge the robot."""

    def __init__(self, name: str, targets: List[List[float]]):
        super().__init__(name)
        self.interface = skills.Search(targets)

    def initialise(self):
        self.interface.initialize_search()

    def update(self) -> pt.common.Status:
        status = self.interface.get_search_status()
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

    def __init__(self, name: str, goal_ID: str, goal_pose: List[float] = None):
        super().__init__(name)
        self.interface = skills.Pick()
        self.goal_ID = goal_ID
        self.goal_pose = goal_pose

    def initialise(self):
        if self.goal_pose is not None:
            self.interface.initialize_pick(
                goal_ID=self.goal_ID, goal_pose=self.goal_pose
            )
        else:
            self.interface.initialize_pick(goal_ID=self.goal_ID)

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

    def __init__(self, name: str, goal_ID: str, goal_pose: List[float]):
        super().__init__(name)
        self.interface = skills.Place()
        self.goal_ID = goal_ID
        self.goal_pose = goal_pose

    def initialise(self):
        self.interface.initialize_place(goal_pose=self.goal_pose, goal_ID=self.goal_ID)

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


class RSequence(pt.composites.Selector):
    """
    Rsequence for py_trees.
    Reactive sequence overidding sequence with memory, py_trees' only available sequence.
    Author: Chrisotpher Iliffe Sprague, sprague@kth.se
    """

    def __init__(self, name: str = "Sequence", children: List[Any] = None):
        super().__init__(name=name, children=children)

    def tick(self):
        """
        Run the tick behaviour for this selector.
        Note that the status of the tick is always determined by its children,
        not by the user customized update function.
        Yields
        ------
            class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children.
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # Required behaviour for *all* behaviours and composites is
        # for tick() to check if it isn't running and initialise
        if self.status != pt.common.Status.RUNNING:
            # selectors dont do anything specific on initialisation
            #   - the current child is managed by the update, never needs to be 'initialised'
            # run subclass (user) handles
            self.initialise()
        # run any work designated by a customized instance of this class
        self.update()
        previous = self.current_child
        for child in self.children:
            for node in child.tick():
                yield node
                if node is child and (
                    node.status == pt.common.Status.RUNNING
                    or node.status == pt.common.Status.FAILURE
                ):
                    self.current_child = child
                    self.status = node.status
                    if previous is None or previous != self.current_child:
                        # we interrupted, invalidate everything at a lower priority
                        passed = False
                        for sibling in self.children:
                            if passed and sibling.status != pt.common.Status.INVALID:
                                sibling.stop(pt.common.Status.INVALID)
                            if sibling == self.current_child:
                                passed = True
                    yield self
                    return
        # all children succeded,
        # set succeed ourselves and current child to the last bugger who failed us
        self.status = pt.common.Status.SUCCESS
        try:
            self.current_child = self.children[-1]
        except IndexError:
            self.current_child = None
        yield self
