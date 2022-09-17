"""Define SMACH states for the sequential FSM."""

from typing import List

import mobile_manip_demo.robot_interface as skills
import numpy as np
import rospy
import smach


class Move(smach.State):
    def __init__(
        self,
        name: str,
        goal_ID: int,
        ref_frame: str,
        goal_pose: List[float],
        outcomes: List[str],
    ):
        self.outcomes = outcomes
        super().__init__(outcomes=outcomes)

        self.name = name
        self.interface = skills.Move()
        self.goal_ID = goal_ID
        self.ref_frame = ref_frame
        self.goal_pose = np.array(goal_pose) if goal_pose is not None else goal_pose
        self.target_pose = (
            self.goal_pose if self.goal_pose is not None else self.goal_ID
        )
        self.initialized = False

        self.condition = skills.RobotAtPose("panda")

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state MOVE!")
        self.interface.initialize_navigation(
            goal_pose=self.goal_pose, ref_frame=self.ref_frame, goal_ID=self.goal_ID
        )
        return True

    def execute(self, userdata):
        if not self.initialized:
            self.initialized = self.initialize()
        rospy.Rate(1).sleep()
        rospy.loginfo("Executing state MOVE!")

        running = True
        while running:
            status = self.interface.get_navigation_status()
            if status == 0 or status == 1:
                continue
            elif status == 3 and self.condition.at_pose(
                target_pose=self.target_pose, tolerance=0.17
            ):
                rospy.logwarn(f"Target pose: {self.target_pose}")
                rospy.loginfo(f"Behavior {self.name} returned SUCCESS!")
                running = False
                return self.outcomes[0]
            else:
                running = False
                return "FAILURE"
                # rospy.signal_shutdown(f"Task failed in {self.name} state!")


class Pick(smach.State):
    def __init__(
        self,
        name: str,
        goal_ID: str,
        goal_pose: List[float],
        outcomes: List[str],
    ):
        self.outcomes = outcomes
        super().__init__(outcomes=outcomes)

        self.name = name
        self.interface = skills.Pick()
        self.goal_ID = goal_ID
        self.goal_pose = np.array(goal_pose) if goal_pose is not None else goal_pose
        self.initialized = False

        self.condition = skills.InHand()

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state PICK!")
        self.interface.initialize_pick(goal_ID=self.goal_ID, goal_pose=self.goal_pose)
        return True

    def execute(self, userdata):
        if not self.initialized:
            self.initialized = self.initialize()
        rospy.Rate(1).sleep()
        rospy.loginfo("Executing state PICK!")

        running = True
        while running:
            status = self.interface.get_pick_status()
            if status == 0 or status == 1:
                continue
            elif status == 3 and self.condition.in_hand():
                rospy.loginfo(f"Behavior {self.name} returned SUCCESS!")
                running = False
                return self.outcomes[0]
            else:
                running = False
                return "FAILURE"
                # rospy.signal_shutdown(f"Task failed in {self.name} state!")


class Place(smach.State):
    def __init__(
        self,
        name: str,
        goal_ID: str,
        place_target: List[float],
        goal_pose: List[float],
        outcomes: List[str],
    ):
        self.outcomes = outcomes
        super().__init__(outcomes=outcomes)

        self.name = name
        self.interface = skills.Place()
        self.goal_ID = goal_ID
        self.place_target = np.array(place_target)
        self.goal_pose = np.array(goal_pose)
        self.initialized = False

        self.condition = skills.ObjectAtPose(goal_ID, "cubes")

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state PLACE!")
        self.interface.initialize_place(
            goal_pose=self.place_target, goal_ID=self.goal_ID
        )
        return True

    def execute(self, userdata):
        if not self.initialized:
            self.initialized = self.initialize()
        rospy.Rate(1).sleep()
        rospy.loginfo("Executing state PLACE!")

        running = True
        while running:
            status = self.interface.get_place_status()
            if status == 0 or status == 1:
                continue
            elif status == 3 and self.condition.at_pose(
                target_pose=self.goal_pose, tolerance=np.array([0.5, 0.5, 0.1])
            ):
                rospy.loginfo(f"Behavior {self.name} returned SUCCESS!")
                running = False
                return self.outcomes[0]
            else:
                running = False
                return "FAILURE"
                # rospy.signal_shutdown(f"Task failed in {self.name} state!")


class Dummy(smach.State):
    def __init__(self, outcomes: List[str]):
        self.outcomes = outcomes
        super().__init__(outcomes=outcomes)

    def execute(self, userdata):
        return self.outcomes[0]
