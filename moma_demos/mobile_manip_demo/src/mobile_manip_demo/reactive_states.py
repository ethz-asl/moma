"""Define SMACH states for the FSM."""

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
        self.interface = skills.Move(approach=True)
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

        status = self.interface.get_navigation_status()
        if status == 0 or status == 1:
            return "RUNNING"
        elif status == 3 and self.condition.at_pose(
            target_pose=self.target_pose, tolerance=0.25
        ):
            rospy.logwarn(f"Target pose:{self.target_pose}")
            rospy.loginfo(f"Behavior {self.name} returned SUCCESS!")
            return self.outcomes[0]
        else:
            self.initialized = False
            return "FAILURE"


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

        status = self.interface.get_pick_status()
        if status == 0 or status == 1:
            return "RUNNING"
        elif status == 3 and self.condition.in_hand():
            rospy.loginfo(f"Behavior {self.name} returned SUCCESS!")
            return self.outcomes[0]
        else:
            self.initialized = False
            return "FAILURE"


class Place(smach.State):
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
        self.interface = skills.Place()
        self.goal_ID = goal_ID
        self.goal_pose = np.array(goal_pose)
        self.initialized = False

        self.condition = skills.ObjectAtPose(goal_ID, "cubes")

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state PLACE!")
        self.interface.initialize_place(goal_pose=self.goal_pose, goal_ID=self.goal_ID)
        return True

    def execute(self, userdata):
        if not self.initialized:
            self.initialized = self.initialize()
        rospy.Rate(1).sleep()
        rospy.loginfo("Executing state PLACE!")

        status = self.interface.get_place_status()
        if status == 0 or status == 1:
            return "RUNNING"
        elif status == 3 and self.condition.at_pose(
            target_pose=self.goal_pose, tolerance=0.5
        ):
            rospy.loginfo(f"Behavior {self.name} returned SUCCESS!")
            return self.outcomes[0]
        else:
            self.initialized = False
            return "FAILURE"


class Dummy(smach.State):
    def __init__(self, outcomes: List[str]):
        self.outcomes = outcomes
        super().__init__(outcomes=outcomes)

    def execute(self, userdata):
        return self.outcomes[0]


# define state IDLE
class IDLE(smach.State):
    def __init__(
        self,
        name: str,
        goal_dict: dict,
        outcomes: List[str],
    ):
        self.outcomes = outcomes
        smach.State.__init__(
            self,
            outcomes=self.outcomes,
        )

        self.name = name
        self.goal_dict = goal_dict

        self.move_condition = skills.RobotAtPose("panda")
        self.pick_condition = skills.InHand()
        self.place_condition = skills.ObjectAtPose(goal_dict["place"][1], "cubes")

        self.initialized = False

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state GRASP!")
        self.interface.initialize_pick(self.goal)
        return True

    def execute(self, userdata):
        rospy.loginfo("Recovery behavior: IDLE!")
        rospy.sleep(3)
        outcome = self.__get_next_state()
        rospy.loginfo(f"Behavior {self.name} returned {outcome}!")
        return outcome

    def __get_next_state(self):
        # TODO: implement IDLE logic, considering that all transitions are in self.outcomes
        # The order of the self.outcomes list depends on the PLAN given in the knowledge base
        if self.move_condition.at_pose(
            target_pose=self.goal_dict["move_1"][1], tolerance=0.25
        ):
            # Move action successfull: return next action
            return self.outcomes[1]
        elif self.pick_condition.in_hand():
            # Pick action successfull: return next action
            return self.outcomes[2]
        elif self.move_condition.at_pose(
            target_pose=self.goal_dict["move_2"][1], tolerance=0.25
        ):
            # Move action successfull: return next action
            return self.outcomes[3]
        elif self.place_condition.at_pose(
            target_pose=self.goal_dict["place"][1], tolerance=0.5
        ):
            # Move action successfull: return next action
            return self.outcomes[4]
        else:
            # restart the task?
            return self.outcomes[0]
