"""Define SMACH states for the FSM."""

from typing import List

import mobile_manip_demo.robot_interface as skills
import numpy as np
import rospy
import smach


class Search(smach.State):
    def __init__(
        self,
        name: str,
        locations: np.ndarray or list,
        IDs: List[int],
        outcomes: List[str],
    ):
        self.outcomes = outcomes
        super().__init__(outcomes=outcomes)

        self.name = name
        self.locations = locations
        self.interface = skills.Search(locations)
        self.IDs = IDs

        self.initialized = False

        self.condition = skills.Found()
        self.battery_condition = skills.BatteryLv()

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state SEARCH!")
        self.interface.initialize_search()
        return True

    def execute(self, userdata):
        if self.condition.found(self.IDs):
            self.interface.cancel_goal()
            self.initialized = False
            return self.outcomes[0]
        elif not self.initialized:
            self.initialized = self.initialize()
        rospy.Rate(1).sleep()
        rospy.loginfo("Executing state SEARCH!")

        status = self.interface.get_search_status()

        if self.battery_condition.battery_lv("lower", 20.0):
            self.interface.cancel_goal()
            self.initialized = False
            return self.outcomes[-1]
        elif status == 0 or status == 1:
            return "RUNNING"
        elif status == 3 and self.condition.found(self.IDs):
            self.initialized = False
            return self.outcomes[0]
        else:
            self.initialized = False
            return "FAILURE"


class Recharge(smach.State):
    def __init__(self, name: str, outcomes: List[str]):
        self.outcomes = outcomes
        super().__init__(outcomes=outcomes)

        self.name = name
        self.interface = skills.Recharge()

        self.initialized = False

        self.condition = skills.BatteryLv()

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state RECHARGE!")
        self.interface.initialize_recharge()
        return True

    def execute(self, userdata):
        if self.condition.battery_lv("greater", 20.0):
            self.interface.cancel_goal()
            self.initialized = False
            return self.outcomes[0]
        elif not self.initialized:
            self.initialized = self.initialize()
        rospy.Rate(1).sleep()
        rospy.loginfo("Executing state RECHARGE!")

        status = self.interface.get_recharge_status()
        if status == 0 or status == 1:
            return "RUNNING"
        elif status == 3:
            self.initialized = False
            return self.outcomes[0]
        else:
            self.initialized = False
            return self.outcomes[0]


class Dock(smach.State):
    def __init__(self, name: str, target_pose: List[float], outcomes: List[str]):
        self.outcomes = outcomes
        super().__init__(outcomes=outcomes)

        self.name = name
        self.interface = skills.Dock()

        self.initialized = False

        self.condition = skills.RobotAtPose("panda")
        self.battery_condition = skills.BatteryLv()
        self.target_pose = np.array(target_pose)

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state RECHARGE!")
        self.interface.initialize_docking()
        return True

    def check_done(self) -> bool:
        return self.condition.at_pose(target_pose=self.target_pose, tolerance=0.17)

    def execute(self, userdata):
        if not self.initialized:
            self.initialized = self.initialize()
        rospy.Rate(1).sleep()
        rospy.loginfo("Executing state DOCK!")

        status = self.interface.get_docking_status()
        if self.battery_condition.battery_lv("lower", 20.0):
            self.interface.cancel_goal()
            self.initialized = False
            return self.outcomes[-1]
        elif status == 0 or status == 1:
            return "RUNNING"
        elif status == 3 and self.check_done():
            self.initialized = False
            rospy.logwarn(f"Target pose:{self.target_pose}")
            rospy.loginfo(f"Behavior {self.name} returned SUCCESS!")
            return self.outcomes[0]
        else:
            self.initialized = False
            return "FAILURE"


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
        self.battery_condition = skills.BatteryLv()

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state MOVE!")
        self.interface.initialize_navigation(
            goal_pose=self.goal_pose, ref_frame=self.ref_frame, goal_ID=self.goal_ID
        )
        return True

    def check_done(self) -> bool:
        return self.condition.at_pose(target_pose=self.target_pose, tolerance=0.17)

    def execute(self, userdata):
        if not self.initialized:
            self.initialized = self.initialize()
        rospy.Rate(1).sleep()
        rospy.loginfo("Executing state MOVE!")

        status = self.interface.get_navigation_status()
        if self.battery_condition.battery_lv("lower", 20.0):
            self.interface.cancel_goal()
            self.initialized = False
            return self.outcomes[-1]
        elif status == 0 or status == 1:
            return "RUNNING"
        elif status == 3 and self.check_done():
            self.initialized = False
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
        self.battery_condition = skills.BatteryLv()

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
        if self.battery_condition.battery_lv("lower", 20.0):
            self.interface.cancel_goal()
            self.initialized = False
            return self.outcomes[-1]
        elif status == 0 or status == 1:
            return "RUNNING"
        elif status == 3 and self.condition.in_hand():
            self.initialized = False
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
        self.battery_condition = skills.BatteryLv()

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state PLACE!")
        rospy.logerr(f"with target {self.goal_pose}")
        self.interface.initialize_place(
            goal_pose=self.place_target, goal_ID=self.goal_ID
        )
        return True

    def check_done(self) -> bool:
        return self.condition.at_pose(
            target_pose=self.goal_pose, tolerance=np.array([0.5, 0.5, 0.1])
        )

    def execute(self, userdata):
        if self.check_done():
            self.interface.cancel_goal()
            self.initialized = False
            return self.outcomes[0]
        elif not self.initialized:
            self.initialized = self.initialize()
        rospy.Rate(1).sleep()
        rospy.loginfo("Executing state PLACE!")

        status = self.interface.get_place_status()
        if self.battery_condition.battery_lv("lower", 20.0):
            self.interface.cancel_goal()
            self.initialized = False
            return self.outcomes[-1]
        elif status == 0 or status == 1:
            return "RUNNING"
        elif status == 3 and self.check_done():
            self.initialized = False
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
        out_dict: dict,
        outcomes: List[str],
    ):
        self.out_dict = out_dict
        self.outcomes = outcomes + list(out_dict.values())
        smach.State.__init__(
            self,
            outcomes=self.outcomes,
        )

        self.name = name
        self.goal_dict = goal_dict

        self.move_condition = skills.RobotAtPose("panda")
        self.pick_condition = skills.InHand()
        self.place_condition = skills.ObjectAtPose(goal_dict["pick"][1], "cubes")
        self.battery_condition = skills.BatteryLv()
        self.search_condition = skills.Found()

    def execute(self, userdata):
        rospy.loginfo("Recovery behavior: IDLE!")
        rospy.sleep(3)
        outcome = self.__get_next_state()
        rospy.loginfo(f"Behavior {self.name} returned {outcome}!")
        return outcome

    def __get_next_state(self):
        # The order of the self.outcomes list depends on the PLAN given in the knowledge base
        if self.battery_condition.battery_lv("lower", 20.0):
            return self.out_dict["recharge"]
        elif self.move_condition.at_pose(
            target_pose=self.goal_dict["dock"][1], tolerance=0.15
        ):
            # If we are at the docking table, the task is solved
            rospy.logwarn(f"IDLE returning: {self.out_dict['dock']}")
            return self.out_dict["dock"]
        elif self.place_condition.at_pose(
            target_pose=self.goal_dict["place"][1], tolerance=np.array([0.5, 0.5, 0.1])
        ):
            # Task solved
            rospy.logwarn(f"IDLE returning: {self.out_dict['place']}")
            return self.out_dict["place"]
        elif self.pick_condition.in_hand():
            # The robot is holding the object, so we can move and place
            # If already at pose, place
            if self.move_condition.at_pose(
                target_pose=self.goal_dict["move_2"][1], tolerance=0.17
            ):
                # If we are at the delivery table, just place it
                rospy.logwarn(f"IDLE returning: {self.out_dict['move_2']}")
                return self.out_dict["move_2"]
            # Otherwise move
            else:

                rospy.logwarn(f"IDLE returning: {self.out_dict['pick']}")
                return self.out_dict["pick"]
        else:
            # The robot is not holding the cube, so go to pick it
            # If already in sight of the cube, just pick it
            if self.move_condition.at_pose(
                target_pose=self.goal_dict["move_1"][1], tolerance=0.17
            ):
                rospy.logwarn(f"IDLE returning: {self.out_dict['move_1']}")
                return self.out_dict["move_1"]
            # Otherwise make sure that you know where it is
            # elif self.search_condition.found(self.goal_dict["search"][1]):
            #     return self.out_dict["search"]
            else:
                # Restart the task
                return self.outcomes[0]
