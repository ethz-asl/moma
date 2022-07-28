"""Define SMACH states for the FSM."""

from enum import IntEnum
from typing import List

import mobile_manip_demo.robot_interface as skills
import rospy
import smach


def state_linker(
    name: str,
    current_state: "WorldState",
    outcomes: List[str],
    terminal_transition: str,
    *args,
) -> smach.State or None:
    """Link the name of the state to add to the function realizing it."""
    if name.startswith("grasp"):
        # TODO: match the string with a template to get the target for the action
        goal_ID = ""
        return Grasp(name, goal_ID, current_state, outcomes, terminal_transition)

    elif name == "IDLE":
        return IDLE(name, goal_ID, current_state, outcomes, terminal_transition)

    return None


class WorldState(IntEnum):
    """Definition of a state in the State Machine Simulator."""

    POSE = 0
    HAS_CUBE = 1
    CUBE_PLACED = 2
    BATTERY = 3


# define state Grasp
class Grasp(smach.State):
    def __init__(
        self,
        name: str,
        goal_ID: str,
        current_state: "WorldState",
        outcomes: List[str],
        terminal_transition: str,
    ):
        self.name = name
        self.state = current_state
        self.outcomes = outcomes
        self.task_succeeded = terminal_transition
        smach.State.__init__(
            self,
            outcomes=self.outcomes,
        )

        self.interface = skills.Pick()
        self.goal = goal_ID
        self.initialized = False

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state GRASP!")
        self.interface.initialize_pick(self.goal)
        return True

    def execute(self, userdata):
        if not self.initialized:
            self.initialized = self.initialize()
        rospy.loginfo("Executing state GRASP!")

        status = self.interface.get_pick_status()
        if status == 0 or status == 1:
            outcome = "RUNNING"
        elif status == 3:
            outcome = "SUCCESS"
        else:
            outcome = "FAILURE"

        # TODO: update WorldState
        rospy.loginfo(f"Behavior {self.name} returned {outcome}!")
        if outcome == "SUCCESS":
            return self.task_succeeded
        else:
            return outcome


# define state IDLE
class IDLE(smach.State):
    def __init__(
        self,
        name: str,
        goal_ID: str,
        current_state: "WorldState",
        outcomes: List[str],
        terminal_transition: str,
    ):
        self.name = name
        self.state = current_state
        self.outcomes = outcomes
        self.task_succeeded = terminal_transition
        smach.State.__init__(
            self,
            outcomes=self.outcomes,
        )

        self.interface = skills.Pick()
        self.goal = goal_ID
        self.initialized = False

    def initialize(self) -> bool:
        rospy.loginfo("Initializing state GRASP!")
        self.interface.initialize_pick(self.goal)
        return True

    def execute(self, userdata):
        rospy.loginfo("Recovery behavior: IDLE!")
        outcome = self.__get_next_state(self.state)
        rospy.loginfo(f"Behavior {self.name} returned {outcome}!")
        return outcome

    def __get_next_state(self, state: "WorldState"):
        # TODO: implement IDLE logic, considering that all transitions are in self.outcomes
        # The order of the self.outcomes list depends on the PLAN given in the knowledge base
        state[WorldState.BATTERY] -= 10
        if state[WorldState.BATTERY] < 40:
            return "battery low"
        elif state[WorldState.CUBE_PLACED]:
            return "cube1 on cupboard"
        elif state[WorldState.HAS_CUBE]:
            if state[WorldState.POSE] == (0.174, 1.343, 0.303):
                return "(0.174, 1.343, 0.303) in-reach"
            else:
                return "cube1 in-hand"
        else:
            if state[WorldState.POSE] == "cube1":
                return "cube1 in-reach"
            else:
                return "start the task"
