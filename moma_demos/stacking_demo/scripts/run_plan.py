#! /usr/bin/env python3

import rospy
import numpy as np

from smach import State, StateMachine
from smach_ros import ServiceState, SimpleActionState
from std_srvs.srv import Trigger
from stacking_demo.srv import MoveToTower, PlanEasyGrasp, GetTowerPrediction, MoveToObjID
from stacking_demo.msg import GraspAction, SelectGraspAction, DropAction


"""
stacking demo

there are 2 positions, hardcoded
1. object to be picked
2. tower

the objective is to pick the object and stack it on the tower
as follows:
1. [optional] scan the object to get the grasp pose
2. [optional] plan the grasp pose
3. execute the grasp and pick the object
4. move to middle position and measure the force of the object
5. wait for model to predict the tower position
6. move to the tower position with a y translation offset from rosparam
7. drop the object on the tower
"""

class Sleep5s(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded"])

    def execute(self, userdata):
        rospy.sleep(5)
        return "succeeded"
    
class SelectRandomObject(State):
    def __init__(self):
        State.__init__(self, outcomes=["0", "1", "2"])

    def execute(self, userdata):
        obj_id = np.random.randint(3)
        return str(obj_id)
    
def main():
    rospy.init_node("stacking_demo", log_level=rospy.INFO)

    # Construct the state machine
    sm = construct_state_machine()

    # Execute SMACH plan
    sm.execute()

    # Wait for keyboard interrupt
    rospy.spin()


def construct_state_machine():
    """Define the states and their transitions"""

    sm = StateMachine(outcomes=["succeeded", "aborted", "preempted"])
    y_offset = 0.0

    with sm:

        StateMachine.add(
            "MOVE_TO_HOME",
            ServiceState("move_to_home", Trigger),
            transitions={
                "succeeded": "SELECT_RANDOM_OBJECT",
            },
        )
        # add Time for the operator to check the tower
        # TODO would be better to save the obj_id in a variable, since we later need it

        StateMachine.add(
            "SELECT_RANDOM_OBJECT",
            SelectRandomObject(),
            transitions={
                "0": "MOVE_TO_OBJECT_0",
                "1": "MOVE_TO_OBJECT_1",
                "2": "MOVE_TO_OBJECT_2",
            },
        )

        StateMachine.add(
            "MOVE_TO_OBJECT_0",
            ServiceState("move_to_object_id", MoveToObjID, request=0), 
            transitions={
                "succeeded": "GET_TOWER_PREDICTION",
            },
        )

        StateMachine.add(
            "MOVE_TO_OBJECT_1",
            ServiceState("move_to_object_id", MoveToObjID, request=1),
            transitions={
                "succeeded": "GET_TOWER_PREDICTION",
            },
        )

        StateMachine.add(
            "MOVE_TO_OBJECT_2",
            ServiceState("move_to_object_id", MoveToObjID, request=2),
            transitions={
                "succeeded": "GET_TOWER_PREDICTION",
            },
        )

        # TODO this should get the obj_id as input
        StateMachine.add(
            "GET_TOWER_PREDICTION",
            ServiceState("get_tower_prediction", GetTowerPrediction, response_slots=["y_offset"]),
            transitions={"succeeded": "MOVE_TO_TOWER"},
        )

        StateMachine.add(
            "MOVE_TO_TOWER",
            ServiceState(
                "move_to_tower", MoveToTower, request_slots=["y_offset"]),
            transitions={
                "succeeded": "SLEEP",
                
            },
        )
        # Time to check if the tower is stable
        StateMachine.add(
            "SLEEP",
            Sleep5s(),
            transitions={
                "succeeded": "REMOVE_OBJECT_FROM_TOWER",
            },
        )

        StateMachine.add(
            "REMOVE_OBJECT_FROM_TOWER",
            ServiceState(
                "remove_object_from_tower", MoveToTower, request_slots=["y_offset"]),
            transitions={
                "succeeded": "RETURN_OBJECT",
                
            },
        )

        # TODO correctly add here the obj_id logic, currently it always drops it at the same place

        StateMachine.add(
            "RETURN_OBJECT",
            ServiceState("return_object", MoveToObjID, request=0),
            transitions={
                "succeeded": "MOVE_TO_HOME",
            },
        )


        # StateMachine.add(
        #     "MOVE_TO_MIDDLE",
        #     SimpleActionState(
        #         "move_to_middle", Trigger, goal_slots=["target_pose"]
        #     ),
        #     transitions={"succeeded": "MEASURE_FORCE",
                         
        #     },
        # )

 

 
    return sm


if __name__ == "__main__":
    main()
