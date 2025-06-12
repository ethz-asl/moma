#! /usr/bin/env python3

import rospy
from smach import StateMachine
from smach_ros import ServiceState, SimpleActionState
from std_srvs.srv import Trigger
from stacking_demo.srv import MoveToTower, PlanEasyGrasp
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

    with sm:
        StateMachine.add(
            "PLAN_EASY_GRASP",
            ServiceState("plan_easy_grasp", PlanEasyGrasp, 
                         response_slots=["target_grasp_pose"]),
            transitions={
                "succeeded": "EXECUTE_GRASP",
            },
        )
        StateMachine.add(
            "EXECUTE_GRASP",
            SimpleActionState(
                "grasp_execution_action", GraspAction, goal_slots=["target_grasp_pose"]
            ),
            transitions={
                "succeeded": "DROP_OBJECT",
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

        # StateMachine.add(
        #     "WAIT_FOR_MODEL",
        #     ServiceState("wait_for_model", Trigger, request_slots=["y_offset"]),
        #     transitions={"succeeded": "MOVE_TO_TOWER"},
        # )

        # StateMachine.add(
        #     "MOVE_TO_TOWER",
        #     ServiceState(
        #         "move_to_tower", MoveToTower, request_slots=["y_offset"] 
        #     ),
        #     transitions={
        #         "succeeded": "DROP_OBJECT",
                
        #     },
        # )

        # StateMachine.add(
        #     "DROP_OBJECT",
        #     SimpleActionState("drop_action", DropAction),
        #     transitions={"succeeded": "RESET"},
        # )

    return sm


if __name__ == "__main__":
    main()
