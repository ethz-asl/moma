#! /usr/bin/env python

import rospy
from smach import StateMachine
from smach_ros import ServiceState, SimpleActionState
import std_srvs.srv

from grasp_demo.msg import *


def construct_state_machine():
    """Define the states and their transitions"""

    sm = StateMachine(outcomes=["succeeded", "aborted", "preempted"])

    with sm:
        StateMachine.add(
            "RESET",
            ServiceState("reset", std_srvs.srv.Trigger),
            transitions={"succeeded": "RECONSTRUCT_SCENE"},
        )

        StateMachine.add(
            "RECONSTRUCT_SCENE",
            SimpleActionState(
                "scan_action", ScanSceneAction, result_slots=["pointcloud_scene"]
            ),
            transitions={"succeeded": "PLAN_GRASP", "aborted": "RESET",},
        )

        StateMachine.add(
            "PLAN_GRASP",
            SimpleActionState(
                "grasp_selection_action",
                SelectGraspAction,
                goal_slots=["pointcloud_scene"],
                result_slots=["target_grasp_pose"],
            ),
            transitions={"succeeded": "EXECUTE_GRASP", "aborted": "RESET",},
        )

        StateMachine.add(
            "EXECUTE_GRASP",
            SimpleActionState(
                "grasp_execution_action", GraspAction, goal_slots=["target_grasp_pose"]
            ),
            transitions={"succeeded": "DROP_OBJECT", "aborted": "RESET",},
        )

        StateMachine.add(
            "DROP_OBJECT",
            SimpleActionState("drop_action", DropAction),
            transitions={"succeeded": "RESET"},
        )

    return sm


def main():
    rospy.init_node("grasp_demo", log_level=rospy.INFO)

    # Construct the state machine
    sm = construct_state_machine()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for keyboard interrupt
    rospy.spin()


if __name__ == "__main__":
    main()
