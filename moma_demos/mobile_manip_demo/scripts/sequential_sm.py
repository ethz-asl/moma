#!/usr/bin/env python
"""Build the sequntial FSM."""

import sys

from mobile_manip_demo import states
from mobile_manip_demo.environment import get_place_pose
import numpy as np
import random
import rospy
import smach
import smach_ros


def state_machine(cube_ID: int, visualize=False):

    rospy.init_node("state_machine")

    # Parameters
    delivery = rospy.get_param("moma_demo/delivery_station")
    place_target = rospy.get_param("moma_demo/place_pose")
    place_pose = get_place_pose(np.array(delivery), np.array(place_target))

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["SUCCESS", "FAILURE"])

    with sm:
        smach.StateMachine.add(
            f"Move-To cube{cube_ID}",
            states.Move(
                name=f"Move-To cube{cube_ID}",
                goal_ID=cube_ID,
                ref_frame="map",
                goal_pose=None,
                outcomes=[f"Cube{cube_ID} reached", "FAILURE"],
            ),
            transitions={
                f"Cube{cube_ID} reached": f"Pick cube{cube_ID}",
                "FAILURE": "Failure",
            },
        )

        smach.StateMachine.add(
            f"Pick cube{cube_ID}",
            states.Pick(
                name=f"Pick cube{cube_ID}",
                goal_ID=cube_ID,
                goal_pose=None,
                outcomes=[f"Cube{cube_ID} in Hand", "FAILURE"],
            ),
            transitions={
                f"Cube{cube_ID} in Hand": "Move-To delivery",
                "FAILURE": "Failure",
            },
        )

        smach.StateMachine.add(
            f"Move-To delivery",
            states.Move(
                name=f"Move-To delivery",
                goal_ID=None,
                ref_frame="map",
                goal_pose=delivery,
                outcomes=[f"Delivery pose reached", "FAILURE"],
            ),
            transitions={
                f"Delivery pose reached": f"Place cube{cube_ID}",
                "FAILURE": "Failure",
            },
        )

        smach.StateMachine.add(
            f"Place cube{cube_ID}",
            states.Place(
                name=f"Place cube{cube_ID}",
                goal_ID=cube_ID,
                place_target=place_target,
                goal_pose=place_pose,
                outcomes=[f"Cube{cube_ID} placed", "FAILURE"],
            ),
            transitions={
                f"Cube{cube_ID} placed": "Success",
                "FAILURE": "Failure",
            },
        )

        smach.StateMachine.add(
            "Failure",
            states.Dummy(outcomes=["FAILURE"]),
            transitions={"FAILURE": "FAILURE"},
        )

        smach.StateMachine.add(
            "Success",
            states.Dummy(outcomes=["SUCCESS"]),
            transitions={"SUCCESS": "SUCCESS"},
        )

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer("server_name", sm, "/SM_ROOT")
    sis.start()

    if not visualize:
        # Execute SMACH plan
        outcome = sm.execute()
        if outcome == "SUCCESS":
            rospy.signal_shutdown("Task completed succesfully!!")
        else:
            rospy.signal_shutdown("Task failed!!")

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    random.seed(100)
    if len(sys.argv) < 2:
        state_machine(2, False)
        print("Usage: <node> arg1")
        print("arg1: terminal OR sequence OR fallback OR connected")
    else:
        state_machine(2, sys.argv[1])
