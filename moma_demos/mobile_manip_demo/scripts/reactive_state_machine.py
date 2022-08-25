#!/usr/bin/env python
"""Build the FSM."""

import sys

from mobile_manip_demo import reactive_states
import rospy
import smach
import smach_ros


def reactive_state_machine(cube_ID: int, visualize=False):

    rospy.init_node("state_machine")

    # Parameters
    delivery = rospy.get_param("moma_demo/delivery_station")
    place_pose = rospy.get_param("moma_demo/place_pose")

    move_1_name = f"Move-To cube{cube_ID}"
    move_1_outcome = f"Cube{cube_ID} reached"

    pick_name = f"Pick cube{cube_ID}"
    pick_outcome = f"Cube{cube_ID} in Hand"

    move_2_name = "Move-To delivery"
    move_2_outcome = "Delivery pose reached"

    place_name = f"Place cube{cube_ID}"
    place_outcome = f"Cube{cube_ID} placed"

    goal_dictionary = {
        "move_1": [move_1_name, cube_ID],
        "pick": [pick_name, cube_ID],
        "move_2": [move_2_name, delivery],
        "place": [place_name, place_pose],
    }

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["SUCCESS"])

    with sm:
        smach.StateMachine.add(
            move_1_name,
            reactive_states.Move(
                name=move_1_name,
                goal_ID=cube_ID,
                ref_frame="map",
                goal_pose=None,
                outcomes=[move_1_outcome, "RUNNING", "FAILURE"],
            ),
            transitions={
                move_1_outcome: pick_name,
                "RUNNING": move_1_name,
                "FAILURE": "IDLE",
            },
        )

        smach.StateMachine.add(
            pick_name,
            reactive_states.Pick(
                name=pick_name,
                goal_ID=cube_ID,
                goal_pose=None,
                outcomes=[pick_outcome, "RUNNING", "FAILURE"],
            ),
            transitions={
                pick_outcome: move_2_name,
                "RUNNING": pick_name,
                "FAILURE": "IDLE",
            },
        )

        smach.StateMachine.add(
            move_2_name,
            reactive_states.Move(
                name=move_2_name,
                goal_ID=None,
                ref_frame="map",
                goal_pose=delivery,
                outcomes=[move_2_outcome, "RUNNING", "FAILURE"],
            ),
            transitions={
                move_2_outcome: place_name,
                "RUNNING": move_2_name,
                "FAILURE": "IDLE",
            },
        )

        smach.StateMachine.add(
            place_name,
            reactive_states.Place(
                name=place_name,
                goal_ID=cube_ID,
                goal_pose=place_pose,
                outcomes=[place_outcome, "RUNNING", "FAILURE"],
            ),
            transitions={
                place_outcome: "Success",
                "RUNNING": place_name,
                "FAILURE": "IDLE",
            },
        )

        smach.StateMachine.add(
            "IDLE",
            reactive_states.IDLE(
                name="IDLE",
                goal_dict=goal_dictionary,
                outcomes=[
                    "Restart the task",
                    move_1_outcome,
                    pick_outcome,
                    move_2_outcome,
                    place_outcome,
                    "RUNNING",
                ],
            ),
            transitions={
                "Restart the task": move_1_name,
                move_1_outcome: pick_name,
                pick_outcome: move_2_name,
                move_2_outcome: place_name,
                place_outcome: "Success",
                "RUNNING": "IDLE",
            },
        )

        smach.StateMachine.add(
            "Success",
            reactive_states.Dummy(outcomes=["SUCCESS"]),
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
    if len(sys.argv) < 2:
        reactive_state_machine(2, False)
        print("Usage: <node> arg1")
        print("arg1: terminal OR sequence OR fallback OR connected")
    else:
        reactive_state_machine(2, sys.argv[1])
