#!/usr/bin/env python
"""Build the fault tolerant FSM."""

import sys

from mobile_manip_demo import reactive_states
from mobile_manip_demo.environment import get_place_pose
import numpy as np
import random
import rospy
import smach
import smach_ros


def reactive_state_machine(cube_ID: int, visualize=False):

    rospy.init_node("state_machine")

    # Parameters
    delivery = rospy.get_param("moma_demo/delivery_station")
    place_target = rospy.get_param("moma_demo/place_pose")
    place_pose = get_place_pose(np.array(delivery), np.array(place_target))
    dock_pose = rospy.get_param("moma_demo/inspection_station")
    cube_locations = rospy.get_param("moma_demo/search_waypoints")
    search_IDs = [0, 1, 2]

    move_1_name = f"Move-To cube{cube_ID}"
    move_1_outcome = f"Cube{cube_ID} reached"

    pick_name = f"Pick cube{cube_ID}"
    pick_outcome = f"Cube{cube_ID} in Hand"

    move_2_name = "Move-To delivery"
    move_2_outcome = "Delivery pose reached"

    place_name = f"Place cube{cube_ID}"
    place_outcome = f"Cube{cube_ID} placed"

    recharge_name = "Recharge!"
    recharge_condition = "Battery below 20%"

    search_name = "Search Cubes"
    search_outcome = "All cubes found"

    dock_name = "Dock"
    dock_outcome = "Dock pose reached"

    goal_dictionary = {
        # "search": [search_name, search_IDs],
        "move_1": [move_1_name, cube_ID],
        "pick": [pick_name, cube_ID],
        "move_2": [move_2_name, delivery],
        "place": [place_name, place_pose],
        "dock": [dock_name, dock_pose],
        "recharge": [recharge_name, None],
    }

    outcome_dictionary = {
        # "search": search_outcome,
        "move_1": move_1_outcome,
        "pick": pick_outcome,
        "move_2": move_2_outcome,
        "place": place_outcome,
        "dock": dock_outcome,
        "recharge": recharge_condition,
    }

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["SUCCESS"])

    with sm:
        # smach.StateMachine.add(
        #     search_name,
        #     reactive_states.Search(
        #         name=search_name,
        #         locations=cube_locations,
        #         IDs=search_IDs,
        #         outcomes=[search_outcome, "RUNNING", "FAILURE", recharge_condition],
        #     ),
        #     transitions={
        #         search_outcome: move_1_name,
        #         "RUNNING": search_name,
        #         "FAILURE": "IDLE",
        #         recharge_condition: recharge_name,
        #     },
        # )

        smach.StateMachine.add(
            move_1_name,
            reactive_states.Move(
                name=move_1_name,
                goal_ID=cube_ID,
                ref_frame="map",
                goal_pose=None,
                # outcomes=[move_1_outcome, "RUNNING", "FAILURE"],
                outcomes=[move_1_outcome, "RUNNING", "FAILURE", recharge_condition],
            ),
            transitions={
                move_1_outcome: pick_name,
                "RUNNING": move_1_name,
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            pick_name,
            reactive_states.Pick(
                name=pick_name,
                goal_ID=cube_ID,
                goal_pose=None,
                # outcomes=[pick_outcome, "RUNNING", "FAILURE"],
                outcomes=[pick_outcome, "RUNNING", "FAILURE", recharge_condition],
            ),
            transitions={
                pick_outcome: move_2_name,
                "RUNNING": pick_name,
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            move_2_name,
            reactive_states.Move(
                name=move_2_name,
                goal_ID=None,
                ref_frame="map",
                goal_pose=delivery,
                # outcomes=[move_2_outcome, "RUNNING", "FAILURE"],
                outcomes=[move_2_outcome, "RUNNING", "FAILURE", recharge_condition],
            ),
            transitions={
                move_2_outcome: place_name,
                "RUNNING": move_2_name,
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            place_name,
            reactive_states.Place(
                name=place_name,
                goal_ID=cube_ID,
                place_target=place_target,
                goal_pose=place_pose,
                # outcomes=[place_outcome, "RUNNING", "FAILURE"],
                outcomes=[place_outcome, "RUNNING", "FAILURE", recharge_condition],
            ),
            transitions={
                # place_outcome: "SUCCESS",
                place_outcome: dock_name,
                "RUNNING": place_name,
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            dock_name,
            reactive_states.Dock(
                name=dock_name,
                target_pose=dock_pose,
                # outcomes=[dock_outcome, "RUNNING", "FAILURE"],
                outcomes=[dock_outcome, "RUNNING", "FAILURE", recharge_condition],
            ),
            transitions={
                # dock_outcome: "SUCCESS",
                dock_outcome: "Success",
                "RUNNING": dock_name,
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            recharge_name,
            reactive_states.Recharge(
                name=recharge_name,
                outcomes=["SUCCESS", "RUNNING"],
            ),
            transitions={
                "RUNNING": recharge_name,
                "SUCCESS": "IDLE",
            },
        )

        smach.StateMachine.add(
            "IDLE",
            reactive_states.IDLE(
                name="IDLE",
                goal_dict=goal_dictionary,
                out_dict=outcome_dictionary,
                outcomes=[
                    "Restart the task",
                    "RUNNING",
                ],
            ),
            transitions={
                "Restart the task": move_1_name,
                # search_outcome: move_1_name,
                move_1_outcome: pick_name,
                pick_outcome: move_2_name,
                move_2_outcome: place_name,
                # place_outcome: "SUCCESS",
                place_outcome: dock_name,
                # dock_outcome: "SUCCESS",
                dock_outcome: "Success",
                recharge_condition: recharge_name,
                "RUNNING": "IDLE",
            },
        )

        # Note: comment this if you wish to count the number of states and transitions!
        # If so, rename the IDLE and last state transitions from 'Success' to 'SUCCESS'
        smach.StateMachine.add(
            "Success",
            reactive_states.Dummy(outcomes=["SUCCESS"]),
            transitions={"SUCCESS": "SUCCESS"},
        )

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer("server_name", sm, "/SM_ROOT")
    sis.start()

    print("Number of states: ", len(list(sm._states.keys())) + 1)
    n_transitions = 0
    for state in sm._transitions.keys():
        n_transitions += len(list(sm._transitions[state].keys()))

    print("Number of transitions: ", n_transitions)

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
        reactive_state_machine(2, False)
        print("Usage: <node> arg1")
        print("arg1: terminal OR sequence OR fallback OR connected")
    else:
        reactive_state_machine(2, sys.argv[1])
