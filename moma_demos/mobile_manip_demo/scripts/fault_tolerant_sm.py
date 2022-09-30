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


def reactive_state_machine():

    rospy.init_node("state_machine")

    cube_ID = 2

    # Parameters
    delivery = rospy.get_param("moma_demo/delivery_station")
    place_target = rospy.get_param("moma_demo/place_pose")
    place_pose = get_place_pose(np.array(delivery), np.array(place_target))
    dock_pose = rospy.get_param("moma_demo/inspection_station")
    cube_locations = rospy.get_param("moma_demo/search_waypoints")
    search_IDs = [0, 1, 2]

    task_type = rospy.get_param("moma_demo/experiment")
    visualization_only = rospy.get_param("moma_demo/visualization_only")

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
    }
    if task_type == 2 or task_type == 3:
        goal_dictionary["recharge"] = [recharge_name, None]
    if task_type == 3:
        goal_dictionary["dock"] = [dock_name, dock_pose]

    outcome_dictionary = {
        # "search": search_outcome,
        "move_1": move_1_outcome,
        "pick": pick_outcome,
        "move_2": move_2_outcome,
        "place": place_outcome,
    }
    if task_type == 2 or task_type == 3:
        outcome_dictionary["recharge"] = recharge_condition
    if task_type == 3:
        outcome_dictionary["dock"] = dock_outcome

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

        # Move Action 1
        move_1_tr = {
            move_1_outcome: pick_name,
            "RUNNING": move_1_name,
            "FAILURE": "IDLE",
        }
        move_1_out = [move_1_outcome, "RUNNING", "FAILURE"]
        if task_type == 2 or task_type == 3:
            move_1_tr[recharge_condition] = recharge_name
            move_1_out.append(recharge_condition)

        smach.StateMachine.add(
            move_1_name,
            reactive_states.Move(
                name=move_1_name,
                goal_ID=cube_ID,
                ref_frame="map",
                goal_pose=None,
                outcomes=move_1_out,
            ),
            transitions=move_1_tr,
        )

        # Pick action
        pick_tr = {
            pick_outcome: move_2_name,
            "RUNNING": pick_name,
            "FAILURE": "IDLE",
        }
        pick_out = [pick_outcome, "RUNNING", "FAILURE"]
        if task_type == 2 or task_type == 3:
            pick_tr[recharge_condition] = recharge_name
            pick_out.append(recharge_condition)

        smach.StateMachine.add(
            pick_name,
            reactive_states.Pick(
                name=pick_name,
                goal_ID=cube_ID,
                goal_pose=None,
                outcomes=pick_out,
            ),
            transitions=pick_tr,
        )

        # Move Action 2
        move_2_tr = {
            move_2_outcome: place_name,
            "RUNNING": move_2_name,
            "FAILURE": "IDLE",
        }
        move_2_out = [move_2_outcome, "RUNNING", "FAILURE"]
        if task_type == 2 or task_type == 3:
            move_2_tr[recharge_condition] = recharge_name
            move_2_out.append(recharge_condition)

        smach.StateMachine.add(
            move_2_name,
            reactive_states.Move(
                name=move_2_name,
                goal_ID=None,
                ref_frame="map",
                goal_pose=delivery,
                outcomes=move_2_out,
            ),
            transitions=move_2_tr,
        )

        # Place Action
        after_place = dock_name if task_type == 3 else "Success"  # "SUCCESS"
        place_tr = {
            place_outcome: after_place,
            "RUNNING": place_name,
            "FAILURE": "IDLE",
        }
        place_out = [place_outcome, "RUNNING", "FAILURE"]
        if task_type == 2 or task_type == 3:
            place_tr[recharge_condition] = recharge_name
            place_out.append(recharge_condition)

        smach.StateMachine.add(
            place_name,
            reactive_states.Place(
                name=place_name,
                goal_ID=cube_ID,
                place_target=place_target,
                goal_pose=place_pose,
                outcomes=place_out,
            ),
            transitions=place_tr,
        )

        # Dock Action
        if task_type == 3:
            smach.StateMachine.add(
                dock_name,
                reactive_states.Dock(
                    name=dock_name,
                    target_pose=dock_pose,
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

        if task_type == 2 or task_type == 3:
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

        # IDLE
        idle_tr = {
            "Restart the task": move_1_name,
            # search_outcome: move_1_name,
            move_1_outcome: pick_name,
            pick_outcome: move_2_name,
            move_2_outcome: place_name,
            place_outcome: after_place,
            "RUNNING": "IDLE",
        }
        if task_type == 2 or task_type == 3:
            idle_tr[recharge_condition] = recharge_name
        if task_type == 3:
            idle_tr[dock_outcome] = "Success"  # "SUCCESS"

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
            transitions=idle_tr,
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

    if not visualization_only:
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
    reactive_state_machine()
