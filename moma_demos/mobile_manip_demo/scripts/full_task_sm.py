#!/usr/bin/env python
"""Build the fault tolerant FSM to fetch 5 items, recharge and dock."""

import os
import sys

from mobile_manip_demo import reactive_states
import networkx as nx
import numpy as np
import rospy
import smach
import smach_ros


def reactive_state_machine(cube_ID: int, visualize=False):
    """Note: this is not working but just for visualization."""

    rospy.init_node("state_machine")

    recharge_name = "Recharge!"
    recharge_condition = "Battery below 20%"

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["SUCCESS"])

    with sm:
        smach.StateMachine.add(
            recharge_name,
            reactive_states.Dummy(
                outcomes=["SUCCESS", "RUNNING"],
            ),
            transitions={
                "RUNNING": recharge_name,
                "SUCCESS": "IDLE",
            },
        )

        smach.StateMachine.add(
            "Search Cubes",
            reactive_states.Dummy(
                outcomes=[
                    "All cubes found",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "All cubes found": "Move-To cube1",
                "RUNNING": "Search Cubes",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        # CUBE1
        smach.StateMachine.add(
            "Move-To cube1",
            reactive_states.Dummy(
                outcomes=[
                    "Cube1 reached",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube1 reached": "Pick cube1",
                "RUNNING": "Move-To cube1",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Pick cube1",
            reactive_states.Dummy(
                outcomes=[
                    "Cube1 in Hand",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube1 in Hand": "Move-To delivery1",
                "RUNNING": "Pick cube1",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Move-To delivery1",
            reactive_states.Dummy(
                outcomes=[
                    "Delivery1 pose reached",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Delivery1 pose reached": "Place cube1",
                "RUNNING": "Move-To delivery1",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Place cube1",
            reactive_states.Dummy(
                outcomes=[
                    "Cube1 placed",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube1 placed": "Move-To cube2",
                "RUNNING": "Place cube1",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        # CUBE2
        smach.StateMachine.add(
            "Move-To cube2",
            reactive_states.Dummy(
                outcomes=[
                    "Cube2 reached",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube2 reached": "Pick cube2",
                "RUNNING": "Move-To cube2",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Pick cube2",
            reactive_states.Dummy(
                outcomes=[
                    "Cube2 in Hand",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube2 in Hand": "Move-To delivery2",
                "RUNNING": "Pick cube2",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Move-To delivery2",
            reactive_states.Dummy(
                outcomes=[
                    "Delivery2 pose reached",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Delivery2 pose reached": "Place cube2",
                "RUNNING": "Move-To delivery2",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Place cube2",
            reactive_states.Dummy(
                outcomes=[
                    "Cube2 placed",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube2 placed": "Move-To cube3",
                "RUNNING": "Place cube2",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        # CUBE3
        smach.StateMachine.add(
            "Move-To cube3",
            reactive_states.Dummy(
                outcomes=[
                    "Cube3 reached",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube3 reached": "Pick cube3",
                "RUNNING": "Move-To cube3",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Pick cube3",
            reactive_states.Dummy(
                outcomes=[
                    "Cube3 in Hand",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube3 in Hand": "Move-To delivery3",
                "RUNNING": "Pick cube3",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Move-To delivery3",
            reactive_states.Dummy(
                outcomes=[
                    "Delivery3 pose reached",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Delivery3 pose reached": "Place cube3",
                "RUNNING": "Move-To delivery3",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Place cube3",
            reactive_states.Dummy(
                outcomes=[
                    "Cube3 placed",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube3 placed": "Move-To cube4",
                "RUNNING": "Place cube3",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        # CUBE4
        smach.StateMachine.add(
            "Move-To cube4",
            reactive_states.Dummy(
                outcomes=[
                    "Cube4 reached",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube4 reached": "Pick cube4",
                "RUNNING": "Move-To cube4",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Pick cube4",
            reactive_states.Dummy(
                outcomes=[
                    "Cube4 in Hand",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube4 in Hand": "Move-To delivery4",
                "RUNNING": "Pick cube4",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Move-To delivery4",
            reactive_states.Dummy(
                outcomes=[
                    "Delivery4 pose reached",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Delivery4 pose reached": "Place cube4",
                "RUNNING": "Move-To delivery4",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Place cube4",
            reactive_states.Dummy(
                outcomes=[
                    "Cube4 placed",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube4 placed": "Move-To cube5",
                "RUNNING": "Place cube4",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        # CUBE5
        smach.StateMachine.add(
            "Move-To cube5",
            reactive_states.Dummy(
                outcomes=[
                    "Cube5 reached",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube5 reached": "Pick cube5",
                "RUNNING": "Move-To cube5",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Pick cube5",
            reactive_states.Dummy(
                outcomes=[
                    "Cube5 in Hand",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube5 in Hand": "Move-To delivery5",
                "RUNNING": "Pick cube5",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Move-To delivery5",
            reactive_states.Dummy(
                outcomes=[
                    "Delivery5 pose reached",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Delivery5 pose reached": "Place cube5",
                "RUNNING": "Move-To delivery5",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        smach.StateMachine.add(
            "Place cube5",
            reactive_states.Dummy(
                outcomes=[
                    "Cube5 placed",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Cube5 placed": "Dock",
                "RUNNING": "Place cube5",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        # DOCK
        smach.StateMachine.add(
            "Dock",
            reactive_states.Dummy(
                outcomes=[
                    "Dock pose reached",
                    "RUNNING",
                    "FAILURE",
                    recharge_condition,
                ],
            ),
            transitions={
                "Dock pose reached": "SUCCESS",
                "RUNNING": "Dock",
                "FAILURE": "IDLE",
                recharge_condition: recharge_name,
            },
        )

        # IDLE
        smach.StateMachine.add(
            "IDLE",
            reactive_states.Dummy(
                outcomes=[
                    "Restart the task",
                    "All cubes found",
                    "Cube1 reached",
                    "Cube1 in Hand",
                    "Delivery1 pose reached",
                    "Cube1 placed",
                    "Cube2 reached",
                    "Cube2 in Hand",
                    "Delivery2 pose reached",
                    "Cube2 placed",
                    "Cube3 reached",
                    "Cube3 in Hand",
                    "Delivery3 pose reached",
                    "Cube3 placed",
                    "Cube4 reached",
                    "Cube4 in Hand",
                    "Delivery4 pose reached",
                    "Cube4 placed",
                    "Cube5 reached",
                    "Cube5 in Hand",
                    "Delivery5 pose reached",
                    "Cube5 placed",
                    "Dock pose reached",
                    recharge_condition,
                    "RUNNING",
                ],
            ),
            transitions={
                "Restart the task": "Search Cubes",
                "All cubes found": "Move-To cube1",
                "Cube1 reached": "Pick cube1",
                "Cube1 in Hand": "Move-To delivery1",
                "Delivery1 pose reached": "Place cube1",
                "Cube1 placed": "Move-To cube2",
                "Cube2 reached": "Pick cube2",
                "Cube2 in Hand": "Move-To delivery2",
                "Delivery2 pose reached": "Place cube2",
                "Cube2 placed": "Move-To cube3",
                "Cube3 reached": "Pick cube3",
                "Cube3 in Hand": "Move-To delivery3",
                "Delivery3 pose reached": "Place cube3",
                "Cube3 placed": "Move-To cube4",
                "Cube4 reached": "Pick cube4",
                "Cube4 in Hand": "Move-To delivery4",
                "Delivery4 pose reached": "Place cube4",
                "Cube4 placed": "Move-To cube5",
                "Cube5 reached": "Pick cube5",
                "Cube5 in Hand": "Move-To delivery5",
                "Delivery5 pose reached": "Place cube5",
                "Cube5 placed": "Dock",
                "Dock pose reached": "SUCCESS",
                recharge_condition: recharge_name,
                "RUNNING": "IDLE",
            },
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


def compute_graph_elements():
    script_folder = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(script_folder)
    dot_folder = os.path.join(root, "dot")

    full_fsm = nx.DiGraph(
        nx.drawing.nx_pydot.read_dot(os.path.join(dot_folder, "full_FSM.dot"))
    )
    print(f"Full task FSM: {full_fsm}")


if __name__ == "__main__":
    reactive_state_machine(2, True)
