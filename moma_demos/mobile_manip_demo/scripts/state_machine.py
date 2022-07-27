"""Build the FSM."""

import os
import sys

from mobile_manip_demo import states
import rospy
import smach
import smach_ros
from state_machine_tutorial.state_machine_generation import utils as api


def main():

    rospy.init_node("smach_example_state_machine")

    current_state = [None] * (len(states.WorldState))
    current_state[states.WorldState.POSE] = (0.0, 0.0, 0.0)
    current_state[states.WorldState.HAS_CUBE] = False
    current_state[states.WorldState.CUBE_PLACED] = False
    current_state[states.WorldState.BATTERY] = 100

    # Plan to reproduce:
    """
    Found plan:
    nav-in-reach {'current_pos': 'origin', 'goal_pos': 'cube1', 'gid': 'grasp0', 'rob': 'robot1'}
    grasp {'obj': 'cube1', 'gid': 'grasp0', 'rob': 'robot1'}
    nav-in-reach {'current_pos': 'cube1', 'goal_pos': 'position_sample_1', 'gid': 'grasp0', 'rob': 'robot1'}
    place-on-220414113908 {'cupboard': 'cupboard', 'cube1': 'cube1', 'robot1': 'robot1', 'grasp0': 'grasp0', 'position_sample_1': 'position_sample_1'}
    """

    example = {
        0: {
            "name": "nav-in-reach",
            "parameters": [(0.0, 0.0, 0.0), "cube1", ("link0", "top")],
            "preconditions": {},
            "postconditions": {"in-reach": ["cube1"]},
        },
        1: {
            "name": "grasp",
            "parameters": ["cube1", ("link0", "top")],
            "preconditions": {"in-reach": ["cube1"]},
            "postconditions": {"in-hand": ["cube1"]},
        },
        2: {
            "name": "nav-in-reach",
            "parameters": ["cube1", (0.174, 1.343, 0.303), ("link0", "top")],
            "preconditions": {},
            "postconditions": {"in-reach": [(0.174, 1.343, 0.303)]},
        },
        3: {
            "name": "place-on",
            "parameters": [
                "cupboard",
                "cube1",
                ("link0", "top"),
                (0.174, 1.343, 0.303),
            ],
            "preconditions": {
                "in-reach": [(0.174, 1.343, 0.303)],
                "in-hand": ["cube1"],
            },
            "postconditions": {"on": ["cupboard", "cube1"]},
        },
    }

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["SUCCESS"])

    # build reactive FSM
    api.build_reactive_SM(
        sm=sm,
        preprocessed_knowledge=example,
        world_state=current_state,
        verbose=False,
        generation_function=states.state_linker,
    )

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer("server_name", sm, "/SM_ROOT")
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
