#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from moma_mission.core.state_ros import StateRos


class StateMachineRos(smach.StateMachine):
    def __init__(self, outcomes=["Success", "Failure"], input_keys=[], output_keys=[]):
        smach.StateMachine.__init__(
            self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys
        )

    def add(self, label, state, transitions=None, remapping=None, constants=None):
        if isinstance(state, (StateMachineRos, smach.StateMachine)):
            super(StateMachineRos, self).add(
                label, state, transitions, remapping, constants
            )
        elif issubclass(state, StateRos):
            super(StateMachineRos, self).add(
                label, state(ns=label), transitions, remapping, constants
            )
        else:
            super(StateMachineRos, self).add(
                label, state, transitions, remapping, constants
            )


if __name__ == "__main__":
    state_machine = StateMachineRos(outcomes=["Success", "Failure"])
    with state_machine:
        state_machine.add(
            "STATE_A",
            StateRos,
            transitions={"Completed": "STATE_B", "Failure": "Failure"},
        )
        state_machine.add(
            "STATE_B",
            StateRos,
            transitions={"Completed": "SEQUENCE", "Failure": "Failure"},
        )
        sequence = StateMachineRos(outcomes=["Sequence_DONE", "Failure"])
        with sequence:
            sequence.add(
                "STATE_C",
                StateRos,
                transitions={"Completed": "Sequence_DONE", "Failure": "Failure"},
            )
        state_machine.add(
            "SEQUENCE",
            sequence,
            transitions={"Sequence_DONE": "Success", "Failure": "Failure"},
        )

    # run
    rospy.init_node("test_mission")
    introspection_server = smach_ros.IntrospectionServer(
        "mission_server", state_machine, "/mission"
    )
    introspection_server.start()
    outcome = state_machine.execute()
    rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))
    introspection_server.stop()
