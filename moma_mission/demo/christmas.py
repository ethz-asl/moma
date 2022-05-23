#!/usr/bin/env python
"""
Christmas 2021 Demo
A simple door opening task, given a known grasp preset
"""
import rospy
import smach
import smach_ros
from moma_mission.core import StateMachineRos
from moma_mission.missions.door_opening.states import DoorManipulation
from moma_mission.states.gripper import GripperControl
from moma_mission.states.manipulation import JointsConfigurationAction

rospy.init_node("christmas_demo")


# Build the state machine
state_machine = StateMachineRos(outcomes=["Success", "Failure"])
with state_machine:

    state_machine.add(
        "OPEN_GRIPPER",
        GripperControl,
        transitions={"Completed": "HOME_ROBOT", "Failure": "Failure"},
    )

    state_machine.add(
        "HOME_ROBOT",
        JointsConfigurationAction,
        transitions={"Completed": "GRASP_PRESET", "Failure": "Failure"},
    )

    state_machine.add(
        "GRASP_PRESET",
        JointsConfigurationAction,
        transitions={"Completed": "CLOSE_GRIPPER", "Failure": "Failure"},
    )

    state_machine.add(
        "CLOSE_GRIPPER",
        GripperControl,
        transitions={"Completed": "DOOR_MANIPULATION", "Failure": "Failure"},
    )

    state_machine.add(
        "DOOR_MANIPULATION",
        DoorManipulation,
        transitions={"Completed": "Success", "Failure": "Failure"},
    )

    # state_machine.add('OPEN_GRIPPER_FINAL', GripperControl, transitions={'Completed': 'HOME_ROBOT_FINAL',
    #                                                                      'Failure': 'Failure'})

    # state_machine.add('HOME_ROBOT_FINAL', JointsConfigurationAction, transitions={'Completed': 'Success',
    #                                                                               'Failure': 'Failure'})

# Create and start the introspection server
introspection_server = smach_ros.IntrospectionServer(
    "piloting_mission_server", state_machine, "/mission_planner"
)
introspection_server.start()

# Execute state machine
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

# Wait for ctrl-c to stop the application
introspection_server.stop()
