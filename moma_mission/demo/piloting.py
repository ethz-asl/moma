#!/usr/bin/env python
import rospy
import smach
import smach_ros
from moma_mission.missions.piloting.sequences import *

rospy.init_node('piloting_mission')

# Init data
Valve.init_from_ros()
Frames.init_from_ros()
Frames.print_summary()

# Build the state machine
state_machine = StateMachineRos(outcomes=['Success', 'Failure'])
with state_machine:
    state_machine.add('HOME_ROBOT_START',
                      homing_sequence_factory(),
                      transitions={'Success': 'REACH_DETECTION_HOTSPOT', 'Failure': 'Failure'})

    state_machine.add('REACH_DETECTION_HOTSPOT',
                      NavigationState,
                      transitions={'Completed': 'DETECTION',
                                   'Failure': 'Failure'})

    state_machine.add('DETECTION',
                      detection_sequence_factory(),
                      transitions={'Success': 'LATERAL_MANIPULATION',
                                   'Failure': 'Failure'})

    state_machine.add('LATERAL_MANIPULATION',
                      lateral_manipulation_sequence_factory(),
                      transitions={'Success': 'HOMING_FINAL',
                                   'Failure': 'Failure'})

    homing_sequence_final = StateMachineRos(outcomes=['Success', 'Failure'])
    with homing_sequence_final:
        homing_sequence_final.add('OPEN_GRIPPER', GripperUSB,
                                  transitions={'Completed': 'HOME_ROBOT_FINAL',
                                               'Failure': 'Failure'})

        homing_sequence_final.add('HOME_ROBOT_FINAL', JointsConfigurationAction,
                                   transitions={'Completed': 'Success',
                                                'Failure': 'Failure'})

    state_machine.add('HOMING_FINAL',
                  homing_sequence_final,
                  transitions={'Success': 'Success',
                               'Failure': 'Failure'})

# Create and start the introspection server
introspection_server = smach_ros.IntrospectionServer('piloting_mission_server', state_machine, '/mission_planner')
introspection_server.start()

# Execute state machine
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

# Wait for ctrl-c to stop the application
introspection_server.stop()