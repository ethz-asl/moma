#!/usr/bin/env python3

import rospy
import smach_ros
from moma_mission.core import StateMachineRos
from moma_mission.missions.piloting.manipulation import ValveManipulationUrdfState
from moma_mission.states.gripper import GripperControl
from moma_mission.states.path_visitor import PathVisitorState

rospy.init_node('piloting_sim')


# Build the state machine
state_machine = StateMachineRos(outcomes=['Success', 'Failure'])
with state_machine:
    state_machine.add('OPEN_GRIPPER', GripperControl, transitions={'Completed': 'MODEL_VALVE',
                                                                   'Failure': 'Failure'})

    state_machine.add('MODEL_VALVE', ValveManipulationUrdfState, transitions={'Completed': 'APPROACH_VALVE',
                                                                              'Failure': 'Failure'})

    state_machine.add('APPROACH_VALVE', PathVisitorState, transitions={'Completed': 'GRASP_VALVE',
                                                                       'Failure': 'Failure'})

    state_machine.add('GRASP_VALVE', PathVisitorState, transitions={'Completed': 'CLOSE_GRIPPER',
                                                                       'Failure': 'Failure'})

    state_machine.add('CLOSE_GRIPPER', GripperControl, transitions={'Completed': 'MANIPULATE_VALVE',
                                                                    'Failure': 'MANIPULATE_VALVE'})

    state_machine.add('MANIPULATE_VALVE', PathVisitorState, transitions={'Completed': 'RELEASE_GRIPPER',
                                                                         'Failure': 'Failure'})

    state_machine.add('RELEASE_GRIPPER', GripperControl, transitions={'Completed': 'BACKOFF_VALVE',
                                                                      'Failure': 'Failure'})

    state_machine.add('BACKOFF_VALVE', PathVisitorState, transitions={'Completed': 'Success',
                                                                         'Failure': 'Failure'})

# Create and start the introspection server
introspection_server = smach_ros.IntrospectionServer('piloting_sim_mission_server', state_machine, '/mission_planner')
introspection_server.start()

# Execute state machine
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

# Wait for ctrl-c to stop the application
introspection_server.stop()
