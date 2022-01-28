#!/usr/bin/env python3

"""
Johnson & Johnson Project Demo
A medical autoinjector perception and pickup task
"""

import rospy
import smach_ros
from moma_mission.core import StateMachineRos
from moma_mission.states.observation import SphericalSamplerState
from moma_mission.states.model_fit_autoinjector import ModelFitAutoinjectorState
from moma_mission.states.gripper import GripperControl
from moma_mission.states.transform_visitor import TransformVisitorState

rospy.init_node('johnson')


# Build the state machine
state_machine = StateMachineRos(outcomes=['Success', 'Failure'])
with state_machine:
    # state_machine.add('OPEN_GRIPPER', GripperControl, transitions={'Completed': 'OBSERVATION_POSE',
    #                                                                'Failure': 'Failure'})
    #
    state_machine.add('OBSERVATION_POSE', SphericalSamplerState, transitions={'Completed': 'OBSERVATION_APPROACH',
                                                                              'Failure': 'Failure'})

    state_machine.add('OBSERVATION_APPROACH', TransformVisitorState, transitions={'Completed': 'MODEL_FIT_AUTOINJECTOR',
                                                                                  'Failure': 'OBSERVATION_POSE'})

    state_machine.add('MODEL_FIT_AUTOINJECTOR', ModelFitAutoinjectorState, transitions={'Completed': 'OBJECT_APPROACH',
                                                                                        'Failure': 'OBSERVATION_POSE'})

    state_machine.add('OBJECT_APPROACH', TransformVisitorState, transitions={'Completed': 'Success', #'CLOSE_GRIPPER',
                                                                                          'Failure': 'OBSERVATION_POSE'})

    # state_machine.add('CLOSE_GRIPPER', GripperControl, transitions={'Completed': 'TARGET_POSE',
    #                                                                 'Failure': 'Failure'})
    #
    # state_machine.add('TARGET_POSE', JointsConfigurationAction, transitions={'Completed': 'Success',
    #                                                                          'Failure': 'Failure'})

# Create and start the introspection server
introspection_server = smach_ros.IntrospectionServer('autoinjector_mission_server', state_machine, '/mission_planner')
introspection_server.start()

# Execute state machine
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

# Wait for ctrl-c to stop the application
introspection_server.stop()
