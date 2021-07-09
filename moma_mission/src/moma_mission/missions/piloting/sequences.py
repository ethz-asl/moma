# !/usr/bin/env python
import rospy
import smach
import smach_ros

from moma_mission.core import StateMachineRos
from moma_mission.missions.piloting.states import *
from moma_mission.states.gripper import GripperUSB
from moma_mission.states.manipulation import JointsConfigurationAction
from moma_mission.states.detection import DetectionState


def homing_sequence_factory():
    homing_sequence = StateMachineRos(outcomes=['Success', 'Failure'])
    with homing_sequence:
        homing_sequence.add('OPEN_GRIPPER', GripperUSB,
                            transitions={'Completed': 'HOME_ROBOT',
                                         'Failure': 'Failure'})

        homing_sequence.add('HOME_ROBOT', JointsConfigurationAction,
                            transitions={'Completed': 'Success',
                                         'Failure': 'Failure'})
    return homing_sequence


def detection_sequence_factory():
    detection_sequence = StateMachineRos(outcomes=['Success', 'Failure'])
    with detection_sequence:
        detection_sequence.add('REACH_VIEWPOINT', JointsConfigurationAction,
                               transitions={'Completed': 'FETCH_DETECTION',
                                            'Failure': 'Failure'})

        detection_sequence.add('FETCH_DETECTION', DetectionState,
                               transitions={'Completed': 'HOME_ROBOT',
                                            'Failure': 'Failure'})

        detection_sequence.add('HOME_ROBOT', JointsConfigurationAction,
                               transitions={'Completed': 'Success',
                                            'Failure': 'Failure'})
    return detection_sequence


def lateral_manipulation_sequence_factory():
    lateral_manipulation_sequence = StateMachineRos(outcomes=['Success', 'Failure'])
    with lateral_manipulation_sequence:
        lateral_manipulation_sequence.add('LATERAL_GRASP', LateralGraspState,
                                          transitions={'Completed': 'CLOSE_GRIPPER',
                                                       'Failure': 'Failure'})

        lateral_manipulation_sequence.add('CLOSE_GRIPPER', GripperUSB,
                                          transitions={'Completed': 'MANIPULATE_VALVE',
                                                       'Failure': 'Failure'})

        lateral_manipulation_sequence.add('MANIPULATE_VALVE', LateralManipulation,
                                          transitions={'Completed': 'OPEN_GRIPPER',
                                                       'Failure': 'Failure'})

        lateral_manipulation_sequence.add('OPEN_GRIPPER', GripperUSB,
                                          transitions={'Completed': 'RESET_LATERAL_GRASP',
                                                       'Failure': 'Failure'})

        lateral_manipulation_sequence.add('RESET_LATERAL_GRASP', PostLateralGraspState,
                                          transitions={'Completed': 'LATERAL_GRASP',
                                                       'Failure': 'Failure',
                                                       'FullRotationDone': 'Success'})
    return lateral_manipulation_sequence
