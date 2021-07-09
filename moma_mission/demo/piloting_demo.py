#!/usr/bin/env python

import rospy
import smach
import smach_ros
from kinova_valve_opening.states import *

"""
Implementation of the state machine for the PILOTING demo
"""

rospy.init_node('piloting_mission')

# Init data
valve_traj_data.init_from_ros()
valve_traj_data.print_summary()

# Build the state machine
state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
with state_machine:
    #######################################################################
    #                      Homing Subsequence
    ####################################################################### 
    
    homing_sequence = smach.StateMachine(outcomes=['Success', 'Failure'])
    with homing_sequence:

        smach.StateMachine.add('OPEN_GRIPPER', GripperUSB(ns='open_gripper_usb'),
                                transitions={'Completed': 'HOME_ROBOT',
                                             'Failure': 'Failure'})

        smach.StateMachine.add('HOME_ROBOT', JointsConfigurationAction(ns="home_action"),
                                transitions={'Completed': 'Success',
                                             'Failure': 'Failure'})
        
        # smach.StateMachine.add('HOME_ROBOT', HomePoseJointConfiguration(ns="home_pose_joints"),
        #                         transitions={'Completed': 'Success',
        #                                      'Failure': 'Failure'})

        # smach.StateMachine.add('HOME_ROBOT', HomePose(ns="home_pose"),
        #                         transitions={'Completed': 'Success',
        #                                      'Failure': 'Failure'})

    smach.StateMachine.add('HOME_ROBOT_START', homing_sequence,
                           transitions={'Success': 'REACH_DETECTION_HOTSPOT', 'Failure': 'Failure'})

    #######################################################################
    #                      Navigation Subsequence
    ####################################################################### 
    smach.StateMachine.add('REACH_DETECTION_HOTSPOT', NavigationState(ns="navigation_state"),
                           transitions={'Completed': 'REACH_VIEWPOINT',
                                        'Aborted': 'Failure'})

    #######################################################################
    #                      Detection Subsequence
    ####################################################################### 
    smach.StateMachine.add('REACH_VIEWPOINT', JointsConfigurationAction(ns='reach_viewpoint'),
                           transitions={'Completed': 'FETCH_DETECTION',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('FETCH_DETECTION', DetectionState(ns='valve_detection'),
                           transitions={'Completed': 'HOME_ROBOT',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('HOME_ROBOT', JointsConfigurationAction(ns="home_action"),
                        transitions={'Completed': 'LATERAL_MANIPULATION_SEQUENCE',
                                     'Failure': 'Failure'})

    #######################################################################
    #                      Lateral Grasp Subsequence
    ####################################################################### 
    lateral_grasp_sm = smach.StateMachine(outcomes=['Success', 'Failure'])
    with lateral_grasp_sm:
        smach.StateMachine.add('LATERAL_GRASP', LateralGraspState(ns='grasp'),
                                transitions={'Completed': 'CLOSE_GRIPPER',
                                'Failure': 'Failure'})

        smach.StateMachine.add('CLOSE_GRIPPER', GripperUSB(ns='close_gripper_usb'),
                                transitions={'Completed': 'MANIPULATE_VALVE',
                                             'Failure': 'Failure'})

        smach.StateMachine.add('MANIPULATE_VALVE', LateralManipulation(ns='manipulate_valve'),
                             transitions={'Completed': 'OPEN_GRIPPER',
                                          'Failure': 'Failure'})
  
  
        smach.StateMachine.add('OPEN_GRIPPER', GripperUSB(ns='open_gripper_usb'),
                             transitions={'Completed': 'RESET_LATERAL_GRASP',
                                          'Failure': 'Failure'})

        smach.StateMachine.add('RESET_LATERAL_GRASP', PostLateralGraspState(ns='grasp'),
                                transitions={'Completed': 'LATERAL_GRASP',
                                             'Failure': 'Failure',
                                             'FullRotationDone': 'Success'})
    

    smach.StateMachine.add('LATERAL_MANIPULATION_SEQUENCE', lateral_grasp_sm, transitions={'Success': 'HOME_ROBOT_END', 'Failure': 'Failure'})

    #######################################################################
    #                      Final Homing Sequence (same as initial)
    #######################################################################
    smach.StateMachine.add('HOME_ROBOT_END', homing_sequence, 
                           transitions={'Success': 'Success', 
                                        'Failure': 'Failure'})

# Create and start the introspection server
introspection_server = smach_ros.IntrospectionServer('mission_server', state_machine, '/mission_planner')
introspection_server.start()

# Execute state machine.
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

# Wait for ctrl-c to stop the application
introspection_server.stop()

# #######################################################################
# #                      Frontal Grasp Subsequence
# #######################################################################
# frontal_grasp_sm = smach.StateMachine(outcomes=['Success', 'Failure'])
# with frontal_grasp_sm:
#     smach.StateMachine.add('FRONTAL_GRASP', FrontalGraspState(ns='grasp'),
#                             transitions={'Completed': 'CLOSE_GRIPPER',
#                             'Failure': 'Failure'})

#     smach.StateMachine.add('CLOSE_GRIPPER', GripperControl(ns='close_gripper'),
#                            transitions={'Completed': 'MANIPULATE_VALVE',
#                                         'Failure': 'Failure'})

#     smach.StateMachine.add('MANIPULATE_VALVE', FrontalManipulation(ns='manipulate_valve'),
#                          transitions={'Completed': 'OPEN_GRIPPER',
#                                       'Failure': 'Failure'})

#     smach.StateMachine.add('OPEN_GRIPPER', GripperControl(ns='open_gripper'),
#                          transitions={'Completed': 'RESET_FRONTAL_GRASP',
#                                       'Failure': 'Failure'})

#     smach.StateMachine.add('RESET_FRONTAL_GRASP', PostFrontalGraspState(ns='grasp'),
#                             transitions={'Completed': 'FRONTAL_GRASP',
#                                          'Failure': 'Failure',
#                                          'FullRotationDone': 'Success'})
# smach.StateMachine.add('FRONTAL_MANIPULATION_SEQUENCE', frontal_grasp_sm, transitions={'Success': 'HOME_ROBOT_END', 'Failure': 'Failure'})
