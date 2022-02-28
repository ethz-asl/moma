#!/usr/bin/env python
import sys
import rospy
import smach
import smach_ros
from moma_mission.core.state_ros import *
from moma_mission.missions.piloting.states import *
from moma_mission.missions.piloting.sequences import *


# Init ros
rospy.init_node("piloting_demo")

# Print frames info
Frames.print_summary()

# Build the state machine
state_machine = StateMachineRos(outcomes=['Success', 'Failure'])

try:
    with state_machine:
        state_machine.add('SETUP', SetUp, transitions={'Completed': 'HOME_ROBOT_START',
                                                       'Failure': 'Failure'})

        state_machine.add('HOME_ROBOT_START', homing_sequence_factory(), transitions={'Success': 'IDLE',
                                                                                      'Failure': 'Failure'})

        state_machine.add('IDLE', Idle, transitions={'ExecuteInspectionPlan': 'WAYPOINT_FOLLOWING',
                                                     'ExecuteManipulationPlan': 'REACH_DETECTION_HOTSPOT',
                                                     'Failure': 'Failure'})
        
        state_machine.add('REACH_DETECTION_HOTSPOT', NavigationState, transitions={'Completed': 'DETECTION',
                                                                                   'Failure': 'Failure'})

        state_machine.add('WAYPOINT_FOLLOWING', WaypointNavigationState,  transitions={'Completed': 'Success',
                                                                                       'Failure': 'Failure',
                                                                                       'NextWaypoint': 'WAYPOINT_FOLLOWING'})

        state_machine.add('DETECTION', detection_sequence_factory(), transitions={'Success': 'LATERAL_MANIPULATION',
                                                                                  'Failure': 'Failure'})

        state_machine.add('LATERAL_MANIPULATION', lateral_manipulation_sequence_factory(), transitions={'Success': 'HOMING_FINAL',
                                                                                                        'Failure': 'Failure'})

        homing_sequence_final = StateMachineRos(
            outcomes=['Success', 'Failure'])

        with homing_sequence_final:
            homing_sequence_final.add('OPEN_GRIPPER', GripperControl, transitions={'Completed': 'HOME_ROBOT_FINAL',
                                                                                   'Failure': 'Failure'})

            homing_sequence_final.add('HOME_ROBOT_FINAL', JointsConfigurationAction, transitions={'Completed': 'Success',
                                                                                                  'Failure': 'Failure'})

        state_machine.add('HOMING_FINAL', homing_sequence_final, transitions={'Success': 'Success',
                                                                              'Failure': 'Failure'})
except Exception as exc:
    rospy.logerr(exc)
    sys.exit(0)

# Create and start the introspection server
introspection_server = smach_ros.IntrospectionServer(
    'piloting_mission_server', state_machine, '/mission_planner')
introspection_server.start()

# Execute state machine
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

# Wait for ctrl-c to stop the application
introspection_server.stop()
