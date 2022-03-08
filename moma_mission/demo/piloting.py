#!/usr/bin/env python
import sys
import rospy
import smach
import smach_ros
from moma_mission.core.state_ros import *
from moma_mission.missions.piloting.states import *
from moma_mission.missions.piloting.sequences import *
from moma_mission.states.observation import FOVSamplerState
from moma_mission.states.transform_visitor import TransformVisitorState
from moma_mission.states.path_visitor import PathVisitorState


# Init ros
rospy.init_node("piloting_demo")

# Init data
Valve.init_from_ros()
Frames.init_from_ros()
Frames.print_summary()

# Build the state machine
state_machine = StateMachineRos(outcomes=['Success', 'Failure'])

try:
    with state_machine:
        state_machine.add('SETUP',
                          SetUp,
                          transitions={'Completed': 'HOME_ROBOT_START',
                                       'Failure': 'Failure'})

        state_machine.add('HOME_ROBOT_START',
                          homing_sequence_factory(),
                          transitions={'Success': 'IDLE',
                                       'Failure': 'Failure'})

        state_machine.add('IDLE',
                          Idle,
                          transitions={'ExecuteInspectionPlan': 'WAYPOINT_FOLLOWING',
                                       'ExecuteManipulationPlan': 'REACH_DETECTION_HOTSPOT',
                                       'Failure': 'Failure'})

        state_machine.add('REACH_DETECTION_HOTSPOT',
                          NavigationState,
                          transitions={'Completed': 'DETECTION_DECISION',
                                       'Failure': 'Failure'})

        state_machine.add('WAYPOINT_FOLLOWING',
                          WaypointNavigationState,
                          transitions={'Completed': 'Success',
                                       'Failure': 'Failure',
                                       'NextWaypoint': 'WAYPOINT_FOLLOWING'})

        # Hacky, but avoids to define an almost empty class with additional outcomes
        state_machine.add('DETECTION_DECISION',
                          StateRos,
                          transitions={'Completed': 'OBSERVATION_POSE',
                                       'Failure': 'PLAN_URDF_VALVE'})

        state_machine.add('OBSERVATION_POSE',
                          FOVSamplerState,
                          transitions={'Completed': 'OBSERVATION_APPROACH',
                                       'Failure': 'Failure'})

        state_machine.add('OBSERVATION_APPROACH',
                          TransformVisitorState,
                          transitions={'Completed': 'MODEL_FIT_VALVE',
                                       'Failure': 'OBSERVATION_POSE'})

        state_machine.add('MODEL_FIT_VALVE',
                          ModelFitValveState,
                          transitions={'Completed': 'PLAN_MODEL_VALVE',
                                       'Failure': 'Failure',
                                       'NextDetection': 'OBSERVATION_POSE'})

        state_machine.add('PLAN_MODEL_VALVE',
                          ValveManipulationModelState,
                          transitions={'Completed': 'APPROACH_VALVE',
                                       'Failure': 'Failure'})

        state_machine.add('PLAN_URDF_VALVE',
                          ValveManipulationUrdfState,
                          transitions={'Completed': 'APPROACH_VALVE',
                                       'Failure': 'Failure'})

        state_machine.add('APPROACH_VALVE',
                          PathVisitorState,
                          transitions={'Completed': 'GRASP_VALVE',
                                       'Failure': 'DETECTION_DECISION'})

        state_machine.add('GRASP_VALVE',
                          PathVisitorState,
                          transitions={'Completed': 'CLOSE_GRIPPER',
                                       'Failure': 'DETECTION_DECISION'})

        state_machine.add('CLOSE_GRIPPER',
                          GripperControl,
                          transitions={'Completed': 'MANIPULATE_VALVE',
                                       'Failure': 'MANIPULATE_VALVE'})

        state_machine.add('MANIPULATE_VALVE',
                          PathVisitorState,
                          transitions={'Completed': 'OPEN_GRIPPER',
                                       'Failure': 'OPEN_GRIPPER'})

        state_machine.add('OPEN_GRIPPER',
                          GripperControl,
                          transitions={'Completed': 'BACKOFF_VALVE',
                                       'Failure': 'Failure'})

        state_machine.add('BACKOFF_VALVE',
                          PathVisitorState,
                          transitions={'Completed': 'HOMING_FINAL',
                                       'Failure': 'Failure'})

        homing_sequence_final = StateMachineRos(
            outcomes=['Success', 'Failure'])

        with homing_sequence_final:
            homing_sequence_final.add('HOME_ROBOT_FINAL',
                                      JointsConfigurationAction,
                                      transitions={'Completed': 'Success',
                                                   'Failure': 'Failure'})

        state_machine.add('HOMING_FINAL',
                          homing_sequence_final,
                          transitions={'Success': 'Success',
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
