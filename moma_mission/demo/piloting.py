#!/usr/bin/env python3
import sys
import rospy
import argparse
from moma_mission.core.state_ros import *
from moma_mission.missions.piloting.states import *
from moma_mission.missions.piloting.sequences import *
from moma_mission.states.observation import FOVSamplerState
from moma_mission.states.transform_visitor import TransformVisitorState
from moma_mission.states.transform_recorder import TransformRecorderState
from moma_mission.states.path_visitor import PathVisitorState
from moma_mission.states.waypoint_bridge import (
    WaypointBroadcasterState,
    WaypointReachedState,
)
from moma_mission.states.manipulation import JointsConfigurationAction


# Init ros
rospy.init_node("piloting_demo")

# Init data
Valve.init_from_ros()
Frames.init_from_ros()
Frames.print_summary()

# Parse args
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument(
    "--standalone",
    help="Test state machine without gRCS integration",
    default=False,
    required=False,
)
arg_parser.add_argument(
    "--sim",
    help="Test state machine in simulation",
    default=False,
    required=False,
)
args, unknown = arg_parser.parse_known_args()
standalone = args.standalone
sim = args.sim

# Build the state machine
state_machine = StateMachineRos(outcomes=["Success", "Failure"])

rospy.loginfo(f"Running in {'standalone' if standalone else 'gRCS'} mode")

try:
    with state_machine:
        if not standalone:
            rospy.loginfo("Setup")
            state_machine.add(
                "SETUP",
                SetUp,
                transitions={"Completed": "HOMING_START", "Failure": "Failure"},
            )

        rospy.loginfo("Homing start")
        state_machine.add(
            "HOMING_START",
            homing_sequence_factory(),
            transitions={
                "Success": "IDLE" if not standalone else "REACH_DETECTION_HOTSPOT_FAR",
                "Failure": "Failure",
            },
        )

        rospy.loginfo("Idle")
        state_machine.add(
            "IDLE",
            Idle,
            transitions={
                "ExecuteInspectionPlan": "WAYPOINT_BROADCAST",
                "ExecuteDummyPlan": "REACH_DETECTION_HOTSPOT_FAR",
                "ManipulateValve": "VALVE_SEQUENCE",
                "Failure": "Failure",
            },
        )

        rospy.loginfo("Broadcast waypoint")
        state_machine.add(
            "WAYPOINT_BROADCAST",
            WaypointBroadcasterState,
            transitions={
                "POSE": "WAYPOINT_FOLLOWING",
                "ACTION_VISUAL": "VALVE_SEQUENCE",
                "Failure": "Failure",
            },
        )

        rospy.loginfo("Reach detection hotspot far")
        state_machine.add(
            "REACH_DETECTION_HOTSPOT_FAR",
            NavigationState,
            transitions={
                # Use wholebody controller in sim
                # "Completed": "REACH_DETECTION_HOTSPOT_MEDIUM"
                # if not sim
                # else "REACH_DETECTION_HOTSPOT_CLOSE",
                "Completed": "REACH_DETECTION_HOTSPOT_MEDIUM",
                "Failure": "REACH_DETECTION_HOTSPOT_FAR",
            },
        )

        rospy.loginfo("Reach detection hotspot medium")
        state_machine.add(
            "REACH_DETECTION_HOTSPOT_MEDIUM",
            NavigationState,
            transitions={
                "Completed": "VALVE_SEQUENCE",
                "Failure": "REACH_DETECTION_HOTSPOT_FAR",
            },
        )

        rospy.loginfo("Reach detection hotspot close")
        state_machine.add(
            "REACH_DETECTION_HOTSPOT_CLOSE",
            TransformVisitorState,
            transitions={
                "Completed": "VALVE_SEQUENCE",
                "Failure": "REACH_DETECTION_HOTSPOT_FAR",
            },
        )

        rospy.loginfo("Waypoint following")
        state_machine.add(
            "WAYPOINT_FOLLOWING",
            NavigationState,
            transitions={
                "Completed": "WAYPOINT_REACHED",
                "Failure": "Failure",
            },
        )

        rospy.loginfo("Waypoint reached")
        state_machine.add(
            "WAYPOINT_REACHED",
            WaypointReachedState,
            transitions={
                "Next": "IDLE",
                "Completed": "IDLE",
                "Failure": "Failure",
            },
        )

        valve_sequence = StateMachineRos(outcomes=["Success", "Failure", "Warning"])
        with valve_sequence:
            rospy.loginfo("New valve")
            valve_sequence.add(
                "NEW_VALVE",
                ValveManipulationPrePostState,
                transitions={
                    "Completed": "VALVE_ANGLE_CONTROLLER",
                    "Failure": "Failure",
                },
            )

            rospy.loginfo("Valve angle controller")
            valve_sequence.add(
                "VALVE_ANGLE_CONTROLLER",
                ValveAngleControllerState,
                transitions={
                    "Completed": "Success",
                    "Failure": "DETECTION_DECISION",
                },
            )

            # Hacky, but avoids to define an almost empty class with additional outcomes
            rospy.loginfo("Detection decision")
            valve_sequence.add(
                "DETECTION_DECISION",
                StateRosDummy,
                transitions={
                    "Completed": "HOMING_DETECTION",
                    "Failure": "PLAN_URDF_VALVE" if standalone else "Failure",
                },
                constants={"get_next_pose": False, "continue_valve_fitting": False},
            )

            # On retrying the DETECTION_DECISION, the robot is not homed -> Do it now
            rospy.loginfo("Homing detection")
            state_machine.add(
                "HOMING_DETECTION",
                homing_sequence_factory(),
                transitions={
                    "Success": "OBSERVATION_POSE",
                    "Failure": "Failure",
                },
            )

            rospy.loginfo("Observation pose")
            valve_sequence.add(
                "OBSERVATION_POSE",
                FOVSamplerState,
                transitions={"Completed": "OBSERVATION_APPROACH", "Failure": "Warning"},
            )

            rospy.loginfo("Observation approach")
            valve_sequence.add(
                "OBSERVATION_APPROACH",
                TransformVisitorState,
                transitions={
                    "Completed": "MODEL_FIT_VALVE",
                    "Failure": "OBSERVATION_POSE",
                },
                constants={"get_next_pose": True},
            )

            rospy.loginfo("Model fit valve")
            valve_sequence.add(
                "MODEL_FIT_VALVE",
                ModelFitValveState,
                transitions={
                    "Completed": "PLAN_MODEL_VALVE",
                    "Failure": "Warning",
                    "NextDetection": "CONTINUE_VALVE_FITTING",
                },
            )

            rospy.loginfo("Continue valve fitting")
            valve_sequence.add(
                "CONTINUE_VALVE_FITTING",
                StateRosDummy,
                transitions={
                    "Completed": "OBSERVATION_POSE",
                    "Failure": "Failure",
                },
                constants={"continue_valve_fitting": True},
            )

            rospy.loginfo("Plan model valve")
            valve_sequence.add(
                "PLAN_MODEL_VALVE",
                ValveManipulationModelState,
                transitions={"Completed": "APPROACH_HOMING", "Failure": "Failure"},
            )

            # URDF planner only works if valve urdf is published, i. e. in standalone mode
            if standalone:
                rospy.loginfo("Plan urdf valve")
                valve_sequence.add(
                    "PLAN_URDF_VALVE",
                    ValveManipulationUrdfState,
                    transitions={"Completed": "APPROACH_HOMING", "Failure": "Failure"},
                )

            # On retrying the DETECTION_DECISION, the robot is not homed -> Do it now
            rospy.loginfo("Homing detection")
            state_machine.add(
                "APPROACH_HOMING",
                JointsConfigurationAction,
                transitions={
                    "Completed": "APPROACH_VALVE",
                    "Failure": "Failure",
                },
            )

            rospy.loginfo("Approach valve")
            valve_sequence.add(
                "APPROACH_VALVE",
                PathVisitorState,
                transitions={
                    "Completed": "GRASP_VALVE",
                    "Failure": "DETECTION_DECISION",
                },
            )

            rospy.loginfo("Grasp valve")
            valve_sequence.add(
                "GRASP_VALVE",
                PathVisitorState,
                transitions={
                    "Completed": "CLOSE_GRIPPER",
                    "Failure": "Failure",  # This is a critical error that needs manual intervention, as we do not know how entangled the robot is with the valve
                },
            )

            rospy.loginfo("Close gripper")
            valve_sequence.add(
                "CLOSE_GRIPPER",
                GripperGrasp,
                transitions={
                    "Completed": "MANIPULATE_VALVE",
                    "Failure": "MANIPULATE_VALVE",
                },
            )

            rospy.loginfo("Manipulate valve")
            valve_sequence.add(
                "MANIPULATE_VALVE",
                PathVisitorState,
                transitions={
                    "Completed": "MANIPULATE_VALVE_STEP_COMPLETED",
                    "Failure": "MANIPULATE_VALVE_STEP_COMPLETED",
                },
            )

            rospy.loginfo("Valve manipulation step completed")
            valve_sequence.add(
                "MANIPULATE_VALVE_STEP_COMPLETED",
                ValveManipulationPrePostState,
                transitions={
                    "Completed": "STORE_FINAL_POSE",
                    "Failure": "Failure",
                },
            )

            rospy.loginfo("Store final pose")
            valve_sequence.add(
                "STORE_FINAL_POSE",
                TransformRecorderState,
                transitions={"Completed": "APPROACH_FINAL_POSE", "Failure": "Failure"},
            )

            rospy.loginfo("Approach final pose")
            valve_sequence.add(
                "APPROACH_FINAL_POSE",
                TransformVisitorState,
                transitions={"Completed": "OPEN_GRIPPER", "Failure": "Failure"},
            )

            rospy.loginfo("Open gripper")
            valve_sequence.add(
                "OPEN_GRIPPER",
                GripperGrasp,
                transitions={"Completed": "BACKOFF_VALVE", "Failure": "Failure"},
            )

            rospy.loginfo("Backoff valve")
            valve_sequence.add(
                "BACKOFF_VALVE",
                TransformVisitorState,
                transitions={
                    "Completed": "HOMING_FINAL",
                    "Failure": "Failure",  # This is a critical error that needs manual intervention, as we do not know how entangled the robot is with the valve
                },
            )

            rospy.loginfo("Homing final")
            state_machine.add(
                "HOMING_FINAL",
                homing_sequence_factory(),
                transitions={
                    "Success": "VALVE_ANGLE_CONTROLLER",
                    "Failure": "Failure",
                },
            )

        rospy.loginfo("Valve sequence")
        state_machine.add(
            "VALVE_SEQUENCE",
            valve_sequence,
            transitions={
                "Success": "WAYPOINT_VALVE_REACHED" if not standalone else "Success",
                "Failure": "Failure",
                "Warning": "WAYPOINT_VALVE_REACHED"
                if not standalone
                else "VALVE_SEQUENCE",
            },
        )

        rospy.loginfo("Waypoint valve reached")
        state_machine.add(
            "WAYPOINT_VALVE_REACHED",
            WaypointReachedState,
            transitions={
                "Next": "HOMING_START",  # To ensure that the robot is homed even after a "warning" state, we issue another homing command
                "Completed": "HOMING_START",  # To ensure that the robot is homed even after a "warning" state, we issue another homing command
                "Failure": "HOMING_START",  # Can happen if the valve manipulation is triggered manually by a high-level action and not by an ACTION waypoint
            },
        )

except Exception as exc:
    rospy.logerr(exc)
    sys.exit(0)

# Execute state machine
rospy.loginfo("\n\nRunning the mission state machine!\n\n")
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))
if outcome != "Success":
    sys.exit(1)
sys.exit(0)

# Wait for ctrl-c to stop the application
# introspection_server.stop()
