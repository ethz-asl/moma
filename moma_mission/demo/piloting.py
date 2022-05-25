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
args, unknown = arg_parser.parse_known_args()
standalone = args.standalone

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
                "ManipulateValve": "DETECTION_DECISION",
                "Failure": "Failure",
            },
        )

        rospy.loginfo("Broadcast waypoint")
        state_machine.add(
            "WAYPOINT_BROADCAST",
            WaypointBroadcasterState,
            transitions={
                "Completed": "WAYPOINT_FOLLOWING",
                "Failure": "Failure",
            },
        )

        rospy.loginfo("Reach detection hotspot far")
        state_machine.add(
            "REACH_DETECTION_HOTSPOT_FAR",
            NavigationState,
            transitions={
                "Completed": "REACH_DETECTION_HOTSPOT_MEDIUM",
                "Failure": "REACH_DETECTION_HOTSPOT_FAR",
            },
        )

        rospy.loginfo("Reach detection hotspot medium")
        state_machine.add(
            "REACH_DETECTION_HOTSPOT_MEDIUM",
            NavigationState,
            transitions={
                "Completed": "DETECTION_DECISION",
                "Failure": "REACH_DETECTION_HOTSPOT_FAR",
            },
        )

        rospy.loginfo("Reach detection hotspot close")
        state_machine.add(
            "REACH_DETECTION_HOTSPOT_CLOSE",
            TransformVisitorState,
            transitions={
                "Completed": "DETECTION_DECISION",
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
                "Completed": "REACH_DETECTION_HOTSPOT_CLOSE",
                "Failure": "Failure",
            },
        )

        # Hacky, but avoids to define an almost empty class with additional outcomes
        rospy.loginfo("Detection decision")
        state_machine.add(
            "DETECTION_DECISION",
            StateRosDummy,
            transitions={"Completed": "OBSERVATION_POSE", "Failure": "PLAN_URDF_VALVE"},
            constants={"get_next_pose": False, "continue_valve_fitting": False},
        )

        rospy.loginfo("Observation pose")
        state_machine.add(
            "OBSERVATION_POSE",
            FOVSamplerState,
            transitions={"Completed": "OBSERVATION_APPROACH", "Failure": "IDLE"},
        )

        rospy.loginfo("Observation approach")
        state_machine.add(
            "OBSERVATION_APPROACH",
            TransformVisitorState,
            transitions={"Completed": "MODEL_FIT_VALVE", "Failure": "OBSERVATION_POSE"},
        )

        rospy.loginfo("Model fit valve")
        state_machine.add(
            "MODEL_FIT_VALVE",
            ModelFitValveState,
            transitions={
                "Completed": "PLAN_MODEL_VALVE",
                "Failure": "IDLE",
                "NextDetection": "CONTINUE_VALVE_FITTING",
            },
        )

        state_machine.add(
            "CONTINUE_VALVE_FITTING",
            StateRosDummy,
            transitions={
                "Completed": "OBSERVATION_POSE",
                "Failure": "Failure",
            },
            constants={"get_next_pose": True, "continue_valve_fitting": True},
        )

        rospy.loginfo("Plan model valve")
        state_machine.add(
            "PLAN_MODEL_VALVE",
            ValveManipulationModelState,
            transitions={"Completed": "APPROACH_VALVE", "Failure": "Failure"},
        )

        rospy.loginfo("Plan urdf valve")
        state_machine.add(
            "PLAN_URDF_VALVE",
            ValveManipulationUrdfState,
            transitions={"Completed": "APPROACH_VALVE", "Failure": "Failure"},
        )

        rospy.loginfo("Approach valve")
        state_machine.add(
            "APPROACH_VALVE",
            PathVisitorState,
            transitions={"Completed": "GRASP_VALVE", "Failure": "DETECTION_DECISION"},
        )

        rospy.loginfo("Grasp valve")
        state_machine.add(
            "GRASP_VALVE",
            PathVisitorState,
            transitions={"Completed": "CLOSE_GRIPPER", "Failure": "DETECTION_DECISION"},
        )

        rospy.loginfo("Close gripper")
        state_machine.add(
            "CLOSE_GRIPPER",
            GripperGrasp,
            transitions={
                "Completed": "MANIPULATE_VALVE",
                "Failure": "MANIPULATE_VALVE",
            },
        )

        rospy.loginfo("Manipulate valve")
        state_machine.add(
            "MANIPULATE_VALVE",
            PathVisitorState,
            transitions={
                "Completed": "STORE_FINAL_POSE",
                "Failure": "STORE_FINAL_POSE",
            },
        )

        rospy.loginfo("Store final pose")
        state_machine.add(
            "STORE_FINAL_POSE",
            TransformRecorderState,
            transitions={"Completed": "APPROACH_FINAL_POSE", "Failure": "Failure"},
        )

        rospy.loginfo("Approach final pose")
        state_machine.add(
            "APPROACH_FINAL_POSE",
            TransformVisitorState,
            transitions={"Completed": "OPEN_GRIPPER", "Failure": "Failure"},
        )

        rospy.loginfo("Open gripper")
        state_machine.add(
            "OPEN_GRIPPER",
            GripperGrasp,
            transitions={"Completed": "BACKOFF_VALVE", "Failure": "Failure"},
        )

        rospy.loginfo("Backoff valve")
        state_machine.add(
            "BACKOFF_VALVE",
            TransformVisitorState,
            transitions={"Completed": "HOMING_FINAL", "Failure": "Failure"},
        )

        rospy.loginfo("Homing final")
        state_machine.add(
            "HOMING_FINAL",
            homing_sequence_factory(),
            transitions={
                "Success": "IDLE" if not standalone else "Success",
                "Failure": "Failure",
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
