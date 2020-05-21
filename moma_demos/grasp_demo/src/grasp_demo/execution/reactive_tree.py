import py_trees
import py_trees_ros

from grasp_demo.msg import (
    DetectionAction,
    DetectionGoal,
    ScanSceneAction,
    ScanSceneActionResult,
    ScanSceneGoal,
    ScanSceneResult,
    SelectGraspAction,
    SelectGraspGoal,
    GraspAction,
    GraspGoal,
    DropGoal,
    DropAction,
)

import sensor_msgs.msg

from action_client import ActionClient_ResultSaver

import std_msgs

def generate_selection_goal_msg(msg):
    goal = SelectGraspGoal()
    goal.pointcloud_scene = msg.pointcloud_scene
    return goal


def get_bt_topics2bb():    
    # ScannedCloudBB = py_trees_ros.subscribers.ToBlackboard(
    #     name="ScannedCloudBB",
    #     topic_name="/scan_action/result",
    #     topic_type=sensor_msgs.msg.PointCloud2,
    #     blackboard_variables={"varScannedCloudBB": None},
    #     initialise_variables={"varScannedCloudBB": None}
    # )
    ScannedBB = py_trees_ros.subscribers.ToBlackboard(
        name="ScannedBB",
        topic_name="/bt_BB/ScannedBB",
        topic_type=std_msgs.msg.Bool,
        blackboard_variables={"varScannedBB": 'data'},
        initialise_variables={"varScannedBB": False}
    )
    
    
    GripperBB = py_trees_ros.subscribers.ToBlackboard(
        name="GripperBB",
        topic_name="/bt_BB/GripperBB",
        topic_type=std_msgs.msg.Bool,
        blackboard_variables={"varGripperContent": 'data'},
        initialise_variables={"varGripperContent": False}
    )

    TargetBB = py_trees_ros.subscribers.ToBlackboard(
        name="TargetBB",
        topic_name="/bt_BB/TargetBB",
        topic_type=std_msgs.msg.Bool,
        blackboard_variables={"varObjAtTarget": 'data'},
        initialise_variables={"varObjAtTarget": False}
    )

    DetectedBB = py_trees_ros.subscribers.ToBlackboard(
        name="DetectedBB",
        topic_name="/bt_BB/DetectedBB",
        topic_type=std_msgs.msg.Bool,
        blackboard_variables={"varDetectedBB": 'data'},
        initialise_variables={"varDetectedBB": True}
    )
    TrackLockBB = py_trees_ros.subscribers.ToBlackboard(
        name="TrackLockBB",
        topic_name="/bt_BB/TrackLockBB",
        topic_type=std_msgs.msg.Bool,
        blackboard_variables={"varTrackLockBB": 'data'},
        initialise_variables={"varTrackLockBB": True}
    )
    TrackMoveBB = py_trees_ros.subscribers.ToBlackboard(
        name="TrackMoveBB",
        topic_name="/bt_BB/TrackMoveBB",
        topic_type=std_msgs.msg.Bool,
        blackboard_variables={"varTrackMoveBB": 'data'},
        initialise_variables={"varTrackMoveBB": False}
    )
    gpdBB = py_trees_ros.subscribers.ToBlackboard(
        name="gpdBB",
        topic_name="/bt_BB/gpdBB",
        topic_type=std_msgs.msg.Bool,
        blackboard_variables={"vargpdBB": 'data'},
        initialise_variables={"vargpdBB": False}
    )
    SelectionBB = py_trees_ros.subscribers.ToBlackboard(
        name="SelectionBB",
        topic_name="/bt_BB/SelectionBB",
        topic_type=std_msgs.msg.Bool,
        blackboard_variables={"varSelectionBB": 'data'},
        initialise_variables={"varSelectionBB": False}
    )


    # topics2bb = py_trees.behaviours.Success(name = "Topics2BB")
    topics2bb = py_trees.composites.Parallel("Topics2BB",\
        children=[
            # ScannedCloudBB,
            ScannedBB,
            GripperBB,
            TargetBB,
            DetectedBB,
            TrackLockBB,
            gpdBB,
            SelectionBB
            ]
        )
    topics2bb.blackbox_level = py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX
    print("return topics2bb")
    return topics2bb


def get_bt_actions():
    selTask = py_trees.composites.Selector("Task")
    # selTask = py_trees.behaviours.Success("Task")
    # ====== *
    check_object_at_target = py_trees.blackboard.CheckBlackboardVariable(
        name="Object at target?",
        variable_name="varObjAtTarget",
        expected_value=True,
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )
    selGripper = py_trees.composites.Selector(name="Gripper Check")
    # ------ #
    selTask.add_children([check_object_at_target,selGripper])
    # ====== *
    seqGripper = py_trees.composites.Sequence(name="Gripper Check")
    seqObject = py_trees.composites.Sequence(name="Get Object")
    # ------ #
    selGripper.add_children([seqGripper,seqObject])
    # ====== #
    checkGripper = py_trees.blackboard.CheckBlackboardVariable(
        name="Check Gripper",
        variable_name="varGripperContent",
        expected_value = True,
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )
    action_drop = py_trees_ros.actions.ActionClient(
        name="action_drop",
        action_spec=DropAction,
        action_goal=DropGoal(),
        action_namespace="drop_action",
        override_feedback_message_on_running="dropping"
        )
    setGripper = py_trees.blackboard.SetBlackboardVariable(
        name="Stop Drop",
        variable_name= "varGripperContent",
        variable_value= False,
    )
    # ------ #
    seqGripper.add_children([checkGripper,action_drop,setGripper])
    # ====== #
    selDetect = py_trees.composites.Selector("Detection")

    selTrack = py_trees.composites.Selector("Tracking")

    selScan = py_trees.composites.Selector("Scanning")

    selGPD = py_trees.composites.Selector("GPDing")

    selSelection = py_trees.composites.Selector("Selecting")

    # action_grasp = py_trees_ros.actions.ActionClient(
    #     name="action_grasp",
    #     action_spec=GraspAction,
    #     action_goal=GraspGoal(),
    #     action_namespace="grasp_execution_action",
    #     override_feedback_message_on_running="grasping"
    #     )
    action_grasp = py_trees.behaviours.Success("Success Action")
    # ------ #
    seqObject.add_children([selDetect,selTrack,selScan,selGPD,selSelection,action_grasp])
    # successElem = py_trees.behaviours.Success("Success")
    # seqObject.add_child(successElem)
    # ====== #
    checkDetected = py_trees.blackboard.CheckBlackboardVariable(
        name="Check Detected",
        variable_name="varDetectedBB",
        expected_value = True,
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )

    action_obj_detection = py_trees_ros.actions.ActionClient(
        name="action_detection",
        action_spec=DetectionAction,
        action_goal=DetectionGoal(),
        action_namespace="detection_execution_action",
        override_feedback_message_on_running="detecting"
    )

    # setDetection = py_trees.blackboard.SetBlackboardVariable(
    #     name="Detected",
    #     variable_name= "varDetectedBB",
    #     variable_value= True,
    # )
    # ------ #
    # selDetect.add_children([checkDetected,action_obj_detection,setDetection])
    successElem1 = py_trees.behaviours.Success("Success 1")
    selDetect.add_child(successElem1)
    # ====== #
    checkPresence = py_trees.blackboard.CheckBlackboardVariable(
        name="Check Presence",
        variable_name="varDetectedBB", # TODO
        expected_value = True,
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )
    checkMovement = py_trees.blackboard.CheckBlackboardVariable(
        name="Check Movement",
        variable_name="varTrackMoveBB",
        expected_value = False,
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )
    # ------ #
    # selTrack.add_children([checkPresence,checkMovement])
    successElem2 = py_trees.behaviours.Success("Success 2")
    selTrack.add_child(successElem2)
    # ====== #
    check_scene_scanned = py_trees.blackboard.CheckBlackboardVariable(
        name="Check scanned scene",
        variable_name="varScannedBB",
        expected_value = True,
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )
    
    action_scan = py_trees_ros.actions.ActionClient(
        name="action_scan",
        action_spec=ScanSceneAction,
        action_goal=ScanSceneGoal(),
        action_namespace="scan_action",
        override_feedback_message_on_running="scanning"
    )
    
    # ------ #
    selScan.add_children([check_scene_scanned,action_scan])
    # ====== #
    check_grasp_options = py_trees.blackboard.CheckBlackboardVariable(
        name="Check scanned scene",
        variable_name="vargpdBB",
        expected_value = True,
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )
    # TODO
    seq_action_gpd_options = py_trees.composites.Sequence(name="Action Seq: GPD")
    # ------ #
    # seq_action_gpd_options.add_children([check_grasp_options,seq_action_gpd_options])
    successElem3 = py_trees.behaviours.Success("Success 3")
    # selGPD.add_children([check_grasp_options,seq_action_gpd_options])
    selGPD.add_child(successElem3)
    # ====== #
    action_gpd_options = py_trees_ros.actions.ActionClient(
        name="action_gpd_options",
        action_spec=SelectGraspAction,
        action_goal=SelectGraspGoal,
        action_namespace="grasp_selection_action",
        override_feedback_message_on_running="locking for grasps"
    )
    # action_gpd_options = ActionClient_BBgoal(
    #     name="action_select",
    #     action_spec=SelectGraspAction,
    #     action_namespace="grasp_selection_action",
    #     goal_gen_callback=generate_selection_goal_msg,
    #     bb_goal_var_name="action_scan_result",
    #     set_flag_instead_result=False,
    # )
    

    set_var_grasp_options = py_trees.blackboard.SetBlackboardVariable(
        name="Set GPD found",
        variable_name= "vargpdBB",
        variable_value= True,
    )
    # ------ #
    # seq_action_gpd_options.add_children([action_gpd_options,set_var_grasp_options])
    # ====== #
    # successElem4 = py_trees.behaviours.Success("Success 4")
    # seq_action_gpd_options.add_children([check_grasp_options,seq_action_gpd_options])
    # ====== #
    check_grasp_selection = py_trees.blackboard.CheckBlackboardVariable(
        name="Check scanned scene",
        variable_name="varSelectionBB",
        expected_value = True,
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )
    # TODO
    action_gpd = py_trees_ros.actions.ActionClient(
        name="action_scan",
        action_spec=GraspAction,
        action_goal=GraspGoal(),
        action_namespace="select_action",
        override_feedback_message_on_running="selecting"
    )
    # action_gpd = ActionClient_BBgoal(
    #     name="action_grasp",
    #     action_spec=GraspAction,
    #     action_namespace="grasp_execution_action",
    #     goal_gen_callback=generate_grasp_goal_msg,
    #     bb_goal_var_name="action_select_result",
    #     set_flag_instead_result=True,
    # )

    set_var_grasp_selection = py_trees.blackboard.SetBlackboardVariable(
        name="Set scanned scene",
        variable_name= "varSelectionBB",
        variable_value= True,
    )
    # ------ #
    # selSelection.add_children([check_grasp_selection,action_gpd,set_var_grasp_selection])
    successElem4 = py_trees.behaviours.Success("Success 4")
    selSelection.add_child(successElem4)
    # ====== #
    return selTask


def get_root():
    print("enter root")
    # Subscriber
    subscriber_root = get_bt_topics2bb()

    # Actions
    action_root = get_bt_actions()

    root = py_trees.composites.Parallel(
        children=[subscriber_root,action_root]
    )
    # root =  subscriber_root
    print("return root")
    return root


class Reactive_PandaTree:
    def __init__(self, debug=False):

        if debug:
            py_trees.logging.level = py_trees.logging.Level.DEBUG

        self._root = get_root()
        self.tree = py_trees_ros.trees.BehaviourTree(self._root)

        self.show_tree_console()

    def show_tree_console(self):
        print("=" * 20)
        print("Behavior tree:")
        print("-" * 20)
        py_trees.display.print_ascii_tree(self.tree.root)
        print("=" * 20)

    def setup(self):
        self.tree.setup(timeout=0)
