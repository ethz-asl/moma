import py_trees
import py_trees_ros

from grasp_demo.msg import (
    DetectionAction,
    ScanSceneAction,
    ScanSceneActionResult,
    ScanSceneGoal,
    SelectGraspAction,
    SelectGraspGoal,
    GraspAction,
    GraspGoal,
    DropGoal,
    DropAction,
)
from action_client import ActionClient_ResultSaver, ActionClient_BBgoal, RepeatAction

import std_msgs

def generate_grasp_goal_msg(msg):
    goal = GraspGoal()
    goal.target_grasp_pose = msg.target_grasp_pose
    return goal


def generate_selection_goal_msg(msg):
    goal = SelectGraspGoal()
    goal.pointcloud_scene = msg.pointcloud_scene
    return goal


def get_bt_topics2bb():
    print("enter topics2bb")
    
    GripperBB = py_trees_ros.subscribers.EventToBlackboard(
        name="GripperBB",
        topic_name="/bt_BB/GripperBB",
        variable_name="varGripperBB"
    )
    # TargetBB = py_trees_ros.subscribers.EventToBlackboard(
    #     name="TargetBB",
    #     topic_name="/bt_BB/TargetBB",
    #     variable_name="action_drop_result"
    # )
    TargetBB = py_trees_ros.subscribers.ToBlackboard(
        name="TargetBB",
        topic_name="/bt_BB/TargetBB",
        topic_type=std_msgs.msg.Bool,
        blackboard_variables={"action_drop_result": None},
        initialise_variables={"action_drop_result": False}
    )
    GripperBB = py_trees_ros.subscribers.EventToBlackboard(
        name="GripperBB",
        topic_name="/bt_BB/GripperBB",
        variable_name="varGripperBB"
    )
    # ScannedBB = py_trees_ros.subscribers.EventToBlackboard(
    #     name="ScannedBB",
    #     topic_name="/bt_BB/ScannedBB",
    #     variable_name="varScannedBB"
    # )
    ScannedBB = py_trees_ros.subscribers.ToBlackboard(
        name="ScannedBB",
        topic_name="/scan_action/result",
        topic_type=ScanSceneActionResult,
        blackboard_variables={"varScannedBB": 'result'},
        initialise_variables={"varScannedBB": False}
    )
    DetectedBB = py_trees_ros.subscribers.EventToBlackboard(
        name="DetectedBB",
        topic_name="/bt_BB/DetectedBB",
        variable_name="varDetectedBB"
    )
    TrackLockBB = py_trees_ros.subscribers.EventToBlackboard(
        name="TrackLockBB",
        topic_name="/bt_BB/TrackLockBB",
        variable_name="varTrackLockBB"
    )
    TrackMoveBB = py_trees_ros.subscribers.EventToBlackboard(
        name="TrackMoveBB",
        topic_name="/bt_BB/TrackMoveBB",
        variable_name="varTrackMoveBB"
    )
    gpdBB = py_trees_ros.subscribers.EventToBlackboard(
        name="gpdBB",
        topic_name="/bt_BB/gpdBB",
        variable_name="vargpdBB"
    )
    SelectionBB = py_trees_ros.subscribers.EventToBlackboard(
        name="SelectionBB",
        topic_name="/bt_BB/SelectionBB",
        variable_name="varSelectionBB"
    )


    # Old BB variables
        # # Perception
        # targetObj2bb = py_trees.behaviours.Success(name="targetObj2bb")
        # # targetObj2bb = py_trees_ros.subscribers.ToBlackboard(
        # #     name="Target object",
        # #     topic_name="topname_2",
        # #     variable_name="varname_1",
        # #     topic_type = std_msgs.msg.String,
        # #     blackboard_variables = {'name':'data'}
        # # )
        # preseceObj2bb = py_trees_ros.subscribers.EventToBlackboard(
        #     name="Obj detection",
        #     topic_name="topname_2",
        #     variable_name="varname_2"
        # )
        # lockObj2bb = py_trees_ros.subscribers.EventToBlackboard(
        #     name="Track lock",
        #     topic_name="topname_3",
        #     variable_name="varname_3"
        # )
        # moveObj2bb = py_trees_ros.subscribers.EventToBlackboard(
        #     name="Track movement",
        #     topic_name="topname_4",
        #     variable_name="varname_4"
        # )

        # # Robot States
        # inGripper2bb = py_trees_ros.subscribers.EventToBlackboard(
        #     name="Gripper content",
        #     topic_name="topname_5",
        #     variable_name="varname_5"
        # )
        # moveRobot2bb = py_trees_ros.subscribers.EventToBlackboard(
        #     name="Robot movement",
        #     topic_name="topname_6",
        #     variable_name="varname_6"
        # )

        # # Action States
        # scannedSzene2bb = py_trees_ros.subscribers.EventToBlackboard(
        #     name="Scan result",
        #     topic_name="topname_7",
        #     variable_name="varname_7"
        # )
        # graspSelect2bb = py_trees_ros.subscribers.EventToBlackboard(
        #     name="GPD result",
        #     topic_name="topname_8",
        #     variable_name="varname_8"
        # )
        # grasExec2bb = py_trees_ros.subscribers.EventToBlackboard(
        #     name="Trajectory result",
        #     topic_name="topname_9",
        #     variable_name="varname_9"
        # )
        # dropObj2bb = py_trees_ros.subscribers.EventToBlackboard(
        #     name="Drop actions",
        #     topic_name="topname_10",
        #     variable_name="varname_10"
        # )


    # topics2bb = py_trees.behaviours.Success(name = "Topics2BB")
    topics2bb = py_trees.composites.Sequence("Topics2BB",\
        children=[
            GripperBB,
            TargetBB,
            ScannedBB,
            DetectedBB,
            TrackLockBB,
            gpdBB,
            SelectionBB
            # # Perceptiopn
            # targetObj2bb,
            # preseceObj2bb,
            # lockObj2bb,
            # moveObj2bb,
            # # Robot States
            # inGripper2bb,
            # moveRobot2bb,
            # # Action States
            # scannedSzene2bb,
            # graspSelect2bb,
            # grasExec2bb,
            # dropObj2bb
            ]
        )
    topics2bb.blackbox_level = py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX
    print("return topics2bb")
    return topics2bb


def get_bt_actions():
    print('enter actions')

    # ====== Seq /Sel ======= #

    # --------  Sel  -------- #
    selTask = py_trees.composites.Selector("Task")
    selGripper = py_trees.composites.Selector("Gripper Check")
    
    selDetect = py_trees.composites.Selector("Detection")
    selTrack = py_trees.composites.Selector("Tracking")
    selScan = py_trees.composites.Selector("Scanning")
    selGPD = py_trees.composites.Selector("GPDing")
    selSelection = py_trees.composites.Selector("Selecting")

    # --------  Seq  -------- #
    seqGripper = py_trees.composites.Sequence(name="Gripper Check")
    seqObject = py_trees.composites.Sequence("Get Object")
    
    # ======   Actions   ===== #
    # --------  Drop  ------- #
    action_drop_goal = DropGoal()
    action_drop = ActionClient_ResultSaver(
        name="action_drop",
        action_spec=DropAction,
        action_goal=action_drop_goal,
        action_namespace="drop_action",
        set_flag_instead_result=True,
    )
    
    # --- Object detection ---- #
    action_obj_detection = ActionClient_ResultSaver(
        name="action_detection",
        # TODO
        action_spec=DetectionAction,
        action_goal=action_drop_goal,
        action_namespace="grasp_selection_action",
        set_flag_instead_result=False,
    )
    # --- Object detection ---- #
    # Detection is running in the background

    # -----  Scan Scene  ----- #
    action_scan_goal = ScanSceneGoal()
    action_scan = ActionClient_ResultSaver(
        name="action_scan",
        action_spec=ScanSceneAction,
        action_goal=action_scan_goal,
        action_namespace="scan_action",
        set_flag_instead_result=False,
    )
    # --- GPD ---- #
    # Searches for grasps
    action_gpd = ActionClient_BBgoal(
        name="action_gpd",
        action_spec=SelectGraspAction,
        action_namespace="grasp_selection_action",
        goal_gen_callback=generate_selection_goal_msg,
        bb_goal_var_name="action_scan_result",
        set_flag_instead_result=False,
    )

    # --- Grasp Selection ---- #
    # TODO
    action_gpd_select = ActionClient_BBgoal(
        name="action_select",
        action_spec=SelectGraspAction,
        action_namespace="grasp_selection_action",
        goal_gen_callback=generate_selection_goal_msg,
        bb_goal_var_name="action_scan_result",
        set_flag_instead_result=False,
    )

    # --- Execute Grasp ---- #
    # TODO
    action_grasp = ActionClient_BBgoal(
        name="action_grasp",
        action_spec=GraspAction,
        action_namespace="grasp_execution_action",
        goal_gen_callback=generate_grasp_goal_msg,
        bb_goal_var_name="action_select_result",
        set_flag_instead_result=True,
    )
    # #compTraj = py_trees.behaviours.Running("Compute Trajectory")
        # compTraj = py_trees.behaviours.Success(name = "Compute Trajectory")
        # # compTraj = py_trees_ros.actions.ActionClient(name='Compute Trajectory',
        # #                                               #action_spec=None,
        # #                                               #action_goal=None,
        # #                                               #action_namespace='/action',
        # #                                               #override_feedback_message_on_running='moving'
        # #                                               )

        # detectObj = py_trees.behaviours.Running("Object detection")
        # detectObj = py_trees_ros.actions.ActionClient(name='Detect Object',
        #                                               #action_spec=None,
        #                                               #action_goal=None,
        #                                               #action_namespace='/action',
        #                                               #override_feedback_message_on_running='moving'
        #                                               )

        #execGrasp = py_trees.behaviours.Running("Execute Grasp")
        # execGrasp = py_trees.behaviours.Success("Execute Grasp")
        # execGrasp = py_trees_ros.actions.ActionClient(name="Execute Grasp",
        #                                               #action_spec=None,
        #                                               #action_goal=None,
        #                                               #action_namespace='/action',
        #                                               #override_feedback_message_on_running='moving'
        #            from action_client import ActionClient_ResultSaver, ActionClient_BBgoal, RepeatAction


    # ======   Checks   ====== #
    # -----  Object at Target  ----- #
    check_object_at_target = py_trees.blackboard.CheckBlackboardVariable(
        name="Object at target?",
        variable_name="action_drop_result",
        expected_value=True,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )
    # -----  Check Gripper  ----- #
    # checkGripper = py_trees.blackboard.CheckBlackboardVariable(name, variable_name='gripper_state_BB', expected_value=True)
    checkGripper = py_trees.blackboard.CheckBlackboardVariable(
        name = "Check Gripper",
        expected_value = True,
    )
    # -----  Detected Obj  ----- #
    # checkGripper = py_trees.blackboard.CheckBlackboardVariable(name, variable_name='gripper_state_BB', expected_value=True)
    checkDetected = py_trees.blackboard.CheckBlackboardVariable(
        name = "Check Detected",
        expected_value = True,
    )
    # -----  Track Obj pres  ----- #
    # checkGripper = py_trees.blackboard.CheckBlackboardVariable(name, variable_name='gripper_state_BB', expected_value=True)
    checkPresence = py_trees.blackboard.CheckBlackboardVariable(
        name = "Check Precence",
        expected_value = True,
    )
    # -----  Detected Obj  ----- #
    # checkGripper = py_trees.blackboard.CheckBlackboardVariable(name, variable_name='gripper_state_BB', expected_value=True)
    checkMovement = py_trees.blackboard.CheckBlackboardVariable(
        name = "Check Movement",
        expected_value = False,
    )
    # -----  Scan Scene  ----- #
    check_scene_scanned = py_trees.blackboard.CheckBlackboardVariable(
        name="Scene scanned?",
        variable_name="action_scan_result",
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )
    # -----  Grasp Options  ----- #
    check_grasp_options = py_trees.blackboard.CheckBlackboardVariable(
        name="Grasp options",
        variable_name="action_gpd_result",
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )
    # --- Grasp Selection ---- #
    check_grasp_selected = py_trees.blackboard.CheckBlackboardVariable(
        name="Grasp selected?",
        variable_name="action_select_result",
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )     

    
    

    # #checkRange = py_trees.blackboard.CheckBlackboardVariable(name = "Check Range", expected_value = armReach)
        # checkRange = py_trees.behaviours.Success(name = "Check Range")

        # blinkLED = py_trees.behaviours.Failure("Blink LED")

        # seqCheck1 = py_trees.composites.Sequence("Check 1")
        # seqCheck2 = py_trees.composites.Sequence("Check 2")

        # checkTraj = py_trees.blackboard.CheckBlackboardVariable(
        #     name = "Trajectory computed",
        #     expected_value = True
        # )

        # checkObj1 = py_trees.blackboard.CheckBlackboardVariable(
        #     name = "Object present? 1"
        # )

        # sel1 = py_trees.composites.Selector("Selector 1")
        
        # checkObj2 = py_trees.blackboard.CheckBlackboardVariable(
        #     name = "Object present? 2",
        #     variable_name="obj_prsc",
        #     expected_value=True
        # )
    

    ### Building tree ###

    selTask.add_children([check_object_at_target,selGripper])
    # selGripper.add_children([seqGripper,seqObject])
    selGripper.add_children([seqGripper])
    seqGripper.add_children([checkGripper,action_drop])
    # seqObject.add_children([selDetect,selTrack,selScan,selGPD,selSelection,action_grasp])
    # selDetect.add_children([checkDetected,action_obj_detection])
    # selTrack.add_children([checkPresence,checkMovement])
    # selScan.add_children([check_scene_scanned,action_scan])
    # selGPD.add_children([check_grasp_options,action_gpd_select])
    # selSelection.add_children([check_grasp_selected,action_gpd])
    # # selRange.add_children([checkRange,blinkLED])
        # # checkRange abandoned
        # selTraj.add_children([seqCheck1,seqCheck2])
        # seqCheck1.add_children([checkTraj,checkObj1,sel1,compTraj])
        # sel1.add_children([checkObj2,detectObj])

    print("return actions")
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
