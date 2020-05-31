import py_trees
import py_trees_ros

from grasp_demo.msg import (
    ScanSceneAction,
    ScanSceneGoal,
    SelectGraspAction,
    SelectGraspGoal,
    GraspAction,
    GraspGoal,
    DropGoal,
    DropAction,
    DetectionAction,
    DetectionGoal,
)
from action_client import ActionClient_ResultSaver, ActionClient_BBgoal, RepeatAction

import std_msgs

import operator

def generate_detection_goal_msg():
    goal = DetectionGoal()
    goal.name = "orange"
    return goal


def generate_grasp_goal_msg(msg):
    goal = GraspGoal()
    goal.target_grasp_pose = msg.target_grasp_pose
    return goal


def generate_selection_goal_msg(msg):
    goal = SelectGraspGoal()
    goal.pointcloud_scene = msg.pointcloud_scene
    return goal


def get_bt_reset(condition_variable_names, reset_all):
    action_name = "reset" if reset_all else "repeat"

    button_reset = py_trees_ros.subscribers.WaitForData(
        name="Button reset?",
        topic_name="/manipulation_actions/" + action_name,
        topic_type=std_msgs.msg.Empty,
    )
    var_reset = py_trees.blackboard.SetBlackboardVariable(
        variable_name="do_" + action_name, variable_value=True
    )
    reset_root = py_trees.composites.Sequence(
        name="Check " + action_name, children=[button_reset, var_reset]
    )
    # reset_root.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    reset_root.blackbox_level = py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX

    # Reset exec
    check_var_reset = py_trees.blackboard.CheckBlackboardVariable(
        name="Check reset var", variable_name="do_" + action_name, expected_value=True
    )
    clear_var_reset = py_trees.blackboard.ClearBlackboardVariable(
        variable_name="do_" + action_name
    )
    reset_action = RepeatAction(
        name=action_name, variable_names=condition_variable_names, repeat_all=reset_all,
    )
    reset_exec_root = py_trees.composites.Sequence(
        name="Do " + action_name,
        children=[check_var_reset, clear_var_reset, reset_action],
    )
    # reset_exec_root.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    reset_exec_root.blackbox_level = py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX
    return reset_root, reset_exec_root


def get_bt_reset_perception(condition_variable_names, reset_all):
    action_name = "reset" if reset_all else "repeat"

    # button_reset = py_trees_ros.subscribers.WaitForData(
    #     name="Button reset?",
    #     topic_name="/manipulation_actions/" + action_name,
    #     topic_type=std_msgs.msg.Empty,
    # )
    var_reset = py_trees.blackboard.SetBlackboardVariable(
        variable_name="do_" + action_name, variable_value=True
    )
    reset_root = py_trees.composites.Sequence(
        name="Check " + action_name, children=[var_reset]
        # name="Check " + action_name, children=[button_reset, var_reset]
    )
    # reset_root.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    reset_root.blackbox_level = py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX

    # Reset exec
    check_var_reset = py_trees.blackboard.CheckBlackboardVariable(
        name="Check reset var", variable_name="do_" + action_name, expected_value=True
    )
    clear_var_reset = py_trees.blackboard.ClearBlackboardVariable(
        variable_name="do_" + action_name
    )
    reset_action = RepeatAction(
        name=action_name, variable_names=condition_variable_names, repeat_all=reset_all,
    )
    reset_exec_root = py_trees.composites.Sequence(
        name="Do " + action_name,
        children=[clear_var_reset, reset_action],
        # children=[check_var_reset, clear_var_reset, reset_action],
    )
    # reset_exec_root.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    reset_exec_root.blackbox_level = py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX
    return reset_root, reset_exec_root


def get_bt_repeat(condition_variable_names):
    button_repeat = py_trees_ros.subscribers.WaitForData(
        name="Button repeat?",
        topic_name="/manipulation_actions/repeat",
        topic_type=std_msgs.msg.Empty,
    )
    var_repeat = py_trees.blackboard.SetBlackboardVariable(
        variable_name="do_repeat", variable_value=True
    )
    repeat_root = py_trees.composites.Sequence(children=[button_repeat, var_repeat])
    repeat_root.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    # Repeat exec
    check_var_repeat = py_trees.blackboard.CheckBlackboardVariable(
        name="Check repeat var", variable_name="do_repeat", expected_value=True
    )
    clear_var_repeat = py_trees.blackboard.ClearBlackboardVariable(
        variable_name="do_repeat"
    )
    repeat_action = RepeatAction(
        name="Repeat last", variable_names=condition_variable_names, repeat_all=False
    )
    click_next_button = py_trees.blackboard.SetBlackboardVariable(
        name="Click next", variable_name="button_pressed_override", variable_value=True
    )
    repeat_exec_root = py_trees.composites.Sequence(
        children=[check_var_repeat, clear_var_repeat, repeat_action, click_next_button]
    )
    # repeat_exec_root.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    repeat_exec_root.blackbox_level = py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX


def get_bt_topics2bb():
    button_next_2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Next button listener",
        topic_name="/manipulation_actions/next",
        variable_name="button_pressed",
    )

    tracker_lock = py_trees_ros.subscribers.ToBlackboard(
        name="Object Tracker Lock",
        topic_name="/object_tracker/objLock",
        topic_type=std_msgs.msg.String,
        blackboard_variables={'object_locked': 'data'},
        # blackboard_variables='object_locked',
        # initialise_variables=False
    )

    tracker_motion = py_trees_ros.subscribers.ToBlackboard(
        name="Object Tracker Motion",
        topic_name="/object_tracker/objRest",
        topic_type=std_msgs.msg.String,
        blackboard_variables={'object_at_rest': 'data'},
        # blackboard_variables='object_at_rest',
        # initialise_variables="moving"
    )

    # grasp_state = py_trees_ros.subscribers.EventToBlackboard(
    #     name="Proximity: TCP_Obj",
    #     topic_name="/proximity_detector",
    #     topic_type=std_msgs.msg.Float64,
    #     blackboard_variables={'grasp_state': 'data'},
    #     # initialise_variables="open"
    # )

    # grasp_state = py_trees_ros.subscribers.CheckData(
    #     name="Proximity: TCP_Obj",
    #     topic_name="/proximity_detector",
    #     topic_type=std_msgs.msg.Float64,
    #     blackboard_variables={'grasp_state': 'data'},
    #     fail_if_no_data = True,
    #     fail_if_bad_comparison = True,
    # )

    # wait_4_grasp_state = py_trees_ros.subscribers.WaitForData(
    #         name='Wait 4 Proximity',
    #         topic_name="/proximity_detector",
    #         topic_type=std_msgs.msg.Float64,
    #     )

    # # grasp_state = py_trees.behaviours.Success(
    # #     name="Proximity: TCP_Obj",
    # # )

    # root_proximity = py_trees.composites.Selector(
    #     name="Selector",
    #     children=[
    #         py_trees.decorators.RunningIsFailure(   # Otherwhise the robot actions will fail
    #             wait_4_grasp_state,
    #         ),
    #         grasp_state
    #     ]
    # )

    gripper_proximity = py_trees_ros.subscribers.ToBlackboard(
        name="Proximity: TCP_Obj",
        topic_name="/proximity_detector",
        topic_type=std_msgs.msg.String,
        blackboard_variables={'gripper_proximity': 'data'},
        # initialise_variables="open"
    )

    topics2bb = py_trees.composites.Parallel("Topics2BB",
        children=[
            button_next_2bb,
            tracker_lock,
            tracker_motion,
            gripper_proximity,
            ]
        )
    # topics2bb.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    topics2bb.blackbox_level = py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX
    return topics2bb


# def get_button_next_check():
    #     # vars_to_check = ["button_pressed_override", "button_pressed"]
    #     vars_to_check = ["button_pressed"]
    #     children = []
    #     for var in vars_to_check:
    #         child = py_trees.composites.Sequence(
    #             name=var,
    #             children=[
    #                 py_trees.blackboard.WaitForBlackboardVariable(
    #                     name="Check " + var, variable_name=var, expected_value=True,
    #                 ),
    #                 py_trees.blackboard.ClearBlackboardVariable(
    #                     name="Clear " + var, variable_name=var
    #                 ),
    #             ],
    #         )
    #         if var == "button_pressed_override":
    #             child = py_trees.decorators.RunningIsFailure(child=child)
    #         children.append(child)
    #     button_next = py_trees.composites.Selector(name="Button next?", children=children)
    #     # button_next.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    #     button_next.blackbox_level = py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX
    #     return button_next

def get_bt_perception(subtree=None):  
    # Action: detect
    action_detection_goal = DetectionGoal()
    action_detection = ActionClient_ResultSaver(
        name="action_detect",
        action_spec=DetectionAction,
        action_goal=action_detection_goal,
        action_namespace="detection_action",
        set_flag_instead_result=False,
    )

    condition_variable_names = [
        "action_drop_result",
        "action_grasp_result",
        "action_select_result",
        "action_scan_result",
        "action_detect_result",
        "action_tracking_result"
    ]

    # Add reset button
    reset_root, reset_exec_root = get_bt_reset_perception(condition_variable_names, reset_all=True)

    root_reset_detection = py_trees.composites.Sequence(
        children=[
            reset_exec_root,
            action_detection,
        ]
    )

    check_object_detected = py_trees.blackboard.CheckBlackboardVariable(
        name="Object detected?",
        variable_name="action_detect_result",
        expected_value=True,
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )


    root_detection = py_trees.composites.Selector(
        children=[
            check_object_detected,
            root_reset_detection,
        ]
    )

    # Tracking Variables
    check_obj_locked_1 = py_trees.blackboard.CheckBlackboardVariable(
        name="Tracking locked?",
        variable_name="object_locked",
        expected_value="locked",
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
    )

    check_obj_locked_2= py_trees.blackboard.CheckBlackboardVariable(
        name="Tracking locked?",
        variable_name="object_locked",
        expected_value="locked",
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
    )

    # check_obj_rest = py_trees.blackboard.WaitForBlackboardVariable(
    #     name="Object at rest?",
    #     # variable_name="object_at_rest/data",
    #     variable_name="object_at_rest",
    #     expected_value="resting",
    #     # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    #     clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
    # )

    check_obj_rest = py_trees.blackboard.CheckBlackboardVariable(
        name="Object at rest?",
        variable_name="object_at_rest",
        expected_value="resting",
        # clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
    )
    
    check_gripper_proximity = py_trees.blackboard.CheckBlackboardVariable(
        name="Gripper proximity?",
        variable_name="proximity_detector",
        expected_value="close",
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )

    # check_gripper_proximity = py_trees.blackboard.CheckBlackboardVariable(
    #     name="Gripper proximity?",
    #     variable_name="proximity_detector",
    #     clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    #     expected_value=0.5,
    #     comparison_operator=operator.le,
    # )

    root_var_tracker = py_trees.composites.Sequence(
        children=[
            check_obj_rest,
            check_obj_locked_1,
            # py_trees.behaviours.Success("Successor 1"),
            # py_trees.behaviours.Failure("Failure 1"),
            # py_trees.behaviours.SuccessEveryN(name="Successor 2",n=1),
            # check_obj_rest,
            # check_obj_locked
            # py_trees.behaviours.SuccessEveryN(name="Successor 3",n=1),
        ]    
    )

    root_tracker_occlusion = py_trees.composites.Selector(
        children=[
            # grasp_state,
            # check_grasp_selected,
            check_gripper_proximity,
            root_var_tracker,
            # py_trees.behaviours.Success("Successor 1"),
            # py_trees.behaviours.Failure("Failure 1"),
            # py_trees.behaviours.SuccessEveryN(name="Successor 2",n=1),
            # check_obj_rest,
            # check_obj_locked
            # py_trees.behaviours.SuccessEveryN(name="Successor 3",n=1),
        ]    
    )

    root_tracking = py_trees.composites.Sequence(
        children=[
            root_detection,
            py_trees.decorators.FailureIsRunning(   # Otherwhise the robot actions will fail
            check_obj_locked_2,
            )
        ]
    )


    root_perception = py_trees.composites.Selector(
        children=[
            root_tracker_occlusion,
            root_tracking,
        ]
    )

    return root_perception

def get_bt_track_scan_select_grasp_drop(subtree=None):
    # Action: scan
    action_scan_goal = ScanSceneGoal()
    action_scan = ActionClient_ResultSaver(
        name="action_scan",
        action_spec=ScanSceneAction,
        action_goal=action_scan_goal,
        action_namespace="scan_action",
        set_flag_instead_result=False,
    )

    check_scene_scanned = py_trees.blackboard.CheckBlackboardVariable(
        name="Scene scanned?",
        variable_name="action_scan_result",
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )

    # button_next = get_button_next_check()

    root_scan = py_trees.composites.Selector(
        children=[
            check_scene_scanned,
            action_scan,
        ]
    )


    # Action: select grasp
    action_grasp_select = ActionClient_BBgoal(
        name="action_select",
        action_spec=SelectGraspAction,
        action_namespace="grasp_selection_action",
        goal_gen_callback=generate_selection_goal_msg,
        bb_goal_var_name="action_scan_result",
        set_flag_instead_result=False,
    )

    check_grasp_selected = py_trees.blackboard.CheckBlackboardVariable(
        name="Grasp selected?",
        variable_name="action_select_result",
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )


    root_grasp_select = py_trees.composites.Selector(
        children=[
            check_grasp_selected,
            py_trees.composites.Sequence(
                children=[
                    root_scan,
                    action_grasp_select
                ]
            ),
        ]
    )

    # Action: grasp
    action_grasp = ActionClient_BBgoal(
        name="action_grasp",
        action_spec=GraspAction,
        action_namespace="grasp_execution_action",
        goal_gen_callback=generate_grasp_goal_msg,
        bb_goal_var_name="action_select_result",
        set_flag_instead_result=True,
    )

    check_object_in_hand = py_trees.blackboard.CheckBlackboardVariable(
        name="Object in hand?",
        variable_name="action_grasp_result",
        expected_value=True,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )


    root_grasp = py_trees.composites.Selector(
        children=[
            check_object_in_hand,
            py_trees.composites.Sequence(
                children=[
                    root_grasp_select,
                    action_grasp
                ]
            ),
        ]
    )


    # Action: drop
    action_drop_goal = DropGoal()
    action_drop = ActionClient_ResultSaver(
        name="action_drop",
        action_spec=DropAction,
        action_goal=action_drop_goal,
        action_namespace="drop_action",
        set_flag_instead_result=True,
    )

    check_object_at_target = py_trees.blackboard.CheckBlackboardVariable(
        name="Object at target?",
        variable_name="action_drop_result",
        expected_value=True,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )


    root_drop = py_trees.composites.Selector(
        children=[
            check_object_at_target,
            py_trees.composites.Sequence(
                children=[
                    root_grasp,
                    action_drop,
                ]
            ),
        ]
    )

    return root_drop


def get_root():
    # For a sketch of the tree layout, see here (slide 2): https://docs.google.com/presentation/d/1swC5c1mbVn2TRDar-y0meTbrC9BUHnT9XWYPeFJlNxM/edit#slide=id.g70bc070381_0_32

    condition_variable_names = [
        "action_drop_result",
        "action_grasp_result",
        "action_select_result",
        "action_scan_result",
        "action_detect_result",
        "action_tracking_result"
    ]

    # Add reset button
    reset_root, reset_exec_root = get_bt_reset(condition_variable_names, reset_all=True)


    # Subscriber
    subscriber_root = get_bt_topics2bb()

    # Add nodes with condition checks and actions
    root_robot = get_bt_track_scan_select_grasp_drop()

    root_perception = get_bt_perception()

    root_action = py_trees.composites.Parallel(
        children=[
            root_perception,
            root_robot,
        ]
    )

    root_action = root_action

    # Assemble tree
    action_root = py_trees.composites.Selector(
        children=[
            reset_exec_root,
            # repeat_exec_root,
            root_action]
    )
    root = py_trees.composites.Parallel(
        policy="SUCCESS_ON_ONE",
        children=[
            reset_root,
            # repeat_root,
            subscriber_root,
            action_root
        ]
    )

    return root


class PandaTree:
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
