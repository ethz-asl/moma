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
    reset_root.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

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
        blackbox_level=py_trees.common.BlackBoxLevel.DETAIL,
    )
    reset_exec_root.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
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
    repeat_exec_root = py_trees.composites.Sequence(
        children=[check_var_repeat, clear_var_repeat, repeat_action],
        blackbox_level=py_trees.common.BlackBoxLevel.DETAIL,
    )
    repeat_exec_root.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL


def get_bt_scan_select_grasp_drop(subtree=None):
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

    button_next = py_trees_ros.subscribers.WaitForData(
        name="Button next?",
        topic_name="/manipulation_actions/next",
        topic_type=std_msgs.msg.Empty,
    )

    root_scan = py_trees.composites.Selector(
        children=[
            check_scene_scanned,
            py_trees.composites.Sequence(
                children=[button_next, action_scan]
                if subtree is None
                else [subtree, button_next, action_scan]
            ),
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

    button_next = py_trees_ros.subscribers.WaitForData(
        name="Button next?",
        topic_name="/manipulation_actions/next",
        topic_type=std_msgs.msg.Empty,
    )

    root_grasp_select = py_trees.composites.Selector(
        children=[
            check_grasp_selected,
            py_trees.composites.Sequence(
                children=[root_scan, button_next, action_grasp_select]
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

    button_next = py_trees_ros.subscribers.WaitForData(
        name="Button next?",
        topic_name="/manipulation_actions/next",
        topic_type=std_msgs.msg.Empty,
    )

    root_grasp = py_trees.composites.Selector(
        children=[
            check_object_in_hand,
            py_trees.composites.Sequence(
                children=[root_grasp_select, button_next, action_grasp]
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

    button_next = py_trees_ros.subscribers.WaitForData(
        name="Button next?",
        topic_name="/manipulation_actions/next",
        topic_type=std_msgs.msg.Empty,
    )

    root_drop = py_trees.composites.Selector(
        children=[
            check_object_at_target,
            py_trees.composites.Sequence(
                children=[root_grasp, button_next, action_drop]
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
    ]

    # Add reset button
    reset_root, reset_exec_root = get_bt_reset(condition_variable_names, reset_all=True)

    # Add repeat button
    repeat_root, repeat_exec_root = get_bt_reset(
        condition_variable_names, reset_all=False
    )

    # Add nodes with condition checks and actions
    root_drop = get_bt_scan_select_grasp_drop()

    # Assemble tree
    action_root = py_trees.composites.Selector(
        children=[reset_exec_root, repeat_exec_root, root_drop]
    )
    root = py_trees.composites.Parallel(children=[reset_root, repeat_root, action_root])

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
        self.tree.setup(timeout=15)
