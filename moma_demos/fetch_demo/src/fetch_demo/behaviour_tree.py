import py_trees
import py_trees_ros

from grasp_demo.msg import (
    ScanSceneAction,
    ScanSceneGoal,
    GraspAction,
    GraspGoal,
    DropGoal,
    DropAction,
)
from fetch_demo.msg import SearchAction, SearchGoal, ApproachAction, ApproachGoal
from grasp_demo.execution.action_client import (
    ActionClient_ResultSaver,
    ActionClient_BBgoal,
    RepeatAction,
)

import std_msgs


def generate_grasp_goal_msg(target_grasp):
    goal = GraspGoal()
    goal.target_grasp_pose = target_grasp.selected_grasp_pose
    return goal


def get_root():

    condition_variable_names = [
        "action_drop_result",
        "action_grasp_result",
        "action_scan_result",
        "action_approach_result",
        "action_search_result",
    ]

    # -------- Add reset button -----------------------------------------

    button_reset = py_trees_ros.subscribers.WaitForData(
        name="Button reset?",
        topic_name="/manipulation_actions/reset",
        topic_type=std_msgs.msg.Empty,
    )
    var_reset = py_trees.blackboard.SetBlackboardVariable(
        variable_name="do_reset", variable_value=True
    )
    reset_root = py_trees.composites.Sequence(children=[button_reset, var_reset])
    reset_root.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    # Reset exec
    check_var_reset = py_trees.blackboard.CheckBlackboardVariable(
        name="Check reset var", variable_name="do_reset", expected_value=True
    )
    clear_var_reset = py_trees.blackboard.ClearBlackboardVariable(
        variable_name="do_reset"
    )
    reset_action = RepeatAction(
        name="Reset all", variable_names=condition_variable_names, repeat_all=True
    )
    reset_exec_root = py_trees.composites.Sequence(
        children=[check_var_reset, clear_var_reset, reset_action],
        blackbox_level=py_trees.common.BlackBoxLevel.DETAIL,
    )
    reset_exec_root.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    # -------- Add repeat button -----------------------------------------

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

    # -------- Add nodes with condition checks and actions -----------------------------

    # Action: follow search waypoints
    action_search_goal = SearchGoal()
    action_search = ActionClient_ResultSaver(
        name="action_search",
        action_spec=SearchAction,
        action_goal=action_search_goal,
        action_namespace="search_action",
        set_flag_instead_result=True,
    )

    check_obj_pos_known = py_trees.blackboard.CheckBlackboardVariable(
        name="Object position known?",
        variable_name="action_search_result",
        expected_value=True,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )

    button_next = py_trees_ros.subscribers.WaitForData(
        name="Button next?",
        topic_name="/manipulation_actions/next",
        topic_type=std_msgs.msg.Empty,
    )

    root_search = py_trees.composites.Selector(
        children=[
            check_obj_pos_known,
            py_trees.composites.Sequence(children=[button_next, action_search]),
        ]
    )

    # Action: approach object
    action_approach_goal = ApproachGoal()
    action_approach = ActionClient_ResultSaver(
        name="action_approach",
        action_spec=ApproachAction,
        action_goal=action_approach_goal,
        action_namespace="approach_action",
        set_flag_instead_result=True,
    )

    check_obj_in_reach = py_trees.blackboard.CheckBlackboardVariable(
        name="Object in reach?",
        variable_name="action_approach_result",
        expected_value=True,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    )

    button_next = py_trees_ros.subscribers.WaitForData(
        name="Button next?",
        topic_name="/manipulation_actions/next",
        topic_type=std_msgs.msg.Empty,
    )

    root_approach = py_trees.composites.Selector(
        children=[
            check_obj_in_reach,
            py_trees.composites.Sequence(
                children=[root_search, button_next, action_approach]
            ),
        ]
    )

    # Action: scan
    action_scan_goal = ScanSceneGoal()
    action_scan = ActionClient_ResultSaver(
        name="action_scan",
        action_spec=ScanSceneAction,
        action_goal=action_scan_goal,
        action_namespace="pointcloud_scan_action",
        set_flag_instead_result=False,
    )

    check_grasp_computed = py_trees.blackboard.CheckBlackboardVariable(
        name="Grasp computed?",
        variable_name="action_scan_result",
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )

    button_next = py_trees_ros.subscribers.WaitForData(
        name="Button next?",
        topic_name="/manipulation_actions/next",
        topic_type=std_msgs.msg.Empty,
    )

    root_scan = py_trees.composites.Selector(
        children=[
            check_grasp_computed,
            py_trees.composites.Sequence(
                children=[root_approach, button_next, action_scan]
            ),
        ]
    )

    # Action: grasp
    action_grasp = ActionClient_BBgoal(
        name="action_grasp",
        action_spec=GraspAction,
        action_namespace="grasp_action",
        goal_gen_callback=generate_grasp_goal_msg,
        bb_goal_var_name="action_scan_result",
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
                children=[root_scan, button_next, action_grasp]
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

    action_root = py_trees.composites.Selector(
        children=[reset_exec_root, repeat_exec_root, root_drop]
    )

    # -------- Return root -----------------------------------------
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

