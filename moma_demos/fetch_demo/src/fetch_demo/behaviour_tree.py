import py_trees
import py_trees_ros

from fetch_demo.msg import SearchAction, SearchGoal, ApproachAction, ApproachGoal
from grasp_demo.execution.action_client import (
    ActionClient_ResultSaver,
    ActionClient_BBgoal,
    RepeatAction,
)

from grasp_demo.execution.behaviour_tree import (
    get_bt_scan_select_grasp_drop,
    get_bt_reset,
    generate_grasp_goal_msg,
    get_button_next_check,
    get_bt_topics2bb,
)

import std_msgs


def get_bt_search_approach(subtree=None):
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

    button_next = get_button_next_check()

    root_search = py_trees.composites.Selector(
        children=[
            check_obj_pos_known,
            py_trees.composites.Sequence(
                children=[button_next, action_search]
                if subtree is None
                else [subtree, button_next, action_search]
            ),
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

    button_next = get_button_next_check()

    root_approach = py_trees.composites.Selector(
        children=[
            check_obj_in_reach,
            py_trees.composites.Sequence(
                children=[root_search, button_next, action_approach]
            ),
        ]
    )
    return root_approach


def get_root():

    condition_variable_names = [
        "action_drop_result",
        "action_grasp_result",
        "action_select_result",
        "action_scan_result",
        "action_approach_result",
        "action_search_result",
    ]

    # Add reset button
    reset_root, reset_exec_root = get_bt_reset(condition_variable_names, reset_all=True)

    # Add repeat button
    repeat_root, repeat_exec_root = get_bt_reset(
        condition_variable_names, reset_all=False
    )

    # Subscriber
    subscriber_root = get_bt_topics2bb()

    # Add nodes with condition checks and actions
    root_approach = get_bt_search_approach()
    root_drop = get_bt_scan_select_grasp_drop(subtree=root_approach)

    # Assemble tree
    action_root = py_trees.composites.Selector(
        children=[reset_exec_root, repeat_exec_root, root_drop]
    )
    root = py_trees.composites.Parallel(
        children=[reset_root, repeat_root, subscriber_root, action_root]
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
        self.tree.setup(timeout=15)

