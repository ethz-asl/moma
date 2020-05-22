#!/usr/bin/env python

from __future__ import print_function

import py_trees
import time
import rospy
import py_trees_ros

import std_msgs

from grasp_demo.msg import (
    DropAction,
    DropGoal,
    ScanSceneAction,
    ScanSceneGoal,
)

# from grasp_demo.execution.behaviour_tree import PandaTree

DEBUG = False
PRINT_TREE = True

def get_bb():
    # read_Sub = py_trees_ros.subscribers.EventToBlackboard(
    #     name="Read Subscriber from Publisher",
    #     topic_name="/test_topic/value",
    #     variable_name="value",
    # )

    wait4scan = py_trees_ros.subscribers.ToBlackboard(
        name="do_scan",
        topic_name="/bt_BB/ScannedBB",
        topic_type=std_msgs.msg.Bool,
        blackboard_variables={"do_action_scan": 'data'},
        initialise_variables={"do_action_scan": False}
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB", children=[wait4scan])
    # topics2bb.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    topics2bb.blackbox_level = py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX
    return topics2bb

def get_arm():
    top_element = py_trees.composites.Sequence(name="tol Elem")
    first_elem = py_trees.composites.Selector(name="first Elem")

    check_var_scan = py_trees.blackboard.CheckBlackboardVariable(
        name="do_scan",
        variable_name="do_action_scan",
        expected_value=True,
    )
    # action_scan = py_trees.behaviours.Success(name="Action Scan")
    action_scan = py_trees_ros.actions.ActionClient(
        name="Action Scan",
        action_spec=ScanSceneAction,
        action_goal=ScanSceneGoal(),
        action_namespace="scan_action",
        override_feedback_message_on_running="scanning"
        )

    # action_drop = py_trees_ros.actions.ActionClient(
    #     name="Action Drop",
    #     action_spec=DropAction,
    #     action_goal=DropGoal(),
    #     action_namespace="drop_action_node",
    #     override_feedback_message_on_running="dropping"
    #     )
    # set_var_scan = py_trees.blackboard.SetBlackboardVariable(
    #     name="stop_scan",
    #     variable_name= "do_action_scan",
    #     variable_value= True,
    # )

    # subBranch = py_trees.composites.Sequence("subBranche")
    # subBranch.add_children([action_scan,set_var_scan])

    successor = py_trees.behaviours.Success(name="successor")

    top_element.add_children([first_elem,successor])
    # first_elem.add_children([check_var_scan,subBranch])
    first_elem.add_children([check_var_scan,action_scan])
    return top_element

def get_root():
    blackboard = get_bb()

    arm = get_arm()

    root = py_trees.composites.Parallel(
        children=([blackboard,arm])
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

def main():
    rospy.init_node("behaviour_tree_node")

    pt = PandaTree(debug=DEBUG)
    pt.setup()

    index = 1
    rospy.loginfo("Starting loop...")
    while not rospy.is_shutdown():
        if PRINT_TREE:
            print("\n--------- Tick {0} ---------\n".format(index))
        pt.tree.tick()
        if PRINT_TREE:
            py_trees.display.print_ascii_tree(pt.tree.root, show_status=True)
        index += 1
        time.sleep(0.5)  # Not really needed. Just for debugging.

    # Alternative to the while loop:
    # pt.tree.tick_tock(sleep_ms=1000, number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK)


if __name__ == "__main__":
    main()
