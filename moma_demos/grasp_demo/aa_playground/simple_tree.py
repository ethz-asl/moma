#!/usr/bin/env python

from __future__ import print_function

import py_trees
import time
import rospy
import py_trees_ros

import std_msgs

# from grasp_demo.execution.behaviour_tree import PandaTree

DEBUG = False
PRINT_TREE = True

def get_bb():
    # read_Sub = py_trees_ros.subscribers.EventToBlackboard(
    #     name="Read Subscriber from Publisher",
    #     topic_name="/test_topic/value",
    #     variable_name="value",
    # )

    read_Sub = py_trees_ros.subscribers.ToBlackboard(
        name="Read Subscriber from Publisher",
        topic_name="/test_topic/value",
        topic_type=std_msgs.msg.String,
        blackboard_variables={"action_drop_result": 'data'},
        initialise_variables={"action_drop_result": 'text'}
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB", children=[read_Sub])
    topics2bb.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    return topics2bb

def get_arm():
    first_elem = py_trees.composites.Sequence(name="first Elem")
    successor = py_trees.behaviours.Success(name="Successor")
    first_elem.add_child(successor)
    return first_elem

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
