#!/usr/bin/env python

from __future__ import print_function

import py_trees
import time
import rospy

from grasp_demo.execution.behaviour_tree_reactive import PandaTree
# from grasp_demo.execution.behaviour_tree_reactive import PandaTree

DEBUG = False
PRINT_TREE = False
RENDER_TREE = True

def main():
    rospy.init_node("behaviour_tree_node")

    pt = PandaTree(debug=DEBUG,render_tree=RENDER_TREE)
    pt.setup()

    index = 1
    rospy.loginfo("Starting loop...")
    while not rospy.is_shutdown():
        if PRINT_TREE:
            print("\n--------- Tick {0} ---------\n".format(index))
        pt.tree.tick()
        if PRINT_TREE:
            py_trees.display.print_ascii_tree(pt.tree.root, show_status=True)
            # py_trees.display.render_dot_tree(pt)
        index += 1
        freqencie = rospy.get_param("/moma_demo/tick_tock_times")
        time.sleep(float(1/freqencie))  # Not really needed. Just for debugging.

    # Alternative to the while loop:
    # pt.tree.tick_tock(sleep_ms=1000, number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK)


if __name__ == "__main__":
    main()
