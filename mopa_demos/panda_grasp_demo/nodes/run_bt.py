#!/usr/bin/env python

import py_trees
import time
import rospy

from panda_grasp_demo.execution.behaviour_tree import PandaTree

DEBUG = False
PRINT_TREE = True

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
        time.sleep(1.0)   # Not really needed. Just for debugging.

    # Alternative to the while loop:
    # pt.tree.tick_tock(sleep_ms=1000, number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK)

if __name__ == "__main__":
    main()
