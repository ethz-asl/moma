import py_trees
import time
import rospy

from execution.behaviour_tree import PandaTree


def main():
    rospy.init_node("behaviour_tree_node")

    pt = PandaTree(debug=False)
    
    index = 1
    while not rospy.is_shutdown():
        print("\n--------- Tick {0} ---------\n".format(index))
        pt.tree.tick()
        print("\n")
        py_trees.display.print_ascii_tree(pt.tree.root, show_status=True)
        index += 1
        time.sleep(1.0)   # Not really needed. Just for debugging.

    # Alternative to the while loop:
    # pt.tree.tick_tock(sleep_ms=500, number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK)

if __name__ == "__main__":
    main()
