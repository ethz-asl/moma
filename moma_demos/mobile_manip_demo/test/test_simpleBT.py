#!/usr/bin/env python

from typing import Any, List
from geometry_msgs.msg import Pose
from mobile_manip_demo.behaviors import Move, RobotAtPose

import rospy
import tf2_ros

import numpy as np
import py_trees


class SimpleBT:
    def __init__(self):
        """Initialize ROS nodes."""
        # Parameters

        self.root = py_trees.composites.Selector(name="Root")
        self.root.add_children(
            [
                RobotAtPose(
                    name="Robot at cube 2?", robot_name="panda", pose=2, tolerance=0.2
                ),
                Move(name="Move to cube 2!", goal_ID=2),
            ]
        )

    def run(self):
        bt = py_trees.trees.BehaviourTree(self.root)
        py_trees.display.render_dot_tree(bt.root, name="bt")
        while not rospy.is_shutdown():
            print(self.root.status)
            self.root.tick_once()


def main():
    rospy.init_node("simpleBT")
    node = SimpleBT()

    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
