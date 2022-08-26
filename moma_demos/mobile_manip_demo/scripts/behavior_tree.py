#!/usr/bin/env python

import mobile_manip_demo.behaviors as bt
from mobile_manip_demo.environment import get_place_pose

import rospy

import functools
import numpy as np
import py_trees


def post_tick_handler(snapshot_visitor, behavior_tree):
    """Prints an ascii tree with the current snapshot status."""
    print(
        "\n"
        + py_trees.display.unicode_tree(
            root=behavior_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited,
        )
    )
    name_ = "root" + str(behavior_tree.count)


class MoMaBT:
    def __init__(self, cube_ID: int):
        """Initialize ROS nodes."""
        # Parameters
        self.delivery = rospy.get_param("moma_demo/delivery_station")
        self.place_target = rospy.get_param("moma_demo/place_pose")
        self.place_pose = get_place_pose(
            np.array(self.delivery), np.array(self.place_target)
        )

        # Mobile Picking:
        move_to_pick = py_trees.composites.Selector(name="Fallback")
        move_to_pick.add_children(
            [
                bt.RobotAtPose(
                    name=f"Robot-At cube{cube_ID}?",
                    robot_name="panda",
                    pose=cube_ID,
                    tolerance=0.1,
                ),
                bt.Move(name=f"Move-To cube{cube_ID}!", goal_ID=cube_ID),
            ]
        )

        pick_sequence = bt.RSequence(name="Sequence")
        pick_sequence.add_children(
            [
                move_to_pick,
                bt.Pick(name=f"Pick cube{cube_ID}!", goal_ID=cube_ID),
            ]
        )
        pick = py_trees.composites.Selector(name="Fallback")
        pick.add_children(
            [
                bt.InHand(name=f"Cube{cube_ID} in Hand?"),
                pick_sequence,
            ]
        )

        # Mobile Placing:
        move_to_place = py_trees.composites.Selector(name="Fallback")
        move_to_place.add_children(
            [
                bt.RobotAtPose(
                    name=f"Robot-At delivery?",
                    robot_name="panda",
                    pose=np.array(self.delivery),
                    tolerance=0.1,
                ),
                bt.Move(name=f"Move-To delivery!", goal_pose=np.array(self.delivery)),
            ]
        )

        place_sequence = bt.RSequence(name="Sequence")
        place_sequence.add_children(
            [
                pick,
                move_to_place,
                bt.Place(
                    name=f"Place cube{cube_ID}!",
                    goal_ID=cube_ID,
                    goal_pose=self.place_target,
                ),
            ]
        )

        # MoMa
        self.root = py_trees.composites.Selector(name="Fallback")
        self.root.add_children(
            [
                bt.ObjectAtPose(
                    name=f"Cube{cube_ID} in delivery?",
                    object_id=cube_ID,
                    model_type="cubes",
                    pose=self.place_pose,
                    tolerance=np.array([0.5, 0.5, 0.1]),
                ),
                place_sequence,
            ]
        )

        self.tree = py_trees.trees.BehaviourTree(self.root)
        py_trees.display.render_dot_tree(self.tree.root, name="moma_bt")

    def get_root(self) -> py_trees.composites.Selector:
        return self.root

    def run(self):
        self.tree.visitors.append(py_trees.visitors.DebugVisitor())
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            functools.partial(post_tick_handler, snapshot_visitor)
        )
        self.tree.visitors.append(snapshot_visitor)
        self.tree.setup(timeout=15)

        while not rospy.is_shutdown():
            rospy.Rate(3).sleep()
            self.tree.tick()


def main():
    rospy.init_node("BehaviorTree")
    node = MoMaBT(2)

    try:
        node.run()
        pass
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
