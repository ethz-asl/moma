#!/usr/bin/env python

import mobile_manip_demo.behaviors as bt
from mobile_manip_demo.environment import get_place_pose
from mobile_manip_demo.visualizer import BTVisualizer

import rospy

import functools
import networkx as nx
import numpy as np
import py_trees
import random


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
    def __init__(self):
        """Initialize ROS nodes."""
        cube_ID = 2
        # Parameters
        delivery = rospy.get_param("moma_demo/delivery_station")
        place_target = rospy.get_param("moma_demo/place_pose")
        place_pose = get_place_pose(np.array(delivery), np.array(place_target))
        search_locations = rospy.get_param("moma_demo/search_waypoints")
        dock_pose = rospy.get_param("moma_demo/inspection_station")

        task_type = rospy.get_param("moma_demo/experiment")
        self.visualization_only = rospy.get_param("moma_demo/visualization_only")

        # Search for Cubes
        # search = py_trees.composites.Selector(name="Fallback")
        # search.add_children(
        #     [
        #         bt.Found(
        #             name=f"Cubes found?",
        #             IDs=[2],
        #         ),
        #         bt.Search(name=f"Search cubes!", targets=search_locations),
        #     ]
        # )

        # move_sequence = bt.RSequence(name="Sequence")
        # move_sequence.add_children(
        #     [
        #         search,
        #         bt.Move(name=f"Move-To cube{cube_ID}!", goal_ID=cube_ID),
        #     ]
        # )

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
                    pose=np.array(delivery),
                    tolerance=0.15,
                ),
                bt.Move(name=f"Move-To delivery!", goal_pose=np.array(delivery)),
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
                    goal_pose=place_target,
                ),
            ]
        )

        # MoMa
        moma = py_trees.composites.Selector(name="Fallback")
        moma.add_children(
            [
                bt.ObjectAtPose(
                    name=f"Cube{cube_ID} in delivery?",
                    object_id=cube_ID,
                    model_type="cubes",
                    pose=place_pose,
                    tolerance=np.array([0.5, 0.5, 0.1]),
                ),
                place_sequence,
            ]
        )

        # Recharge
        recharge = py_trees.composites.Selector(name="Fallback")
        recharge.add_children(
            [
                bt.BatteryLv(
                    name="Battery > 20%?",
                    relation="greater",
                    value=20.0,
                ),
                bt.Recharge(name="Recharge!"),
            ]
        )

        # Dock
        dock = py_trees.composites.Selector(name="Fallback")
        dock.add_children(
            [
                bt.RobotAtPose(
                    name=f"Robot-At inspection?",
                    robot_name="panda",
                    pose=np.array(dock_pose),
                    tolerance=0.1,
                ),
                bt.Dock(name="Dock!"),
            ]
        )

        self.root = bt.RSequence(name="Sequence")
        if task_type == 2:
            self.root.add_children([recharge, moma])
        elif task_type == 3:
            self.root.add_children([recharge, moma, dock])
        else:
            self.root = moma

        self.tree = py_trees.trees.BehaviourTree(self.root)

    def get_root(self) -> py_trees.composites.Selector:
        return self.root

    def visualize(self, bt_name: str):
        """Compute the number of nodes and transition in a BT and save it as figure."""
        py_trees.display.render_dot_tree(self.tree.root, name=bt_name)
        graph = nx.DiGraph(
            nx.drawing.nx_pydot.from_pydot(py_trees.display.dot_tree(self.tree.root))
        )
        print(f"{bt_name}, {graph}")

    def run(self):
        """The BT execution is visualized in the terminal."""
        self.tree.visitors.append(py_trees.visitors.DebugVisitor())
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            functools.partial(post_tick_handler, snapshot_visitor)
        )
        self.tree.visitors.append(snapshot_visitor)
        self.tree.setup(timeout=15)

        while not rospy.is_shutdown():
            rospy.Rate(1).sleep()
            self.tree.tick()

    def run_online(self):
        """The BT execution is visualized in a Chrome page that opens upon execution."""
        viz = BTVisualizer(self.tree)

        self.tree.add_post_tick_handler(viz.update_graph)
        while not rospy.is_shutdown():
            rospy.Rate(1).sleep()
            # viz.tick()
            self.tree.tick()


def main():
    rospy.init_node("BehaviorTree")
    node = MoMaBT()

    try:
        node.visualize("big_moma")
        if not node.visualization_only:
            node.run_online()
        pass
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
