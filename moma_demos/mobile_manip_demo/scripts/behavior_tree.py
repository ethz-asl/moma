#!/usr/bin/env python

import mobile_manip_demo.behaviors as bt

import rospy

import py_trees


class MoMaBT:
    def __init__(self, cube_ID: int):
        """Initialize ROS nodes."""
        # Parameters
        self.delivery = rospy.get_param("moma_demo/delivery_station")
        self.place_pose = rospy.get_param("moma_demo/place_pose")

        # Mobile Picking:
        move_to_pick = py_trees.composites.Selector(name="Fallback")
        move_to_pick.add_children(
            [
                bt.RobotAtPose(
                    name=f"Robot-At cube{cube_ID}?",
                    robot_name="panda",
                    pose=cube_ID,
                    tolerance=0.2,
                ),
                bt.Move(name=f"Move-To cube{cube_ID}!", goal_ID=cube_ID),
            ]
        )

        pick_sequence = py_trees.composites.Sequence(name="Sequence")
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
                    pose=self.delivery,
                    tolerance=0.4,
                ),
                bt.Move(name=f"Move-To delivery!", goal_pose=self.delivery),
            ]
        )

        place_sequence = py_trees.composites.Sequence(name="Sequence")
        place_sequence.add_children(
            [
                pick,
                move_to_place,
                bt.Place(
                    name=f"Place cube{cube_ID}!",
                    goal_ID=cube_ID,
                    goal_pose=self.place_pose,
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
                    tolerance=0.4,
                ),
                place_sequence,
            ]
        )

        self.tree = py_trees.trees.BehaviourTree(self.root)
        py_trees.display.render_dot_tree(self.tree.root, name="moma_bt")

    def get_root(self) -> py_trees.composites.Selector:
        return self.root

    def run(self):
        while not rospy.is_shutdown():
            # print(self.root.status)
            self.root.tick_once()


def main():
    rospy.init_node("BehaviorTree")
    node = MoMaBT(2)

    try:
        # node.run()
        pass
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
