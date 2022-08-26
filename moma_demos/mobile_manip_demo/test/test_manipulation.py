#!/usr/bin/env python

"""
Test the manipulation skills of grasping and dropping.

Note: this test works only if the robot spawns in front of the right table.
In run_gazebo.launch, set the following parameters:
<arg name="initial_pose_x" default="-1.4" />
<arg name="initial_pose_y" default="1.96" />
<arg name="initial_pose_yaw" default="3.14" />

Then, after spawning the robot, move it closer to the table.
"""

from geometry_msgs.msg import PoseStamped
from mobile_manip_demo.robot_interface import InHand, Pick, Place, ObjectAtPose
from moma_utils.ros.moveit import MoveItClient

import rospy
import tf2_ros

import numpy as np


class ManipulationNode:
    def __init__(self):
        """Initialize ROS nodes."""
        # Parameters
        self.with_id = True

        self.pick_action = Pick()
        self.place_action = Place()

        self.pick_condition = InHand()
        self.place_condition = ObjectAtPose(2)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.moveit = MoveItClient("panda_arm")

    def check_place_condition(self):
        condition = self.place_condition.at_pose(
            target_pose=np.array([-1.2, -2.0, 0.75]),
            tolerance=np.array([0.5, 0.5, 0.1]),
        )
        rospy.logwarn(f"Place condition: {condition}")

    def send_manip_request(self):
        # Put arm on a named configuration
        self.moveit.goto("ready")

        self.pick_action.cancel_goal()

        pick_goal, place_goal = self.get_manip_goals()

        if self.with_id:
            self.pick_action.initialize_pick(goal_ID=2)
        else:
            self.pick_action.initialize_pick(pick_goal.pose, goal_ID=2)
            rospy.loginfo(f"Sending goal:\n {pick_goal.pose.position}.")

        pick_done = False
        while not pick_done:
            rospy.Rate(1).sleep()
            rospy.logwarn(f"Pick condition: {self.pick_condition.in_hand()}")
            status = self.pick_action.get_pick_status()
            if status == 0 or status == 1:
                rospy.loginfo("pick RUNNING")
            elif status == 3:
                rospy.loginfo("pick SUCCESS")
                pick_done = True
            else:
                rospy.loginfo(str(status))
                rospy.loginfo("pick FAILURE")
                pick_done = True

        # If pick is done, place where you picked
        if pick_done:
            self.place_action.initialize_place(goal_pose=place_goal.pose, goal_ID=2)

            place_done = False
            while not place_done:
                rospy.Rate(1).sleep()
                status = self.place_action.get_place_status()
                if status == 0 or status == 1:
                    rospy.loginfo("place RUNNING")
                elif status == 3:
                    self.check_place_condition()
                    rospy.loginfo("place SUCCESS")
                    place_done = True
                else:
                    self.check_place_condition()
                    rospy.loginfo("place FAILURE")
                    place_done = True

    def get_manip_goals(self):
        done = False
        attempts = 0
        while not done and attempts < 15:
            try:
                msg1 = self.tf_buffer.lookup_transform(
                    "task",
                    "tag_2",
                    rospy.Time(),
                    rospy.Duration(5),
                )
                msg2 = self.tf_buffer.lookup_transform(
                    "panda_link0",
                    "tag_2",
                    rospy.Time(),
                    rospy.Duration(5),
                )
                done = True
            except Exception:
                attempts += 1
                rospy.Rate(1).sleep()
                rospy.logerr("Could not get transform, retrying...")

        pick_goal = PoseStamped()
        pick_goal.header.frame_id = "task"
        pick_goal.header.stamp = rospy.Time.now()
        pick_goal.pose.position.x = msg1.transform.translation.x
        pick_goal.pose.position.y = msg1.transform.translation.y
        pick_goal.pose.position.z = msg1.transform.translation.z
        pick_goal.pose.orientation = msg1.transform.rotation

        place_goal = PoseStamped()
        place_goal.header.frame_id = "panda_link0"
        place_goal.header.stamp = rospy.Time.now()
        place_goal.pose.position.x = msg2.transform.translation.x
        place_goal.pose.position.y = msg2.transform.translation.y
        place_goal.pose.position.z = msg2.transform.translation.z + 0.05
        place_goal.pose.orientation.x = 1.0
        place_goal.pose.orientation.y = 0.0
        place_goal.pose.orientation.z = 0.0
        place_goal.pose.orientation.w = 0.0

        return pick_goal, place_goal


def main():
    rospy.init_node("manipulation_tester_node")
    node = ManipulationNode()

    try:
        node.send_manip_request()
        # node.check_place_condition()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
