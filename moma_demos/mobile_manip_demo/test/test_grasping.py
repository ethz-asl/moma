#!/usr/bin/env python

from geometry_msgs.msg import Pose
from mobile_manip_demo.robot_interface import Pick, ObjectAtPose

import rospy
import tf2_ros
import tf2_geometry_msgs

import numpy as np


class GraspingNode:
    def __init__(self):
        """Initialize ROS nodes."""
        # Parameters

        self.goal = Pose()
        self.goal.position.x = -2.50
        self.goal.position.y = 2.0
        self.goal.position.z = 0.69
        self.goal.orientation.x = 0.0225540390197495
        self.goal.orientation.y = -0.026169554672720013
        self.goal.orientation.z = -0.7232745208065945
        self.goal.orientation.w = 0.6896959020351779

        self.pick_action = Pick()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def send_pick_request(self):
        self.pick_action.cancel_goal()

        pose_transformed = self.__transform_pose(self.goal, "map", "task")

        done = False
        while not done:
            try:
                msg = self.tf_buffer.lookup_transform(
                    "tag_2",
                    "task",
                    rospy.Time(),
                    rospy.Duration(5),
                )
                done = True
            except Exception:
                rospy.logerr("Could not get transform, retrying...")

        goal = Pose()
        goal.position.x = msg.transform.translation.x
        goal.position.y = msg.transform.translation.y
        goal.position.z = msg.transform.translation.z
        goal.orientation = msg.transform.rotation

        self.pick_action.initialize_pick(goal)

        # pick_done = False
        # while not pick_done:
        #     status = self.pick_action.get_pick_status()
        #     if status == 0 or status == 1:
        #         rospy.loginfo("pick RUNNING")
        #     elif status == 3:
        #         rospy.loginfo("pick SUCCESS")
        #         pick_done = True
        #     else:
        #         rospy.loginfo(str(status))
        #         rospy.loginfo("pick FAILURE")
        #         pick_done = True

    def __transform_pose(
        self, input_pose: Pose, from_frame: str, to_frame: str
    ) -> Pose:
        # **Assuming /tf2 topic is being broadcasted

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = self.tf_buffer.transform(
                pose_stamped, to_frame, rospy.Duration(5)
            )
            return output_pose_stamped.pose
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            raise


def main():
    rospy.init_node("grasping_tester_node")
    node = GraspingNode()

    try:
        node.send_pick_request()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
