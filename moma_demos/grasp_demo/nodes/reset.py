#!/usr/bin/env python3

import argparse
import geometry_msgs.msg
import rospy
import std_srvs.srv
import sensor_msgs.msg
from sensor_msgs.point_cloud2 import create_cloud_xyz32
import std_msgs.msg
import tf2_ros

from vgn.rviz import Visualizer
from moma_utils.spatial import Transform
from moma_utils.ros.conversions import to_transform_msg
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaArmClient, PandaGripperClient


class ResetNode(object):
    def __init__(self, arm_id):
        self.arm_id = arm_id
        # self.base_frame_id = rospy.get_param("moma_demo/base_frame_id")
        self.base_frame_id = f"{self.arm_id}_link0"
        self.task_frame_id = rospy.get_param("moma_demo/task_frame_id")
        self.table_height = rospy.get_param("moma_demo/table_height")
        rospy.set_param("moma_demo/workspace", 0)
        self.init_robot_connection()
        self.vis = Visualizer()
        self.cloud_pub = rospy.Publisher("/scene_cloud", sensor_msgs.msg.PointCloud2)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.Service("reset", std_srvs.srv.Trigger, self.reset)
        rospy.loginfo("Reset service ready")

    def init_robot_connection(self):
        # Establish robot node connections
        self.arm = PandaArmClient()
        self.gripper = PandaGripperClient()
        self.moveit = MoveItClient(f"{self.arm_id}_arm")

        # Reset gripper
        self.gripper.grasp()

        # Add a collision box for the table
        msg = geometry_msgs.msg.PoseStamped()
        msg.header.frame_id = self.base_frame_id
        msg.pose.position.x = 0.4
        msg.pose.position.z = self.table_height
        self.moveit.scene.add_box("table", msg, size=(0.6, 0.6, 0.02))

    def reset(self, req):
        self.reset_arm()
        self.broadcast_roi()
        self.reset_vis()
        return std_srvs.srv.TriggerResponse()

    def reset_arm(self):
        if self.arm.has_error:
            self.arm.recover()
        self.gripper.release()
        self.moveit.goto("ready", velocity_scaling=0.2)

    def broadcast_roi(self):
        l = 0.3
        n = len(rospy.get_param("moma_demo/workspaces"))
        i = rospy.get_param("moma_demo/workspace")
        T_base_task = Transform.translation(
            [0.3, -n * l / 2 + i * l, self.table_height - 0.05]
        )
        msg = geometry_msgs.msg.TransformStamped()
        msg.header.frame_id = self.base_frame_id
        msg.child_frame_id = self.task_frame_id
        msg.transform = to_transform_msg(T_base_task)
        self.static_broadcaster.sendTransform(msg)
        rospy.sleep(1.0)  # wait for tf tree to be updated

    def reset_vis(self):
        self.vis.clear()
        self.vis.roi(self.task_frame_id, 0.3)
        self.cloud_pub.publish(
            create_cloud_xyz32(std_msgs.msg.Header(frame_id=self.base_frame_id), [])
        )


def main():
    rospy.init_node("reset")
    parser = argparse.ArgumentParser()
    parser.add_argument("-arm_id", type=str, default="panda")
    args, _ = parser.parse_known_args()
    ResetNode(arm_id=args.arm_id)
    rospy.spin()


if __name__ == "__main__":
    main()
