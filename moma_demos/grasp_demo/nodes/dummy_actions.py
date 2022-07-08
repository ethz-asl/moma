#!/usr/bin/env python3

import actionlib
from grasp_demo.msg import (
    ScanSceneAction,
    ScanSceneResult,
    GraspAction,
    GraspResult,
    DropAction,
    DropResult,
    SelectGraspAction,
    SelectGraspResult,
)
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import rospy


class DummyActionServers:
    def __init__(self):
        action_name = "scan_action"
        self.action_server_scan = actionlib.SimpleActionServer(
            action_name, ScanSceneAction, execute_cb=self.scan_cb, auto_start=False
        )
        self.action_server_scan.start()
        rospy.loginfo("Scan action server running.")

        action_name = "grasp_selection_action"
        self.action_server_grasp_select = actionlib.SimpleActionServer(
            action_name,
            SelectGraspAction,
            execute_cb=self.grasp_selection_cb,
            auto_start=False,
        )
        self.action_server_grasp_select.start()
        rospy.loginfo("Grasp select server running.")

        action_name = "grasp_execution_action"
        self.action_server_grasp = actionlib.SimpleActionServer(
            action_name, GraspAction, execute_cb=self.grasp_cb, auto_start=False
        )
        self.action_server_grasp.start()
        rospy.loginfo("Grasp action server running.")

        action_name = "drop_action"
        self.action_server_drop = actionlib.SimpleActionServer(
            action_name, DropAction, execute_cb=self.drop_cb, auto_start=False
        )
        self.action_server_drop.start()
        rospy.loginfo("Drop action server running.")

    def scan_cb(self, goal):
        rospy.loginfo("Scanning action was triggered")
        result = ScanSceneResult()

        pc = PointCloud2()

        rospy.sleep(1.0)

        rospy.loginfo("Success")
        result.pointcloud_scene = pc
        self.action_server_scan.set_succeeded(result)

    def grasp_selection_cb(self, goal):
        rospy.loginfo("Grasp selection was triggered")
        result = SelectGraspResult()

        grasp_pose = PoseStamped()

        rospy.sleep(1.0)

        rospy.loginfo("Success")
        result.target_grasp_pose = grasp_pose
        self.action_server_grasp_select.set_succeeded(result)

    def grasp_cb(self, goal):
        rospy.loginfo("Grasping action was triggered")
        result = GraspResult()
        rospy.sleep(1.0)
        rospy.loginfo("Success")
        self.action_server_grasp.set_succeeded(result)

    def drop_cb(self, goal):
        rospy.loginfo("Drop action was triggered")
        result = DropResult()
        rospy.sleep(5.0)
        rospy.loginfo("Success")
        self.action_server_drop.set_succeeded(result)


def main():
    rospy.init_node("grasp_demo_dummy_actions_node")

    das = DummyActionServers()

    rospy.spin()


if __name__ == "__main__":
    main()
