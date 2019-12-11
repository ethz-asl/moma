#!/usr/bin/env python

from __future__ import print_function

import copy
import pickle

import actionlib
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
import tf
from std_srvs.srv import Trigger

from grasp_demo.msg import ScanSceneAction, ScanSceneResult
from grasp_demo.panda_commander import PandaCommander

class ScanActionNode(object):
    def __init__(self):
        # Panda commander
        self.panda_commander = PandaCommander("panda_arm")

        # Scan joint configurations
        self.scan_joints = [
            [
                -0.13044640511344546,
                -1.3581389637834451,
                -0.7902883262964496,
                -2.2897565394284434,
                0.04394716020347789,
                1.2693998432689242,
                1.0239726927690207,
            ],
            [
                1.1617,
                -0.5194,
                -1.6787,
                -1.345415587151993,
                0.27857345296034125,
                0.9858770641287168,
                0.48917187201250106,
            ],
            [
                0.03534398500304972,
                0.0792812212202517,
                0.00024001071061285442,
                -1.2297476580268456,
                0.07415731295971527,
                0.9772190555466544,
                0.8901656288974905,
            ],
            [
                -1.0761406668405233,
                -0.9812572320940955,
                1.0510634288286071,
                -1.4203576079753408,
                0.2339650344716178,
                0.9772807318199377,
                0.6811464487329869,
            ],
            [
                -0.3038625468067955,
                -1.2148889997380417,
                0.812242864175488,
                -1.754142465942784,
                0.16873402494854398,
                1.1185604270829095,
                0.9999408414952058,
            ],
        ]

        # Service proxys to control the point cloud fusion
        rospy.wait_for_service("/point_cloud_reconstruction/reset")

        self.reset_fusion = rospy.ServiceProxy(
            "/point_cloud_reconstruction/reset", Trigger
        )
        self.start_fusion = rospy.ServiceProxy(
            "/point_cloud_reconstruction/start", Trigger
        )
        self.stop_fusion = rospy.ServiceProxy(
            "/point_cloud_reconstruction/stop", Trigger
        )

        # Create publisher for stitched point cloud
        self.stitched_point_cloud_pub = rospy.Publisher(
            "cloud", PointCloud2, queue_size=10
        )

        # Set up action server
        action_name = "pointcloud_scan_action"
        self._as = actionlib.SimpleActionServer(
            action_name, ScanSceneAction, execute_cb=self.execute_cb, auto_start=False
        )
        self._as.start()

        rospy.loginfo("Scan action server ready")

    def execute_cb(self, goal):
        rospy.loginfo("Scanning action was triggered")
        result = ScanSceneResult()

        if goal.num_scan_poses > len(self.scan_joints):
            rospy.info("Invalid goal set")
            self._as.set_aborted(result)
            return

        self.reset_fusion()
        self.start_fusion()

        # Perform the scan trajectory
        for i in range(goal.num_scan_poses):

            if self._as.is_preempt_requested():
                rospy.loginfo("Got preempted")
                self._as.set_preempted()
                return

            self.panda_commander.goto_joint_target(
                self.scan_joints[i], max_velocity_scaling=0.5
            )

        # Wait for the latest reconstruction
        stitched_cloud = rospy.wait_for_message(
            "/point_cloud_reconstruction/point_cloud", PointCloud2
        )
        self.stop_fusion()

        # Send reconstructed point cloud to GPD
        self.stitched_point_cloud_pub.publish(stitched_cloud)

        rospy.loginfo("Scanning action succeeded")
        self._as.set_succeeded(result)

def main():
    try:
        rospy.init_node("scan_action_node")
        ScanActionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
