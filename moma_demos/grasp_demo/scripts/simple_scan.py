#!/usr/bin/env python

from __future__ import print_function

import copy

from actionlib import SimpleActionServer
import numpy as np
from geometry_msgs.msg import TransformStamped
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points, create_cloud
import tf
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from grasp_demo.msg import ScanSceneAction, ScanSceneResult
from panda_control.panda_commander import PandaCommander


class SimpleScanAction(object):
    """Move robot to a set of predefined scan poses and stitch together the captured point clouds."""

    def __init__(self):
        self._as = SimpleActionServer(
            "scan_action", ScanSceneAction, execute_cb=self.execute_cb, auto_start=False
        )

        self.panda_commander = PandaCommander("panda_arm")
        self.listener = tf.TransformListener()
        self.latest_cloud_data = None

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

        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.point_cloud_cb)
        self.cloud_pub = rospy.Publisher("~cloud", PointCloud2, queue_size=1)

        self._as.start()
        rospy.loginfo("Scan action server ready")

    def point_cloud_cb(self, data):
        self.latest_cloud_data = copy.deepcopy(data)

    def execute_cb(self, goal):
        """Move to each scan pose, capture a point cloud and stitch them together."""
        rospy.loginfo("Scanning action was triggered")

        captured_clouds = []

        for joints in self.scan_joints:

            if self._as.is_preempt_requested():
                rospy.loginfo("Got preempted")
                self._as.set_preempted()
                return

            self.panda_commander.goto_joint_target(joints, max_velocity_scaling=0.5)
            cloud = self.capture_point_cloud()
            captured_clouds.append(cloud)

        # Stitch the cloud
        cloud = self.stitch_point_clouds(captured_clouds)

        # Publish stitched cloud for visualization only
        self.cloud_pub.publish(cloud)

        result = ScanSceneResult(pointcloud_scene=cloud)
        self._as.set_succeeded(result)

        rospy.loginfo("Scan scene action succeeded")

    def capture_point_cloud(self):
        self.latest_cloud_data = None
        rospy.sleep(0.1)  # wait for the latest point cloud to be published

        if self.latest_cloud_data is None:
            rospy.logwarn("Didn't receive any pointcloud data yet.")
            return None

        point_cloud_msg = self.transform_pointcloud(self.latest_cloud_data)
        rospy.loginfo("Captured point cloud")

        return point_cloud_msg

    def transform_pointcloud(self, msg):
        frame = msg.header.frame_id
        translation, rotation = self.listener.lookupTransform(
            "panda_link0", frame, rospy.Time()
        )
        transform_msg = TransformStamped()
        transform_msg.transform.translation.x = translation[0]
        transform_msg.transform.translation.y = translation[1]
        transform_msg.transform.translation.z = translation[2]
        transform_msg.transform.rotation.x = rotation[0]
        transform_msg.transform.rotation.y = rotation[1]
        transform_msg.transform.rotation.z = rotation[2]
        transform_msg.transform.rotation.w = rotation[3]
        transformed_msg = do_transform_cloud(msg, transform_msg)
        transformed_msg.header = msg.header
        transformed_msg.header.frame_id = "panda_link0"
        return transformed_msg

    def stitch_point_clouds(self, clouds):
        points_out = []
        for cloud in clouds:
            points_out += list(read_points(cloud))
        return create_cloud(cloud.header, cloud.fields, points_out)


if __name__ == "__main__":
    try:
        rospy.init_node("scan_action_node")
        SimpleScanAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
