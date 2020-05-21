#!/usr/bin/env python

from __future__ import print_function

import copy
import sys

from actionlib import SimpleActionServer
import numpy as np
from geometry_msgs.msg import TransformStamped
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points, create_cloud
import tf
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from grasp_demo.msg import ScanSceneAction, ScanSceneResult
from grasp_demo.utils import create_robot_connection

from voxbloxpp_scan import ScanAction


class SimpleScanAction(ScanAction):
    """Move robot to a set of predefined scan poses and stitch together the captured point clouds."""

    def __init__(self):
        super(SimpleScanAction, self).__init__()

        self.base_frame_id = rospy.get_param("/moma_demo/base_frame_id")

        self.listener = tf.TransformListener()
        self.latest_cloud_data = None

        rospy.Subscriber(
            "/wrist_camera/depth/color/points", PointCloud2, self.point_cloud_cb
        )
        self.cloud_pub = rospy.Publisher("~cloud", PointCloud2, queue_size=1)
        # self.result_pub = rospy.Publisher("/bt_BB/ScannedBB", Bool, queue_size=1)

    def point_cloud_cb(self, data):
        self.latest_cloud_data = copy.deepcopy(data)

    def execute_cb(self, goal):
        """Move to each scan pose, capture a point cloud and stitch them together."""
        rospy.loginfo("Scanning action was triggered")

        captured_clouds = []

        for joints in self._scan_joints:
            if self._as.is_preempt_requested():
                rospy.loginfo("Got preempted")
                self._as.set_preempted()
                return

            self._robot_arm.goto_joint_target(
                joints, max_acceleration_scaling=0.2, max_velocity_scaling=0.4
            )
            rospy.sleep(0.5)
            cloud = self.capture_point_cloud()
            captured_clouds.append(cloud)

        # Stitch the cloud
        cloud = self.stitch_point_clouds(captured_clouds)

        # Publish stitched cloud for visualization only
        self.cloud_pub.publish(cloud)

        # Move home
        self._robot_arm.goto_joint_target(
            self._scan_joints[0], max_velocity_scaling=0.4
        )

        result_pub = True
        result = ScanSceneResult(pointcloud_scene=cloud,result = result_pub)
        # pub.publish(result_pub)
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
            self.base_frame_id, frame, rospy.Time()
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
        transformed_msg.header.frame_id = self.base_frame_id
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
