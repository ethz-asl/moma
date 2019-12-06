#!/usr/bin/env python

import time
import pickle

from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import open3d
import rospy
from sensor_msgs.msg import Image, PointCloud2
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import tf2_ros

from moma_utils import ros_conversions


class PointCloudStitching(object):
    def __init__(self):
        self.base_frame_id = "panda_link0"
        self.camera_frame_id = "camera_depth_optical_frame"
        self.camera_topic = "/camera/depth/image_rect_raw"

        self.intrinsic = open3d.camera.PinholeCameraIntrinsic(
            width=640,
            height=480,
            fx=383.2657775878906,
            fy=383.2657775878906,
            cx=319.3994140625,
            cy=242.43833923339844,
        )
        self.depth_trunc = 0.5
        self.downsample_voxel_size = 0.005

        self.stitched_cloud = open3d.geometry.PointCloud()
        self.reset(TriggerRequest())

        # Subscribe to the camera
        rospy.Subscriber(self.camera_topic, Image, self.camera_cb, queue_size=1)

        # Create tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Advertise services to reset the point cloud integration
        rospy.Service("~reset", Trigger, self.reset)

        # Publish stitched point cloud at a fixed rate
        self.stitched_cloud_pub = rospy.Publisher(
            "~stitched_cloud", PointCloud2, queue_size=1
        )
        rospy.Timer(
            rospy.Duration(0.5), self.publish_stitched_cloud,
        )

        rospy.loginfo("Ready to stitch point clouds")

    def reset(self, request):
        self.stitched_cloud.clear()
        return TriggerResponse()

    def camera_cb(self, depth_img_msg):
        rospy.loginfo("Processing incoming image")

        cv_bridge = CvBridge()
        depth_img = cv_bridge.imgmsg_to_cv2(depth_img_msg)
        stamp = depth_img_msg.header.stamp

        # Process image
        depth_img = depth_img.astype(np.float32) / 1000.0
        depth_img[depth_img > self.depth_trunc] = 0.0

        # Lookup camera pose
        try:
            T_camera_base_msg = self.tf_buffer.lookup_transform(
                self.camera_frame_id, self.base_frame_id, stamp, rospy.Duration(0.1)
            )
        except Exception:
            return

        T_camera_base = ros_conversions.from_transform_msg(T_camera_base_msg.transform)

        # Construct a point cloud from the depth image
        point_cloud = open3d.geometry.PointCloud.create_from_depth_image(
            open3d.geometry.Image(depth_img),
            self.intrinsic,
            T_camera_base.as_matrix(),
            depth_scale=1.0,
            depth_trunc=self.depth_trunc,
        )

        # Stitch point clouds
        self.stitched_cloud += point_cloud

        # Downsample to keep memory bounded
        self.stitched_cloud = self.stitched_cloud.voxel_down_sample(
            voxel_size=self.downsample_voxel_size
        )

    def publish_stitched_cloud(self, event):
        points = np.asarray(self.stitched_cloud.points)
        if len(points) == 0:
            return
        msg = ros_conversions.to_point_cloud_msg(points, frame_id=self.base_frame_id)
        self.stitched_cloud_pub.publish(msg)


if __name__ == "__main__":
    try:
        rospy.init_node("point_cloud_stitching")
        PointCloudStitching()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
