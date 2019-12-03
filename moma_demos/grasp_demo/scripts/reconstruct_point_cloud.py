#!/usr/bin/env python

import random
import time

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points, create_cloud
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros


class PointCloudReconstruction(object):
    def __init__(self):
        self.base_frame = "panda_link0"
        self.sensor_topic_name = "/camera/depth/color/points"
        self.max_num_points = 360 * 360

        self.running = False

        # Subscribe to the sensor
        rospy.Subscriber(
            self.sensor_topic_name,
            PointCloud2,
            self.sensor_cb,
            queue_size=1,
            buff_size=2 * 6144000,  # default buff_size is too small
        )

        # Create tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Advertise services to reset, start, and stop the point cloud integration
        rospy.Service("~reset", Trigger, self.reset)
        rospy.Service("~start", Trigger, self.start)
        rospy.Service("~stop", Trigger, self.stop)

        # Publish stitched point cloud
        self.point_cloud_pub = rospy.Publisher(
            "~point_cloud", PointCloud2, queue_size=10
        )

        rospy.loginfo("Point cloud reconstruction node ready")

    def reset(self, request):
        self.points = None
        return TriggerResponse()

    def start(self, request):
        self.running = True
        return TriggerResponse()

    def stop(self, request):
        self.running = False
        return TriggerResponse()

    def sensor_cb(self, point_cloud_msg):
        if not self.running:
            return

        header = point_cloud_msg.header
        fields = point_cloud_msg.fields

        # Lookup robot pose
        T_robot_camera = self.tf_buffer.lookup_transform(
            self.base_frame, header.frame_id, header.stamp, rospy.Duration(0.1)
        )

        # Transform point cloud
        point_cloud_msg = do_transform_cloud(point_cloud_msg, T_robot_camera)
        point_cloud_msg.header = header
        point_cloud_msg.header.frame_id = self.base_frame

        # Stitch point cloud
        if self.points is None:
            self.points = list(read_points(point_cloud_msg))
        else:
            self.points += list(read_points(point_cloud_msg))

        # Downsample by randomly selecting points
        num_points = min(len(self.points), self.max_num_points)
        self.points = random.sample(self.points, num_points)

        # Publish stitched point cloud
        self.point_cloud_pub.publish(create_cloud(header, fields, self.points))


if __name__ == "__main__":
    try:
        rospy.init_node("point_cloud_reconstruction")
        PointCloudReconstruction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
