#!/usr/bin/env python

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
        self.running = False
        self.count = 0

        # Subscribe to the sensor
        rospy.Subscriber(self.sensor_topic_name, PointCloud2, self.sensor_cb)

        # Create tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Advertise services to reset, start, and stop the point cloud integration
        rospy.Service("stop", Trigger, self.reset)
        rospy.Service("reset", Trigger, self.stop)
        rospy.Service("start", Trigger, self.start)

        # Publish stitched point cloud
        self.point_cloud_pub = rospy.Publisher(
            "~point_cloud", PointCloud2, queue_size=10
        )

        rospy.sleep(1.0)
        self.reset(TriggerRequest())
        self.start(TriggerRequest())

    def reset(self, request):
        self.point_cloud = None

    def start(self, request):
        self.running = True

    def stop(self, request):
        self.running = False

    def sensor_cb(self, point_cloud):
        self.count += 1
        if not self.running or self.count % 60 != 0:
            return

        rospy.loginfo("Processing point cloud")

        # Lookup robot pose
        camera_frame = point_cloud.header.frame_id
        stamp = point_cloud.header.stamp
        T_robot_camera = self.tf_buffer.lookup_transform(
            self.base_frame, camera_frame, stamp, rospy.Duration(0.1)
        )

        # TODO(mbreyer) sometimes an ExtrapolationException is raised if no robot pose with the current time can
        # be found, probably it would be good to increase the publish rate of the robot state

        # Transform point cloud
        transformed_point_cloud = do_transform_cloud(point_cloud, T_robot_camera)
        transformed_point_cloud.header = point_cloud.header
        transformed_point_cloud.header.frame_id = self.base_frame

        # Stitch point cloud
        if self.point_cloud is None:
            self.point_cloud = transformed_point_cloud
        else:
            header = self.point_cloud.header
            fields = self.point_cloud.fields
            points_out = list(read_points(self.point_cloud)) + list(
                read_points(transformed_point_cloud)
            )
            self.point_cloud = create_cloud(header, fields, points_out)

        # Publish new point cloud
        print("Hi")
        self.point_cloud_pub.publish(self.point_cloud)


if __name__ == "__main__":
    try:
        rospy.init_node("point_cloud_reconstruction")
        PointCloudReconstruction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
