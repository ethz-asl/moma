#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from dynamic_reconfigure.server import Server
from moma_bringup.cfg import HSVConfig  # You will need to create this cfg file
import struct
# import pcl
import numpy as np
import cv2  # OpenCV for HSV conversion
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class PointCloudSplitter:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pointcloud_splitter', anonymous=True)
        self.bridge = CvBridge()
        # Publisher for test image visualization of the reference color
        self.image_pub = rospy.Publisher('/reference_color_image', Image, queue_size=1)

        # Dynamic reconfigure server
        self.srv = Server(HSVConfig, self.dynamic_reconfigure_callback)

        # Set default HSV color range (will be modified via dynamic reconfigure)
        self.hsv_lower = np.array([35, 50, 50])  # Lower bound for HSV
        self.hsv_upper = np.array([85, 255, 255])  # Upper bound for HSV

        # Subscribe to the input PointCloud2 topic
        self.pc_sub = rospy.Subscriber('/concatenated_cloud_passthrough_xyz_downsampled', PointCloud2, self.pointcloud_callback)

        # Publishers for the inliers and outliers PointCloud2
        self.pc_inliers_pub = rospy.Publisher('/inliers_pointcloud', PointCloud2, queue_size=1)
        self.pc_outliers_pub = rospy.Publisher('/outliers_pointcloud', PointCloud2, queue_size=1)


    def dynamic_reconfigure_callback(self, config, level):
        print('dynamic_reconfigure_callback')
        """Dynamic reconfigure callback to update the HSV range."""
        self.hsv_lower = np.array([config.h_lower, config.s_lower, config.v_lower])
        self.hsv_upper = np.array([config.h_upper, config.s_upper, config.v_upper])
        rospy.loginfo("Updated HSV range: Lower: %s Upper: %s", self.hsv_lower, self.hsv_upper)
        
        # Publish the reference color as an image for visualization
        self.publish_test_image(self.hsv_lower, self.hsv_upper)
        return config

    def publish_test_image(self, hsv_lower, hsv_upper):
        print('publish_test_image')
        """Create and publish a test image with the reference color."""
        # Generate a solid color image of the average reference color
        avg_hsv = (hsv_lower + hsv_upper) / 2
        color = np.uint8([[avg_hsv]])  # 1x1 HSV pixel
        rgb_color = cv2.cvtColor(color, cv2.COLOR_HSV2BGR)[0][0]  # Convert to RGB

        # Create a 100x100 image with this color
        image = np.full((100, 100, 3), rgb_color, dtype=np.uint8)

        # Convert to ROS Image message and publish
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_pub.publish(img_msg)

    def pointcloud_callback(self, msg):
        print('pointcloud_callback')
        # Extract the point cloud data
        points_list = []
        inliers = []
        outliers = []

        # Convert PointCloud2 message to point list
        for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "rgb")):
            x, y, z, rgb = point

            # Extract the RGB values
            # r = int((rgb >> 16) & 0xFF)
            # g = int((rgb >> 8) & 0xFF)
            # b = int(rgb & 0xFF)
            packed_rgb = struct.unpack('I', struct.pack('f', rgb))[0]
            r = (packed_rgb >> 16) & 0xFF
            g = (packed_rgb >> 8) & 0xFF
            b = packed_rgb & 0xFF

            # Convert RGB to HSV using OpenCV
            rgb_array = np.uint8([[[r, g, b]]])
            hsv_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2HSV)
            h, s, v = hsv_array[0][0]

            # Compare with predefined HSV range
            if (self.hsv_lower <= np.array([h, s, v])).all() and (np.array([h, s, v]) <= self.hsv_upper).all():
                # Add to inliers if color is within range
                inliers.append([x, y, z, rgb])
            else:
                # Add to outliers otherwise
                outliers.append([x, y, z, rgb])

        # Convert the lists back to PointCloud2 messages
        inliers_cloud = self.create_pointcloud(msg.header, inliers)
        outliers_cloud = self.create_pointcloud(msg.header, outliers)

        # Publish the point clouds
        self.pc_inliers_pub.publish(inliers_cloud)
        self.pc_outliers_pub.publish(outliers_cloud)

    def create_pointcloud(self, header, points):
        """Converts a list of points into a PointCloud2 message."""
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1),
        ]

        # Create PointCloud2 message
        return pc2.create_cloud(header, fields, points)


if __name__ == '__main__':
    try:
        pointcloud_splitter = PointCloudSplitter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
