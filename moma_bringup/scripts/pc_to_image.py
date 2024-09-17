#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import cv2
import numpy as np
import struct
from cv_bridge import CvBridge

class PointCloudToRGB:
    def __init__(self):
        # Initialize the node
        rospy.init_node('pointcloud_to_rgb')

        # Subscribe to the point cloud topic
        rospy.Subscriber('/concatenated_cloud_passthrough_xyz_downsampled', PointCloud2, self.pointcloud_callback)

        # Publisher for the RGB image
        self.image_pub = rospy.Publisher('/output_image', Image, queue_size=10)

        # OpenCV Bridge for converting between ROS Image and OpenCV Image
        self.bridge = CvBridge()

    def pointcloud_callback(self, pointcloud_msg):
        # Initialize lists for storing points and colors
        points = []
        colors = []

        # Extract point cloud data (x, y, z, rgb)
        for point in pc2.read_points(pointcloud_msg, skip_nans=True, field_names=("x", "y", "z", "rgb")):
            x, y, z, rgb = point

            # Extract RGB values from packed float
            r = (int(rgb) >> 16) & 0xFF
            g = (int(rgb) >> 8) & 0xFF
            b = int(rgb) & 0xFF

            # Store the 2D projection (x, y) and the corresponding color
            points.append((x, y))
            colors.append((r, g, b))

        # Create a blank image for visualization
        image_width = 800
        image_height = 600
        image = np.zeros((image_height, image_width, 3), dtype=np.uint8)

        # Scale and project points onto the image
        for point, color in zip(points, colors):
            x, y = point
            img_x = int((x + 10) * image_width / 20)  # Adjust projection scaling
            img_y = int((y + 10) * image_height / 20)

            if 0 <= img_x < image_width and 0 <= img_y < image_height:
                image[img_y, img_x] = color

        # Display the image using OpenCV
        cv2.imshow('Projected RGB Image', image)
        cv2.waitKey(1)

        # Convert the OpenCV image to a ROS Image message and publish it
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_pub.publish(ros_image)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PointCloudToRGB()
        node.run()
    except rospy.ROSInterruptException:
        pass
