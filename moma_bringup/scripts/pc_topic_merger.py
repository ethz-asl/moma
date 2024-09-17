#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class PointCloudMerger:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pointcloud_merger', anonymous=True)
        
        # Publishers
        self.pub = rospy.Publisher('/combined_cloud', PointCloud2, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/rs_435_1/depth/color/points', PointCloud2, self.callback_cloud1)
        rospy.Subscriber('/rs_435_2/depth/color/points', PointCloud2, self.callback_cloud2)
        rospy.Subscriber('/rs_435_3/depth/color/points', PointCloud2, self.callback_cloud3)
        
        # Variables to store point clouds
        self.cloud1 = None
        self.cloud2 = None
        self.cloud3 = None

    def callback_cloud1(self, msg):
        print('Received cloud 1')
        self.pub.publish(msg)

    def callback_cloud2(self, msg):
        print('Received cloud 2')
        self.pub.publish(msg)

    def callback_cloud3(self, msg):
        print('Received cloud 3')
        self.pub.publish(msg)

    def combine_and_publish(self):
        # Ensure all clouds are received before combining
        if self.cloud1 and self.cloud2 and self.cloud3:
            # Extract point data from each point cloud
            points1 = list(pc2.read_points(self.cloud1, skip_nans=True))
            points2 = list(pc2.read_points(self.cloud2, skip_nans=True))
            points3 = list(pc2.read_points(self.cloud3, skip_nans=True))
            
            # Combine points
            combined_points = points1 + points2 + points3

            # Create a new header
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.cloud1.header.frame_id  # Assuming all clouds share the same frame

            # Create a new PointCloud2 message
            combined_cloud_msg = pc2.create_cloud(header, self.cloud1.fields, combined_points)
            
            # Publish the combined point cloud
            self.pub.publish(combined_cloud_msg)

if __name__ == '__main__':
    try:
        pointcloud_merger = PointCloudMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
