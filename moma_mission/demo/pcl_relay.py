#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

rospy.init_node("pcl_relay")

def pcl_callback(msg):
    msg.header.stamp = rospy.get_rostime()
    relay.publish(msg)

relay = rospy.Publisher("/zedm/zed_node/point_cloud/cloud_registered_relay", PointCloud2, queue_size=1)
pcl_sub = rospy.Subscriber("/zedm/zed_node/point_cloud/cloud_registered", PointCloud2, pcl_callback, queue_size=1)

rospy.spin()

