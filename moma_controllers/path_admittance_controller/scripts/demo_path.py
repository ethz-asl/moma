#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("demo_path")
    demo_path_topic = rospy.get_param("~demo_path_topic", "/demo_path")
    demo_path_frame = rospy.get_param("~demo_path_frame", "end_effector")

    rospy.loginfo("Publishing over topic {} in frame {}".format(demo_path_topic, demo_path_frame))
    demo_path_publisher = rospy.Publisher(demo_path_topic, Path, queue_size=1)
    demo_path = Path()
    demo_path.header.frame_id = demo_path_frame

    # draw a square
    for i in range(4):
        pose = PoseStamped()
        pose.header.frame_id = demo_path_frame
        pose.header.stamp = demo_path.header.stamp + rospy.Duration(5.0 * i)
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = (0 < i < 3) * 0.2
        pose.pose.position.y = (i > 1) * 0.2
        pose.pose.position.z = 0.0
        demo_path.poses.append(pose)
    demo_path.header.stamp = rospy.get_rostime()
    while demo_path_publisher.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscribers...")
        rospy.sleep(1.0)

    demo_path_publisher.publish(demo_path)
    rospy.loginfo("Path published!")
    rospy.sleep(2.0)
