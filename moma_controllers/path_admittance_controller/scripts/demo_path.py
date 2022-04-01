#!/usr/bin/env python
import tf2_ros
import tf2_geometry_msgs

import rospy
from copy import deepcopy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("demo_path")
    demo_path_topic = rospy.get_param("~demo_path_topic", "/demo_path")
    demo_path_frame = rospy.get_param("~demo_path_frame", "end_effector")
    demo_path_fixed_frame = rospy.get_param("~demo_path_fixed_frame", "world")
    rospy.loginfo(
        "Publishing over topic {} in frame {}".format(demo_path_topic, demo_path_frame)
    )

    tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))  # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform(
        demo_path_fixed_frame,
        demo_path_frame,  # source frame
        rospy.Time(0),  # get the tf at first available time
        rospy.Duration(3.0),
    )  # wait for 1 second

    # draw a square
    demo_path = Path()
    demo_path.header.frame_id = demo_path_fixed_frame
    demo_path.header.stamp = rospy.get_rostime()
    for i in range(4):
        local_pose = PoseStamped()
        local_pose.pose.orientation.w = 1.0
        local_pose.pose.position.x = (0 < i < 3) * 0.2
        local_pose.pose.position.y = (i > 1) * 0.2
        local_pose.pose.position.z = 0.0

        global_pose = PoseStamped()
        global_pose = tf2_geometry_msgs.do_transform_pose(local_pose, transform)
        global_pose.header.frame_id = demo_path_fixed_frame
        global_pose.header.stamp = demo_path.header.stamp + rospy.Duration(5.0 * i)
        print(global_pose.header.stamp.to_sec())

        demo_path.poses.append(deepcopy(global_pose))

    demo_path_publisher = rospy.Publisher(demo_path_topic, Path, queue_size=1)
    while demo_path_publisher.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscribers...")
        rospy.sleep(1.0)

    while not rospy.is_shutdown():
        demo_path_publisher.publish(demo_path)
        rospy.sleep(20)
        rospy.loginfo("Path published!")

    rospy.sleep(2.0)
