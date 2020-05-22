#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose, PoseArray, PoseStamped

from std_msgs.msg import String, Header

from time import sleep

def sub_grasp_candidates():
    rospy.init_node('grasp_candidate_selection', anonymous=True)
    rospy.Subscriber('/grasp_candidates', PoseArray, selector_CB)
    rospy.spin()


def selector_CB(data):
    pub = rospy.Publisher('/grasp_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        rospy.loginfo(data)
        sleep(1)
        msg = PoseStamped()
        msg.header = Header()
        msg.header.frame_id = data.header.frame_id
        msg.pose = data.poses[0]
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    sub_grasp_candidates()
    