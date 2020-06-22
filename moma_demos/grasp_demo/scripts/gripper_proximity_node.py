#!/usr/bin/env python

import sys
import numpy as np
import rospy
import tf

from math import pow, sqrt
from std_msgs.msg import Float64, String

from geometry_msgs.msg import PoseStamped

import tf2_ros
import tf2_geometry_msgs

import py_trees

# grasp_pose = None

# distance lower than this threshold set detector from far to close
proximity_threshold = 0.6 

proximity_detextor_pub = rospy.Publisher(name="/proximity_detector", data_class=String, queue_size=1)

def get_tf_position(frame,base):
    while not rospy.is_shutdown():
        try:
            # (trans,rot) = listener.lookupTransform('/panda_default_ee', '/panda_link0', rospy.Time(0))
            (trans,rot) = listener.lookupTransform(base, frame, rospy.Time(0))
            return trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def grasp_CB(data):
    grasp_pose = data
    return grasp_pose

def get_distance(pose,position):

    x1 = pose.pose.position.x
    y1 = pose.pose.position.y
    z1 = pose.pose.position.z

    # rospy.loginfo(position)
    x2 = position[0]
    y2 = position[1]
    z2 = position[2]

    distance = float(sqrt(pow((x2-x1),2)+pow((y2-y1),2)+pow((z2-z1),2)))

    return distance


def main():
    rospy.init_node('proximity_detector',anonymous=False)
    listener = tf.TransformListener()
    grasp_pose = rospy.wait_for_message(
        "/grasp_pose",
        PoseStamped,
        timeout=480
    )
    rospy.loginfo(grasp_pose)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform("/panda_link0","/panda_default_ee", rospy.Time(0))
            print(3)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print(3)    
        distance = get_distance(grasp_pose,trans)
        if distance <= proximity_threshold:
            proximity_detextor_pub.publish("close")
        else: proximity_detextor_pub.publish("far")

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
