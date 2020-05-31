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

# proximity_detextor_pub = rospy.Publisher(name="/proximity_detector", data_class=Float64, queue_size=1)
proximity_detextor_pub = rospy.Publisher(name="/proximity_detector", data_class=String, queue_size=1)

def get_tf_position(frame,base):
    # rospy.loginfo("waiting for grasp to publish")
    # rospy.loginfo("created listener")
    # rate = rospy.Rate(1.0)
    # rospy.loginfo("r    ate")
    while not rospy.is_shutdown():
        try:
            # rospy.loginfo("try")
            # (trans,rot) = listener.lookupTransform('/panda_default_ee', '/panda_link0', rospy.Time(0))
            (trans,rot) = listener.lookupTransform(base, frame, rospy.Time(0))
            return trans
            # rospy.loginfo(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # rospy.loginfo("expect")
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
    # rospy.Subscriber(name="/grasp_pose",data_class=PoseStamped, callback=grasp_CB, queue_size=1)
    print(1)
    grasp_pose = rospy.wait_for_message(
        "/grasp_pose",
        PoseStamped,
        timeout=480
    )
    print(2)
    rospy.loginfo(grasp_pose)

    while not rospy.is_shutdown():
        # tcp = get_tf_position("panda_default_ee","panda_link0")
        try:
            # rospy.loginfo("try")
            # (trans,rot) = listener.lookupTransform('/panda_default_ee', '/panda_link0', rospy.Time(0))
            (trans,rot) = listener.lookupTransform("/panda_link0","/panda_default_ee", rospy.Time(0))
            print(3)
            # rospy.loginfo(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # rospy.loginfo("expect")
            continue
        print(3)    
        distance = get_distance(grasp_pose,trans)
        if distance <= 0.6:
            proximity_detextor_pub.publish("close")
        else: proximity_detextor_pub.publish("far")
        # rospy.loginfo(distance)
        # print(distance)
        # proximity_detextor_pub.publish(distance)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
