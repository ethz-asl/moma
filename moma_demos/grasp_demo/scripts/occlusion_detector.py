#!/usr/bin/env python

import sys
import numpy as np
import rospy
import tf

from math import pow, sqrt
from std_msgs.msg import Float64, String, Bool
from grasp_demo.msg import BoundingBox, DetectionActionResult

from geometry_msgs.msg import Pose, PoseStamped
import cv2 as cv

import tf2_ros
import tf2_geometry_msgs

import py_trees

# grasp_pose = None

# proximity_detextor_pub = rospy.Publisher(name="/proximity_detector", data_class=Float64, queue_size=1)
occlusion_detector_pub = rospy.Publisher(name="/occlusion_detector", data_class=Bool, queue_size=1)

def transform2position(input_pose, from_frame, to_frame):

        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        # print("F1")
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        # print("F2")
        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose.position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

def create_pose(trans,rot):
    pose=Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose

def position2imgPoints(position):
            position_transformed = np.array([[position.x,position.y,position.z]])
            position_transformed = position_transformed.T
            cameraMatrix = np.array([602.1849879340944, 0.0, 320.5, 0.0, 602.1849879340944, 240.5, 0.0, 0.0, 1.0]).reshape(3,3)
            distCoeffs=np.array([0.0, 0.0, 0.0, 0.0, 0.0])
            rvec = np.array([0,0,0], np.float) # rotation vector
            tvec = np.array([0,0,0], np.float) # translation vector
            imagePoints, jac = cv.projectPoints(position_transformed, rvec=rvec, tvec=tvec, cameraMatrix=cameraMatrix,distCoeffs=distCoeffs)
            return [imagePoints[0][0][0],imagePoints[0][0][1]]

def main():
    rospy.init_node('occlusion_detector',anonymous=False)
    listener = tf.TransformListener()
    rate = rospy.Rate(5)
    try:
        grasp_pose = rospy.wait_for_message(
            "/grasp_pose",
            PoseStamped,
            timeout=480
        )
        grasp_position_inCAM = transform2position(grasp_pose.pose,'panda_link0', 'fixed_camera_depth_optical_frame')
        grasp_imgPoints = position2imgPoints(grasp_position_inCAM)
        print(grasp_position_inCAM)
    except rospy.ROSException:
            rospy.loginfo("did not get info")
            return []

    box_margin = 250
    xmin = grasp_imgPoints[0] - box_margin
    xmax = grasp_imgPoints[0] + box_margin
    ymin = grasp_imgPoints[1] - box_margin
    ymax = grasp_imgPoints[1] + box_margin

    while not rospy.is_shutdown():

        time = rospy.Time.now()
        try:
            (trans,rot) = listener.lookupTransform("/panda_link8","/fixed_camera_depth_optical_frame", rospy.Time(0))
            # (trans,rot) = listener.lookupTransform("/panda_link0","/panda_default_ee", rospy.Time(0))
            ee_pose_inCAM = create_pose(trans,rot)
            # ee_position_inCAM = transform2position(ee_pose,'panda_link8', 'fixed_camera_depth_optical_frame')
            ee_imgPoints = position2imgPoints(ee_pose_inCAM.position)
            print(ee_imgPoints)

            if not ((xmin <= ee_imgPoints[0] and ee_imgPoints[0] <= xmax) and (ymin <= ee_imgPoints[1] and ee_imgPoints[1] <= ymax)):
                occlusion_detector_pub.publish(False)
                # print("false x: ",xmin ,"<=", ee_imgPoints[0] ,"<=", xmax," | ",ymin ,"<=", ee_imgPoints[1] ,"<=", ymax)
                # print("false %s",rospy.Time.now()-time)
            else:
                occlusion_detector_pub.publish(True)
                # print("true  x: %s <= %s <= %s | x: %s <= %s <= %",xmin ,ee_imgPoints[0], xmax,ymin ,ee_imgPoints[1], ymax)
                # print("true  %s",rospy.Time.now()-time)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # print("Gripper:")
        # print(ee_imgPoints)
        # print("Box:")
        # print(grasp_imgPoints)
        # print(rospy.Time.now()-time)
        # rospy.Time.now()
        # print(rospy.Time.now())
    
    rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
