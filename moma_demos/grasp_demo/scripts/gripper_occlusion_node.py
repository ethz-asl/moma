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

occlusion_detector_pub = rospy.Publisher(name="/gripper_occlusion", data_class=String, queue_size=1)

def transform2position(input_pose, from_frame, to_frame):
        # Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

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

# projecting point from 3D into the image frame
def position2imgPoints(position):
    position_transformed = np.array([[position.x,position.y,position.z]])
    position_transformed = position_transformed.T
    cameraMatrix = np.array([602.1849879340944, 0.0, 320.5, 0.0, 602.1849879340944, 240.5, 0.0, 0.0, 1.0]).reshape(3,3)
    distCoeffs=np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    rvec = np.array([0,0,0], np.float) # rotation vector
    tvec = np.array([0,0,0], np.float) # translation vector
    imagePoints, jac = cv.projectPoints(position_transformed, rvec=rvec, tvec=tvec, cameraMatrix=cameraMatrix,distCoeffs=distCoeffs)
    # returns the (x,y) pixel positions from the 3D point in the camera frame
    return [imagePoints[0][0][0],imagePoints[0][0][1]]

def main():
    rospy.init_node('occlusion_detector',anonymous=False)
    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    try:
        grasp_pose = rospy.wait_for_message(
            "/grasp_pose",
            PoseStamped,
            timeout=480
        )
        grasp_position_inCAM = transform2position(grasp_pose.pose,'panda_link0', 'fixed_camera_depth_optical_frame')
        grasp_imgPoints = position2imgPoints(grasp_position_inCAM)
    except rospy.ROSException:
        rospy.loginfo("did not get info")
        return []

    # box margin defined from ROS parameter server, tracking uses same value for drawing frame
    box_margin = rospy.get_param("/moma_demo/occlusion_margin")
    xmin = grasp_imgPoints[0] - box_margin
    xmax = grasp_imgPoints[0] + box_margin
    ymin = grasp_imgPoints[1] - box_margin
    ymax = grasp_imgPoints[1] + box_margin

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform("/fixed_camera_depth_optical_frame","/panda_default_ee", rospy.Time(0))
            ee_pose_inCAM = create_pose(trans,rot)
            ee_imgPoints = position2imgPoints(ee_pose_inCAM.position)

            # checks if the end effector gets into close probximity of the tracked object
            if not ((xmin <= ee_imgPoints[0] and ee_imgPoints[0] <= xmax) and (ymin <= ee_imgPoints[1] and ee_imgPoints[1] <= ymax)):
                occlusion_detector_pub.publish("visible")
            else:
                occlusion_detector_pub.publish("occluded")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
