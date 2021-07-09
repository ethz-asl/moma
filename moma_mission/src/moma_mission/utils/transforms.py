#! /usr/bin/env python
import numpy as np
import pinocchio as pin

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped
from moma_mission.utils.rotation import CompatibleRotation as R


def get_transform(target, source):
    """ Retrieve transform. Let it fail if unable to get the transform """
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    res = tf_buffer.can_transform(target, source, rospy.Time(0), rospy.Duration(10), return_debug_tuple=True)
    for r in res:
        print(r)
    transform = tf_buffer.lookup_transform(target,
                                           source,
                                           rospy.Time(0),  # tf at first available time
                                           rospy.Duration(3))
    return tf_to_se3(transform)


def numpy_to_pose_stamped(translation, orientation, frame_id):
    pose = PoseStamped()
    pose.pose.position.x = translation[0]
    pose.pose.position.y = translation[1]
    pose.pose.position.z = translation[2]
    pose.pose.orientation.x = orientation[0]
    pose.pose.orientation.y = orientation[1]
    pose.pose.orientation.z = orientation[2]
    pose.pose.orientation.w = orientation[3]
    pose.header.stamp = rospy.get_rostime()
    pose.header.frame_id = frame_id
    return pose


def se3_to_pose_ros(se3pose):
    pose_ros = Pose()
    pose_ros.position.x = se3pose.translation[0]
    pose_ros.position.y = se3pose.translation[1]
    pose_ros.position.z = se3pose.translation[2]
    q = R.from_dcm(se3pose.rotation).as_quat()
    pose_ros.orientation.x = q[0]
    pose_ros.orientation.y = q[1]
    pose_ros.orientation.z = q[2]
    pose_ros.orientation.w = q[3]
    return pose_ros


def tf_to_se3(transform):
    q = pin.Quaternion(transform.transform.rotation.w,
                       transform.transform.rotation.x,
                       transform.transform.rotation.y,
                       transform.transform.rotation.z)
    t = np.array([transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z])
    return pin.SE3(q, t)


def tf_to_pose(transform):
    pose = Pose()
    pose.orientation.w = transform.transform.rotation.w
    pose.orientation.x = transform.transform.rotation.x
    pose.orientation.y = transform.transform.rotation.y
    pose.orientation.z = transform.transform.rotation.z

    pose.position.x = transform.transform.translation.x
    pose.position.y = transform.transform.translation.y
    pose.position.z = transform.transform.translation.z
    return pose


def pose_to_se3(pose):
    q = pin.Quaternion(pose.orientation.w,
                       pose.orientation.x,
                       pose.orientation.y,
                       pose.orientation.z)
    t = np.array([pose.position.x,
                  pose.position.y,
                  pose.position.z])
    return pin.SE3(q, t)
