#! /usr/bin/env python
import numpy as np
import pinocchio as pin

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, PoseArray
from moma_mission.utils.rotation import CompatibleRotation as R


def yaw_from_quaternion(q):
    rot = R.from_quat(q).as_matrix()
    return yaw_from_rotation_matrix(rot)

def yaw_from_quaternion_ros(q_ros):
    q = np.array([q_ros.x, q_ros.y, q_ros.z, q_ros.w])
    return yaw_from_quaternion(q)

def yaw_from_rotation_matrix(rot):
    assert rot.shape == (3,3)
    return np.arctan2(rot[0, 1], rot[0, 0])

def poses_with_cov_to_pose_array(poses_list, frame):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = frame
    poses.poses = [pose.pose.pose for pose in poses_list]
    return poses


def numpy_to_pose(translation, orientation):
    pose = Pose()
    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose


def numpy_to_pose_stamped(translation, orientation, frame_id):
    pose = PoseStamped()
    pose.pose = numpy_to_pose(translation, orientation)
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


def se3_to_pose_stamped(se3pose, frame_id):
    pose = PoseStamped()
    pose.pose = se3_to_pose_ros(se3pose)
    pose.header.stamp = rospy.get_rostime()
    pose.header.frame_id = frame_id
    return pose


def se3_to_transform(se3pose, stamp, frame_id, child_frame_id):
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = frame_id
    tf.child_frame_id = child_frame_id
    tf.transform.translation.x = se3pose.translation[0]
    tf.transform.translation.y = se3pose.translation[1]
    tf.transform.translation.z = se3pose.translation[2]
    q = R.from_matrix(se3pose.rotation).as_quat()
    tf.transform.rotation.x = q[0]
    tf.transform.rotation.y = q[1]
    tf.transform.rotation.z = q[2]
    tf.transform.rotation.w = q[3]
    return tf

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
