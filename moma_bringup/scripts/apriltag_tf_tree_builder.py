#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, Transform

import tf.transformations as tft
import numpy as np

class AprilTagTransformPublisher:
    def __init__(self):
        self.camera_root_frame_id = rospy.get_param('~camera_root_frame_id', default='root_cam1')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'april_tag_0')
        print(self.camera_root_frame_id)
        print(self.base_frame_id)
        # Initialize a tf2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to detection topic
        self.sub = rospy.Subscriber('apriltag_detection', AprilTagDetectionArray, self.tag_callback)

    def tag_callback(self, msg):
        # Iterate through the detected tags
        for detection in msg.detections:
            # get T_cam_tag
            T_cam_tag_tf = detection.pose.pose.pose
            T_cam_tag = tft.quaternion_matrix([
                T_cam_tag_tf.orientation.x,
                T_cam_tag_tf.orientation.y,
                T_cam_tag_tf.orientation.z,
                T_cam_tag_tf.orientation.w])
            T_cam_tag[0:3,-1] = np.array([
                T_cam_tag_tf.position.x,
                T_cam_tag_tf.position.y,
                T_cam_tag_tf.position.z
            ])

            # get T_camroot_cam
            try:
                # Lookup the transform from 'target_frame' to 'source_frame'
                T_camroot_cam_tfs = self.tf_buffer.lookup_transform(detection.pose.header.frame_id, self.camera_root_frame_id, rospy.Time(0), rospy.Duration(4.0))

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(e)

            T_camroot_cam_tf = T_camroot_cam_tfs.transform
            T_camroot_cam = tft.quaternion_matrix([
                T_camroot_cam_tf.rotation.x,
                T_camroot_cam_tf.rotation.y,
                T_camroot_cam_tf.rotation.z,
                T_camroot_cam_tf.rotation.w])
            T_camroot_cam[0:3,-1] = np.array([
                T_camroot_cam_tf.translation.x,
                T_camroot_cam_tf.translation.y,
                T_camroot_cam_tf.translation.z
            ])

            # get T_camroot_tag
            T_camroot_tag = tft.concatenate_matrices(T_camroot_cam, T_cam_tag)

            # get T_tag_camroot
            T_tag_camroot = tft.inverse_matrix(T_camroot_tag)

            # send off
            q_tag_camroot = tft.quaternion_from_matrix(T_tag_camroot)
            T_tag_camroot_tfs = geometry_msgs.msg.TransformStamped()
            T_tag_camroot_tfs.transform.rotation.x = q_tag_camroot[0]
            T_tag_camroot_tfs.transform.rotation.y = q_tag_camroot[1]
            T_tag_camroot_tfs.transform.rotation.z = q_tag_camroot[2]
            T_tag_camroot_tfs.transform.rotation.w = q_tag_camroot[3]
            T_tag_camroot_tfs.transform.translation.x = T_tag_camroot[0,-1]
            T_tag_camroot_tfs.transform.translation.y = T_tag_camroot[1,-1]
            T_tag_camroot_tfs.transform.translation.z = T_tag_camroot[2,-1]
            T_tag_camroot_tfs.header = detection.pose.header
            T_tag_camroot_tfs.header.frame_id = self.base_frame_id
            T_tag_camroot_tfs.child_frame_id = self.camera_root_frame_id
            # publish the transform
            self.tf_broadcaster.sendTransform(T_tag_camroot_tfs)

if __name__ == '__main__':
    rospy.init_node('april_tag_transform_publisher')
    AprilTagTransformPublisher()
    rospy.spin()
