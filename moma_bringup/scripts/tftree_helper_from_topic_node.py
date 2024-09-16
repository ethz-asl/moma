#!/usr/bin/env python
# template written by chatGPT.

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, Transform
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest, SetBool, SetBoolResponse, SetBoolRequest
from dynamic_reconfigure.server import Server
from moma_bringup.cfg import ExtrinsicsConfig

import tf.transformations as tft
import numpy as np


class TfTreeHelperFromTopicNode:
    def __init__(self):
        self.camera_root_frame_id = rospy.get_param(
            '~camera_root_frame_id', default='root_cam1')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'april_tag_0')
        self.invert_tf = rospy.get_param('~invert_tf', False)
        # Initialize a tf2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.calibrate_trigger_srv = rospy.Service(
            'calibrate', Trigger, self.calibrate_callback_srv_cb)
        self.set_calib_mode_srv = rospy.Service(
            'set_calibration_mode', SetBool, self.set_calibration_mode_srv_cb)

        self.first_msg = True
        self.continuous_calibration = False

        self.t_x_offset = 0.0
        self.t_y_offset = 0.0
        self.t_z_offset = 0.0
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.yaw_offset = 0.0

        self.last_T_tag_camroot_tfs = None
        self.dynrec_srv = Server(ExtrinsicsConfig, self.dynrec_callback)

        # Subscribe to detection topic
        self.sub = rospy.Subscriber(
            'apriltag_detection', AprilTagDetectionArray, self.detection_cb)

    def dynrec_callback(self, config, level):
        print('dynrec_callback')
        self.t_x_offset = config.tx
        self.t_y_offset = config.ty
        self.t_z_offset = config.tz
        self.roll_offset = config.roll
        self.pitch_offset = config.pitch
        self.yaw_offset = config.yaw
        if self.last_T_tag_camroot_tfs is not None:
            self.calculate_transform()
        else:
            print('self.last_T_tag_camroot_tfs is none')
        return config
    
    def calculate_transform(self):
        print('calculate_transform')
        # try:
        #     T_Aold_Anew_tfs = self.tf_buffer.lookup_transform(
        #         self.frame_A_old, self.frame_A_new, rospy.Time(0), rospy.Duration(1.0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #     rospy.logerr(e)
        #     return

        # T_Aold_Anew = self.homog_mat_from_tfs(T_Aold_Anew_tfs)

        # try:
        #     T_Bold_Bnew_tfs = self.tf_buffer.lookup_transform(
        #         self.frame_B_old, self.frame_B_new, rospy.Time(0), rospy.Duration(1.0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #     rospy.logerr(e)
        #     return

        # T_Bold_Bnew = self.homog_mat_from_tfs(T_Bold_Bnew_tfs)

        # T_Aold_Bnew = tft.concatenate_matrices(self.T_Aold_Bold, T_Bold_Bnew)
        # T_Anew_Bnew = tft.concatenate_matrices(
        #     tft.inverse_matrix(T_Aold_Anew), T_Aold_Bnew)

        # extract stuff
        T_tag_camroot_tfs = self.last_T_tag_camroot_tfs
        quat_tag_camroot_tfs = np.array([T_tag_camroot_tfs.transform.rotation.x, T_tag_camroot_tfs.transform.rotation.y, T_tag_camroot_tfs.transform.rotation.z, T_tag_camroot_tfs.transform.rotation.w])
        transl_tag_camroot_tfs = np.array([T_tag_camroot_tfs.transform.translation.x, T_tag_camroot_tfs.transform.translation.y, T_tag_camroot_tfs.transform.translation.z])
        T_Anew_Bnew  = tft.quaternion_matrix(quat_tag_camroot_tfs)
        T_Anew_Bnew[0, -1] = transl_tag_camroot_tfs[0]
        T_Anew_Bnew[1, -1] = transl_tag_camroot_tfs[1]
        T_Anew_Bnew[2, -1] = transl_tag_camroot_tfs[2]

        # correct with offset parameters
        T_dynrec_offset = tft.euler_matrix(
            self.roll_offset, self.pitch_offset, self.yaw_offset, 'sxyz')
        T_dynrec_offset[0, -1] += self.t_x_offset
        T_dynrec_offset[1, -1] += self.t_y_offset
        T_dynrec_offset[2, -1] += self.t_z_offset
        T_Anew_Bnew_offset = tft.concatenate_matrices(
            T_dynrec_offset, T_Anew_Bnew)
        T_Anew_Bnew = T_Anew_Bnew_offset

        q_Anew_Bnew = tft.quaternion_from_matrix(T_Anew_Bnew)
        T_tag_camroot_new_tfs = geometry_msgs.msg.TransformStamped()
        T_tag_camroot_new_tfs.transform.rotation.x = q_Anew_Bnew[0]
        T_tag_camroot_new_tfs.transform.rotation.y = q_Anew_Bnew[1]
        T_tag_camroot_new_tfs.transform.rotation.z = q_Anew_Bnew[2]
        T_tag_camroot_new_tfs.transform.rotation.w = q_Anew_Bnew[3]
        T_tag_camroot_new_tfs.transform.translation.x = T_Anew_Bnew[0, -1]
        T_tag_camroot_new_tfs.transform.translation.y = T_Anew_Bnew[1, -1]
        T_tag_camroot_new_tfs.transform.translation.z = T_Anew_Bnew[2, -1]
        T_tag_camroot_new_tfs.header.stamp = rospy.Time.now()
        T_tag_camroot_new_tfs.header.frame_id = T_tag_camroot_tfs.header.frame_id
        T_tag_camroot_new_tfs.child_frame_id = T_tag_camroot_tfs.child_frame_id

        self.static_tf_broadcaster.sendTransform(T_tag_camroot_new_tfs)
        rospy.sleep(1)
        self.static_tf_broadcaster.sendTransform(T_tag_camroot_new_tfs)
        print('send off!')

    def detection_cb(self, msg):
        self.last_msg = msg
        if self.first_msg:
            self.calibrate_callback_srv_cb(TriggerRequest)
            self.first_msg = False
        if self.continuous_calibration:
            self.calibrate_callback_srv_cb(TriggerRequest)
        resp = TriggerResponse()
        resp.success = True
        return resp

    def set_calibration_mode_srv_cb(self, request):
        self.continuous_calibration = request.data
        if not self.continuous_calibration:
            self.first_msg = True
        resp = SetBoolResponse()
        resp.success = True
        return resp

    def calibrate_callback_srv_cb(self, request):
        print('calibrate_callback_srv_cb: ', self.camera_root_frame_id)
        # Iterate through the detected tags
        for detection in self.last_msg.detections:
            # get T_cam_tag
            T_cam_tag_tf = detection.pose.pose.pose
            T_cam_tag = tft.quaternion_matrix([
                T_cam_tag_tf.orientation.x,
                T_cam_tag_tf.orientation.y,
                T_cam_tag_tf.orientation.z,
                T_cam_tag_tf.orientation.w])
            T_cam_tag[0:3, -1] = np.array([
                T_cam_tag_tf.position.x,
                T_cam_tag_tf.position.y,
                T_cam_tag_tf.position.z
            ])

            # get T_camroot_cam
            try:
                # Lookup the transform from 'target_frame' to 'source_frame'
                T_camroot_cam_tfs = self.tf_buffer.lookup_transform(
                    self.camera_root_frame_id, detection.pose.header.frame_id, rospy.Time(0), rospy.Duration(4.0))

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(e)

            T_camroot_cam_tf = T_camroot_cam_tfs.transform
            T_camroot_cam = tft.quaternion_matrix([
                T_camroot_cam_tf.rotation.x,
                T_camroot_cam_tf.rotation.y,
                T_camroot_cam_tf.rotation.z,
                T_camroot_cam_tf.rotation.w])
            T_camroot_cam[0:3, -1] = np.array([
                T_camroot_cam_tf.translation.x,
                T_camroot_cam_tf.translation.y,
                T_camroot_cam_tf.translation.z
            ])

            # get T_camroot_tag
            T_camroot_tag = tft.concatenate_matrices(T_camroot_cam, T_cam_tag)

            # get T_tag_camroot
            if self.invert_tf:
                # hack
                T_tag_camroot = T_camroot_tag
            else:   
                T_tag_camroot = tft.inverse_matrix(T_camroot_tag)

            # send off
            q_tag_camroot = tft.quaternion_from_matrix(T_tag_camroot)
            T_tag_camroot_tfs = geometry_msgs.msg.TransformStamped()
            T_tag_camroot_tfs.transform.rotation.x = q_tag_camroot[0]
            T_tag_camroot_tfs.transform.rotation.y = q_tag_camroot[1]
            T_tag_camroot_tfs.transform.rotation.z = q_tag_camroot[2]
            T_tag_camroot_tfs.transform.rotation.w = q_tag_camroot[3]
            T_tag_camroot_tfs.transform.translation.x = T_tag_camroot[0, -1]
            T_tag_camroot_tfs.transform.translation.y = T_tag_camroot[1, -1]
            T_tag_camroot_tfs.transform.translation.z = T_tag_camroot[2, -1]
            T_tag_camroot_tfs.header = detection.pose.header
            if self.invert_tf:
                T_tag_camroot_tfs.header.frame_id = self.camera_root_frame_id
                T_tag_camroot_tfs.child_frame_id = self.base_frame_id
            else:
                T_tag_camroot_tfs.header.frame_id = self.base_frame_id
                T_tag_camroot_tfs.child_frame_id = self.camera_root_frame_id
            # publish the transform
            if self.continuous_calibration:
                self.tf_broadcaster.sendTransform(T_tag_camroot_tfs)
            else:
                self.static_tf_broadcaster.sendTransform(T_tag_camroot_tfs)
            self.last_T_tag_camroot_tfs = T_tag_camroot_tfs


if __name__ == '__main__':
    rospy.init_node('tf_tree_helper_from_topic_node')
    TfTreeHelperFromTopicNode()
    rospy.spin()
