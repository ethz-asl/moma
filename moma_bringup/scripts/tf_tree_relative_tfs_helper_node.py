#!/usr/bin/env python
# template written by chatGPT.

import rospy
import tf
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import geometry_msgs.msg

from moma_bringup.cfg import ExtrinsicsConfig
from dynamic_reconfigure.server import Server


class TfTreeRelTfHelper:
    def __init__(self):
        self.init_node()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # infra RS2-RS3 (images_2024-09-06-12-07-07.bag)
        self.frame_A_old = rospy.get_param(
            '~frame_A_old', 'rs_435_2_infra1_optical_frame')
        self.frame_B_old = rospy.get_param(
            '~frame_B_old', 'rs_435_3_infra1_optical_frame')
        self.frame_A_new = rospy.get_param('~frame_A_new', 'rs_435_2_link')
        self.frame_B_new = rospy.get_param('~frame_B_new', 'rs_435_3_link')

        self.t_Aold_Bold = rospy.get_param('~t_Aold_Bold', [
            -0.41437184, -0.01970905,  0.0101194,
        ])
        self.q_Aold_Bold = rospy.get_param('~q_Aold_Bold', [
            0.00412074, -0.01980342,  0.99930248,  0.03139113,
        ])

        self.T_Aold_Bold = self.compute_T_Aold_Bold()

        srv = Server(ExtrinsicsConfig, self.dynrec_callback)

    def init_node(self):
        rospy.init_node('kalibr_helper_node')

    def compute_T_Aold_Bold(self):
        T_Aold_Bold = tft.quaternion_matrix([
            self.q_Aold_Bold[0],
            self.q_Aold_Bold[1],
            self.q_Aold_Bold[2],
            self.q_Aold_Bold[3]
        ])
        T_Aold_Bold[0:3, -1] = np.array([
            self.t_Aold_Bold[0],
            self.t_Aold_Bold[1],
            self.t_Aold_Bold[2]
        ])
        return T_Aold_Bold

    def homog_mat_from_tfs(self, T_tfs):
        T_tf = T_tfs.transform
        T = tft.quaternion_matrix([
            T_tf.rotation.x,
            T_tf.rotation.y,
            T_tf.rotation.z,
            T_tf.rotation.w])
        T[0:3, -1] = np.array([
            T_tf.translation.x,
            T_tf.translation.y,
            T_tf.translation.z
        ])
        return T

    def dynrec_callback(self, config, level):
        self.t_x_offset = config.tx
        self.t_y_offset = config.ty
        self.t_z_offset = config.tz
        self.roll_offset = config.roll
        self.pitch_offset = config.pitch
        self.yaw_offset = config.yaw
        self.calculate_transform()
        return config

    def calculate_transform(self):
        try:
            T_Aold_Anew_tfs = self.tf_buffer.lookup_transform(
                self.frame_A_old, self.frame_A_new, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return

        T_Aold_Anew = self.homog_mat_from_tfs(T_Aold_Anew_tfs)

        try:
            T_Bold_Bnew_tfs = self.tf_buffer.lookup_transform(
                self.frame_B_old, self.frame_B_new, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return

        T_Bold_Bnew = self.homog_mat_from_tfs(T_Bold_Bnew_tfs)

        T_Aold_Bnew = tft.concatenate_matrices(self.T_Aold_Bold, T_Bold_Bnew)
        T_Anew_Bnew = tft.concatenate_matrices(
            tft.inverse_matrix(T_Aold_Anew), T_Aold_Bnew)

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
        T_Anew_Bnew_tfs = geometry_msgs.msg.TransformStamped()
        T_Anew_Bnew_tfs.transform.rotation.x = q_Anew_Bnew[0]
        T_Anew_Bnew_tfs.transform.rotation.y = q_Anew_Bnew[1]
        T_Anew_Bnew_tfs.transform.rotation.z = q_Anew_Bnew[2]
        T_Anew_Bnew_tfs.transform.rotation.w = q_Anew_Bnew[3]
        T_Anew_Bnew_tfs.transform.translation.x = T_Anew_Bnew[0, -1]
        T_Anew_Bnew_tfs.transform.translation.y = T_Anew_Bnew[1, -1]
        T_Anew_Bnew_tfs.transform.translation.z = T_Anew_Bnew[2, -1]
        T_Anew_Bnew_tfs.header.stamp = rospy.Time.now()
        T_Anew_Bnew_tfs.header.frame_id = self.frame_A_new
        T_Anew_Bnew_tfs.child_frame_id = self.frame_B_new

        self.static_tf_broadcaster.sendTransform(T_Anew_Bnew_tfs)
        rospy.sleep(1)
        self.static_tf_broadcaster.sendTransform(T_Anew_Bnew_tfs)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = TfTreeRelTfHelper()
    node.calculate_transform()
    node.run()
