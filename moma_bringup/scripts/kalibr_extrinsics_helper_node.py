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

def homog_mat_from_tfs(T_tfs):
        T_tf = T_tfs.transform
        T = tft.quaternion_matrix([
            T_tf.rotation.x,
            T_tf.rotation.y,
            T_tf.rotation.z,
            T_tf.rotation.w])
        T[0:3,-1] = np.array([
            T_tf.translation.x,
            T_tf.translation.y,
            T_tf.translation.z
        ])
        return T

def dynrec_callback(config, level):
    print('dynrec_callback')
    rospy.loginfo("""Reconfigure Request: 
    tx: {tx}, 
    ty: {ty}, 
    tz: {tz}, 
    roll: {roll}, 
    pitch: {pitch}, 
    yaw: {yaw}""".format(**config))
    return config

def calculate_transform():
    print('calculate_transform')
    # Initialize the ROS node
    rospy.init_node('tf_calculator')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Retrieve parameters from the parameter server
    frame_A_old = rospy.get_param('~frame_A_old', 'rs_435_2_color_optical_frame')
    frame_B_old = rospy.get_param('~frame_B_old', 'rs_435_3_color_optical_frame')
    frame_A_new = rospy.get_param('~frame_A_new', 'rs_435_2_link')
    frame_B_new = rospy.get_param('~frame_B_new', 'rs_435_3_link')

    # Get the transformation parameters from the parameter server
    # wednesday's snapshot bag with Paula
    t_Aold_Bold = rospy.get_param('~t_Aold_Bold', [
        -0.39136812, -0.05739385,  0.01090286,
        ])
    q_Aold_Bold = rospy.get_param('~q_Aold_Bold', [
        -0.01346537,  0.04188808,  0.99875846,  0.02335846,
        ])


    T_Aold_Bold = tft.quaternion_matrix([
                q_Aold_Bold[0],
                q_Aold_Bold[1],
                q_Aold_Bold[2],
                q_Aold_Bold[3]
                ])
    T_Aold_Bold[0:3,-1] = np.array([
        t_Aold_Bold[0],
        t_Aold_Bold[1],
        t_Aold_Bold[2]
    ])

    # query T_Aold_Anew from tree.
    try:
        # Lookup the transform from 'target_frame' to 'source_frame'
        T_Aold_Anew_tfs = tf_buffer.lookup_transform(frame_A_old, frame_A_new, rospy.Time(0), rospy.Duration(1.0))

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)
    T_Aold_Anew = homog_mat_from_tfs(T_Aold_Anew_tfs)

    # query T_Bold_Bnew from tree.
    try:
        # Lookup the transform from 'target_frame' to 'source_frame'
        T_Bold_Bnew_tfs = tf_buffer.lookup_transform(frame_B_old, frame_B_new, rospy.Time(0), rospy.Duration(1.0))

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)
    T_Bold_Bnew = homog_mat_from_tfs(T_Bold_Bnew_tfs)

    # compute T_Aold_Bnew
    T_Aold_Bnew = tft.concatenate_matrices(T_Aold_Bold, T_Bold_Bnew)

    # compute T_Anew_Bnew
    # T_Aold_Bnew = T_Aold_Anew*T_Anew_Bnew <-> inv(T_Aold_Anew)*T_Aold_Bnew = T_Anew_Bnew
    T_Anew_Bnew = tft.concatenate_matrices(tft.inverse_matrix(T_Aold_Anew), T_Aold_Bnew)

    # send off          
    q_Anew_Bnew = tft.quaternion_from_matrix(T_Anew_Bnew)
    T_Anew_Bnew_tfs = geometry_msgs.msg.TransformStamped()
    T_Anew_Bnew_tfs.transform.rotation.x = q_Anew_Bnew[0]
    T_Anew_Bnew_tfs.transform.rotation.y = q_Anew_Bnew[1]
    T_Anew_Bnew_tfs.transform.rotation.z = q_Anew_Bnew[2]
    T_Anew_Bnew_tfs.transform.rotation.w = q_Anew_Bnew[3]
    T_Anew_Bnew_tfs.transform.translation.x = T_Anew_Bnew[0,-1]
    T_Anew_Bnew_tfs.transform.translation.y = T_Anew_Bnew[1,-1]
    T_Anew_Bnew_tfs.transform.translation.z = T_Anew_Bnew[2,-1]
    T_Anew_Bnew_tfs.header.stamp = rospy.Time.now()
    T_Anew_Bnew_tfs.header.frame_id = frame_A_new
    T_Anew_Bnew_tfs.child_frame_id = frame_B_new
    # publish the transform
    static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_tf_broadcaster.sendTransform(T_Anew_Bnew_tfs)
    rospy.sleep(1)
    static_tf_broadcaster.sendTransform(T_Anew_Bnew_tfs)
    
if __name__ == '__main__':
    rospy.init_node("kalibr_helper_node")
    srv = Server(ExtrinsicsConfig, dynrec_callback)
    rospy.spin()
    # try:
    #     calculate_transform()
    # except rospy.ROSInterruptException:
    #     pass
