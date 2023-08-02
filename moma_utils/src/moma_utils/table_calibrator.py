#!/usr/bin/env python3
import rospy

import tf2_ros
import geometry_msgs.msg
import std_srvs.srv
import json

from moma_utils.spatial import Transform
from moma_utils.ros.conversions import *


class TableCalibrator:
    def __init__(self):
        # Initialize TF buffers and stuff.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Initialize the relevant transforms
        self._T_w_t = Transform.identity()
        self._T_t_c = Transform.identity()

        # Get ROS params
        self.get_parameters()

        # Create service calls
        self._calibrate_camera_srv = rospy.Service(
            '~calibrate_camera', std_srvs.srv.Empty, self.calibrate_camera)
        self._calibrate_table_srv = rospy.Service(
            '~calibrate_table', std_srvs.srv.Empty, self.calibrate_table)
        self._write_to_file_srv = rospy.Service(
            '~write_to_file', std_srvs.srv.Empty, self.write_srv)
        self._read_from_file_srv = rospy.Service(
            '~read_from_file', std_srvs.srv.Empty, self.read_srv)

    def get_parameters(self):
        self._global_frame = rospy.get_param('~global_frame', 'world')
        self._table_frame = rospy.get_param('~table_frame', 'table')
        self._tag_frame = rospy.get_param('~tag_frame', 'tag_0')
        self._camera_base_frame = rospy.get_param(
            '~camera_base_frame', 'fixed_camera_link')
        self._calibration_file_path = rospy.get_param(
            '~file_path', 'calibration.yaml')
        self._load_from_params = rospy.get_param('~load_from_params', False)

        if self._load_from_params:
            self.load_from_parameters()
        else:
            self.load_from_file(self._calibration_file_path)

    def load_from_file(self, path):
        with open(path, 'r') as file:
            line = file.readline()
            line = line.replace('T_w_t: ', '')
            line = line.replace('\"', '')
            list_var = json.loads(line)
            print("T_w_t: ", list_var)
            self._T_w_t = Transform.from_list(list_var)

            line = file.readline()
            line = line.replace('T_t_c: ', '')
            line = line.replace('\"', '')
            list_var = json.loads(line)
            print("T_t_c: ", list_var)
            self._T_t_c = Transform.from_list(list_var)
            rospy.loginfo('Read calibration file from: %s', path)

        self.publish_transforms()

    def load_from_parameters(self):
        try:
            T_w_t_str = rospy.get_param('~T_w_t')
            T_t_c_str = rospy.get_param('~T_t_c')
            self._T_w_t = Transform.from_list(json.loads(T_w_t_str))
            self._T_t_c = Transform.from_list(json.loads(T_t_c_str))
            rospy.loginfo('Loaded calibration from parameter server.')
        except (KeyError, json.JSONDecodeError):
            rospy.logwarn(
                "Couldn't look up transforms from the parameter server!")

        self.publish_transforms()

    def write_to_file(self, path):
        with open(path, 'w') as file:
            file.write('T_w_t: \"[')
            list_var = self._T_w_t.to_list()
            file.write(', '.join(map(str, list_var)))
            file.write(']\"\n')

            file.write('T_t_c: \"[')
            list_var = self._T_t_c.to_list()
            file.write(', '.join(map(str, list_var)))
            file.write(']\"\n')
            rospy.loginfo('Output calibration file to: %s', path)

    def write_srv(self, req):
        self.write_to_file(self._calibration_file_path)
        return []

    def read_srv(self, req):
        self.load_from_file(self._calibration_file_path)
        return []

    def publish_transforms(self):
        """ Publishes world to table transform and world to camera transform.
        Call this function again to refresh if it's been calibrated."""
        # World to table.
        msg = geometry_msgs.msg.TransformStamped()
        msg.transform = to_transform_msg(self._T_w_t)
        msg.header.frame_id = self._global_frame
        msg.child_frame_id = self._table_frame
        msg.header.stamp = rospy.Time.now()
        self._tf_broadcaster.sendTransform(msg)

        # World to cam.
        msg = geometry_msgs.msg.TransformStamped()
        msg.transform = to_transform_msg(self._T_w_t * self._T_t_c)
        msg.header.frame_id = self._global_frame
        msg.child_frame_id = self._camera_base_frame
        msg.header.stamp = rospy.Time.now()
        self._tf_broadcaster.sendTransform(msg)

    def calibrate_camera(self, req):
        """ Calibrates a static camera to the global frame via a pre-calibrated
        table. """
        # Look up the transform from camera to the tag:
        try:
            T_t_c_msg = self._tf_buffer.lookup_transform(
                self._camera_base_frame, self._tag_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn(
                "Couldn't look up transform from camera to tag. "
                "Is the tag in view?")
            return None

        self._T_t_c = from_transform_msg(T_t_c_msg.transform).inverse()

        # We should already have the transform from the table frame to the
        # global frame, otherwise identity.
        T_w_c = self._T_w_t * self._T_t_c

        print(
            "T from camera to world ([tx ty tz qx qy qz qw]): ",
            T_w_c.to_list())

        self.publish_transforms()
        return []

    def calibrate_table(self, req):
        """ Calibrates the table to the global frame via a pre-calibrated camera. """
        try:
            T_w_t_msg = self._tf_buffer.lookup_transform(
                self._global_frame, self._tag_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn(
                "Couldn't look up transform from global frame to tag. "
                "Is the tag in view?")
            return None

        self._T_w_t = from_transform_msg(T_w_t_msg.transform)

        print(
            "T from table to world ([tx ty tz qx qy qz qw]): ",
            self._T_w_t.to_list())

        self.publish_transforms()
        return []


if __name__ == '__main__':
    rospy.init_node('table_calibrator')

    # Create a table calibrator object
    table_calibrator = TableCalibrator()

    rospy.spin()
