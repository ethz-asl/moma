#!/usr/bin/env python3
import subprocess
import sys

import numpy as np
import pinocchio as pin
import rospy
import tf2_ros
import xacro
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose


class TransformListener:
    """A base class for a transform listener"""

    def __init__(self, reference_frame: str, child_frame: str) -> None:
        self.reference_frame = reference_frame
        self.child_frame = child_frame
        self.link_pose = None

    def update_pose(self) -> None:
        raise NameError("'update_pose' must be implemented")

    def get_pose(self) -> pin.SE3:
        self.update_pose()
        if self.link_pose is None:
            raise NameError(
                f"could not listen to pose {self.child_frame} to {self.reference_frame}"
            )
        else:
            rospy.loginfo(
                f"Transfrom from {self.child_frame} to {self.reference_frame} is\n{self.link_pose}"
            )
        return self.link_pose


class GazeboTransformListener(TransformListener):
    def __init__(self, robot_name: str, link_name: str):
        TransformListener.__init__(self, reference_frame="world", child_frame=link_name)
        self.robot_name = robot_name
        self.gazebo_link_name = f"{robot_name}::{link_name}"
        self.states_sub = rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self.callback
        )

    def update_pose(self) -> None:
        max_attempts = 10
        attempts = 0
        while self.link_pose is None and attempts < max_attempts:
            rospy.sleep(1.0)
            attempts += 1

    def callback(self, data):
        try:
            ind = data.name.index(self.gazebo_link_name)
            gazebo_link_pose: Pose = data.pose[ind]
            self.link_pose = pin.XYZQUATToSE3(
                [
                    gazebo_link_pose.position.x,
                    gazebo_link_pose.position.y,
                    gazebo_link_pose.position.z,
                    gazebo_link_pose.orientation.x,
                    gazebo_link_pose.orientation.y,
                    gazebo_link_pose.orientation.z,
                    gazebo_link_pose.orientation.w,
                ]
            )
        except ValueError:
            rospy.logerr(f"No link named {self.link_name} in gazebo link states.")


class RosTransformListener(TransformListener):
    def __init__(self, reference_frame: str, child_frame: str) -> None:
        TransformListener.__init__(
            self, reference_frame=reference_frame, child_frame=child_frame
        )
        self.transform_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transform_buffer)

    def update_pose(self):
        try:
            transform = self.transform_buffer.lookup_transform(
                self.reference_frame,
                self.child_frame,
                rospy.Time(0),
                rospy.Duration(10),
            )
            self.link_pose = pin.XYZQUATToSE3(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
            )
            return self.link_pose
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logerr(exc)


if __name__ == "__main__":
    """
    # 1. Read transform of robot to world from gazebo T_w_r
    # 2. Read transform of map to robot from icp (ROS) T_r_f
    # 3. Get T_w_f = T_w_r * T_r_f
    # 4. Read from param object pose T_f_o
    # 5. Get T_w_o = T_w_f * T_f_o
    # 6. Spawn object in gazebo using gazebo service and T_w_o
    """
    try:
        rospy.init_node("gazebo_smart_spawner", anonymous=True)
        robot_name = rospy.get_param("~robot_name")
        robot_link = rospy.get_param("~robot_link")
        fixed_link = rospy.get_param("~fixed_link")
        fix_position = rospy.get_param("~fix_position")
        fix_rpy = rospy.get_param("~fix_rpy")
        fix_rotation = pin.rpy.rpyToMatrix(*fix_rpy)

        asset_name = rospy.get_param("~asset_name")
        asset_xacro_file = rospy.get_param("~asset_xacro_file")
        asset_xacro_arguments = rospy.get_param("~asset_xacro_argument")
        asset_description_name = f"{asset_name}_description"

        gtf_listener = GazeboTransformListener(
            robot_name=robot_name, link_name=robot_link
        )
        rtf_listener = RosTransformListener(
            reference_frame=fixed_link, child_frame=robot_link
        )

        rospy.loginfo(f"Reading {asset_name} pose in gazebo world...")
        T_f_o = pin.SE3(fix_rotation, np.asarray(fix_position))
        T_w_r = gtf_listener.get_pose()
        T_f_r = rtf_listener.get_pose()
        T_w_o = T_w_r.act(T_f_r.actInv(T_f_o))

        xyz = T_w_o.translation
        rpy = pin.rpy.matrixToRpy(T_w_o.rotation)
        rospy.loginfo(
            f"{asset_name} pose in gazebo world frame is (xyz, rpy)\n{xyz}\n{rpy}"
        )

        xacro_cmd = f"$(xacro {asset_xacro_file} x:={xyz[0]} y:={xyz[1]} z:={xyz[2]} roll:={rpy[0]} pitch:={rpy[1]} yaw:={rpy[2]}) {asset_xacro_arguments}"
        load_description_cmd = f'rosparam set {asset_description_name} "{xacro_cmd}"'
        rospy.loginfo(
            f"Loading {asset_description_name} running:\n {load_description_cmd}"
        )
        load_description_rc = subprocess.Popen(
            load_description_cmd, shell=True, stdout=subprocess.PIPE
        ).wait(10.0)
        if load_description_rc != 0:
            rospy.logerr(
                f"Failed to load the description. Exit code {load_description_rc}"
            )

        rospy.loginfo(f"Spawning {asset_name} in gazebo.")
        spawn_model_cmd = f"rosrun gazebo_ros spawn_model -param {asset_description_name} -urdf -model {asset_name}"
        spawn_model_rc = subprocess.Popen(
            spawn_model_cmd, shell=True, stdout=subprocess.PIPE
        ).wait(10.0)
        if spawn_model_rc != 0:
            rospy.logerr(
                f"Failed to spawn the gazebo model. Exit code {spawn_model_rc}"
            )
        rospy.loginfo("Done")

    except (rospy.ROSInterruptException, NameError) as exc:
        rospy.logerr(exc)
