#!/usr/bin/env python

import sys
import subprocess
import rospy
import tf2_ros
import pinocchio as pin
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates


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
            ind = data.name.index(self.link_name)
            gazebo_link_pose: Pose = data.pose[ind]
            self.link_pose = pin.XYZQUATToSE3(
                gazebo_link_pose.position.x,
                gazebo_link_pose.position.y,
                gazebo_link_pose.pose.position.z,
                gazebo_link_pose.orientation.x,
                gazebo_link_pose.orientation.y,
                gazebo_link_pose.orientation.z,
                gazebo_link_pose.orientation.w,
            )
        except ValueError:
            rospy.logerr(f"No link named {self.link_name} in gazebo link states.")


class RosTransformListener(TransformListener):
    def __init__(self, reference_frame: str, child_frame: str) -> None:
        TransformListener(reference_frame=reference_frame, child_frame=child_frame)
        self.transform_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transform_buffer)

    def get_pose(self) -> pin.SE3:
        try:
            transform = self.transform_buffer.lookup_transform(
                self.reference_frame,
                self.source_frame,
                rospy.Time(0),
                rospy.Duration(10),
            )
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
        fix_rotation = pin.rpy.rpyToMatrix(fix_rpy)

        asset_name = rospy.get_param("~asset_name")
        asset_xacro_file = rospy.get_param("~asset_xacro_file")
        asset_xacro_arguments = rospy.get_param("~asset_xacro_argument")

        gtf_listener = GazeboTransformListener(
            robot_name=robot_name, robot_link=robot_link
        )
        rtf_listener = RosTransformListener(
            reference_frame=fixed_link, child_frame=robot_link
        )

        rospy.loginfo("Reading {asset_name} pose in gazebo world...")
        T_f_o = pin.SE3(fix_rotation, fix_position)
        T_w_r = gtf_listener.get_pose()
        T_f_r = rtf_listener.get_pose()
        T_w_o = T_w_r.act(T_f_r.actInv(T_f_o))

        xyz = T_f_o.translation
        rpy = pin.rpy.matrixToRpy(T_f_o.rotation)
        rospy.loginfo(
            f"{asset_name} pose in gazebo world frame is (xyz, rpy)\n{xyz}\n{rpy}"
        )

        rospy.loginfo(f"Loading {asset_name} description.")
        load_description_cmd = """rosparam set {description_name} "$(xacro {asset_xacro_file} x:={xyz[0]} y:={xyz[1]} z:={xyz[2]} roll:={rpy[0]} pitch:={rpy[1]} yaw:=x:={rpy[2]}) {asset_xacro_arguments}" """
        subprocess.Popen(
            load_description_cmd, shell=True, stdout=subprocess.STDOUT
        ).stdout.read()

        rospy.loginfo(f"Waiting for gazebo spawn model service to become available")
        try:
            rospy.wait_for_service("gazebo/spawn_model", timeout=20.0)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            sys.exit(0)

        rospy.loginfo(f"Spawnng {asset_name} in gazebo.")
        spawn_model_cmd = """rosrun gazebo_ros spawn_model -param {description_name} -urdf -model {object_name}"""
        subprocess.Popen(
            spawn_model_cmd, shell=True, stdout=subprocess.STDOUT
        ).stdout.read()

    except (rospy.ROSInterruptException, NameError) as exc:
        rospy.logerr(exc)
