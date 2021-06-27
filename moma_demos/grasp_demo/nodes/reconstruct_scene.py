#!/usr/bin/env python

from __future__ import print_function

import argparse

from actionlib import SimpleActionServer
from geometry_msgs.msg import TransformStamped
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points, create_cloud
import tf
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from grasp_demo.msg import ScanSceneAction, ScanSceneResult
from moma_utils.ros.moveit import MoveItClient


class ReconstructSceneNode(object):
    """Reconstruct scene moving the camera along a fixed trajectory.
    
    The reconstruction can either done with Voxblox++ or via simple point cloud stitching.
    """

    def __init__(self, semantic):
        self.moveit = MoveItClient("panda_arm")
        self.scan_joints = rospy.get_param("moma_demo/scan_joints_arm")

        if semantic:
            execute_cb = self.reconstruct_semantic_scene
        else:
            execute_cb = self.reconstruct_scene
            self.base_frame_id = rospy.get_param("moma_demo/base_frame_id")
            self.listener = tf.TransformListener()
            self.cloud_pub = rospy.Publisher("~cloud", PointCloud2, queue_size=1)

        self.action_server = SimpleActionServer(
            "scan_action", ScanSceneAction, execute_cb=execute_cb, auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("Scan action server ready")

    def reconstruct_scene(self, goal):
        rospy.loginfo("Stitching point clouds")
        captured_clouds = []
        for joints in self.scan_joints:
            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                return

            self.moveit.goto(joints, velocity_scaling=0.4, acceleration_scaling=0.2)
            rospy.sleep(0.5)
            cloud_msg = rospy.wait_for_message(
                "/camera/depth/color/points", PointCloud2
            )
            cloud_msg = self._transform_pointcloud(cloud_msg)
            captured_clouds.append(cloud_msg)

        cloud = self._stitch_point_clouds(captured_clouds)
        self.cloud_pub.publish(cloud)

        result = ScanSceneResult(pointcloud_scene=cloud)
        self.action_server.set_succeeded(result)

    def reconstruct_semantic_scene(self, goal):
        pass

    def _transform_pointcloud(self, msg):
        frame = msg.header.frame_id
        translation, rotation = self.listener.lookupTransform(
            self.base_frame_id, frame, rospy.Time()
        )
        transform_msg = TransformStamped()
        transform_msg.transform.translation.x = translation[0]
        transform_msg.transform.translation.y = translation[1]
        transform_msg.transform.translation.z = translation[2]
        transform_msg.transform.rotation.x = rotation[0]
        transform_msg.transform.rotation.y = rotation[1]
        transform_msg.transform.rotation.z = rotation[2]
        transform_msg.transform.rotation.w = rotation[3]
        transformed_msg = do_transform_cloud(msg, transform_msg)
        transformed_msg.header = msg.header
        transformed_msg.header.frame_id = self.base_frame_id
        return transformed_msg

    def _stitch_point_clouds(self, clouds):
        points_out = []
        for cloud in clouds:
            points_out += list(read_points(cloud))
        return create_cloud(cloud.header, cloud.fields, points_out)


def create_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--semantic", type=str)
    return parser


def main():
    rospy.init_node("scan_action_node")
    ReconstructSceneNode(False)
    rospy.spin()


if __name__ == "__main__":
    main()
