#!/usr/bin/env python

from __future__ import print_function

import sys

from actionlib import SimpleActionServer
from geometry_msgs.msg import TransformStamped
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points, create_cloud
from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


from grasp_demo.msg import ScanSceneAction, ScanSceneResult
from moma_utils.ros.moveit import MoveItClient
from vpp_msgs.srv import GetScenePointcloud, GetScenePointcloudRequest


class ReconstructSceneNode(object):
    """Reconstruct scene moving the camera along a fixed trajectory.
    
    The reconstruction can either done with Voxblox++ or via simple point cloud stitching.
    """

    def __init__(self, semantic):
        self.moveit = MoveItClient("panda_arm")
        self.scan_joints = rospy.get_param("moma_demo/scan_joints_arm")

        if semantic:
            execute_cb = self.reconstruct_semantic_scene
            rospy.loginfo("Waiting for gsm_node")
            rospy.wait_for_service("/gsm_node/reset_map")
            self._reset_map = rospy.ServiceProxy("/gsm_node/reset_map", Empty)
            self._toggle_integration = rospy.ServiceProxy(
                "/gsm_node/toggle_integration", SetBool
            )
            self._query_point_cloud = rospy.ServiceProxy(
                "/gsm_node/get_scene_pointcloud", GetScenePointcloud
            )

        else:
            execute_cb = self.reconstruct_scene
            self.base_frame = rospy.get_param("moma_demo/base_frame_id")
            self.cloud_pub = rospy.Publisher("~cloud", PointCloud2, queue_size=1)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.action_server = SimpleActionServer(
            "scan_action", ScanSceneAction, execute_cb=execute_cb, auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("Scan action server ready")

    def reconstruct_scene(self, goal):
        rospy.loginfo("Stitching point clouds")
        captured_clouds = []
        for joints in self.scan_joints:
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
        rospy.loginfo("Mapping scene")
        self._reset_map(EmptyRequest())

        self._toggle_integration(SetBoolRequest(data=True))

        for joints in self.scan_joints:
            self.moveit.goto(joints, velocity_scaling=0.2, acceleration_scaling=0.2)
            rospy.sleep(1.0)

        self._toggle_integration(SetBoolRequest(data=False))

        # Wait for the scene point cloud
        msg = self._query_point_cloud(GetScenePointcloudRequest())
        cloud = msg.scene_cloud
        # TODO(mbreyer) check frame of point cloud

        result = ScanSceneResult(pointcloud_scene=cloud)
        self.acion_server.set_succeeded(result)
        rospy.loginfo("Scan scene action succeeded")

    def _transform_pointcloud(self, cloud_msg):
        frame = cloud_msg.header.frame_id
        stamp = cloud_msg.header.stamp
        transform = self.tf_buffer.lookup_transform(
            self.base_frame, frame, stamp, rospy.Duration(0.2)
        )
        transformed_cloud_msg = do_transform_cloud(cloud_msg, transform)
        transformed_cloud_msg.header.frame_id = self.base_frame
        return transformed_cloud_msg

    def _stitch_point_clouds(self, clouds):
        points_out = []
        for cloud in clouds:
            points_out += list(read_points(cloud))
        return create_cloud(cloud.header, cloud.fields, points_out)


def main():
    rospy.init_node("scan_action_node")
    semantic = sys.argv[1] in ["True", "true"]
    ReconstructSceneNode(semantic)
    rospy.spin()


if __name__ == "__main__":
    main()
