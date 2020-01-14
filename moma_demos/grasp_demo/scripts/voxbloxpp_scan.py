#!/usr/bin/env python

from __future__ import print_function

import copy
import pickle
import sys

from actionlib import SimpleActionServer
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from gpd_ros.msg import GraspConfigList
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest
from vpp_msgs.srv import GetScenePointcloud, GetScenePointcloudRequest

from grasp_demo.msg import ScanSceneAction, ScanSceneResult
from grasp_demo.utils import create_robot_connection


class VoxbloxPPScanAction(object):
    """
        Move arm along prespecified joint configurations (scanning motion). While doing so,
        let Voxblox++ integrate sensor readings. Finally, obtain a pointcloud of the reconstructed
        scene from Voxblox++ and return it.
    """

    def __init__(self):
        self._read_joint_configurations()
        self._subscribe_voxbloxpp()
        self._connect_yumi()
        self._setup_action_server()
        self._as.start()
        rospy.loginfo("Scan action server ready")

    def _read_joint_configurations(self):
        self._scan_joints = rospy.get_param("scan_joint_values")
        self._search_joints_r = rospy.get_param("search_joints_r")
        self._ready_joints_l = rospy.get_param("ready_joints_l")
    
    def _subscribe_voxbloxpp(self):
        reset_srv_name = "/gsm_node/reset_map"
        rospy.wait_for_service(reset_srv_name)
        self._reset_map = rospy.ServiceProxy(reset_srv_name, Empty)

        integration_srv_name = "/gsm_node/toggle_integration"
        rospy.wait_for_service(integration_srv_name)
        self._toggle_integration = rospy.ServiceProxy(integration_srv_name, SetBool)

        point_cloud_srv_name = "/gsm_node/get_scene_pointcloud"
        rospy.wait_for_service(point_cloud_srv_name)
        self._query_point_cloud = rospy.ServiceProxy(
            point_cloud_srv_name, GetScenePointcloud
        )

    def _connect_yumi(self):
        self._left_arm = create_robot_connection("yumi_left_arm")
        self._right_arm = create_robot_connection("yumi_right_arm")

    def _setup_action_server()
        self._as = SimpleActionServer(
            "scan_action", ScanSceneAction, execute_cb=self.execute_cb, auto_start=False
        )

    def execute_cb(self, goal):
        rospy.loginfo("Scanning action was triggered")

        self._reset_map(EmptyRequest())
        self._left_arm.goto_joint_target(self._ready_joints_l, max_velocity_scaling=0.4)

        self._toggle_integration(SetBoolRequest(data=True))

        for i, joints in enumerate(self.scan_joints):
            if self._as.is_preempt_requested():
                rospy.loginfo("Got preempted")
                self._as.set_preempted()
                return

            self._right_arm.goto_joint_target(
                joints, max_acceleration_scaling=0.2, max_velocity_scaling=0.2
            )
            rospy.sleep(1.0)

        self._toggle_integration(SetBoolRequest(data=False))

        # Wait for the scene point cloud
        msg = self._query_point_cloud(GetScenePointcloudRequest())
        cloud = msg.scene_cloud
        # TODO(mbreyer) check frame of point cloud 

        self._right_arm.goto_joint_target(self._scan_joints[0], max_velocity_scaling=0.4)

        rospy.loginfo("Scan scene action succeeded")
        result = ScanSceneResult(pointcloud_scene=cloud)
        self._as.set_succeeded(result)


if __name__ == "__main__":
    try:
        rospy.init_node("scan_action_node")
        VoxbloxPPScanAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
