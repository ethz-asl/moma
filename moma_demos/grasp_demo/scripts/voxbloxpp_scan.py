#!/usr/bin/env python

from __future__ import print_function

import copy
import pickle

from actionlib import SimpleActionServer
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from gpd_ros.msg import GraspConfigList
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty, EmptyRequest
from vpp_msgs.srv import GetScenePointcloud, GetScenePointcloudRequest

from grasp_demo.msg import ScanSceneAction, ScanSceneFeedback, ScanSceneResult
from panda_control.panda_commander import PandaCommander


class VoxbloxPPScanAction(object):
    def __init__(self):
        self._as = SimpleActionServer(
            "scan_action", ScanSceneAction, execute_cb=self.execute_cb, auto_start=False
        )

        self.panda_commander = PandaCommander("panda_arm")

        self.scan_joints = rospy.get_param("grasp_demo")["scan_joint_values"]
        self.home_joints = self.panda_commander.move_group.get_named_target_values(
            "home"
        )

        # Create service to reset Voxblox++ map
        reset_srv_name = "/gsm_node/reset_map"
        rospy.wait_for_service(reset_srv_name)
        self.reset_map = rospy.ServiceProxy(reset_srv_name, Empty)

        # Create service to trigger Voxblox++ integration
        integration_srv_name = "/gsm_node/toggle_integration"
        rospy.wait_for_service(integration_srv_name)
        self.toggle_integration = rospy.ServiceProxy(integration_srv_name, Empty)

        # Create service to query point cloud
        point_cloud_srv_name = "/gsm_node/get_scene_pointcloud"
        rospy.wait_for_service(point_cloud_srv_name)
        self.query_point_cloud = rospy.ServiceProxy(
            point_cloud_srv_name, GetScenePointcloud
        )

        self._as.start()
        rospy.loginfo("Scan action server ready")

    def execute_cb(self, goal):
        rospy.loginfo("Scanning action was triggered")

        self.panda_commander.move_group.set_max_acceleration_scaling_factor(0.2)
        self.reset_map(EmptyRequest())

        for i, joints in enumerate(self.scan_joints):

            if i == 1:
                self.toggle_integration(EmptyRequest())

            if self._as.is_preempt_requested():
                rospy.loginfo("Got preempted")
                self._as.set_preempted()
                return

            self.panda_commander.goto_joint_target(joints, max_velocity_scaling=0.3)

        self.toggle_integration(EmptyRequest())

        # Wait for the scene point cloud
        msg = self.query_point_cloud(GetScenePointcloudRequest())
        cloud = msg.scene_cloud

        self.panda_commander.goto_joint_target(
            self.home_joints, max_velocity_scaling=0.4
        )

        result = ScanSceneResult(pointcloud_scene=cloud)
        self._as.set_succeeded(result)

        rospy.loginfo("Scan scene action succeeded")


if __name__ == "__main__":
    try:
        rospy.init_node("scan_action_node")
        VoxbloxPPScanAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
