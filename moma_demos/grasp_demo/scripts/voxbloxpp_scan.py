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

from grasp_demo.msg import ScanSceneAction, ScanSceneFeedback, ScanSceneResult
from panda_control.panda_commander import PandaCommander


class VoxbloxPPScanAction(object):
    def __init__(self):
        self._as = SimpleActionServer(
            "scan_action", ScanSceneAction, execute_cb=self.execute_cb, auto_start=False
        )

        self.panda_commander = PandaCommander("panda_arm")

        # Scan joint configurations
        self.scan_joints = [
            [
                -0.13044640511344546,
                -1.3581389637834451,
                -0.7902883262964496,
                -2.2897565394284434,
                0.04394716020347789,
                1.2693998432689242,
                1.0239726927690207,
            ],
            [
                1.1617,
                -0.5194,
                -1.6787,
                -1.345415587151993,
                0.27857345296034125,
                0.9858770641287168,
                0.48917187201250106,
            ],
            [
                0.03534398500304972,
                0.0792812212202517,
                0.00024001071061285442,
                -1.2297476580268456,
                0.07415731295971527,
                0.9772190555466544,
                0.8901656288974905,
            ],
            [
                -1.0761406668405233,
                -0.9812572320940955,
                1.0510634288286071,
                -1.4203576079753408,
                0.2339650344716178,
                0.9772807318199377,
                0.6811464487329869,
            ],
            [
                -0.3038625468067955,
                -1.2148889997380417,
                0.812242864175488,
                -1.754142465942784,
                0.16873402494854398,
                1.1185604270829095,
                0.9999408414952058,
            ],
        ]

        # Create service to trigger Voxblox++ integration
        integration_srv_name = "/gsm_node/toggle_integration"
        rospy.wait_for_service(integration_srv_name)
        self.toggle_integration = rospy.ServiceProxy(integration_srv_name, Empty)

        self._as.start()
        rospy.loginfo("Scan action server ready")

    def execute_cb(self, goal):
        rospy.loginfo("Scanning action was triggered")

        self.toggle_integration(EmptyRequest())

        for joints in self.scan_joints:

            if self._as.is_preempt_requested():
                rospy.loginfo("Got preempted")
                self._as.set_preempted()
                return

            self.panda_commander.goto_joint_target(joints, max_velocity_scaling=0.2)

        self.toggle_integration(EmptyRequest())

        # Wait for the scene point cloud
        # TODO(mbreyer) trigger "/gsm_node/get_scene_pointcloud" here, currently this is a manual process
        cloud = rospy.wait_for_message("/gsm_node/cloud", PointCloud2)

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
