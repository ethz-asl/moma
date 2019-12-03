#!/usr/bin/env python

from __future__ import print_function

import copy
import pickle

import actionlib
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from gpd_ros.msg import GraspConfigList
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points, create_cloud
import tf
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from std_srvs.srv import Trigger, TriggerRequest

from grasp_demo.msg import ScanSceneAction, ScanSceneFeedback, ScanSceneResult
from grasp_demo.panda_commander import PandaCommander


class ScanActionNode(object):
    def __init__(self):
        # Panda commander
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

        # Service proxys to control the point cloud fusion
        rospy.wait_for_service("/point_cloud_reconstruction/reset")

        self.reset_fusion = rospy.ServiceProxy(
            "/point_cloud_reconstruction/reset", Trigger
        )
        self.start_fusion = rospy.ServiceProxy(
            "/point_cloud_reconstruction/start", Trigger
        )
        self.stop_fusion = rospy.ServiceProxy(
            "/point_cloud_reconstruction/stop", Trigger
        )

        # Create publisher for stitched point cloud
        self.stitched_point_cloud_pub = rospy.Publisher(
            "/cloud_stitched", PointCloud2, queue_size=10
        )

        # Create publisher for selected grasp
        self.selected_grasp_pub = rospy.Publisher(
            "/grasp_pose", PoseStamped, queue_size=10
        )

        # Set up action server
        action_name = "pointcloud_scan_action"
        self._as = actionlib.SimpleActionServer(
            action_name, ScanSceneAction, execute_cb=self.execute_cb, auto_start=False
        )
        self._as.start()

        rospy.loginfo("Scan action server ready")

    def execute_cb(self, goal):
        rospy.loginfo("Scanning action was triggered")
        result = ScanSceneResult()

        if goal.num_scan_poses > len(self.scan_joints):
            rospy.info("Invalid goal set")
            self._as.set_aborted(result)
            return

        self.reset_fusion()
        self.start_fusion()

        # Perform the scan trajectory
        for i in range(goal.num_scan_poses):

            if self._as.is_preempt_requested():
                rospy.loginfo("Got preempted")
                self._as.set_preempted()
                return

            self.panda_commander.goto_joint_target(
                self.scan_joints[i], max_velocity_scaling=0.5
            )

        # Wait for the latest reconstruction
        stitched_cloud = rospy.wait_for_message(
            "/point_cloud_reconstruction/point_cloud", PointCloud2
        )
        self.stop_fusion()

        # Send reconstructed point cloud to GPD
        self.stitched_point_cloud_pub.publish(stitched_cloud)

        try:
            grasp_candidates = rospy.wait_for_message(
                "/detect_grasps/clustered_grasps", GraspConfigList, timeout=30
            )
        except rospy.ROSException:
            rospy.loginfo("GPD server timed out")
            self._as.set_aborted(result)
            return

        if len(grasp_candidates.grasps) == 0:
            rospy.loginfo("No grasps detected")
            self._as.set_aborted(result)
            return

        grasp_pose = self.select_grasp_pose(grasp_candidates)
        self.selected_grasp_pub.publish(grasp_pose)

        rospy.loginfo("Scanning action succeeded")
        result.selected_grasp_pose = grasp_pose
        self._as.set_succeeded(result)

    def select_grasp_pose(self, grasp_config_list):
        grasp = grasp_config_list.grasps[0]

        x_axis = np.r_[grasp.axis.x, grasp.axis.y, grasp.axis.z]
        y_axis = np.r_[grasp.binormal.x, grasp.binormal.y, grasp.binormal.z]
        z_axis = np.r_[grasp.approach.x, grasp.approach.y, grasp.approach.z]
        rot_mat = np.vstack([x_axis, y_axis, z_axis]).T

        if np.linalg.det(rot_mat) < 0:
            rospy.loginfo(
                "Grasp pose vectors not a right-handed system. Flipping y-axis."
            )
            y_axis *= -1
            rot_mat = np.vstack([x_axis, y_axis, z_axis]).T

        rot = Rotation.from_dcm(rot_mat)

        if x_axis[0] < 0:
            rospy.loginfo(
                "Flipped grasp pose. x-axis was pointing in negative direction"
            )
            rot = rot * Rotation.from_euler("z", 180, degrees=True)

        offset = rot.apply(
            [0.0, 0.0, 0.04]
        )  # GPD defines points at the hand palm, not the fingertip
        quat = rot.as_quat()

        pose = Pose()
        pose.position.x = grasp.position.x + offset[0]
        pose.position.y = grasp.position.y + offset[1]
        pose.position.z = grasp.position.z + offset[2]

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "panda_link0"

        return pose_stamped


def main():
    try:
        rospy.init_node("scan_action_node")
        ScanActionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
