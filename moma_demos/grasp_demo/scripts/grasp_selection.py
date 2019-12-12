#!/usr/bin/env python

import numpy as np
from actionlib import SimpleActionServer
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from gpd_ros.msg import GraspConfigList
import rospy
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation

from grasp_demo.msg import SelectGraspAction, SelectGraspResult


class GraspSelectionAction(object):
    def __init__(self):
        self._as = SimpleActionServer(
            "grasp_selection_action",
            SelectGraspAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )

        # Publishing a cloud to this topic triggers GPD
        self.gpd_cloud_pub = rospy.Publisher(
            "/cloud_stitched", PointCloud2, queue_size=10
        )

        # # For visualizing the detected grasps
        # self.detected_grasps_pub = rospy.Publisher(Stitched
        #     "/grasp_candidates", PoseArray, queue_size=10
        # )

        self.selected_grasp_pub = rospy.Publisher(
            "/grasp_pose", PoseStamped, queue_size=10
        )

        self._as.start()

    def execute_cb(self, goal_msg):
        cloud = goal_msg.pointcloud_scene

        # Trigger GPD and wait for detected grasps
        self.gpd_cloud_pub.publish(cloud)

        try:
            grasp_candidates = rospy.wait_for_message(
                "/detect_grasps/clustered_grasps", GraspConfigList, timeout=30
            )
        except rospy.ROSException:
            rospy.loginfo("GPD server timed out")
            self._as.set_aborted(SelectGraspResult())
            return

        if len(grasp_candidates.grasps) == 0:
            rospy.loginfo("No grasps detected")
            self._as.set_aborted(result)
            return

        # Wait for the user to select a grasp
        grasp_pose = self.select_grasp_pose(grasp_candidates)

        # Visualize selected grasp
        self.selected_grasp_pub.publish(grasp_pose)

        result = SelectGraspResult()
        result.target_grasp_pose = grasp_pose
        self._as.set_succeeded(result)

        rospy.loginfo("Grasp selection action succeeded")

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


if __name__ == "__main__":
    try:
        rospy.init_node("grasp_selection_action_node")
        GraspSelectionAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
