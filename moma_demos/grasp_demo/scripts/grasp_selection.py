#!/usr/bin/env python

from actionlib import SimpleActionServer
import numpy as np
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped
from gpd_ros.msg import GraspConfigList
import rospy
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation

from grasp_demo.msg import SelectGraspAction, SelectGraspResult
from moma_utils.transform import Transform, Rotation
from moma_utils.ros_conversions import from_point_msg, from_pose_msg, to_pose_msg


def grasp_config_list_to_pose_array(grasp_config_list):
    pose_array_msg = PoseArray()
    pose_array_msg.header.stamp = rospy.Time.now()
    pose_array_msg.header.frame_id = "panda_link0"

    for grasp in grasp_config_list.grasps:

        x_axis = np.r_[grasp.axis.x, grasp.axis.y, grasp.axis.z]
        y_axis = np.r_[grasp.binormal.x, grasp.binormal.y, grasp.binormal.z]
        z_axis = np.r_[grasp.approach.x, grasp.approach.y, grasp.approach.z]

        rot_mat = np.vstack([x_axis, y_axis, z_axis]).T

        if np.linalg.det(rot_mat) < 0:
            # Not a right-handed system, flipping y-axis
            y_axis *= -1
            rot_mat = np.vstack([x_axis, y_axis, z_axis]).T

        rot = Rotation.from_dcm(rot_mat)

        if x_axis[0] < 0:
            # X-axis pointing in negative direction, rotating around Z axis
            rot = rot * Rotation.from_euler("z", 180, degrees=True)

        # GPD defines points at the hand palm, not the fingertip
        position = from_point_msg(grasp.position)
        position += rot.apply([0.0, 0.0, 0.03])

        pose_msg = to_pose_msg(Transform(rot, position))
        pose_array_msg.poses.append(pose_msg)

    return pose_array_msg


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

        # For visualizing the detected grasps
        self.detected_grasps_pub = rospy.Publisher(
            "/grasp_candidates", PoseArray, queue_size=10
        )

        # For visualizing the selected grasp
        self.selected_grasp_pub = rospy.Publisher(
            "/grasp_pose", PoseStamped, queue_size=10
        )

        self._as.start()
        rospy.loginfo("Grasp selection action server ready")

    def execute_cb(self, goal_msg):
        cloud = goal_msg.pointcloud_scene

        # Trigger GPD and wait for detected grasps
        rospy.loginfo("Sending point cloud to GPD")
        self.gpd_cloud_pub.publish(cloud)

        try:
            grasp_config_list = rospy.wait_for_message(
                "/detect_grasps/clustered_grasps", GraspConfigList, timeout=30
            )
        except rospy.ROSException:
            rospy.loginfo("GPD server timed out")
            self._as.set_aborted(SelectGraspResult())
            return

        if len(grasp_config_list.grasps) == 0:
            rospy.loginfo("No grasps detected")
            self._as.set_aborted(SelectGraspResult())
            return

        grasp_candidates = grasp_config_list_to_pose_array(grasp_config_list)

        # Visualize detected grasps
        self.detected_grasps_pub.publish(grasp_candidates)
        rospy.loginfo("Received grasp candidates")

        # Wait for the user to select a grasp
        clicked_point_msg = rospy.wait_for_message("/clicked_point", PointStamped)
        clicked_point = from_point_msg(clicked_point_msg.point)
        rospy.loginfo("User clicked")

        # Select the closest grasp candidate
        distances = []
        for pose in grasp_candidates.poses:
            grasp_point = from_pose_msg(pose).translation
            distances.append(np.linalg.norm(clicked_point - grasp_point))

        selected_grasp_msg = PoseStamped()
        selected_grasp_msg.header.stamp = rospy.Time.now()
        selected_grasp_msg.header.frame_id = "panda_link0"
        selected_grasp_msg.pose = grasp_candidates.poses[np.argmin(distances)]

        # Visualize selected grasp
        self.selected_grasp_pub.publish(selected_grasp_msg)
        rospy.loginfo("Grasp selected")

        result = SelectGraspResult(target_grasp_pose=selected_grasp_msg)
        self._as.set_succeeded(result)

        rospy.loginfo("Grasp selection action succeeded")


if __name__ == "__main__":
    try:
        rospy.init_node("grasp_selection_action_node")
        GraspSelectionAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
