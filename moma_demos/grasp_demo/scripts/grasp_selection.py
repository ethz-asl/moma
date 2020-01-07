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
    """
        Send point cloud to GPD, then let the user select one (using rviz) and return the
        selected grasp pose.
    """

    def __init__(self):
        self._as = SimpleActionServer(
            "grasp_selection_action",
            SelectGraspAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )

        self.gpd_cloud_pub = rospy.Publisher(
            "/cloud_stitched", PointCloud2, queue_size=10
        )  # publishing a cloud to this topic triggers GPD

        self.detected_grasps_pub = rospy.Publisher(
            "/grasp_candidates", PoseArray, queue_size=10
        )

        self.selected_grasp_pub = rospy.Publisher(
            "/grasp_pose", PoseStamped, queue_size=10
        )

        self._as.start()
        rospy.loginfo("Grasp selection action server ready")

    def execute_cb(self, goal_msg):
        grasp_candidates = self.detect_grasps(goal_msg.pointcloud_scene)

        if not grasp_candidates:
            self._as.set_aborted(SelectGraspResult())
            rospy.loginfo("No grasps detected, aborting")
            return
        else:
            rospy.loginfo("{} grasps detected".format(len(grasp_candidates)))

        self.visualize_detected_grasps(grasp_candidates)

        selected_grasp = self.wait_for_user_selection(grasp_candidates)
        rospy.loginfo("Grasp selected")

        self.visualize_selected_grasp(selected_grasp)

        result = SelectGraspResult(target_grasp_pose=selected_grasp)
        self._as.set_succeeded(result)

    def detect_grasps(self, cloud):
        self.gpd_cloud_pub.publish(cloud)

        try:
            grasp_config_list = rospy.wait_for_message(
                "/detect_grasps/clustered_grasps", GraspConfigList, timeout=30
            )
        except rospy.ROSException:
            return []

        return grasp_config_list_to_pose_array(grasp_config_list)

    def visualize_detected_grasps(self, grasp_candidates):
        self.detected_grasps_pub.publish(grasp_candidates)

    def wait_for_user_selection(self, grasp_candidates):
        clicked_point_msg = rospy.wait_for_message("/clicked_point", PointStamped)
        clicked_point = from_point_msg(clicked_point_msg.point)

        distances = []
        for pose in grasp_candidates.poses:
            grasp_point = from_pose_msg(pose).translation
            distances.append(np.linalg.norm(clicked_point - grasp_point))

        selected_grasp_msg = PoseStamped()
        selected_grasp_msg.header.stamp = rospy.Time.now()
        selected_grasp_msg.header.frame_id = "panda_link0"
        selected_grasp_msg.pose = grasp_candidates.poses[np.argmin(distances)]

        return selected_grasp_msg

    def visualize_selected_grasp(self, selected_grasp):
        self.selected_grasp_pub.publish(selected_grasp)


if __name__ == "__main__":
    try:
        rospy.init_node("grasp_selection_action_node")
        GraspSelectionAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
