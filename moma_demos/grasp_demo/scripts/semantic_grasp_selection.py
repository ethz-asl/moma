#!/usr/bin/env python

from actionlib import SimpleActionServer
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped
from gpd_ros.msg import GraspConfigList
import rospy
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation

from grasp_demo.msg import SemanticSelectGraspAction, SemanticSelectGraspResult
from moma_utils.transform import Transform, Rotation
from moma_utils.ros_conversions import from_point_msg, from_pose_msg, to_pose_msg
from grasp_selection import grasp_config_list_to_pose_array, SelectGraspAction
from vpp_msgs.srv import GetAlignedInstanceBoundingBox


class SemanticGraspSelectionAction(object):
    """
        Sets the GPD workspace based on the object instances location. Queries GPD for
        grasp candidates and let's the user select on from rviz.
    """

    def __init__(self):
        self._as = SimpleActionServer(
            "semantic_grasp_selection_action",
            SemanticSelectGraspAction,
            execute_cb=self._set_workspace_execute_cb,
            auto_start=False,
        )

        self.gpd_cloud_pub = rospy.Publisher(
            "/cloud_stitched", PointCloud2, queue_size=10
        )

        self.detected_grasps_pub = rospy.Publisher(
            "/grasp_candidates", PoseArray, queue_size=10
        )

        self.selected_grasp_pub = rospy.Publisher(
            "/grasp_pose", PoseStamped, queue_size=10
        )

        bbox_service_name = "gsm_node/get_aligned_instance_bbox"
        self.instance_bounding_box_service = rospy.ServiceProxy(
            bbox_service_name, GetAlignedInstanceBoundingBox
        )

        self._as.start()
        rospy.loginfo("Semantic grasp selection action server ready")

    def _set_workspace_execute_cb(self, goal_msg):
        self._set_grasping_workspace(goal_msg.instance_id)
        self._execute_cb(goal_msg)

    def _set_grasping_workspace(self, instance_id):
        response = self.instance_bounding_box_service(instance_id)
        pose = response.bbox.pose
        dimensions = response.bbox.dimensions

        vertices = np.zeros((8, 3))
        x = np.array([1.0, 0.0, 0.0]) * dimensions[0]
        y = np.array([0.0, 1.0, 0.0]) * dimensions[1]
        z = np.array([0.0, 0.0, 1.0]) * dimensions[2]
        base_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        for i in range(0, 2):
            for j in range(0, 2):
                for k in range(0, 2):
                    v_index = i * 4 + j * 2 + k
                    vertices[v_index] += base_position + i * x + j * y + k * z
        rotation = Rotation(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        rotated_vertices = rotation.apply(vertices)

        lows = np.min(rotated_vertices, axis=0)
        highs = np.max(rotated_vertices, axis=0)
        workspace_param = " ".join(["{low} {high}" for low, high in zip(lows, highs)])
        rospy.set_param("detect_grasps/workspace", workspace_param)


if __name__ == "__main__":
    try:
        rospy.init_node("semantic_grasp_selection_action_node")
        action = SemanticGraspSelectionAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
