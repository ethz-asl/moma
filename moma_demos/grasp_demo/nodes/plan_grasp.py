#!/usr/bin/env python

import sys

from actionlib import SimpleActionServer
import numpy as np
from geometry_msgs.msg import *
from gpd_ros.msg import GraspConfigList
import rospy
from sensor_msgs.msg import PointCloud2
import tf
import tf2_ros
import ros_numpy

from grasp_demo.msg import SelectGraspAction, SelectGraspResult
from moma_utils.spatial import Transform, Rotation
from moma_utils.ros.conversions import *
from vgn import vis
from vgn.detection import *
from vgn.networks import *
from vpp_msgs.srv import GetMap


class PlanGraspNode(object):
    def __init__(self, method):
        self._read_parameters()
        self._init_visualization()
        self.listener = tf.TransformListener()

        if method == "gpd":
            self._init_gpd()
        elif method == "vgn":
            self._init_vgn()
        else:
            raise ValueError("Invalid grasp detection method")

        self.action_server = SimpleActionServer(
            "grasp_selection_action",
            SelectGraspAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.action_server.start()
        rospy.loginfo("Grasp selection action server ready")

    def execute_cb(self, goal_msg):
        grasps, scores = self.detect_grasps(goal_msg.pointcloud_scene)

        if not grasps or len(grasps.poses) == 0:
            self.action_server.set_aborted(SelectGraspResult())
            rospy.loginfo("No grasps detected, aborting")
            return
        else:
            rospy.loginfo("{} grasps detected".format(len(grasps.poses)))

        self._visualize_detected_grasps(grasps)
        selected_grasp = self._select_grasp(grasps, scores)
        self._visualize_selected_grasp(selected_grasp)
        result = SelectGraspResult(target_grasp_pose=selected_grasp)
        self.action_server.set_succeeded(result)

    def _read_parameters(self):
        self.base_frame_id = rospy.get_param("moma_demo/base_frame_id")
        self.grasp_selection_method = rospy.get_param(
            "moma_demo/grasp_selection_method"
        )

    def _init_gpd(self):
        self.gpd_cloud_pub = rospy.Publisher(
            "detect_grasps/cloud_stitched", PointCloud2, queue_size=10
        )  # publishing a cloud to this topic triggers GPD
        self.detect_grasps = self._detect_grasps_with_gpd

    def _detect_grasps_with_gpd(self, cloud):
        self.gpd_cloud_pub.publish(cloud)
        try:
            grasp_config_list = rospy.wait_for_message(
                "detect_grasps/clustered_grasps", GraspConfigList, timeout=120
            )
        except rospy.ROSException:
            return []
        grasps, scores = grasp_config_list_to_pose_array(grasp_config_list)
        grasps.header.frame_id = self.base_frame_id
        return grasps, scores

    def _init_vgn(self):
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.sleep(0.5)
        self.get_map_srv = rospy.ServiceProxy("/gsm_node/get_map", GetMap)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.net = ConvNet()
        model_path = "/home/franka/catkin_ws/src/vgn_private/data/models/vgn_conv.pth"
        self.net.load_state_dict(torch.load(model_path, map_location=self.device))
        self.detect_grasps = self._detect_grasps_with_vgn

        # Define task space
        self.T_panda_link0_task = Transform(
            Rotation.identity(), np.r_[0.27, -0.13, 0.15]
        )
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "panda_link0"
        msg.child_frame_id = "task"
        msg.transform = to_transform_msg(self.T_panda_link0_task)
        self.static_broadcaster.sendTransform(msg)

    def _detect_grasps_with_vgn(self, cloud):
        voxel_size = 0.0075
        map_cloud = self.get_map_srv().map_cloud
        rospy.loginfo("Received map cloud")

        vis.clear()
        vis.draw_workspace(0.3)

        # Build input from VPP map
        data = ros_numpy.numpify(map_cloud)
        x, y, z = data["x"], data["y"], data["z"]
        points = np.column_stack((x, y, z)) - self.T_panda_link0_task.translation
        d = (data["distance"] + 0.03) / 2.0 / (0.03)  # scale to [0, 1]
        tsdf_vol = np.zeros((1, 40, 40, 40), dtype=np.float32)
        for idx, point in enumerate(points):
            if np.all(point > 0.0) and np.all(point < 0.3):
                i, j, k = np.floor(point / voxel_size).astype(int)
                tsdf_vol[0, i, j, k] = d[idx]
        vis.draw_tsdf(tsdf_vol, voxel_size)
        rospy.loginfo("Constructed VGN input")

        # Detect grasps
        qual_vol, rot_vol, width_vol = predict(tsdf_vol, self.net, self.device)
        qual_vol, rot_vol, width_vol = process(tsdf_vol, qual_vol, rot_vol, width_vol)
        grasps, scores = select(qual_vol, rot_vol, width_vol, 0.90, 1)
        num_grasps = len(grasps)
        if num_grasps > 0:
            idx = np.random.choice(num_grasps, size=min(5, num_grasps), replace=False)
            grasps, scores = np.array(grasps)[idx], np.array(scores)[idx]
        grasps = [from_voxel_coordinates(g, voxel_size) for g in grasps]
        vis.draw_quality(qual_vol, voxel_size, threshold=0.01)
        rospy.loginfo("Detected grasps")

        # Construct output
        grasp_candidates = PoseArray()
        grasp_candidates.header.stamp = rospy.Time.now()

        for grasp in grasps:
            pose_msg = to_pose_msg(self.T_panda_link0_task * grasp.pose)
            grasp_candidates.poses.append(pose_msg)
        grasp_candidates.header.frame_id = self.base_frame_id

        return grasp_candidates, scores

    def _init_visualization(self):
        self.detected_grasps_pub = rospy.Publisher(
            "grasp_candidates", PoseArray, queue_size=10
        )
        self.selected_grasp_pub = rospy.Publisher(
            "grasp_pose", PoseStamped, queue_size=10
        )

    def _select_grasp(self, grasps, scores):
        if self.grasp_selection_method == "manual":
            selected_grasp = self._wait_for_user_selection(grasps)
        elif self.grasp_selection_method == "auto":
            selected_grasp = self._select_best_grasp(grasps, scores)
        else:
            raise NotImplementedError(
                "Grasp selection method {} invalid".format(self.grasp_selection_method)
            )
        rospy.loginfo("Grasp selected")
        return selected_grasp

    def _visualize_detected_grasps(self, grasp_candidates):
        self.detected_grasps_pub.publish(grasp_candidates)

    def _wait_for_user_selection(self, grasp_candidates):
        clicked_point_msg = rospy.wait_for_message("/clicked_point", PointStamped)
        clicked_point = from_point_msg(clicked_point_msg.point)
        translation, _ = self.listener.lookupTransform(
            "base_link", self.base_frame_id, rospy.Time()
        )
        clicked_point += np.asarray(translation)
        distances = []
        for pose in grasp_candidates.poses:
            grasp_point = from_pose_msg(pose).translation
            distances.append(np.linalg.norm(clicked_point - grasp_point))

        selected_grasp_msg = PoseStamped()
        selected_grasp_msg.header.stamp = rospy.Time.now()
        selected_grasp_msg.header.frame_id = self.base_frame_id
        selected_grasp_msg.pose = grasp_candidates.poses[np.argmin(distances)]
        return selected_grasp_msg

    def _select_best_grasp(self, grasps, scores):
        index = np.argmax([p.position.z for p in grasps.poses])
        selected_grasp_msg = PoseStamped()
        selected_grasp_msg.header.stamp = rospy.Time.now()
        selected_grasp_msg.header.frame_id = self.base_frame_id
        selected_grasp_msg.pose = grasps.poses[index]
        return selected_grasp_msg

    def _visualize_selected_grasp(self, selected_grasp):
        self.selected_grasp_pub.publish(selected_grasp)


def grasp_config_list_to_pose_array(grasp_config_list):
    pose_array_msg = PoseArray()
    pose_array_msg.header.stamp = rospy.Time.now()
    scores = []
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
        hand_depth = 0.06
        position = from_point_msg(grasp.position)
        position += rot.apply([0.0, 0.0, hand_depth])
        pose_msg = to_pose_msg(Transform(rot, position))
        pose_array_msg.poses.append(pose_msg)
        scores.append(grasp.score)

    return pose_array_msg, scores


def main():
    rospy.init_node("grasp_selection_action_node")
    method = sys.argv[1]
    PlanGraspNode(method)
    rospy.spin()


if __name__ == "__main__":
    main()
