#!/usr/bin/env python3

from pathlib import Path
from actionlib import SimpleActionServer
import numpy as np
from geometry_msgs.msg import *
import rospy
import tf
import tf2_ros
import ros_numpy

from grasp_demo.msg import SelectGraspAction, SelectGraspResult
from moma_utils.ros.conversions import *
from vgn.grasp import ParallelJawGrasp
from vgn.detection import VGN, select_local_maxima
from vgn.rviz import Visualizer
from vgn.utils import grid_to_map_cloud
from vpp_msgs.srv import GetMap


class PlanGraspNode(object):
    def __init__(self):
        self.listener = tf.TransformListener()
        self.read_parameters()
        self.init_visualization()
        self.init_vgn()

        self.action_server = SimpleActionServer(
            "grasp_selection_action",
            SelectGraspAction,
            execute_cb=self.plan_grasp,
            auto_start=False,
        )
        self.action_server.start()
        rospy.loginfo("Grasp selection action server ready")

    def read_parameters(self):
        self.base_frame_id = rospy.get_param("moma_demo/base_frame_id")
        self.task_frame_id = rospy.get_param("moma_demo/task_frame_id")
        self.grasp_selection = rospy.get_param("moma_demo/grasp_selection")

    def init_visualization(self):
        self.vis = Visualizer()
        self.detected_grasps_pub = rospy.Publisher(
            "grasp_candidates",
            PoseArray,
            queue_size=10,
        )

    def init_vgn(self):
        model_path = Path(rospy.get_param("moma_demo/vgn/model"))
        self.vgn = VGN(model_path)
        self.get_map_srv = rospy.ServiceProxy("/gsm_node/get_map", GetMap)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        msg = self.tf_buffer.lookup_transform(
            "panda_link0", "task", rospy.Time(), rospy.Duration(10.0)
        )
        self.T_base_task = from_transform_msg(msg.transform)

    def plan_grasp(self, goal_msg):
        grasps, scores = self.detect_grasps()

        if len(grasps.poses) == 0:
            self.action_server.set_aborted(SelectGraspResult())
            rospy.loginfo("No grasps detected, aborting")
            return
        else:
            rospy.loginfo("{} grasps detected".format(len(grasps.poses)))

        self.visualize_detected_grasps(grasps)
        selected_grasp = self.select_grasp(grasps, scores)
        self.visualize_selected_grasp(selected_grasp)
        result = SelectGraspResult(target_grasp_pose=selected_grasp)
        self.action_server.set_succeeded(result)

    def detect_grasps(self):
        voxel_size = 0.0075
        map_cloud = self.get_map_srv().map_cloud
        data = ros_numpy.numpify(map_cloud)
        x, y, z = data["x"], data["y"], data["z"]
        points = np.column_stack((x, y, z)) - self.T_base_task.translation
        d = (data["distance"] + 0.03) / 0.06  # scale to [0, 1]
        tsdf_grid = np.zeros((40, 40, 40), dtype=np.float32)
        for idx, point in enumerate(points):
            if np.all(point > 0.0) and np.all(point < 0.3):
                i, j, k = np.floor(point / voxel_size).astype(int)
                tsdf_grid[i, j, k] = d[idx]

        points, distances = grid_to_map_cloud(voxel_size, tsdf_grid)
        self.vis.map_cloud(self.task_frame_id, points, distances)
        rospy.loginfo("Received map cloud")

        out = self.vgn.predict(tsdf_grid)
        self.vis.quality(self.task_frame_id, voxel_size, out.qual)

        grasps, scores = select_local_maxima(voxel_size, out, threshold=0.9)
        rospy.loginfo("Detected grasps")

        grasp_candidates = PoseArray()
        grasp_candidates.header.stamp = rospy.Time.now()
        for grasp in grasps:
            pose_msg = to_pose_msg(self.T_base_task * grasp.pose)
            pose_msg.position.z -= 0.025  # TODO(mbreyer) Investigate this
            grasp_candidates.poses.append(pose_msg)
        grasp_candidates.header.frame_id = self.base_frame_id

        return grasp_candidates, scores

    def visualize_detected_grasps(self, grasp_candidates):
        self.detected_grasps_pub.publish(grasp_candidates)

    def select_grasp(self, grasps, scores):
        if self.grasp_selection == "manual":
            selected_grasp = self.wait_for_user_selection(grasps)
        elif self.grasp_selection == "auto":
            selected_grasp = self.select_best_grasp(grasps, scores)
        else:
            raise NotImplementedError(
                "Grasp selection method {} invalid".format(self.grasp_selection)
            )
        rospy.loginfo("Grasp selected")
        return selected_grasp

    def wait_for_user_selection(self, grasp_candidates):
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

    def select_best_grasp(self, grasps, scores):
        index = np.argmax([p.position.z for p in grasps.poses])
        selected_grasp_msg = PoseStamped()
        selected_grasp_msg.header.stamp = rospy.Time.now()
        selected_grasp_msg.header.frame_id = self.base_frame_id
        selected_grasp_msg.pose = grasps.poses[index]
        return selected_grasp_msg

    def visualize_selected_grasp(self, selected_grasp):
        grasp = ParallelJawGrasp(from_pose_msg(selected_grasp.pose), 0.04)
        self.vis.grasp(self.base_frame_id, grasp, 1.0)


def main():
    rospy.init_node("plan_grasps")
    PlanGraspNode()
    rospy.spin()


if __name__ == "__main__":
    main()
