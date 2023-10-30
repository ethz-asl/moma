#!/usr/bin/env python3

import argparse
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
from vgn.utils import map_cloud_to_grid


class PlanGraspNode(object):
    def __init__(self, arm_id):
        self.arm_id = arm_id

        self.listener = tf.TransformListener()
        self.read_parameters()
        self.init_tf()
        self.init_vgn()
        self.init_visualization()

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

    def init_tf(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def init_vgn(self):
        model_path = Path(rospy.get_param("/vgn/model"))
        self.vgn = VGN(model_path)

    def init_visualization(self):
        self.vis = Visualizer()
        self.grasp_poses_pub = rospy.Publisher("grasp_poses", PoseArray, queue_size=10)

    def plan_grasp(self, goal):
        # Deserialize map cloud message
        voxel_size = goal.voxel_size
        data = ros_numpy.numpify(goal.map_cloud)
        points = np.column_stack((data["x"], data["y"], data["z"]))
        distances = data["distance"]
        tsdf_grid = map_cloud_to_grid(voxel_size, points, distances)

        # Evaluate VGN
        out = self.vgn.predict(tsdf_grid)

        # Visualize grasp quality tensor
        self.vis.quality(self.task_frame_id, voxel_size, out.qual)

        # Filter output
        grasps, scores = select_local_maxima(voxel_size, out, threshold=0.9)

        # Lookup task transform
        msg = self.tf_buffer.lookup_transform(
            f"{self.arm_id}_link0", "task", rospy.Time(), rospy.Duration(10.0)
        )
        T_base_task = from_transform_msg(msg.transform)

        # Serialize grasps
        grasp_poses_msg = PoseArray()
        grasp_poses_msg.header.stamp = rospy.Time.now()
        grasp_poses_msg.header.frame_id = self.base_frame_id
        grasp_poses_msg.poses = [to_pose_msg(T_base_task * g.pose) for g in grasps]

        if len(grasp_poses_msg.poses) == 0:
            self.action_server.set_aborted(SelectGraspResult())
            rospy.loginfo("No grasps detected, aborting")
            # Try detecting grasps in a different workspace
            n = len(rospy.get_param("moma_demo/workspaces"))
            i = rospy.get_param("moma_demo/workspace")
            rospy.set_param("moma_demo/workspace", (i + 1) % n)
            return
        else:
            rospy.loginfo("{} grasps detected".format(len(grasp_poses_msg.poses)))

        self.visualize_grasps(grasp_poses_msg)
        selected_grasp = self.select_grasp(grasp_poses_msg, scores)
        self.visualize_selected_grasp(selected_grasp)
        result = SelectGraspResult(target_grasp_pose=selected_grasp)
        self.action_server.set_succeeded(result)

    def visualize_grasps(self, grasp_candidates):
        self.grasp_poses_pub.publish(grasp_candidates)

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
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm_id", type=str, default="panda")
    args = parser.parse_known_args()
    PlanGraspNode(args.arm_id)
    rospy.spin()


if __name__ == "__main__":
    main()
