#!/usr/bin/env python3

import numpy as np
import rospy
import geometry_msgs.msg
import ros_numpy

import moma_utils.ros.conversions as conv
from vgn.grasp import ParallelJawGrasp
from vgn.detection import VGN, select_local_maxima
from vgn.rviz import Visualizer
from vgn.utils import grid_to_map_cloud


class PlanGrasp(object):
    def __init__(self, model_path, base_frame_id, task_frame_id):
        # Parameters
        self.base_frame_id = base_frame_id
        self.task_frame_id = task_frame_id

        # Init VGN
        self.vgn = VGN(model_path)
        self.vis = Visualizer()

    def detect_grasps(self, map_cloud, T_base_task):
        voxel_size = 0.0075
        data = ros_numpy.numpify(map_cloud)
        x, y, z = data["x"], data["y"], data["z"]
        points = np.column_stack((x, y, z))
        # TODO: pass semantic argument
        # d = (data["distance"] + 0.03) / 0.06  # scale to [0, 1]
        d = data["distance"]
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
        self.vis.grasps("task", grasps, scores)
        rospy.loginfo(f"Detected {len(grasps)} grasps")

        grasp_candidates = geometry_msgs.msg.PoseArray()
        grasp_candidates.header.stamp = rospy.Time.now()
        for grasp in grasps:
            pose_msg = conv.to_pose_msg(T_base_task * grasp.pose)
            # pose_msg.position.z -= 0.025  # TODO(mbreyer) Investigate this
            grasp_candidates.poses.append(pose_msg)
        grasp_candidates.header.frame_id = self.base_frame_id

        return grasp_candidates, scores

    def visualize_selected_grasp(self, selected_grasp):
        grasp = ParallelJawGrasp(conv.from_pose_msg(selected_grasp.pose), 0.04)
        self.vis.grasp(self.base_frame_id, grasp, 1.0)
