#!/usr/bin/env python3

from pathlib import Path
import numpy as np
import rospy
from actionlib import SimpleActionServer
from geometry_msgs.msg import *
import tf
import tf2_ros

from grasp_demo.msg import SelectGraspAction, SelectGraspResult
import moma_utils.ros.conversions as conv
from moma_utils.grasping import PlanGrasp
from vpp_msgs.srv import GetMap


class PlanGraspNode(object):
    def __init__(self):
        self.listener = tf.TransformListener()

        # Parameters
        model_path = Path(rospy.get_param("moma_demo/vgn/model"))
        self.base_frame_id = rospy.get_param("moma_demo/base_frame_id")
        task_frame_id = rospy.get_param("moma_demo/task_frame_id")
        self.grasp_selection = rospy.get_param("moma_demo/grasp_selection")
        tf_buffer = tf2_ros.Buffer()
        msg = tf_buffer.lookup_transform(
            self.base_frame_id, task_frame_id, rospy.Time(), rospy.Duration(10)
        )
        self.T_base_task = conv.from_transform_msg(msg.transform)

        # Publishers and subscribers
        self.detected_grasps_pub = rospy.Publisher(
            "grasp_candidates", PoseArray, queue_size=10
        )
        self.get_map_srv = rospy.ServiceProxy("/gsm_node/get_map", GetMap)

        # Grasp planner
        self.grasp_planner = PlanGrasp(model_path, self.base_frame_id, task_frame_id)

        self.action_server = SimpleActionServer(
            "grasp_selection_action",
            SelectGraspAction,
            execute_cb=self.plan_grasp,
            auto_start=False,
        )
        self.action_server.start()
        rospy.loginfo("Grasp selection action server ready")

    def plan_grasp(self, goal_msg):
        map_cloud = self.get_map_srv().map_cloud
        grasps, scores = self.grasp_planner.detect_grasps(map_cloud, self.T_base_task)

        if len(grasps.poses) == 0:
            self.action_server.set_aborted(SelectGraspResult())
            rospy.loginfo("No grasps detected, aborting")
            return
        else:
            rospy.loginfo("{} grasps detected".format(len(grasps.poses)))

        self.visualize_detected_grasps(grasps)
        selected_grasp = self.select_grasp(grasps, scores)
        self.grasp_planner.visualize_selected_grasp(selected_grasp)
        result = SelectGraspResult(target_grasp_pose=selected_grasp)
        self.action_server.set_succeeded(result)

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
        clicked_point = conv.from_point_msg(clicked_point_msg.point)
        translation, _ = self.listener.lookupTransform(
            "base_link", self.base_frame_id, rospy.Time()
        )
        clicked_point += np.asarray(translation)
        distances = []
        for pose in grasp_candidates.poses:
            grasp_point = conv.from_pose_msg(pose).translation
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


def main():
    rospy.init_node("plan_grasps")
    PlanGraspNode()
    rospy.spin()


if __name__ == "__main__":
    main()
