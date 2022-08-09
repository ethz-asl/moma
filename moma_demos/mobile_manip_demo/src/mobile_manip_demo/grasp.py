#!/usr/bin/env python3

from pathlib import Path

import numpy as np

import rospy
import tf2_ros
from actionlib import SimpleActionServer, SimpleActionClient
import actionlib_msgs.msg
import std_srvs.srv
import mobile_manip_demo.msg
import grasp_demo.msg
import geometry_msgs.msg
from moma_utils.grasping import PlanGrasp
import moma_utils.ros.conversions as conv


class GraspSkill:
    def __init__(self):
        # Parameters
        model_path = Path(rospy.get_param("moma_demo/vgn/model"))
        self.base_frame_id = rospy.get_param("moma_demo/base_frame_id")
        self.task_frame_id = rospy.get_param("moma_demo/task_frame_id")
        self.grasp_selection = rospy.get_param("moma_demo/grasp_selection")
        tf_buffer = tf2_ros.Buffer()
        msg = tf_buffer.lookup_transform(
            self.base_frame_id, self.task_frame_id, rospy.Time(), rospy.Duration(10)
        )
        self.T_base_task = conv.from_transform_msg(msg.transform)

        # VGN interface
        self.grasp_planner = PlanGrasp(
            model_path, self.base_frame_id, self.task_frame_id
        )

        # Publishers and subscribers
        self.detected_grasps_pub = rospy.Publisher(
            "grasp_candidates", geometry_msgs.msg.PoseArray, queue_size=10
        )

        # Service clients
        rospy.wait_for_service("reset")
        self.srv_reset = rospy.ServiceProxy("reset", std_srvs.srv.Trigger)

        # Action clients
        self.client_reconstruct = SimpleActionClient(
            "scan_action", grasp_demo.msg.ScanSceneAction
        )
        self.client_reconstruct.wait_for_server()
        self.client_grasp_execution = SimpleActionClient(
            "grasp_execution_action", grasp_demo.msg.GraspAction
        )
        self.client_grasp_execution.wait_for_server()

        # Action server
        self.action_server = SimpleActionServer(
            "hl_grasp",
            mobile_manip_demo.msg.GraspAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self.action_server.start()

    def execute_callback(self, goal):
        # Reset arm
        resp = self.srv_reset()
        if not resp.success:
            self.report_failure("Calling the reset service failed")
            return

        # Reconstruct scene
        self.client_reconstruct.send_goal(grasp_demo.msg.ScanSceneGoal())
        res = self.wait_monitoring_preemption(self.client_reconstruct)
        if not res:
            self.report_preemption("Preempted during scene reconstruction")
            return
        state = self.client_reconstruct.get_state()
        if state != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            self.report_failure("Scene reconstruction failed")
            return
        map_cloud = self.client_reconstruct.get_result().pointcloud_scene

        # Plan grasp
        grasps, scores = self.grasp_planner.detect_grasps(map_cloud, self.T_base_task)
        num_grasps = len(grasps.poses)
        if num_grasps == 0:
            self.report_failure("No grasps detected")
            return
        rospy.loginfo(f"{num_grasps} grasps detected")
        self.visualize_detected_grasps(grasps)
        grasps_list = [conv.from_pose_msg(grasps.poses[i]) for i in range(num_grasps)]

        # Select grasp
        target_pose = conv.from_pose_msg(goal.target_object_pose)
        distances = [
            np.linalg.norm(grasps_list[i].translation - target_pose.translation)
            for i in range(num_grasps)
        ]
        selected_grasp_idx = np.argmin(distances)
        target_pose_msg = conv.to_pose_stamped_msg(
            grasps_list[selected_grasp_idx], self.base_frame_id
        )

        # Execute grasp
        grasp_goal = grasp_demo.msg.GraspActionGoal(target_grasp_pose=target_pose_msg)
        self.client_grasp_execution.send_goal(grasp_goal)
        res = self.wait_monitoring_preemption(self.client_grasp_execution)
        if not res:
            self.report_preemption("Preempted during grasp execution")
            return
        state = self.client_reconstruct.get_state()
        if state != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            self.report_failure("Grasp execution failed")
            return

        self.report_success("Finished successfully")

    def report_failure(self, msg):
        result = mobile_manip_demo.msg.GraspResult()
        rospy.logerr(msg)
        result.success = False
        result.message = msg
        self.action_server.set_aborted(result)

    def report_preemption(self, msg):
        result = mobile_manip_demo.msg.GraspResult()
        rospy.logwarn(msg)
        result.success = False
        result.message = msg
        self.action_server.set_preempted(result)

    def report_success(self, msg: str):
        result = mobile_manip_demo.msg.GraspResult()
        rospy.loginfo(msg)
        result.success = True
        result.message = msg
        self.action_server.set_succeeded(result)

    def wait_monitoring_preemption(self, client):
        while not client.wait_for_result(timeout=rospy.Duration(1)):
            if self.action_server.is_preempt_requested():
                client.cancel_goal()
                return False
        return True

    def visualize_detected_grasps(self, grasp_candidates):
        self.detected_grasps_pub.publish(grasp_candidates)
