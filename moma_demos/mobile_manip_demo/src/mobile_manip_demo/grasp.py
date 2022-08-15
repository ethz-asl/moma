#!/usr/bin/env python3

from pathlib import Path

from actionlib import SimpleActionServer, SimpleActionClient
import actionlib_msgs.msg
import geometry_msgs.msg
import grasp_demo.msg
from mobile_manip_demo import environment as env
import mobile_manip_demo.msg
from mobile_manip_demo.srv import (
    ForceDrop,
    ForceDropRequest,
    ForceGrasp,
    ForceGraspRequest,
)
from moma_utils.grasping import PlanGrasp
import moma_utils.ros.conversions as conv
import numpy as np
import rospy
import sensor_msgs.msg
import std_srvs.srv
import tf2_ros


class GraspSkill:
    def __init__(self):
        # Parameters
        model_path = Path(rospy.get_param("/vgn/model"))
        self.base_frame_id = rospy.get_param("moma_demo/base_frame_id")
        self.task_frame_id = rospy.get_param("moma_demo/task_frame_id")
        self.grasp_selection = rospy.get_param("moma_demo/grasp_selection")
        self.object_type = rospy.get_param("moma_demo/object_type")

        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)

        msg = self.__compute_tf(self.task_frame_id, self.base_frame_id)
        if msg is None:
            raise ValueError("Could not get the transformation")
        rospy.loginfo(f"Got the transform bTt\n {msg.transform}")
        self.T_base_task = conv.from_transform_msg(msg.transform)

        # VGN interface
        rospy.loginfo("Initializing grasp planner")
        self.grasp_planner = PlanGrasp(
            model_path, self.base_frame_id, self.task_frame_id
        )

        # Publishers and subscribers
        self.detected_grasps_pub = rospy.Publisher(
            "grasp_candidates", geometry_msgs.msg.PoseArray, queue_size=10
        )
        self.pointcloud_pub = rospy.Publisher(
            "stitched_pointcloud", sensor_msgs.msg.PointCloud2, queue_size=10
        )

        # Service clients
        rospy.wait_for_service("reset")
        self.srv_reset = rospy.ServiceProxy("reset", std_srvs.srv.Trigger)

        self.attach_srv = rospy.ServiceProxy("force_grasp", ForceGrasp)
        self.attach_srv.wait_for_service()

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
        rospy.loginfo("Initializing grasping action server")
        self.action_server.start()

    def execute_callback(self, goal):
        # Reset arm
        resp = self.srv_reset()
        if not resp.success:
            self.report_failure("Calling the reset service failed")
            return
        rospy.loginfo("Scene resetted")

        # Reconstruct scene
        self.client_reconstruct.send_goal(grasp_demo.msg.ScanSceneGoal())
        rospy.loginfo("Reconstructing scene")
        res = self.wait_monitoring_preemption(self.client_reconstruct)
        if not res:
            self.report_preemption("Preempted during scene reconstruction")
            return
        state = self.client_reconstruct.get_state()
        if state != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            self.report_failure("Scene reconstruction failed")
            return
        map_cloud = self.client_reconstruct.get_result().pointcloud_scene
        rospy.loginfo("Scene reconstructed")

        self.pointcloud_pub.publish(map_cloud)

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
        if int(goal.goal_id) > -1:
            # get the pose from the ID
            msg = self.__compute_tf("tag_" + str(goal.goal_id), self.task_frame_id)
            if msg is None:
                self.report_failure("Transformation Error")
                return
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = msg.transform.translation.x
            target_pose.position.y = msg.transform.translation.y
            target_pose.position.z = msg.transform.translation.z
            target_pose.orientation = msg.transform.rotation
        else:
            target_pose = conv.from_pose_msg(goal.target_object_pose)
        distances = [
            np.linalg.norm(grasps_list[i].translation - target_pose.translation)
            for i in range(num_grasps)
        ]
        selected_grasp_idx = np.argmin(distances)
        target_pose_msg = conv.to_pose_stamped_msg(
            grasps_list[selected_grasp_idx], self.base_frame_id
        )
        rospy.loginfo(f"Sending target for grasping:\n {target_pose_msg}")

        # Execute grasp
        grasp_goal = grasp_demo.msg.GraspGoal(target_grasp_pose=target_pose_msg)
        self.client_grasp_execution.send_goal(grasp_goal)
        res = self.wait_monitoring_preemption(self.client_grasp_execution)
        if not res:
            self.report_preemption("Preempted during grasp execution")
            return
        state = self.client_reconstruct.get_state()
        if state != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            self.report_failure("Grasp execution failed")
            return

        # Otherwise everything is fine and we can force the grasping
        name, link = env.get_item_by_marker(int(goal.goal_id), self.object_type)
        self.grasp_request = ForceGraspRequest()
        self.grasp_request.model_name = name
        self.grasp_request.ee_name = "panda"
        self.grasp_request.model_link = link
        self.grasp_request.ee_link = "panda::panda_leftfinger"
        response = self.attach_srv.call(self.grasp_request)

        if response.success:
            self.report_success("Finished successfully")
        else:
            self.report_failure("Grasp execution failed")

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

    def __compute_tf(self, target_frame: str, reference_frame: str):
        done = False
        attempts = 0
        while not done or attempts > 15:
            try:
                msg = self.tf_buffer.lookup_transform(
                    reference_frame,
                    target_frame,
                    rospy.Time(),
                    rospy.Duration(5),
                )
                done = True
            except Exception:
                attempts += 1
                rospy.logerr("Could not get transform, retrying...")

        if done:
            return msg
        else:
            return None
