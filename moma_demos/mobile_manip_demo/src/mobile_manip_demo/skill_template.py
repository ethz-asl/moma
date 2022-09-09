#!/usr/bin/env python3
"""Base implementation of a robot skill."""

from typing import Any
import rospy
import tf2_ros


class Skill:
    """Skill template."""

    def __init__(self):

        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)

        self.action_server = None

    def report_failure(self, result_obj: Any, msg: str):
        rospy.logerr(msg)
        result_obj.success = False
        result_obj.message = msg
        self.action_server.set_aborted(result_obj)

    def report_preemption(self, result_obj: Any, msg: str):
        rospy.logwarn(msg)
        result_obj.success = False
        result_obj.message = msg
        self.action_server.set_preempted(result_obj)

    def report_success(self, result_obj: Any, msg: str):
        rospy.loginfo(msg)
        result_obj.success = True
        result_obj.message = msg
        self.action_server.set_succeeded(result_obj)

    def wait_monitoring_preemption(self, client):
        while not client.wait_for_result(timeout=rospy.Duration(1)):
            if self.action_server.is_preempt_requested():
                client.cancel_goal()
                return False
        return True

    def compute_tf(self, target_frame: str, reference_frame: str):
        done = False
        attempts = 0
        while not done and attempts < 20:
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
                rospy.Rate(1).sleep()

        if done:
            return msg
        else:
            return None
