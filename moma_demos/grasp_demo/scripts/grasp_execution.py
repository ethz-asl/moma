#!/usr/bin/env python
import sys

from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionServer
from moveit_commander.conversions import pose_to_list
import numpy as np
import rospy
from scipy.spatial.transform import Rotation

from grasp_demo.msg import GraspAction, GraspResult
from grasp_demo.utils import create_robot_connection

from moma_utils.transform import Transform


class GraspExecutionAction(object):
    """
        Execute a grasp specified by the action goal using MoveIt.
    """

    def __init__(self):
        self.robot_name = sys.argv[1]
        self._load_parameters()
        self._connect_robot()

        self._as = SimpleActionServer(
            "grasp_execution_action",
            GraspAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )

        self.pregrasp_pub = rospy.Publisher(
            "/pregrasp_pose", PoseStamped, queue_size=10
        )

        self._as.start()
        rospy.loginfo("Grasp action server ready")

    def _connect_robot(self):
        full_robot_name = (
            self.robot_name + "_" + self._robot_arm_names[0]
            if len(self._robot_arm_names) > 1
            else self.robot_name
        )
        self._robot_arm = create_robot_connection(full_robot_name)

    def _load_parameters(self):
        self._robot_arm_names = rospy.get_param("/moma_demo/robot_arm_names")
        self._ready_joints = rospy.get_param(
            "/moma_demo/ready_joints_" + self._robot_arm_names[0]
        )
        self._base_frame_id = rospy.get_param("/moma_demo/base_frame_id")
        self._arm_velocity_scaling = rospy.get_param(
            "/moma_demo/arm_velocity_scaling_grasp"
        )
        self._pregrasp_offset_z = rospy.get_param("/moma_demo/pregrasp_offset_z")
        self._grasp_offset_z = rospy.get_param("/moma_demo/grasp_offset_z")

    def execute_cb(self, goal_msg):
        rospy.loginfo("Received grasp pose")

        grasp_pose_msg = goal_msg.target_grasp_pose.pose
        grasp_pose_msg_list = pose_to_list(grasp_pose_msg)
        T_base_grasp = Transform(
            Rotation.from_quat(grasp_pose_msg_list[3:]), grasp_pose_msg_list[:3]
        )
        T_grasp_offset = Transform(
            Rotation.from_quat([0.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, self._grasp_offset_z]),
        )
        T_base_grasp = T_base_grasp * T_grasp_offset

        T_grasp_pregrasp = Transform(
            Rotation.from_quat([0.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, self._pregrasp_offset_z]),
        )
        T_base_pregrasp = T_base_grasp * T_grasp_pregrasp

        msg = T_base_pregrasp.to_pose_stamped(self._base_frame_id)
        self.pregrasp_pub.publish(msg)

        rospy.loginfo("Opening hand")
        self._robot_arm.release()

        rospy.loginfo("Moving to pregrasp pose")
        self._robot_arm.goto_pose_target(
            T_base_pregrasp.to_list(), max_velocity_scaling=self._arm_velocity_scaling
        )

        rospy.loginfo("Moving to grasp pose")
        self._robot_arm.goto_pose_target(
            T_base_grasp.to_list(), max_velocity_scaling=self._arm_velocity_scaling
        )

        rospy.loginfo("Grasping")
        self._robot_arm.grasp()

        rospy.loginfo("Retrieving object")
        self._robot_arm.goto_pose_target(
            T_base_pregrasp.to_list(), max_velocity_scaling=self._arm_velocity_scaling
        )

        self._robot_arm.goto_joint_target(
            self._ready_joints, max_velocity_scaling=self._arm_velocity_scaling
        )

        grasped_something = self._robot_arm.check_object_grasped()

        if not grasped_something:
            self._as.set_aborted()
        else:
            self._as.set_succeeded(GraspResult())


if __name__ == "__main__":
    try:
        rospy.init_node("grasp_execution_node")
        GraspExecutionAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
