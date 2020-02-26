#!/usr/bin/env python
import sys

from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionServer
from moveit_commander.conversions import pose_to_list
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

from grasp_demo.msg import GraspAction, GraspResult
from grasp_demo.utils import create_robot_connection

from moma_utils.transform import Transform


class GraspExecutionAction(object):
    """
        Execute a grasp specified by the action goal using MoveIt.
    """

    def __init__(self):
        self.robot_commander = create_robot_connection(sys.argv[1])

        self._read_joint_configurations()

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

    def _read_joint_configurations(self):
        self._ready_joints_l = rospy.get_param("ready_joints_l")
        self._base_frame_id = rospy.get_param("base_frame_id")

    def execute_cb(self, goal_msg):
        rospy.loginfo("Received grasp pose")

        grasp_pose_msg = goal_msg.target_grasp_pose.pose
        grasp_pose_msg_list = pose_to_list(grasp_pose_msg)
        T_base_grasp = Transform(
            Rotation.from_quat(grasp_pose_msg_list[3:]), grasp_pose_msg_list[:3]
        )

        T_grasp_pregrasp = Transform(
            Rotation.from_quat([0.0, 0.0, 0.0, 1.0]), np.array([0.0, 0.0, -0.05]),
        )
        T_base_pregrasp = T_base_grasp * T_grasp_pregrasp

        msg = T_base_pregrasp.to_pose_stamped(self._base_frame_id)
        self.pregrasp_pub.publish(msg)

        rospy.loginfo("Opening hand")
        # self.robot_commander.move_gripper(width=0.10)
        self.robot_commander.release()

        rospy.loginfo("Moving to pregrasp pose")
        self.robot_commander.goto_pose_target(
            T_base_pregrasp.tolist(), max_velocity_scaling=0.2
        )

        rospy.loginfo("Moving to grasp pose")
        self.robot_commander.goto_pose_target(T_base_grasp, max_velocity_scaling=0.2)

        rospy.loginfo("Grasping")
        # self.robot_commander.grasp(0.05)
        self.robot_commander.grasp()

        rospy.loginfo("Retrieving object")
        self.robot_commander.goto_pose_target(
            T_base_pregrasp.tolist(), max_velocity_scaling=0.2
        )

        self.robot_commander.goto_joint_target(
            self._ready_joints_l, max_velocity_scaling=0.2
        )

        gripper_state = rospy.wait_for_message("/yumi/gripper_states", JointState)
        gripper_width_l = gripper_state.position[0]

        if gripper_width_l < 0.005:
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
