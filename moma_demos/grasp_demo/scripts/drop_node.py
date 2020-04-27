#!/usr/bin/env python

from __future__ import print_function

import sys

from actionlib import SimpleActionServer
import rospy

from grasp_demo.msg import DropAction, DropResult
from grasp_demo.utils import create_robot_connection


class DropActionNode(object):
    def __init__(self):
        self.robot_name = sys.argv[1]
        simulation_mode = True if sys.argv[2] == "true" else False
        self._load_parameters()
        self._connect_robot(simulation_mode)

        self._as = SimpleActionServer(
            "drop_action", DropAction, execute_cb=self.execute_cb, auto_start=False
        )
        self._as.start()

        rospy.loginfo("Drop action server ready")

    def _connect_robot(self, simulation_mode):
        full_robot_name = (
            self.robot_name + "_" + self._robot_arm_names[0]
            if len(self._robot_arm_names) > 1
            else self.robot_name
        )
        self._robot_arm = create_robot_connection(full_robot_name, simulation_mode)

    def _load_parameters(self):
        self._robot_arm_names = rospy.get_param("/moma_demo/robot_arm_names")
        self._home_joints = rospy.get_param(
            "/moma_demo/home_joints_" + self._robot_arm_names[0]
        )
        self._drop_joints = rospy.get_param(
            "/moma_demo/drop_joints_" + self._robot_arm_names[0]
        )
        self._arm_velocity_scaling = rospy.get_param(
            "/moma_demo/arm_velocity_scaling_drop"
        )

    def execute_cb(self, goal):
        rospy.loginfo("Dropping action was triggered")

        self._robot_arm.goto_joint_target(
            self._drop_joints, max_velocity_scaling=self._arm_velocity_scaling
        )
        rospy.sleep(0.5)  # wait for the operator's hand to be placed under the EE
        self._robot_arm.release()
        rospy.sleep(0.2)
        self._robot_arm.goto_joint_target(
            self._home_joints, max_velocity_scaling=self._arm_velocity_scaling
        )

        rospy.loginfo("Dropping action succeeded")
        self._as.set_succeeded(DropResult())


def main():
    try:
        rospy.init_node("drop_action_node")
        DropActionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
