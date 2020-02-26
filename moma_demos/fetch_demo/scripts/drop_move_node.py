#!/usr/bin/env python

import actionlib
from fetch_demo.msg import DropMoveAction, DropMoveResult
import rospy
from grasp_demo.utils import create_robot_connection
from fetch_demo.common import MovingActionServer
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Vector3, Twist


class DropActionServer(MovingActionServer):
    """
        When called, this action should navigate the base and arm to a pre-specified position,
        then open the gripper to drop the grasped object. 
    """

    def __init__(self):
        action_name = "drop_move_action"
        super(DropActionServer, self).__init__(action_name, DropMoveAction)
        self._read_joint_configurations()
        self._connect_ridgeback()
        self._connect_robot_arm()
        self._read_waypoints()

    def _read_waypoints(self):
        waypoints = rospy.get_param("drop_waypoints")
        self._retract_waypoint = waypoints["retract_waypoint"]
        self._drop_waypoints = waypoints["drop_waypoints"]

    def _read_joint_configurations(self):
        self._robot_arm_names = rospy.get_param("robot_arm_names")
        self._home_joints = rospy.get_param("home_joints_" + self._robot_arm_names[0])
        self._drop_joints = rospy.get_param("drop_joints_" + self._robot_arm_names[0])
        self._search_joints = rospy.get_param("drop_joints_" + self._robot_arm_names[1])
        self._arm_velocity_scaling = rospy.get_param("arm_velocity_scaling_drop")

    def _connect_robot_arm(self):
        full_robot_name = (
            self.robot_name + "_" + self._robot_arm_names[0]
            if len(self._robot_arm_names) > 1
            else self.robot_name
        )
        self._robot_arm_grasp = create_robot_connection(full_robot_name)

        if len(self._robot_arm_names) > 1:
            full_robot_name = self.robot_name + "_" + self._robot_arm_names[1]
            self._robot_arm_scan = create_robot_connection(full_robot_name)
        else:
            self._robot_arm_scan = None

    def _connect_ridgeback(self):
        self._base_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def action_callback(self, msg):
        rospy.loginfo("Start drop action")
        result = DropMoveResult()

        vel_msg = Twist(linear=Vector3(-0.1, 0.0, 0.0))
        for i in range(50):
            self._base_vel_pub.publish(vel_msg)
            rospy.sleep(0.1)

        state = self._visit_waypoint(self._retract_waypoint)
        if state == GoalStatus.PREEMPTED:
            rospy.loginfo("Got preemption request")
            self.action_server.set_preempted()
            return
        elif state == GoalStatus.ABORTED:
            rospy.logerr("Failed to navigate to retract waypoint")
            self.action_server.set_aborted()
            return

        # Move arms into a good position for driving
        self._robot_arm_grasp.goto_joint_target(
            self._home_joints, max_velocity_scaling=self._arm_velocity_scaling
        )
        if self._robot_arm_scan is not None:
            self._robot_arm_scan.goto_joint_target(
                self._search_joints, max_velocity_scaling=self._arm_velocity_scaling
            )

        # Follow waypoints to drop location
        for waypoint in self._drop_waypoints:
            state = self._visit_waypoint(waypoint)
            if state == GoalStatus.PREEMPTED:
                rospy.loginfo("Got preemption request")
                self.action_server.set_preempted()
                return
            elif state == GoalStatus.ABORTED:
                rospy.logerr("Failed to navigate to a drop waypoint")
                self.action_server.set_aborted()
                return

        # Drop the object
        self._robot_arm_grasp.goto_joint_target(
            self._drop_joints, max_velocity_scaling=self._arm_velocity_scaling
        )
        self._robot_arm_grasp.release()
        self._robot_arm_grasp.goto_joint_target(
            self._home_joints, max_velocity_scaling=self._arm_velocity_scaling
        )

        rospy.loginfo("Finished dropping")
        self.action_server.set_succeeded(result)


def main():
    rospy.init_node("drop_move_action_node")
    DropActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
