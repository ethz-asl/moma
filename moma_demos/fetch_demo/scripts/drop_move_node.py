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
        self._connect_yumi()
        self._read_waypoints()

    def _read_waypoints(self):
        waypoints = rospy.get_param("drop_waypoints")
        self._retract_waypoint = waypoints["retract_waypoint"]
        self._drop_waypoints = waypoints["drop_waypoints"]

    def _read_joint_configurations(self):
        self._search_joints_r = rospy.get_param("search_joints_r")
        self._ready_joints_l = rospy.get_param("ready_joints_l")
        self._home_joints_l = rospy.get_param("home_joints_l")

    def _connect_yumi(self):
        self._left_arm = create_robot_connection("yumi_left_arm")
        self._right_arm = create_robot_connection("yumi_right_arm")

    def _connect_ridgeback(self):
        self._base_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def action_callback(self, msg):
        rospy.loginfo("Start approaching object")
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
            rospy.logerr("Failed to navigate to approach waypoint")
            self.action_server.set_aborted()
            return

        self._left_arm.goto_joint_target(self._home_joints_l, max_velocity_scaling=0.5)
        self._right_arm.goto_joint_target(
            self._search_joints_r, max_velocity_scaling=0.5
        )

        for waypoint in self._drop_waypoints:
            state = self._visit_waypoint(waypoint)
            if state == GoalStatus.PREEMPTED:
                rospy.loginfo("Got preemption request")
                self.action_server.set_preempted()
                return
            elif state == GoalStatus.ABORTED:
                rospy.logerr("Failed to navigate to approach waypoint " + waypoint)
                self.action_server.set_aborted()
                return

        self._left_arm.goto_joint_target(self._ready_joints_l, max_velocity_scaling=0.5)
        self._left_arm.release()
        self._left_arm.goto_joint_target(self._home_joints_l, max_velocity_scaling=0.5)

        rospy.loginfo("Finished dropping")
        self.action_server.set_succeeded(result)


def main():
    rospy.init_node("drop_move_action_node")
    DropActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
