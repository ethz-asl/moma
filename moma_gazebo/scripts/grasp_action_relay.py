#! /usr/bin/env python3
import rospy
from actionlib.simple_action_client import SimpleActionClient
from actionlib.simple_action_server import SimpleActionServer
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
from franka_gripper.msg import GraspAction
from franka_gripper.msg import GraspGoal
from franka_gripper.msg import GraspResult


class GraspRelay:
    def __init__(self) -> None:
        self.in_action_name = rospy.get_param("~in_action_name")
        self.out_action_name = rospy.get_param("~out_action_name")

        self.grasp_server = SimpleActionServer(
            self.in_action_name, GraspAction, self.relay_action, auto_start=False
        )
        self.gripper_client = SimpleActionClient(
            self.out_action_name, GripperCommandAction
        )
        self.grasp_server.start()

    def relay_action(self, goal: GraspGoal):
        grasp_result = GraspResult()
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.position = goal.width
        gripper_goal.command.max_effort = goal.force

        grasp_result.success = True
        if not self.gripper_client.wait_for_server(rospy.Duration.from_sec(10)):
            rospy.logerr(f"Timeout while waiting for {self.out_action_name}")
            grasp_result.success = False
            grasp_result.error = f"Timeout while waiting for {self.out_action_name}"

        if not self.gripper_client.send_goal_and_wait(
            gripper_goal, rospy.Duration(10.0)
        ):
            rospy.logerr(
                f"Timeout while waiting for result from {self.out_action_name}"
            )
            grasp_result.success = False
            grasp_result.error = (
                f"Timeout while waiting for result from{self.out_action_name}"
            )

        if grasp_result.success:
            self.grasp_server.set_succeeded(grasp_result)
        else:
            self.grasp_server.set_aborted(grasp_result)


if __name__ == "__main__":
    rospy.init_node("grasp_action_relay")
    try:
        relay = GraspRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
