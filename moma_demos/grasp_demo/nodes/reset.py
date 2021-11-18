#!/usr/bin/env python

import geometry_msgs.msg
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import tf2_ros

from moma_utils.spatial import Transform
from moma_utils.ros.conversions import to_transform_msg
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaArmClient, PandaGripperClient


class ResetNode(object):
    def __init__(self):
        self.broadcast_task_frame_transform()
        self.arm = PandaArmClient()
        self.gripper = PandaGripperClient()
        self.moveit = MoveItClient("panda_arm")
        rospy.sleep(1.0)  # wait for the moveit connection to be established
        self.create_planning_scene()

        rospy.Service("reset", Trigger, self.reset)
        rospy.loginfo("Reset service ready")

    def create_planning_scene(self):
        # Add a collision box for the table
        table_height = rospy.get_param("moma_demo/table_height")
        safety_margin = 0.01
        msg = geometry_msgs.msg.PoseStamped()
        msg.header.frame_id = "panda_link0"
        msg.pose.position.x = 0.5
        msg.pose.position.y = 0.0
        msg.pose.position.z = table_height - 0.01 + safety_margin
        self.moveit.scene.add_box("table", msg, size=(0.6, 0.6, 0.02))

    def broadcast_task_frame_transform(self):
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        table_height = rospy.get_param("moma_demo/table_height")
        rospy.sleep(0.5)
        T_panda_link0_task = Transform.translation([0.25, -0.15, table_height - 0.05])
        msg = geometry_msgs.msg.TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "panda_link0"
        msg.child_frame_id = "task"
        msg.transform = to_transform_msg(T_panda_link0_task)
        self.static_broadcaster.sendTransform(msg)

    def reset(self, req):
        if self.arm.has_error:
            self.arm.recover()
        self.moveit.goto("ready", velocity_scaling=0.2)
        # self.gripper.home()
        self.gripper.grasp()
        return TriggerResponse()


def main():
    rospy.init_node("reset")
    ResetNode()
    rospy.spin()


if __name__ == "__main__":
    main()
