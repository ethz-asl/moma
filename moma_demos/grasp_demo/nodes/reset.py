#!/usr/bin/env python3

import geometry_msgs.msg
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import PointField, PointCloud2
from sensor_msgs.point_cloud2 import create_cloud
from std_msgs.msg import Header
import tf2_ros

from vgn.rviz import Visualizer
from moma_utils.spatial import Transform
from moma_utils.ros.conversions import to_transform_msg
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaArmClient, PandaGripperClient


class ResetNode(object):
    def __init__(self):
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.arm = PandaArmClient()
        self.gripper = PandaGripperClient()
        self.moveit = MoveItClient("panda_arm")
        self.vis = Visualizer()
        self.cloud_pub = rospy.Publisher("/gsm_node/cloud", PointCloud2)
        rospy.sleep(1.0)  # wait for connections to be established
        self.broadcast_task_frame_transform()
        self.create_planning_scene()

        rospy.Service("reset", Trigger, self.reset)
        rospy.loginfo("Reset service ready")

    def broadcast_task_frame_transform(self):
        table_height = rospy.get_param("moma_demo/table_height")
        T_panda_link0_task = Transform.translation([0.25, -0.15, table_height - 0.05])
        msg = geometry_msgs.msg.TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "panda_link0"
        msg.child_frame_id = "task"
        msg.transform = to_transform_msg(T_panda_link0_task)
        self.static_broadcaster.sendTransform(msg)
        rospy.sleep(1.0)

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

    def reset(self, req):
        self.reset_arm()
        self.reset_vis()
        return TriggerResponse()

    def reset_arm(self):
        if self.arm.has_error:
            self.arm.recover()
        self.moveit.goto("ready", velocity_scaling=0.2)
        self.gripper.grasp()

    def reset_vis(self):
        self.vis.clear()
        self.vis.roi("task", 0.3)
        header = Header(frame_id="task")
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 8, PointField.FLOAT32, 1),
        ]
        empty_cloud_msg = create_cloud(header, fields, [])
        self.cloud_pub.publish(empty_cloud_msg)


def main():
    rospy.init_node("reset")
    ResetNode()
    rospy.spin()


if __name__ == "__main__":
    main()
