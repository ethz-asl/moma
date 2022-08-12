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
        self.base_frame_id = rospy.get_param("moma_demo/base_frame_id")
        self.task_frame_id = rospy.get_param("moma_demo/task_frame_id")
        self.table_height = rospy.get_param("moma_demo/table_height")
        self.broadcast_roi()
        self.init_robot_connection()
        self.vis = Visualizer()
        self.cloud_pub = rospy.Publisher("/gsm_node/cloud", PointCloud2)
        rospy.Service("reset", Trigger, self.reset)
        rospy.loginfo("Reset service ready")

    def broadcast_roi(self):
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.sleep(1.0)

        T_base_task = Transform.translation([0.3, -0.15, self.table_height - 0.05])

        msg = geometry_msgs.msg.TransformStamped()
        msg.header.frame_id = self.base_frame_id
        msg.child_frame_id = self.task_frame_id
        msg.transform = to_transform_msg(T_base_task)
        self.static_broadcaster.sendTransform(msg)

    def init_robot_connection(self):
        self.arm = PandaArmClient()
        self.gripper = PandaGripperClient()
        self.moveit = MoveItClient("panda_arm")

        rospy.sleep(1.0)

        # Add a collision box for the table
        msg = geometry_msgs.msg.PoseStamped()
        msg.header.frame_id = self.base_frame_id
        msg.pose.position.x = 0.4
        msg.pose.position.z = self.table_height
        self.moveit.scene.add_box("table", msg, size=(0.6, 0.6, 0.02))

    def reset(self, req):
        self.reset_arm()
        rospy.loginfo("Reset arm successful")
        self.reset_vis()
        response = TriggerResponse()
        response.success = True
        return response

    def reset_arm(self):
        if self.arm.has_error:
            self.arm.recover()
        self.moveit.goto("ready", velocity_scaling=0.2)
        rospy.loginfo("Going to ready position")
        self.gripper.grasp()
        self.gripper.release()

    def reset_vis(self):
        self.vis.clear()
        self.vis.roi(self.task_frame_id, 0.3)
        header = Header(frame_id=self.base_frame_id)
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
