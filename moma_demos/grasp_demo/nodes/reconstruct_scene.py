#!/usr/bin/env python3

import sys

from actionlib import SimpleActionServer
import rospy
from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest

from grasp_demo.msg import ScanSceneAction, ScanSceneResult
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaArmClient
from vpp_msgs.srv import GetScenePointcloud, GetScenePointcloudRequest


class ReconstructSceneNode(object):
    """Reconstruct scene moving the camera along a fixed trajectory.

    The reconstruction is done with Voxblox++.
    """

    def __init__(self, semantic):
        self.moveit = MoveItClient("panda_arm")
        self.scan_joints = rospy.get_param("moma_demo/scan_joints")
        self.arm = PandaArmClient()

        if semantic:
            self.connect_to_gsm_node()
            execute_cb = self.reconstruct_scene_with_vpp
        else:
            raise NotImplementedError

        self.action_server = SimpleActionServer(
            "scan_action",
            ScanSceneAction,
            execute_cb=execute_cb,
            auto_start=False,
        )
        self.action_server.start()
        rospy.loginfo("Scan action server ready")

    def connect_to_gsm_node(self):
        rospy.loginfo("Waiting for gsm_node")
        rospy.wait_for_service("/gsm_node/reset_map")
        self.reset_map = rospy.ServiceProxy("/gsm_node/reset_map", Empty)
        self.toggle_integration = rospy.ServiceProxy(
            "/gsm_node/toggle_integration", SetBool
        )
        self.get_scene_cloud = rospy.ServiceProxy(
            "/gsm_node/get_scene_pointcloud", GetScenePointcloud
        )

    def reconstruct_scene_with_vpp(self, goal):
        rospy.loginfo("Mapping scene")
        self.reset_map(EmptyRequest())

        self.toggle_integration(SetBoolRequest(data=True))
        for joints in self.scan_joints:
            self.moveit.goto(joints, velocity_scaling=0.4, acceleration_scaling=0.2)
            rospy.sleep(2.0)
        self.toggle_integration(SetBoolRequest(data=False))

        msg = self.get_scene_cloud(GetScenePointcloudRequest())
        cloud = msg.scene_cloud
        result = ScanSceneResult(pointcloud_scene=cloud)

        self.action_server.set_succeeded(result)
        rospy.loginfo("Scan scene action succeeded")


def main():
    rospy.init_node("scan_action_node")
    semantic = sys.argv[1] in ["True", "true"]
    ReconstructSceneNode(semantic)
    rospy.spin()


if __name__ == "__main__":
    main()
