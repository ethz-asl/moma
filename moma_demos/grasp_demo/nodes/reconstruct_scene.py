#!/usr/bin/env python3

import sys

from actionlib import SimpleActionServer
import rospy
import std_srvs.srv
import vgn.srv
import grasp_demo.msg
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaArmClient
import vpp_msgs.srv


class ReconstructSceneNode(object):
    """Reconstruct scene moving the camera along a fixed trajectory.

    The reconstruction is done with Voxblox++.
    """

    def __init__(self, semantic):
        self.moveit = MoveItClient("panda_arm")
        self.scan_joints = rospy.get_param("moma_demo/scan_joints")
        self.arm = PandaArmClient()

        # Init members
        self.semantic = semantic
        self.reset_map = None
        self.toggle_integration = None
        self.get_scene_cloud = None
        self.get_map = None

        if semantic:
            self.connect_to_gsm_node()
        else:
            self.connect_to_tsdf_node()

        execute_cb = self.reconstruct_scene
        self.action_server = SimpleActionServer(
            "scan_action",
            grasp_demo.msg.ScanSceneAction,
            execute_cb=execute_cb,
            auto_start=False,
        )
        self.action_server.start()
        rospy.loginfo("Scan action server ready")

    def connect_to_gsm_node(self):
        rospy.loginfo("Waiting for gsm_node")
        rospy.wait_for_service("/gsm_node/reset_map")
        self.reset_map = rospy.ServiceProxy("/gsm_node/reset_map", std_srvs.srv.Empty)
        self.toggle_integration = rospy.ServiceProxy(
            "/gsm_node/toggle_integration", std_srvs.srv.SetBool
        )
        self.get_scene_cloud = rospy.ServiceProxy(
            "/gsm_node/get_scene_pointcloud", vpp_msgs.srv.GetScenePointcloud
        )
        self.get_map = rospy.ServiceProxy("/gsm_node/get_map", vpp_msgs.srv.GetMap)

    def connect_to_tsdf_node(self):
        rospy.loginfo("Waiting for TSDF node")
        rospy.wait_for_service("reset_map")
        self.reset_map = rospy.ServiceProxy("reset_map", std_srvs.srv.Trigger)
        self.toggle_integration = rospy.ServiceProxy(
            "toggle_integration", std_srvs.srv.SetBool
        )
        self.get_scene_cloud = rospy.ServiceProxy(
            "get_scene_cloud", vgn.srv.GetSceneCloud
        )
        self.get_map = rospy.ServiceProxy("get_map_cloud", vgn.srv.GetMapCloud)

    def reconstruct_scene(self, goal):
        if self.semantic:
            self.reset_map(std_srvs.srv.EmptyRequest())
            map_request = vpp_msgs.srv.GetMapRequest()
        else:
            self.reset_map(std_srvs.srv.TriggerRequest())
            map_request = vgn.srv.GetMapCloudRequest()

        self.moveit.goto(self.scan_joints[0], velocity_scaling=0.2)

        rospy.loginfo("Mapping scene")
        self.toggle_integration(std_srvs.srv.SetBoolRequest(data=True))
        rospy.sleep(2.0)
        # check with just one view
        for joints in self.scan_joints[1:]:
            self.moveit.goto(joints, velocity_scaling=0.2)
            rospy.sleep(2.0)
        self.toggle_integration(std_srvs.srv.SetBoolRequest(data=False))

        msg = self.get_map(map_request)
        result = grasp_demo.msg.ScanSceneResult(
            pointcloud_scene=msg.map_cloud, voxel_size=msg.voxel_size
        )

        self.action_server.set_succeeded(result)
        rospy.loginfo("Scan scene action succeeded")


def main():
    rospy.init_node("scan_action_node")
    semantic = sys.argv[1] in ["True", "true"]
    ReconstructSceneNode(semantic)
    rospy.spin()


if __name__ == "__main__":
    main()
