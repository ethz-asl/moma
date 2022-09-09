#!/usr/bin/env python3
"""ROS node that attaches links in Gazebo for a robust grasping."""

from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from mobile_manip_demo.srv import (
    ForceDrop,
    ForceDropResponse,
    ForceGrasp,
    ForceGraspResponse,
)
import rospy


class ForceManipulation:
    def __init__(self):
        """Initialize ROS nodes."""
        rospy.init_node("force_manipulation")
        # Advertised Service
        self.grasp_srv = rospy.Service("force_grasp", ForceGrasp, self.force_grasp)

        # TODO: check if we need to pass a target pose
        self.drop_srv = rospy.Service("force_drop", ForceDrop, self.force_drop)

        self.attach_client = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
        self.attach_client.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        self.detach_client = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
        self.detach_client.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    def force_grasp(self, req):
        # Attach them
        new_req = AttachRequest()
        new_req.model_name_1 = req.model_name
        new_req.link_name_1 = req.model_link
        new_req.model_name_2 = req.ee_name
        new_req.link_name_2 = req.ee_link
        rospy.loginfo(
            f"Attaching model {new_req.model_name_1} to {new_req.model_name_2} for grasping."
        )

        response = self.attach_client.call(new_req)
        rospy.loginfo(f"With call: {response.ok}")
        return ForceGraspResponse(response.ok, "")

    def force_drop(self, req):
        # Detach them
        new_req = AttachRequest()
        new_req.model_name_1 = req.model_name
        new_req.link_name_1 = req.model_link
        new_req.model_name_2 = req.ee_name
        new_req.link_name_2 = req.ee_link
        rospy.loginfo(
            f"Detaching model {new_req.model_name_1} from {new_req.model_name_2} for dropping."
        )

        # In case drop item in a given position

        response = self.detach_client.call(new_req)
        return ForceDropResponse(response.ok, "")


if __name__ == "__main__":
    sb = ForceManipulation()
    rospy.spin()
