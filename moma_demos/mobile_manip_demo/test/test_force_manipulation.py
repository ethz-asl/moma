#!/usr/bin/env python

from mobile_manip_demo.srv import (
    ForceDrop,
    ForceDropRequest,
    ForceGrasp,
    ForceGraspRequest,
)
import rospy

import numpy as np


class GraspingNode:
    def __init__(self):
        """Initialize ROS nodes."""
        # Parameters
        model_name = "wood_cube_2"
        robot_name = "panda"
        model_link = "wood_cube_2::link"
        ee_link = "panda::panda_leftfinger"
        self.grasp_request = ForceGraspRequest()
        self.grasp_request.model_name = model_name
        self.grasp_request.ee_name = robot_name
        self.grasp_request.model_link = model_link
        self.grasp_request.ee_link = ee_link

        self.drop_request = ForceDropRequest()
        self.drop_request.model_name = model_name
        self.drop_request.ee_name = robot_name
        self.drop_request.model_link = model_link
        self.drop_request.ee_link = ee_link

        self.attach_srv = rospy.ServiceProxy("force_grasp", ForceGrasp)
        self.attach_srv.wait_for_service()

        self.detach_srv = rospy.ServiceProxy("force_drop", ForceDrop)
        self.detach_srv.wait_for_service()

    def test_pick_and_place(self):
        response = self.attach_srv.call(self.grasp_request)
        # rospy.loginfo(f"Graps request returned: {response.success}")
        rospy.sleep(5.0)
        if response.success:
            self.detach_srv.call(self.drop_request)


def main():
    rospy.init_node("grasping_tester_node")
    node = GraspingNode()

    try:
        node.test_pick_and_place()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
