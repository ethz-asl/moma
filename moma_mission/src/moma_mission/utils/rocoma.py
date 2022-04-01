import rospy
import os
from rocoma_msgs.srv import (
    SwitchController,
    SwitchControllerRequest,
    SwitchControllerResponse,
)


def switch_roco_controller(controller_name, ns=""):
    """
    Switch a roco controller
    :param controller_name:
    :param ns: namespace of the controller manager
    :return: if the switch succeeded or failed
    """
    if controller_name == "":
        rospy.logwarn("Controller name is " ". Could not switch.")
        return False
    switch_service = rospy.ServiceProxy(
        os.path.join(ns, "controller_manager", "switch_controller"), SwitchController
    )
    try:
        switch_service.wait_for_service(10.0)
    except rospy.ROSException:
        rospy.logerr(
            "Failed to contact service: {}".format(switch_service.resolved_name)
        )
        return False

    req = SwitchControllerRequest()
    req.name = controller_name

    res = switch_service.call(req)
    if res.status == SwitchControllerResponse.STATUS_ERROR:
        rospy.logerr(
            "Error when trying to switch the roco controller: {}".format(
                controller_name
            )
        )
        return False
    if res.status == SwitchControllerResponse.STATUS_NA:
        rospy.logerr("The controller {} is not available".format(controller_name))
        return False
    if res.status == SwitchControllerResponse.STATUS_NOTFOUND:
        rospy.logerr("The controller {} has not been found.".format(controller_name))
        return False

    if res.status == SwitchControllerResponse.STATUS_RUNNING:
        rospy.logwarn("The controller {} is already running.".format(controller_name))
        return True
    if res.status == SwitchControllerResponse.STATUS_SWITCHED:
        rospy.loginfo(
            "Successfully switched to the controller {}".format(controller_name)
        )
        return True
