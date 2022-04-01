import os
import sys
import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from controller_manager_msgs.srv import (
    ListControllers,
    ListControllersRequest,
    ListControllersResponse,
)


def get_param_safe(param_name, default=None):
    """
    Return the parameter if it exists, otherwise it raises an exception
    :param param_name:
    :return:
    """
    try:
        return (
            rospy.get_param(param_name)
            if default is None
            else rospy.get_param(param_name, default)
        )
    except KeyError as exc:
        print("ERROR")

        rospy.logerr(exc)
        raise NameError("Failed to parse parameter {}".format(param_name))


def switch_ros_controller(startlist=[], stoplist=[], manager_namespace=""):
    list_controller_service_name = os.path.join(
        manager_namespace, "controller_manager", "list_controllers"
    )
    try:
        rospy.wait_for_service(list_controller_service_name, timeout=2.0)
    except rospy.ROSException as exc:
        rospy.logerr("Failed to call {}".format(list_controller_service_name))
        return False

    # stop all controllers running and not in the whitelist
    controller_stop_list = []
    controller_start_list = []

    rospy.loginfo(
        f"switch_controller_ros:\nstart list={startlist}\nstop list={stoplist}"
    )
    list_service_client = rospy.ServiceProxy(
        list_controller_service_name, ListControllers
    )
    req = ListControllersRequest()
    res = list_service_client.call(req)

    controllers_map = {ctrl.name: ctrl.state for ctrl in res.controller}
    for controller in startlist:
        if controller not in controllers_map.keys():
            rospy.logerr(
                "Cannot start controller [{}]. Have you loaded it?".format(controller)
            )
            return False

        if controllers_map[controller] == "running":
            rospy.logwarn("Controller [{}] is already running".format(controller))
        else:
            controller_start_list.append(controller)

    for controller in stoplist:
        if controller not in controllers_map.keys():
            rospy.logwarn(
                "Conrtoller [{}] is not loaded. Have you unloaded it?".format(
                    controller
                )
            )

        if controllers_map[controller] == "running":
            controller_stop_list.append(controller)
        else:
            rospy.logwarn(
                "Controller [{}] is not running yet, so it cannot be stopped.".format(
                    controller
                )
            )

    rospy.loginfo("Stopping controllers: {}".format(controller_stop_list))
    rospy.loginfo("Starting controllers: {}".format(controller_start_list))
    switch_controller_service_name = os.path.join(
        manager_namespace, "controller_manager", "switch_controller"
    )
    try:
        rospy.wait_for_service(switch_controller_service_name, timeout=2.0)
    except rospy.ROSException as exc:
        rospy.logerr("Failed to call {}".format(switch_controller_service_name))
        return False

    switch_controller_client = rospy.ServiceProxy(
        switch_controller_service_name, SwitchController
    )
    req = SwitchControllerRequest()
    req.start_controllers = controller_start_list
    req.stop_controllers = controller_stop_list
    req.timeout = 5.0
    req.start_asap = True
    req.strictness = req.STRICT

    res = switch_controller_client.call(req)
    if not res.ok:
        rospy.logerr("Failed to switch ros controller")
        return False

    if len(controller_stop_list) > 0:
        rospy.loginfo("Sleeping 3.0 sec before returning")
        rospy.sleep(3.0)
    else:
        rospy.loginfo("No new controller started")

    rospy.loginfo("Successfully switched controllers")
    return True
