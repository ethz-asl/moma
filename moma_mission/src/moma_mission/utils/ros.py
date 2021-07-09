import os
import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from controller_manager_msgs.srv import ListControllers, ListControllersRequest, ListControllersResponse


def get_param_safe(param_name):
    """
    Return the parameter if it exists, otherwise it raises an exception
    :param param_name:
    :return:
    """
    try:
        return rospy.get_param(param_name)
    except KeyError as exc:
        rospy.logerr(exc)
        raise NameError("Failed to parse parameter {}".format(param_name))


def switch_ros_controller(controller_name, manager_namespace='', whitelist=[]):
    list_controller_service_name = os.path.join(manager_namespace, "controller_manager", "list_controllers")
    try:
        rospy.wait_for_service(list_controller_service_name, timeout=2.0)
    except rospy.ROSException as exc:
        rospy.logerr("Failed to call {}".format(list_controller_service_name))
        return False

    rospy.loginfo("Starting the controller {}".format(controller_name))
    # stop all controllers running and not in the whitelist
    controller_stop_list = []
    controller_start_list = []

    list_service_client = rospy.ServiceProxy(list_controller_service_name, ListControllers)
    req = ListControllersRequest()
    res = list_service_client.call(req)
    for controller in res.controller:
        rospy.loginfo("Controller {}, state={}".format(controller.name, controller.state))
        if controller.name == controller_name and controller.state != "running":
            controller_start_list.append(controller.name)
        elif controller.name not in whitelist and controller.state == "running" and controller.name != controller_name:
            controller_stop_list.append(controller.name)

    rospy.loginfo("Stopping controllers: {}".format(controller_stop_list))
    rospy.loginfo("Starting controllers: {}".format(controller_start_list))
    switch_controller_service_name = os.path.join(manager_namespace, "controller_manager", "switch_controller")
    try:
        rospy.wait_for_service(switch_controller_service_name, timeout=2.0)
    except rospy.ROSException as exc:
        rospy.logerr("Failed to call {}".format(switch_controller_service_name))
        return False

    switch_controller_client = rospy.ServiceProxy(switch_controller_service_name, SwitchController)
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

    rospy.loginfo("Successfully switched controllers")
    return True