from controller_manager_msgs.srv import *
import rospy


class ControllerManager(object):
    """Wrapper around the ROS controller manager's service calls.
    
    See http://wiki.ros.org/controller_manager for more information
    about the ROS controller manager.
    """

    def __init__(self, controller_manager_ns="/controller_manager"):

        list_controllers_srv_name = controller_manager_ns + "/list_controllers"
        rospy.wait_for_service(list_controllers_srv_name)
        self.list_controller_srv = rospy.ServiceProxy(
            list_controllers_srv_name, ListControllers
        )

        switch_controller_srv_name = controller_manager_ns + "/switch_controller"
        rospy.wait_for_service(switch_controller_srv_name)
        self.switch_controller_srv = rospy.ServiceProxy(
            switch_controller_srv_name, SwitchController
        )

    def list_controllers(self):
        """Returns a list of loaded controllers and their state."""
        controller_states = self.list_controller_srv(ListControllersRequest())
        return controller_states

    def switch_controller(self, start_controllers=[], stop_controllers=[]):
        """Switch the currently running controllers.

        Args:
            start_controllers: List of controllers to start.
            stop_controllers: List of controllers to stop.

        Returns:
            True if the switch was successful, False otherwise.
        """
        req = SwitchControllerRequest()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        req.strictness = SwitchControllerRequest.BEST_EFFORT
        res = self.switch_controller_srv(req)

        return res.ok
