from moma_mission.core import StateRos
from moma_mission.utils.ros import switch_ros_controller


class StateRosControl(StateRos):
    """
    Switch and send target poses to the controller manager
    """

    def __init__(self, ns, outcomes=['Completed', 'Failure']):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)

        self.controller_name = self.get_scoped_param("controller_name")
        self.manager_namespace = self.get_scoped_param("manager_namespace")
        self.whitelist = self.get_scoped_param("whitelist")

    def do_switch(self):
        return switch_ros_controller(controller_name=self.controller_name,
                                     manager_namespace=self.manager_namespace,
                                     whitelist=self.whitelist)
