import rospy
from moma_mission.core import StateRos
from moma_mission.utils.ros import switch_ros_controller


class StateRosControl(StateRos):
    """
    Switch and send target poses to the controller manager
    """

    def __init__(
        self, ns, outcomes=["Completed", "Failure"], input_keys=[], output_keys=[]
    ):
        StateRos.__init__(
            self,
            ns=ns,
            outcomes=outcomes,
            input_keys=input_keys,
            output_keys=output_keys,
        )

        # The lists are assumed to be in the same namespace as the "manager_namespace"
        self.startlist = self.get_scoped_param("startlist", [])
        self.stoplist = self.get_scoped_param("stoplist", [])
        # The maps are assumed to have the namespaces as keys
        self.startmap = self.get_scoped_param("startmap", {})
        self.stopmap = self.get_scoped_param("stopmap", {})
        self.manager_namespace = self.get_scoped_param("manager_namespace", "")
        self.ee_frame_param = self.get_scoped_param(
            "ee_frame_param",
            "/{}/{}/tool_link".format(self.manager_namespace, self.startlist[0])
            if len(self.startlist) > 0
            else "",
        )
        self.ee_frame = rospy.get_param(self.ee_frame_param, None)

    def do_switch(self):
        manager_namespaces = set(list(self.startmap.keys()) + list(self.stopmap.keys()))
        for ns in manager_namespaces:
            switch_result = switch_ros_controller(
                startlist=self.startmap[ns] if ns in self.startmap else [],
                stoplist=self.stopmap[ns] if ns in self.stopmap else [],
                manager_namespace=ns,
            )
            if not switch_result:
                return switch_result
        return switch_ros_controller(
            startlist=self.startlist,
            stoplist=self.stoplist,
            manager_namespace=self.manager_namespace,
        )
