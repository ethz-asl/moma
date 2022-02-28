import rospy
from moma_mission.core import StateRos
from moma_mission.utils.ros import switch_ros_controller


class StateRosControl(StateRos):
    """
    Switch and send target poses to the controller manager
    """

    def __init__(self, ns, outcomes=['Completed', 'Failure']):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)

        self.startlist = self.get_scoped_param("startlist", [])
        self.stoplist = self.get_scoped_param("stoplist", [])
        self.manager_namespace = self.get_scoped_param("manager_namespace", "")
        self.ee_frame_param = self.get_scoped_param("ee_frame_param",
                                                    "/{}/{}/tool_link".format(self.manager_namespace,
                                                                              self.startlist[0])
                                                    if len(self.startlist) > 0 else "")
        self.ee_frame = rospy.get_param(self.ee_frame_param, None)
        
    def do_switch(self, wait_before_return=3.0):
        return switch_ros_controller(startlist=self.startlist,
                                     stoplist=self.stoplist,   
                                     manager_namespace=self.manager_namespace)
