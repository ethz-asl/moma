#!/usr/bin/env python
import smach
import rospy
from os.path import join

from moma_mission.utils import ros


class StateMachineContext(object):
    def __init__(self):
        self.data = {}


# A global context accessible to all states inheriting from this state
global_context = StateMachineContext()


class StateRos(smach.State):
    """
    Base state adding minimal ROS functionality to the basic smach state
    """

    def __init__(self, outcomes=['Completed', 'Failure'],
                 input_keys=[],
                 output_keys=[],
                 ns=""):
        smach.State.__init__(self,
                             outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.namespace = ns
        global global_context
        self.global_context = global_context

        self.initialization_failure = False

        # parse optional default outcome
        self.default_outcome = self.get_scoped_param("default_outcome", safe=False)
        if self.default_outcome and self.default_outcome not in self.get_registered_outcomes():
            raise NameError("{} is not in default outcomes".format(self.default_outcome))  # prevent accidental typos

    def get_scoped_param(self, param_name, safe=True):
        """
        Get the parameter namespaced under the state name
        e.g get_scoped_param('/my_param') --> looks for '/state_name/my_param'
            get_scoped_param('my_param') --> looks for '/ros_namespace/state_name/my_param'
        """
        if param_name.startswith('/'):
            named_param = join(self.namespace, param_name)
        else:
            named_param = join(rospy.get_namespace(), self.namespace, param_name)

        if safe:
            return ros.get_param_safe(named_param)
        else:
            if rospy.has_param(named_param):
                return rospy.get_param(named_param)
            else:
                return None

    def execute(self, ud):
        if self.default_outcome:
            rospy.loginfo(
                "{state} ==> {outcome} (using default outcome)".format(state=self.namespace, outcome=self.default_outcome))
            return self.default_outcome

        if self.initialization_failure:
            return "Failure"

        return self.run()

    def run(self):
        raise NotImplementedError("run must be implemented by the inheriting state")

    def set_context_data(self, key, data, overwrite=False):
        """
        Set a new data field in the global context accessible to all states which
        are derived from this state
        :param key: data key
        :param data: data content
        :param overwrite: if True, let the user overwrite data if already in global context
        :return: True if read was successful
        """
        if not overwrite and key in self.global_context.data.keys():
            rospy.logwarn("Could not set data in global context. Key {} already exists".format(key))
            return False
        self.global_context.data[key] = data

    def get_context_data(self, key):
        if key in self.global_context.data.keys():
            return self.global_context.data[key]
        else:
            rospy.logwarn("Failed to retrieve [{}] from data".format(key))
            return None


if __name__ == "__main__":

    class StateA(StateRos):
        def __init__(self, ns=""):
            StateRos.__init__(self, ns='state_a')

        def run(self):
            try:
                param = self.get_scoped_param("my_param")
                self.set_context_data("a_data", param)
            except NameError as exc:
                rospy.logerr(exc)
                return 'Failure'
            return 'Completed'


    class StateB(StateRos):
        def __init__(self):
            StateRos.__init__(self, ns='state_b')

        def run(self):
            value = self.get_context_data("a_data")
            rospy.loginfo("value is {}".format(value))
            return 'Completed'


    class StateC(StateRos):
        def __init__(self):
            StateRos.__init__(self, ns='state_c')

        def run(self):
            print("I should not print because of default transition")
            return 'Completed'


    rospy.init_node("state_ros_test")
    rospy.set_param("state_a/my_param", 1)
    rospy.set_param("state_c/default_outcome", "Failure")

    a = StateA()
    a.execute(ud=None)

    b = StateB()
    b.execute(ud=None)

    c = StateC()
    c.execute(ud=None)
