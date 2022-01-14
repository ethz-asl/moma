#!/usr/bin/env python

import rospy
import actionlib

from moma_msgs.msg import JointAction, JointGoal
from moma_mission.core import StateRos, StateRosControl
from moma_mission.utils.ros import switch_ros_controller
from moma_mission.utils.moveit import MoveItPlanner


class RosControlPoseReaching(StateRos):
    """
    Switch and send target poses to the controller manager
    """

    def __init__(self, ns, outcomes=['Completed', 'Failure']):
        StateRos.__init__(self,
                          outcomes=outcomes,
                          ns=ns)

        self.controller_name = self.get_scoped_param("controller_name")
        self.manager_namespace = self.get_scoped_param("manager_namespace")
        self.whitelist = self.get_scoped_param("whitelist")

    def do_switch(self):
        return switch_ros_controller(controller_name=self.controller_name,
                                     manager_namespace=self.manager_namespace,
                                     whitelist=self.whitelist)


class JointsConfigurationVisitor(StateRos):
    """
    This state keeps an internal database of joints configurations (from param server)
    and at each execution steps commands the next configuration. Once the last is reached,
    it starts looping
    Example usage: the arm has an eye-in-hand and needs to scan the scene from different viewpoints
    """

    def __init__(self, ns=""):
        StateRos.__init__(self,
                          outcomes=['Completed', 'Failure'],
                          input_keys=['reset'],
                          ns=ns)

        self.joints_configurations = self.get_scoped_param("joints_configurations")
        self.n_configurations = len(self.joints_configurations)
        rospy.loginfo("Parsed {} joints configurations: {}".format(self.n_configurations, self.joints_configurations))

        self.planner = MoveItPlanner()
        self.idx = 0

    def run(self):

        if self.idx == self.n_configurations:
            rospy.logwarn("No more configurations to visit! Looping")
            self.idx = 0

        rospy.loginfo("Visiting configuration {}: {}".format(self.idx, self.joints_configurations[self.idx]))
        success = self.planner.reach_joint_angles(self.joints_configurations[self.idx], tolerance=0.01)
        self.idx += 1
        if success:
            return 'Completed'
        else:
            return 'Failure'


class MoveItNamedPositionReaching(StateRos):
    """
    In this state the arm reaches a prerecorded and named configuration saved in the moveit config package
    (see the <robot_name>.sdf or explore the package through moveit_setup_assistant)
    """

    def __init__(self, target, ns=""):
        StateRos.__init__(self, outcomes=['Completed', 'Failure'], ns=ns)
        self.planner = MoveItPlanner()
        self.target = target

    def run(self):
        rospy.loginfo("Reaching named position: {}".format(self.target))
        success = self.planner.reach_named_position(self.target)
        if success:
            return 'Completed'
        else:
            return 'Failure'


class MoveItHome(MoveItNamedPositionReaching):
    """ Assumes there exist an home configuration """

    def __init__(self, ns=""):
        MoveItNamedPositionReaching.__init__(self, "home", ns=ns)


class MoveItVertical(MoveItNamedPositionReaching):
    """ Assumes there exist a retract configuration """

    def __init__(self, ns=""):
        MoveItNamedPositionReaching.__init__(self, "vertical", ns=ns)


class JointsConfigurationAction(StateRosControl):
    """
    Send the arm to a joint configuration using the actionlib interface
    """

    def __init__(self, ns=""):
        try:
            StateRosControl.__init__(self, ns=ns)
            self.joints_configurations = self.get_scoped_param("joints_configurations")
            self.n_configurations = len(self.joints_configurations)
            rospy.loginfo("Parsed {} joints configurations: {}".format(self.n_configurations, self.joints_configurations))

            self.action_name = self.get_scoped_param("action_name")
            self.client = actionlib.SimpleActionClient(self.action_name, JointAction)
        except Exception:
            self.initialization_failure = True

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        # Wait some time to let the controller setup (this should not be necessary)
        rospy.sleep(1.0)

        if not self.client.wait_for_server(rospy.Duration(10)):
            rospy.logerr("Failed to connect to server {}".format(self.action_name))
            return 'Failure'

        goal = JointGoal()
        for goal_position in self.joints_configurations:
            goal.position = goal_position
            rospy.loginfo("Going to configuration {}".format(goal_position))
            self.client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(30))
            result = self.client.get_result()
            if not result.success:
                rospy.logerr("Failed to reach the goal configuration")
                return 'Failure'

        return 'Completed'


if __name__ == "__main__":
    from moma_mission.core import StateMachineRos
    state_machine = StateMachineRos(outcomes=['Success', 'Failure'])
    with state_machine:
        state_machine.add('STATE_A',
                          JointsConfigurationAction,
                          transitions={'Completed': 'Success', 'Failure': 'Failure'})
