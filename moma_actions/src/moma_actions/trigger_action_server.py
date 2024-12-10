#! /usr/bin/env python

import rospy
import actionlib
from moma_actions.msg import TriggerAction, TriggerGoal

from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalStatus

from moma_actions.giraffe_interface import GiraffeComponents


class TriggerComponentServer(object):
    """
    Trigger action server, to send cmd to component on robot
    """

    def __init__(self, name) -> None:
        self._action_name = name
        self._robot_name = rospy.get_param("~robot_name", "giraffe")
        self._component = None

        self._action = TriggerAction()
        self._goal = TriggerGoal()

        self._server = actionlib.SimpleActionServer(
            self._action_name, TriggerAction, self.execute_cb, auto_start=False
        )

        if self._robot_name == "giraffe":
            self._component = GiraffeComponents()
        else:
            rospy.logerr(f"Only implemented robot \"giraffe\", yours: {self._robot_name}")
            raise ValueError
        
        rospy.loginfo("Starting trigger component action server")
        
        self._server.start()
        self._cancel_requested = False


    def execute_cb(self, goal: TriggerGoal) -> None:
        
        rospy.loginfo(f"Recieved goal type {goal.component.data}")

        self._server.preempt_request = False

        while True:
            if (
                rospy.is_shutdown()
                or self._server.is_preempt_requested()
                or self._cancel_requested
            ):
                rospy.loginfo("trigger goal preempted")
                self._server.set_preempted()
                self._cancel_requested = True
                success = False
                break

            elif self._component.trigger(goal.component.data, goal.cmd.data):
                self._server.set_succeeded()
                success = True
                break

    def cancel_goal(self):
        """
        Cancel the trigger goal.
        """
        rospy.loginfo("Cancelling trigger goal")
        self._server.set_preempted()  # Preempt the server

        # If the server is not done, preempt the goal
        rospy.logerr(f"state {self._server.get_state()}")
        if self._server.get_state() == actionlib.GoalStatus.DONE:
            rospy.loginfo("Goal was already reached")
            return
        else:
            self._server.set_preempted()


def open_hatch_cb(req):
    rospy.sleep(1.0)
    rospy.loginfo("hatch open!")
    return TriggerResponse(success=True)

def close_hatch_cb(req):
    rospy.sleep(1.0)
    return TriggerResponse(success=True)

def raise_roller_cb(req):
    rospy.sleep(1.0)
    return TriggerResponse(success=True)

def lower_roller_cb(req):
    rospy.sleep(1.0)
    return TriggerResponse(success=True)

def setup_component_srvs():
    rospy.Service("open_hatch", Trigger, open_hatch_cb)
    rospy.Service("close_hatch", Trigger, close_hatch_cb)
    rospy.Service("raise_roller", Trigger, raise_roller_cb)
    rospy.Service("lower_roller", Trigger, lower_roller_cb)

def main():
    rospy.init_node("components")
    setup_component_srvs()
    TriggerComponentServer(rospy.get_name())

    rospy.spin()


if __name__ == "__main__":
    main()
