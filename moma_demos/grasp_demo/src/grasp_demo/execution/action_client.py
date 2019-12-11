#!/usr/bin/env python

import py_trees_ros
import py_trees

from grasp_demo.msg import GraspGoal


# When the result of the action should be written to the blackboard
class ActionClient_ResultSaver(py_trees_ros.actions.ActionClient):
    def __init__(
        self,
        name,
        action_spec,
        action_goal,
        action_namespace,
        override_feedback_message_on_running="moving",
        bb_var_name=None,
        set_flag_instead_result=False,
    ):
        super(ActionClient_ResultSaver, self).__init__(
            name,
            action_spec,
            action_goal,
            action_namespace,
            override_feedback_message_on_running,
        )
        self.blackboard = py_trees.blackboard.Blackboard()
        if bb_var_name is None:
            bb_var_name = name + "_result"
        self.bb_var_name = bb_var_name
        self.set_flag_instead_result = set_flag_instead_result

    def initialise(self):
        super(ActionClient_ResultSaver, self).initialise()
        try:
            delattr(self.blackboard, self.bb_var_name)
        except AttributeError:
            pass

    def update(self):
        ret = super(ActionClient_ResultSaver, self).update()
        if ret == py_trees.Status.SUCCESS:
            result = (
                True
                if self.set_flag_instead_result
                else self.action_client.get_result()
            )
            self.blackboard.set(self.bb_var_name, result, overwrite=True)
        return ret


# When a goal from the blackboard should be sent
class ActionClient_BBgoal(py_trees_ros.actions.ActionClient):
    def __init__(
        self,
        name,
        action_spec,
        action_namespace,
        goal_gen_callback,
        bb_goal_var_name,
        bb_result_var_name=None,
        set_flag_instead_result=True,
        override_feedback_message_on_running="moving",
    ):
        action_goal = None
        super(ActionClient_BBgoal, self).__init__(
            name,
            action_spec,
            action_goal,
            action_namespace,
            override_feedback_message_on_running,
        )
        self.blackboard = py_trees.blackboard.Blackboard()
        self.bb_goal_var_name = bb_goal_var_name
        self.goal_gen_callback = goal_gen_callback
        if bb_result_var_name is None:
            bb_result_var_name = name + "_result"
        self.bb_result_var_name = bb_result_var_name
        self.set_flag_instead_result = set_flag_instead_result

    def initialise(self):
        super(ActionClient_BBgoal, self).initialise()
        self.action_goal = self.goal_gen_callback(
            self.blackboard.get(self.bb_goal_var_name)
        )
        try:
            delattr(self.blackboard, self.bb_result_var_name)
        except AttributeError:
            pass

    def update(self):
        if self.action_goal is None:
            self.feedback_message = "no action_goal, was initialise() called?"
            return py_trees.Status.INVALID
        ret = super(ActionClient_BBgoal, self).update()
        if ret == py_trees.Status.SUCCESS:
            result = (
                True
                if self.set_flag_instead_result
                else self.action_client.get_result()
            )
            self.blackboard.set(self.bb_result_var_name, result, overwrite=True)
        return ret
