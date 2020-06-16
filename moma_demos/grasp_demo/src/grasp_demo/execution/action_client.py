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


# Goes through list of blackboard variables. Clears first one that exists or all of them.
class RepeatAction(py_trees.behaviours.Success):
    def __init__(self, name, variable_names, repeat_all):
        super(RepeatAction, self).__init__(name)
        self.variable_names = variable_names
        self.blackboard = py_trees.blackboard.Blackboard()
        self.repeat_all = repeat_all

    def initialise(self):
        for var in self.variable_names:
            try:
                delattr(self.blackboard, var)
                if not self.repeat_all:
                    break
            except AttributeError:
                continue

class ActionClient_AllCancelation(py_trees_ros.actions.ActionClient):
    def __init__(
        self,
        name,
        action_spec,
        action_namespace,
        action_client,
        # goal_gen_callback,
        # bb_goal_var_name,
        # bb_result_var_name=None,
        # set_flag_instead_result=True,
        # override_feedback_message_on_running="moving",
        # new_status = "SUCCESS"
    ):
        new_status = "SUCCESS"
        action_cancel = None
        super(ActionClient_AllCancelation, self).__init__(
            name,
            action_spec,
            # action_goal,
            action_namespace,
            # override_feedback_message_on_running,
        )
        # self.blackboard = py_trees.blackboard.Blackboard()
        # self.bb_goal_var_name = bb_goal_var_name
        # self.goal_gen_callback = goal_gen_callback
        # if bb_result_var_name is None:
        #     bb_result_var_name = name + "_result"
        # self.bb_result_var_name = bb_result_var_name
        # self.set_flag_instead_result = set_flag_instead_result

    def terminate(self, new_status):
        super(ActionClient_AllCancelation, self).terminate()
        # self.action_goal = self.goal_gen_callback(
        #     self.blackboard.get(self.bb_goal_var_name)
        # )
        # try:
        #     delattr(self.blackboard, self.bb_result_var_name)
        # except AttributeError:
        #     pass
