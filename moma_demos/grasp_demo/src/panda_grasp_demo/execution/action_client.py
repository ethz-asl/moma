#!/usr/bin/env python

import py_trees_ros
import py_trees

from grasp_demo.msg import GraspGoal


class ActionClient_ResultSaver(py_trees_ros.actions.ActionClient):
    def __init__(self, name="Action Client", action_spec=None, action_goal=None, action_namespace="/action",
                 override_feedback_message_on_running="moving", bb_var_name=None):
        super(ActionClient_ResultSaver, self).__init__(name, action_spec, action_goal, action_namespace, override_feedback_message_on_running)
        self.blackboard = py_trees.blackboard.Blackboard()
        if bb_var_name is None:
            bb_var_name = name+"_result"
        self.bb_var_name = bb_var_name

    def initialise(self):
        super(ActionClient_ResultSaver, self).initialise()
        try:
            delattr(self.blackboard, self.bb_var_name)
        except AttributeError:
            pass
        
    def update(self):
        ret = super(ActionClient_ResultSaver, self).update()
        if ret == py_trees.Status.SUCCESS:
            result = self.action_client.get_result()
            self.blackboard.set(self.bb_var_name, result, overwrite=True)
        return ret

# When a goal from the blackboard should be sent
class ActionClient_BBgoal(py_trees_ros.actions.ActionClient):
    def __init__(self, name, action_spec, action_namespace, goal_gen_callback, bb_var_name,
                 override_feedback_message_on_running="moving"):
        action_goal = None
        super(ActionClient_BBgoal, self).__init__(name, action_spec, action_goal, action_namespace, override_feedback_message_on_running)
        self.blackboard = py_trees.blackboard.Blackboard()
        if bb_var_name is None:
            bb_var_name = name+"_goal"
        self.bb_var_name = bb_var_name
        self.goal_gen_callback = goal_gen_callback

    def initialise(self):
        super(ActionClient_BBgoal, self).initialise()
        self.action_goal = self.goal_gen_callback(self.blackboard.get(self.bb_var_name))

    def update(self):
        if self.action_goal is None:
            self.feedback_message = "no action_goal, was initialise() called?"
            return py_trees.Status.INVALID
        return super(ActionClient_BBgoal, self).update()
