#!/usr/bin/env python

from __future__ import print_function

import time
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from fetch_demo.msg import (
    SearchAction,
    ApproachAction,
    ApproachGoal,
    DropMoveAction,
)
from grasp_demo.msg import (
    ScanSceneAction,
    SelectGraspAction,
    SelectGraspGoal,
    GraspAction,
    GraspGoal,
)
from std_msgs.msg import Empty

WAIT_FOR_ENTER = True


def wait_for_enter():
    if WAIT_FOR_ENTER:
        raw_input("Press enter to move to next action...")


class SequentialRunner:
    def __init__(self):
        self.client_search = actionlib.SimpleActionClient("search_action", SearchAction)
        self.client_search.wait_for_server()

        self.client_approach = actionlib.SimpleActionClient(
            "approach_action", ApproachAction
        )
        self.client_approach.wait_for_server()

        self.client_scan = actionlib.SimpleActionClient("scan_action", ScanSceneAction)
        self.client_scan.wait_for_server()

        self.client_plan_grasp = actionlib.SimpleActionClient(
            "grasp_selection_action", SelectGraspAction
        )
        self.client_plan_grasp.wait_for_server()

        self.client_execute_grasp = actionlib.SimpleActionClient(
            "grasp_execution_action", GraspAction
        )
        self.client_execute_grasp.wait_for_server()

        self.client_drop_move = actionlib.SimpleActionClient(
            "drop_move_action", DropMoveAction
        )
        self.client_drop_move.wait_for_server()

    def run(self):
        print("Started demo sequence")

        # Search object (input: none; result: target object position known in global map):
        self.client_search.send_goal(Empty())
        self.client_search.wait_for_result()
        result_state = self.client_search.get_state()
        if result_state is not GoalStatus.SUCCEEDED:
            print("Failure during the search action. Aborting.")
            return
        else:
            result_search = self.client_search.get_result()

        wait_for_enter()

        # Approach object (input: target object position; output: none):
        goal = ApproachGoal(target_object_pose=result_search.target_object_pose)
        self.client_approach.send_goal(goal)
        self.client_approach.wait_for_result()
        result_state = self.client_approach.get_state()
        if result_state is not GoalStatus.SUCCEEDED:
            print("Failure during the approach action. Aborting.")
            return

        wait_for_enter()

        # Scan scene (input: none; output: pointcloud of table top scene in front of robot):
        self.client_scan.send_goal(Empty())
        self.client_scan.wait_for_result()
        result_state = self.client_scan.get_state()
        if result_state is not GoalStatus.SUCCEEDED:
            print("Failure during the scan action. Aborting.")
            return
        else:
            result_scan = self.client_scan.get_result()

        wait_for_enter()

        # Plan grasp (input: pointcloud; output: grasp pose):
        goal = SelectGraspGoal(pointcloud_scene=result_scan.pointcloud_scene)
        self.client_plan_grasp.send_goal(goal)
        self.client_plan_grasp.wait_for_result()
        result_state = self.client_plan_grasp.get_state()
        if result_state is not GoalStatus.SUCCEEDED:
            print("Failure during the grasp planning action. Aborting.")
            return
        else:
            result_plan_grasp = self.client_plan_grasp.get_result()

        wait_for_enter()

        # Execute grasp (input: grasp pose; output: none):
        goal = GraspGoal(target_grasp_pose=result_plan_grasp.target_grasp_pose)
        self.client_execute_grasp.send_goal(goal)
        self.client_execute_grasp.wait_for_result()
        result_state = self.client_execute_grasp.get_state()
        if result_state is not GoalStatus.SUCCEEDED:
            print("Failure during the grasp execution action. Aborting.")
            return

        wait_for_enter()

        # Drop object (input: none; output: none):
        self.client_drop_move.send_goal(Empty())
        self.client_drop_move.wait_for_result()
        result_state = self.client_drop_move.get_state()
        if result_state is not GoalStatus.SUCCEEDED:
            print("Failure during the dropping action. Aborting.")
            return

        # Finished
        print("Finished demo sequence.")


def main():
    rospy.init_node("commander_node")

    sr = SequentialRunner()

    raw_input("Press enter to start the demo sequence...")
    sr.run()


if __name__ == "__main__":
    main()
