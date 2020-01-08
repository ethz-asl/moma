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
    Instances,
    ScanSceneAction,
    SelectGraspAction,
    SelectGraspGoal,
    SemanticSelectGraspAction,
    SemanticSelectGraspGoal,
    GraspAction,
    GraspGoal,
)
from std_msgs.msg import Empty
from vpp_msgs.srv import GetListSemanticInstances

WAIT_FOR_ENTER = True


def wait_for_enter():
    if WAIT_FOR_ENTER:
        raw_input("Press enter to move to next action...")


class SequentialRunner:
    def __init__(self):
        self.semantic_grasping = rospy.get_param("semantic_grasping", False)

        self.client_search = actionlib.SimpleActionClient("search_action", SearchAction)
        self.client_search.wait_for_server()

        self.client_approach = actionlib.SimpleActionClient(
            "approach_action", ApproachAction
        )
        self.client_approach.wait_for_server()

        self.client_scan = actionlib.SimpleActionClient("scan_action", ScanSceneAction)
        self.client_scan.wait_for_server()

        if self.semantic_grasping:
            self.get_instances_service = rospy.ServiceProxy(
                "gsm_node/get_list_semantic_instances", GetListSemanticInstances
            )
            self.instances_publisher = rospy.Publisher("instances", Instances)

            self.client_plan_grasp = actionlib.SimpleActionClient(
                "semantic_grasp_selection_action", SemanticSelectGraspAction
            )
        else:
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
        if self.semantic_grasping:
            result_plan_grasp = self._semantic_grasp(result_scan)
        else:
            result_plan_grasp = self._plain_grasp(result_scan)
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

    def _semantic_grasp(self, result_scan):
        self._update_instance_labels()
        selected_instance = rospy.client.wait_for_message("selected_instance")
        goal = SemanticSelectGraspGoal(
            pointcloud_scene=result_scan.pointcloud_scene,
            instance_id=selected_instance.id,
        )
        self.client_plan_grasp.send_goal(goal)
        self.client_plan_grasp.wait_for_result()
        result_state = self.client_plan_grasp.get_state()
        if result_state is not GoalStatus.SUCCEEDED:
            print("Failure during the semantic grasp planning action. Aborting.")
            return
        return self.client_plan_grasp.get_result()

    def _plain_grasp(self, result_scan):
        goal = SelectGraspGoal(pointcloud_scene=result_scan.pointcloud_scene)
        self.client_plan_grasp.send_goal(goal)
        self.client_plan_grasp.wait_for_result()
        result_state = self.client_plan_grasp.get_state()
        if result_state is not GoalStatus.SUCCEEDED:
            print("Failure during the grasp planning action. Aborting.")
            return
        return self.client_plan_grasp.get_result()

    def _update_instance_labels(self):
        response = self.get_instances_service()
        if len(response.instance_ids) == 0:
            print("No instances to grasp")
            return
        msg = Instances()
        msg.instances_ids = response.instance_ids
        msg.semantic_categories = response.semantic_categories
        self.instances_publisher.publish(msg)


def main():
    rospy.init_node("commander_node")

    sr = SequentialRunner()

    raw_input("Press enter to start the demo sequence...")
    sr.run()


if __name__ == "__main__":
    main()
