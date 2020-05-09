#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
TODO Write text here what is it about
"""

##############################################################################
# Imports
##############################################################################

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
import std_msgs.msg

##############################################################################
# Variables
##############################################################################
armReach = 999

##############################################################################
# Behaviours
##############################################################################

def create_root():
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.
    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    # LEVEL 0
    # Root Tree running Blackboard and Action Tree in parallel
    root = py_trees.composites.Parallel("Reactive Execution")

    # LEVEL 1
    # Blackboard Tree
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    gripper_state_BB = py_trees.behaviours.Success(name="Gripper State BB")
        # gripperStateBB = py_trees_ros.subscribers.EventToBlackboard(
        #     name="Gripper State BB",
        #     topic_name="/gripper/angle",
        #     variable_name="event_scan_button"
        # )
    dist_obj_BB = py_trees.behaviours.Success(name="Distance to object BB")
    obj_range_BB = py_trees.behaviours.Success(name="Object in range BB")
    traj_comp_BB = py_trees.behaviours.Success(name="Trajectory computed BB")
    # obj_pres_BB = py_trees.behaviours.Success(name="Object presence BB")
    obj_pres_BB = py_trees_ros.subscribers.ToBlackboard(
        name="Object presence BB",
        topic_name = '/object_tracker/objLock',
        topic_type = std_msgs.msg.Bool,
        blackboard_variables = {'obj_prsc':'data'}
        )
    obj_moving_BB = py_trees_ros.subscribers.ToBlackboard(
        name="Object moving BB",
        topic_name = '/object_tracker/motion',
        topic_type = std_msgs.msg.Bool,
        blackboard_variables = {'obj_mvng':'data'}
        )
    
    # Action Tree
    selGripper = py_trees.composites.Selector("Gripper Check")
    
    # LEVEL 2
    checkGripper = py_trees.blackboard.CheckBlackboardVariable(name = "Check Gripper", expected_value = True)
    # checkGripper = py_trees.blackboard.CheckBlackboardVariable(name, variable_name='gripper_state_BB', expected_value=True)
    seqObject = py_trees.composites.Sequence("Get Object")

    # LEVEL 3
    selRange = py_trees.composites.Selector("Range Check")
    selTraj = py_trees.composites.Selector("Get Trajectory")
    #execGrasp = py_trees.behaviours.Running("Execute Grasp")
    execGrasp = py_trees.behaviours.Success("Execute Grasp")
    # execGrasp = py_trees_ros.actions.ActionClient(name="Execute Grasp",
    #                                               #action_spec=None,
    #                                               #action_goal=None,
    #                                               #action_namespace='/action',
    #                                               #override_feedback_message_on_running='moving'
    #                                               )

    # LEVEL 4
    # TODO How to make the comporison for the expected result
    #checkRange = py_trees.blackboard.CheckBlackboardVariable(name = "Check Range", expected_value = armReach)
    checkRange = py_trees.behaviours.Success(name = "Check Range")
    #blinkLED = py_trees.behaviours.Running("Blink LED")
    blinkLED = py_trees.behaviours.Failure("Blink LED")
    # blinkLED = py_trees_ros.actions.ActionClient(name="Blink LED",
    #                                               #action_spec=None,
    #                                               #action_goal=None,
    #                                               #action_namespace='/action',
    #                                               #override_feedback_message_on_running='moving'
    #                                               )
    seqCheck1 = py_trees.composites.Sequence("Check 1")
    seqCheck2 = py_trees.composites.Sequence("Check 2")

    # LEVEL 5
    checkTraj = py_trees.blackboard.CheckBlackboardVariable(name = "Trajectory computed", expected_value = True)
    checkObj1 = py_trees.blackboard.CheckBlackboardVariable(name = "Object present? 1")
    sel1 = py_trees.composites.Selector("Selector 1")
    #compTraj = py_trees.behaviours.Running("Compute Trajectory")
    compTraj = py_trees.behaviours.Success(name = "Compute Trajectory")
    # compTraj = py_trees_ros.actions.ActionClient(name='Compute Trajectory',
    #                                               #action_spec=None,
    #                                               #action_goal=None,
    #                                               #action_namespace='/action',
    #                                               #override_feedback_message_on_running='moving'
    #                                               )

    # LEVEL 6
    checkObj2 = py_trees.blackboard.CheckBlackboardVariable(
        name = "Object present? 2",
        variable_name="obj_prsc",
        expected_value=True
        )
    detectObj = py_trees.behaviours.Running("Object detection")
    # detectObj = py_trees_ros.actions.ActionClient(name='Detect Object',
    #                                               #action_spec=None,
    #                                               #action_goal=None,
    #                                               #action_namespace='/action',
    #                                               #override_feedback_message_on_running='moving'
    #                                               )

    ### Building tree ###
    # childs in LEVEL 1
    root.add_child(topics2bb)
    root.add_child(selGripper)
    # childs in LEVEL 2
    topics2bb.add_children([gripper_state_BB,dist_obj_BB,obj_range_BB,traj_comp_BB,obj_pres_BB,obj_moving_BB])
    selGripper.add_child(checkGripper)
    selGripper.add_child(seqObject)
    # childs in LEVEL 3
    seqObject.add_child(selRange)
    seqObject.add_child(selTraj)
    seqObject.add_child(execGrasp)
    # childs in LEVEL 4
    selRange.add_child(checkRange)
    selRange.add_child(blinkLED)
    selTraj.add_child(seqCheck1)
    selTraj.add_child(seqCheck2)
    # childs in LEVEL 5
    seqCheck1.add_child(checkTraj)
    seqCheck1.add_child(checkObj1)
    seqCheck2.add_child(sel1)
    seqCheck2.add_child(compTraj)
    # childs in LEVEL 6
    sel1.add_child(checkObj2)
    sel1.add_child(detectObj)
    return root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the main script.
    """
    rospy.init_node("tree")
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(500)

if __name__ == "__main__":
    main()