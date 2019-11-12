import py_trees
import py_trees_ros

from panda_grasp_demo.msg import ScanSceneAction, ScanSceneGoal
from action_client import ActionClient_ResultSaver

import std_msgs

def get_root():
    # For a sketch of the tree layout, see here (slide 2): https://docs.google.com/presentation/d/1swC5c1mbVn2TRDar-y0meTbrC9BUHnT9XWYPeFJlNxM/edit#slide=id.g70bc070381_0_32

    # -------- Add nodes writing to blackboard -----------------------------------------

    bb_root = py_trees.composites.Sequence()

    # grasp2bb = py_trees_ros.subscribers.ToBlackboard(name="grasp2bb",
    #                                                  topic_name="/panda_demo/grasp_pose",
    #                                                  topic_type=std_msgs.msg.String,
    #                                                  blackboard_variables="grasp_pose",
    #                                                  initialise_variables=None
    #                                                 )
    # bb_root.add_child(grasp2bb)

    # -------- Add nodes with condition checks and actions -----------------------------

    action_root = py_trees.composites.Selector()

    check_obj_in_ws = py_trees.behaviours.Success(name="Object in workspace?")
    action_root.add_child(py_trees.decorators.Inverter(check_obj_in_ws))

    check_obj_in_hand = py_trees.behaviours.Failure(name="Object in hand?")

    # check_grasp_pose_known = py_trees.behaviours.Failure(name="Grasp pose known?")
    check_grasp_pose_known = py_trees.blackboard.CheckBlackboardVariable(name="Grasp pose known?", variable_name="target_grasp_pose")

    scan_goal = ScanSceneGoal()
    scan_goal.num_scan_poses = 2
    # action_get_grasp = py_trees.behaviours.SuccessEveryN(name="Action compute grasp", n=4)
    # action_get_grasp = py_trees.behaviours.Running(name="Action compute grasp")
    action_get_grasp = ActionClient_ResultSaver(name="Action compute grasp",
                                                         action_spec=ScanSceneAction,
                                                         action_goal=scan_goal,
                                                         action_namespace="pointcloud_scan_action",
                                                         bb_var_name="target_grasp_pose"
                                                        )

    composite_compute_grasp = py_trees.composites.Selector(children=[check_grasp_pose_known, action_get_grasp])

    action_grasp = py_trees.behaviours.Running(name="Action do grasp")
    composite_do_grasp = py_trees.composites.Sequence(children=[composite_compute_grasp, action_grasp])

    composite_check_in_hand = py_trees.composites.Selector(children=[check_obj_in_hand, composite_do_grasp])

    action_drop = py_trees.behaviours.Failure(name="Action drop object")
    composite_drop = py_trees.composites.Sequence(children=[composite_check_in_hand, action_drop])

    action_root.add_child(composite_drop)


    # -------- Return root -----------------------------------------
    root = py_trees.composites.Parallel(children=[bb_root, action_root])

    return root

class PandaTree:
    def __init__(self, debug=False):

        if debug:
            py_trees.logging.level = py_trees.logging.Level.DEBUG

        self._root = get_root()
        self.tree = py_trees_ros.trees.BehaviourTree(self._root)

        self.show_tree_console()
        self.tree.setup(timeout=15)

    def show_tree_console(self):
        print("="*20)
        print("Behavior tree:")
        print("-"*20)
        py_trees.display.print_ascii_tree(self.tree.root)
        print("="*20)
