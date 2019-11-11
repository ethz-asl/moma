import py_trees
import py_trees_ros


def get_root():
    root = py_trees.composites.Selector()

    check_obj_in_ws = py_trees.behaviours.Success(name="Object in workspace?")
    root.add_child(py_trees.decorators.Inverter(check_obj_in_ws))

    check_obj_in_hand = py_trees.behaviours.Failure(name="Object in hand?")

    check_grasp_pose_known = py_trees.behaviours.Failure(name="Grasp pose known?")
    action_get_grasp = py_trees.behaviours.Failure(name="Action compute grasp")
    composite_compute_grasp = py_trees.composites.Selector(children=[check_grasp_pose_known, action_get_grasp])

    action_grasp = py_trees.behaviours.Failure(name="Action do grasp")
    composite_do_grasp = py_trees.composites.Sequence(children=[composite_compute_grasp, action_grasp])

    composite_check_in_hand = py_trees.composites.Selector(children=[check_obj_in_hand, composite_do_grasp])

    action_drop = py_trees.behaviours.Failure(name="Action drop object")
    composite_drop = py_trees.composites.Sequence(children=[composite_check_in_hand, action_drop])

    root.add_child(composite_drop)
    return root

class PandaTree:
    def __init__(self, debug=False):

        if debug:
            py_trees.logging.level = py_trees.logging.Level.DEBUG

        self._root = get_root()
        self.tree = py_trees.trees.BehaviourTree(self._root)

        self.show_tree_console()
        self.tree.setup(timeout=15)

    def show_tree_console(self):
        print("="*20)
        print("Behavior tree:")
        print("-"*20)
        py_trees.display.print_ascii_tree(self.tree.root)
        print("="*20)
