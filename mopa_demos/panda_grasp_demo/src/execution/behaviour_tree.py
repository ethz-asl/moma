import py_trees
import py_trees_ros


def create_tree():
    root = py_trees.composites.Selector()

    check_obj_in_ws = py_trees.behaviours.Failure(name="Object in workspace?")
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
