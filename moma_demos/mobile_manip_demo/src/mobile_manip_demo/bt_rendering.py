#!/usr/bin/env python3
"""Methods to render the BT."""

from typing import Any

from mobile_manip_demo.behaviors import RSequence
import py_trees as pt
import pydot


def dot_graph(tree: Any, include_status: bool = False) -> pydot.Dot:
    """Create and return a pydot graph from tree."""
    if isinstance(tree, pt.trees.BehaviourTree):
        root = tree.root
    else:
        root = tree

    graph = pydot.Dot("BT", graph_type="digraph")
    __generate_dot_tree(root, graph, include_status)
    return graph


def __generate_dot_tree(root: Any, graph: pydot.Dot, include_status: bool):
    if hasattr(root, "get_display_name"):
        name = root.get_display_name()
    else:
        name = root.name

    node = root.name

    node_instance = None

    if isinstance(root, RSequence) or isinstance(root, pt.composites.Sequence):
        node_instance = pydot.Node(
            node, label="→", shape="square", margin="0.1", width="0", height="0"
        )
    elif isinstance(root, pt.composites.Selector):
        node_instance = pydot.Node(
            node, label="?", shape="square", margin="0.1", width="0", height="0"
        )
    elif isinstance(root, pt.behaviour.Behaviour):
        if node[-1] == "!":
            node_instance = pydot.Node(node, label=name, shape="box")
        else:
            node_instance = pydot.Node(node, label=name, shape="ellipse")

    if node_instance is not None:
        if include_status:
            if root.status == pt.common.Status.SUCCESS:
                node_instance.set("color", "lightgreen")
                node_instance.set("penwidth", "2")
            elif root.status == pt.common.Status.FAILURE:
                node_instance.set("color", "red")
                node_instance.set("penwidth", "2")
            elif root.status == pt.common.Status.RUNNING:
                node_instance.set("color", "orange")
                node_instance.set("penwidth", "2")
            else:
                node_instance.set("color", "black")
                node_instance.set("penwidth", "1")

        node = node_instance.get_name()
        while node in [x.get_name() for x in graph.get_node_list()]:
            node += " "

        node_instance.set_name(node)

        graph.add_node(node_instance)

    if isinstance(root, pt.composites.Composite):
        for i, child in enumerate(root.children):
            child_node = __generate_dot_tree(child, graph, include_status)
            edge = pydot.Edge(node, child_node)
            if include_status:
                color = graph.get_node(child_node)[0].get("color")
                if color is not None:
                    edge.set("color", color)
                    edge.set("penwidth", "1")
            graph.add_edge(edge)

    return node


def __display_names(root: Any):
    """Create a new tree with empty behaviors and display names."""
    if hasattr(root, "get_display_name"):
        name = root.get_display_name()
    else:
        name = root.name

    if isinstance(root, pt.composites.Composite):
        if root.name == "Sequence":
            # control = RSequence()
            control = pt.composites.Sequence("Sequence", memory=False)
        elif root.name == "Fallback":
            control = pt.composites.Selector("Fallback")
        else:
            raise ValueError("Unknown control flow node " + root.name)

        for child in root.children:
            copy = __display_names(child)
            control.add_child(copy)

        return control
    else:
        return pt.behaviour.Behaviour(name=name)
