"""Setting for behavior_lists for the task."""

from behaviors import behavior_lists as bl
from mobile_manip_demo import behaviors

# Define condition nodes and action nodes
# import the scripts where behaviors are defined

# TODO: decide how to retrieve parameters
# TODO: decide if it is necessary or if we can use only the behaviors script


def get_navgoal_IDs():
    """Parse the config file and get the IDs for the navigation goals."""
    return [""]


def get_behavior_list() -> bl.BehaviorLists:
    """Return the behavior list."""
    condition_nodes = [
        bl.ParameterizedNode(
            name="at pos",
            behavior=behaviors.AtPos,
            parameters=[
                bl.NodeParameter(
                    ["0", "1", "2"], data_type=bl.ParameterTypes.INDEX, placement=0
                ),
                bl.NodeParameter(
                    [],
                    (0, 0, 0),
                    (5, 5, 5),
                    (1, 1, 1),
                    data_type=bl.ParameterTypes.POSITION,
                ),
            ],
            condition=True,
        ),
    ]

    action_nodes = [
        bl.ParameterizedNode(
            name="move",
            behavior=behaviors.Move,
            parameters=[
                bl.NodeParameter(get_navgoal_IDs(), data_type=bl.ParameterTypes.STRING)
            ],
            condition=False,
        ),
    ]

    behavior_list = bl.BehaviorLists(
        condition_nodes=condition_nodes, action_nodes=action_nodes
    )

    return behavior_list
