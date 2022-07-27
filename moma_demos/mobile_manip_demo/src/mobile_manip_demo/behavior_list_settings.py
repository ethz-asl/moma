"""Setting for behavior_lists for the task."""

from behaviors import behavior_lists as bl
from mobile_manip_demo import behaviors

# Define condition nodes and action nodes
# import the scripts where behaviors are defined

# TODO: decide how to retrieve parameters
# TODO: decide if it is necessary or if we can use only the behaviors script


def get_goal_IDs():
    """Parse the config file and get the IDs for the goals."""
    return [""]


def get_behavior_list() -> bl.BehaviorLists:
    """Return the behavior list."""
    condition_nodes = [
        bl.ParameterizedNode(
            name="rob at pos",
            behavior=behaviors.RobotAtPose,
            parameters=[
                bl.NodeParameter(pose_ID, data_type=bl.ParameterTypes.STRING),
            ],
            condition=True,
        ),
        bl.ParameterizedNode(
            name="cube at pos",
            behavior=behaviors.ObjectAtPose,
            parameters=[
                bl.NodeParameter(pose_ID, data_type=bl.ParameterTypes.STRING),
            ],
            condition=True,
        ),
        bl.ParameterizedNode(
            name="in hand",
            behavior=behaviors.InHand,
            parameters=[
                bl.NodeParameter(obj, data_type=bl.ParameterTypes.STRING),
            ],
            condition=True,
        ),
    ]

    action_nodes = [
        bl.ParameterizedNode(
            name="move",
            behavior=behaviors.Move,
            parameters=[
                bl.NodeParameter(get_goal_IDs(), data_type=bl.ParameterTypes.STRING)
            ],
            condition=False,
        ),
        bl.ParameterizedNode(
            name="pick",
            behavior=behaviors.Pick,
            parameters=[
                bl.NodeParameter(get_goal_IDs(), data_type=bl.ParameterTypes.STRING)
            ],
            condition=False,
        ),
        bl.ParameterizedNode(
            name="place",
            behavior=behaviors.Place,
            parameters=[
                bl.NodeParameter(get_goal_IDs(), data_type=bl.ParameterTypes.STRING)
            ],
            condition=False,
        ),
    ]

    behavior_list = bl.BehaviorLists(
        condition_nodes=condition_nodes, action_nodes=action_nodes
    )

    return behavior_list
