from .grasping import get_grasping_description
from .navigate import get_nav_in_reach_description, get_nav_at_description


def get_action_descriptions():
    functions = [
        get_grasping_description,
        get_nav_in_reach_description,
        get_nav_at_description,
    ]
    descr = dict()
    for func in functions:
        temp = func()
        descr[temp[0]] = temp[1]
    return descr
