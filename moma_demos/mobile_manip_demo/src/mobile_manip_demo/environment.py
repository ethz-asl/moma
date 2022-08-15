#!/usr/bin/env python3
"""Definition of utilities in the simulation environment."""


from argparse import ArgumentError
from ast import mod


def get_item_by_marker(marker_id: int, model_type: str = "cubes"):
    """Get model name and link as defined in Gazebo."""
    if model_type == "cubes":
        name = "wood_cube_" + str(marker_id)
        link = name + "::link"
    elif model_type == "cans":
        if marker_id == 0:
            name = "can_coke"
            link = name + "::link_0"
        elif marker_id == 1:
            name = "can_pepsi"
            link = name + "::link_0"
        elif marker_id == 2:
            name = "can_fanta"
            link = name + "::link_0"
        elif marker_id == 3:
            name = "can_sprite"
            link = name + "::link_0"
        else:
            raise ArgumentError(f"Marker ID {marker_id} does not exist.")
    else:
        raise ArgumentError(f"Model type {model_type} not implemented.")

    return (name, link)
