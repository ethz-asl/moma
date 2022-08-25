#!/usr/bin/env python3
"""Definition of utilities in the simulation environment."""


from argparse import ArgumentError
import numpy as np
import os
import yaml


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


def get_closest_robot_target(marker_pose: np.ndarray) -> np.ndarray:
    """Get the robot pose that is closest to the given marker pose."""
    dir_path = os.path.dirname(os.path.abspath(__file__))
    parent_path = os.path.dirname(os.path.dirname(dir_path))
    config_file = os.path.join(parent_path, "config/moma_demo.yaml")
    with open(config_file) as file:
        params = yaml.full_load(file)
        search_waypoints = params["moma_demo"]["search_waypoints"]

    search_waypoints = [np.array(x) for x in search_waypoints]

    dist = 1e10
    idx = 0
    for i, possible in enumerate(search_waypoints):
        current_dist = np.linalg.norm(possible[:2] - marker_pose[:2])
        if current_dist < dist:
            dist = current_dist
            idx = i

    return search_waypoints[idx]
