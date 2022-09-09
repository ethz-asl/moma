#!/usr/bin/env python3
"""Definition of utilities in the simulation environment."""


from argparse import ArgumentError
from copy import copy
import math
from typing import Tuple
import numpy as np
import os
import yaml


def get_item_by_marker(marker_id: int, model_type: str = "cubes") -> Tuple[str, str]:
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


def get_place_pose(
    navigation_target: np.ndarray, place_target: np.ndarray
) -> np.ndarray:
    place_pose = copy(navigation_target)
    # transform from panda_link0 to base_link
    tf_manip_base = np.array([0.210, 0.000, 0.480])
    target_in_base = tf_manip_base + place_target
    # add also 0.5 for the last control we give to mobile base
    place_pose[0] += target_in_base[1]
    place_pose[1] += target_in_base[0] + 0.5
    place_pose[2] += target_in_base[2]

    return place_pose


def angle_from_quaternion(
    quaternion: np.ndarray or list, rotation: str = "yaw"
) -> float:
    """Return the desired rotation angle in radians from a quaternion."""
    x, y, z, w = quaternion if isinstance(quaternion, list) else quaternion.tolist()
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    if rotation == "yaw":
        return yaw_z
    elif rotation == "pitch":
        return pitch_y
    else:
        return roll_x
