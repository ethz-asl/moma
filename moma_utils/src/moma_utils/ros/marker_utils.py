#! /usr/bin/env python

from __future__ import annotations

import rospy
import random
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import scipy.spatial.transform
import numpy as np

from moma_utils.transform import Transform
from moma_utils.ros import conversions


def pose2Transform(pose: Pose) -> Transform:
    position = [pose.position.x, pose.position.y, pose.position.z]
    rotation = scipy.spatial.transform.Rotation(
        [
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]
    )
    return Transform(rotation, position)


def _create_marker_msg(
    marker_type: Marker.type,
    frame: str,
    pose: Transform,
    scale: np.array | list,
    color: str | list,
):
    color = getColorRGBA(color)

    msg = Marker()
    msg.header.frame_id = frame
    msg.header.stamp = rospy.Time()
    msg.type = marker_type
    msg.action = Marker.ADD
    msg.pose = conversions.to_pose_msg(pose)
    msg.scale = conversions.to_vector3_msg(scale)
    msg.color = color
    return msg


def create_cylinder_marker_msg(
    pose: Pose,
    color: list | str = "orange",
    frame: str = "map",
    height: float = 0.5,
    radius: float = 0.05,
):
    pose.position.z = pose.position.z + height / 2.0
    pose = pose2Transform(pose)
    scale = [radius, radius, height]
    msg = _create_marker_msg(Marker.CYLINDER, frame, pose, scale, color)
    return msg


def create_text_marker_msg(
    pose: Pose,
    text: str,
    color: list | str = "white",
    frame: str = "map",
    scale: float = 0.1,
):
    scale = [scale, scale, scale]
    pose = pose2Transform(pose)
    msg = _create_marker_msg(
        Marker.TEXT_VIEW_FACING, frame, pose, scale, color
    )
    msg.text = text
    return msg


def delete_marker_msg(msg: Marker):
    """
    setting specific marker to DELETE
    """
    msg.action = Marker.DELETE
    return msg


def show_marker_msg(msg: Marker):
    """
    setting specific marker to ADD
    """
    msg.action = Marker.ADD
    return msg


def getColorRGBA(color: str | list, transparency: float = 1.0) -> ColorRGBA:
    """
    Convert a color name or RGB value to a ROS ColorRGBA type

    @param color name (string) or RGB color value (tuple or list)

    @return color (ColorRGBA)
    """

    result = ColorRGBA()
    result.a = transparency

    if (type(color) == tuple) or (type(color) == list):
        if len(color) == 3:
            result.r = color[0]
            result.g = color[1]
            result.b = color[2]
        elif len(color) == 4:
            result.r = color[0]
            result.g = color[1]
            result.b = color[2]
            result.a = color[3]
        else:
            raise ValueError(
                "color must have 3 or 4 float values in getColor()"
            )
    elif color == "red":
        result.r = 0.96
        result.g = 0.26
        result.b = 0.21
    elif color == "green":
        result.r = 0.3
        result.g = 0.69
        result.b = 0.31
    elif color == "blue":
        result.r = 0.13
        result.g = 0.59
        result.b = 0.95
    elif (color == "grey") or (color == "gray"):
        result.r = 0.62
        result.g = 0.62
        result.b = 0.62
    elif color == "white":
        result.r = 1.0
        result.g = 1.0
        result.b = 1.0
    elif color == "orange":
        result.r = 1.0
        result.g = 0.44
        result.b = 0.0
    elif color == "translucent_light":
        result.r = 0.1
        result.g = 0.1
        result.b = 0.1
        result.a = 0.1
    elif color == "translucent":
        result.r = 0.1
        result.g = 0.1
        result.b = 0.1
        result.a = 0.25
    elif color == "translucent_dark":
        result.r = 0.1
        result.g = 0.1
        result.b = 0.1
        result.a = 0.5
    elif color == "black":
        result.r = 0.0
        result.g = 0.0
        result.b = 0.0
    elif color == "yellow":
        result.r = 1.0
        result.g = 1.0
        result.b = 0.0
    elif color == "brown":
        result.r = 0.597
        result.g = 0.296
        result.b = 0.0
    elif color == "pink":
        result.r = 0.97
        result.g = 0.73
        result.b = 0.82
    elif color == "lime_green":
        result.r = 0.8
        result.g = 0.86
        result.b = 0.22
    elif color == "clear":
        result.r = 1.0
        result.g = 1.0
        result.b = 1.0
        result.a = 0.0
    elif color == "purple":
        result.r = 0.40
        result.g = 0.23
        result.b = 0.72
    elif color == "light_purple":
        result.r = 0.82
        result.g = 0.77
        result.b = 0.91
    elif color == "teal":
        result.r = 0.00
        result.g = 0.59
        result.b = 0.53
    elif color == "random":
        # Get a random color that is not too light
        while True:
            result.r = random.random()  # random float from 0 to 1
            result.g = random.random()
            result.b = random.random()
            if (result.r + result.g + result.b) > 1.5:  # 0=black, 3=white
                break
    else:
        rospy.logerr(
            "getColor() called with unknown color name '%s', defaulting to 'blue'",
            color,
        )
        result.r = 0.1
        result.g = 0.1
        result.b = 0.8

    return result


def getRandomColor():
    """
    Get a random color.

    @return color (ColorRGBA)
    """

    # Make a list of the color names to choose from
    all_colors = []
    all_colors.append("red")
    all_colors.append("green")
    all_colors.append("blue")
    all_colors.append("grey")
    all_colors.append("white")
    all_colors.append("orange")
    all_colors.append("yellow")
    all_colors.append("brown")
    all_colors.append("pink")
    all_colors.append("lime_green")
    all_colors.append("purple")

    # Chose a random color name
    rand_num = random.randint(0, len(all_colors) - 1)
    rand_color_name = all_colors[rand_num]

    return rand_color_name
