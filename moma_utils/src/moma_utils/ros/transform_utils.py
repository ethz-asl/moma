#!/usr/bin/env python

import rospy
import numpy as np

#  import scipy as sc
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus


def array_from_pose(pose: Pose) -> np.array:
    array = np.array(
        [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    return array

def pose_from_array(position: np.ndarray, orientation: np.ndarray = [0, 0, 0, 1]) -> Pose:

    pose = Pose()
    if len(position) >= 2:
        pose.position.x = position[0]
        pose.position.y = position[1]
        if len(position) == 2:
            pose.position.z = 0.0
        else:
            pose.position.z = position[2]
    
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]

    return pose


def wrap_angle(
    angle: float, min_angle: float = -np.pi, max_angle: float = np.pi
) -> float:
    """
    Wrap an angle around limits
        - default [-pi, pi] 
        - angle unit should match limit 
    """
    range_width = max_angle - min_angle
    return np.mod(angle + max_angle, range_width) + min_angle


def angle_from_quaternion(
    quaternion: np.ndarray, axis: str = "yaw", radians: bool = True
) -> float:
    """
    Convert a quaternion to an Euler angle
        - default radians, rotation around yaw 
        - zyx euler rotation order
    """
    
    # Ensure quaternion has a non-zero norm
    if np.isclose(np.linalg.norm(quaternion), 0):
        raise ValueError("Quaternion has zero norm, cannot calculate angle.")
    
    r = Rotation.from_quat(quaternion)

    if axis == "yaw":
        r = r.as_euler("zyx")[0]
    elif axis == "pitch":
        r = r.as_euler("zyx")[1]
    elif axis == "roll":
        r = r.as_euler("zyx")[2]
    else:
        raise ValueError(
            f"Only [yaw, pitch, roll] are accepted (yours: [{axis}])"
        )

    if radians:
        return r
    else:
        return np.degree(r)
    
def quaternion_from_angle(
        angle: float, axis: str = "yaw", radians: bool = True
) -> np.ndarray:
    """
    Convert an Euler angle to a quaternion
        - default radiuans, rotation around yaw
        - zyx euler rotation order

    return [x, y, z, w]
    """
    if not radians:
        angle = np.radians(angle)

    if axis == "yaw":
        r = Rotation.from_euler("z", angle)
    elif axis == "pitch":
        r = Rotation.from_euler("y", angle)
    elif axis == "roll":
        r = Rotation.from_euler("x", angle)
    else:
        raise ValueError(f"only [yaw, pitch, roll] accepted, ({axis})")
    
    return r.as_quat()    

def empty_pose() -> Pose:
    """
    empty pose with neutral non-zero quaternion
    """
    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0
    pose.orientation.w = 1.0
    return pose
    
def find_midpoint(point_a: list, point_b: list) -> list:

    midpoint = [
        (point_a[0] + point_b[0]) / 2.0,
        (point_a[1] + point_b[1]) / 2.0
    ]
    return midpoint

def find_slope(point_a: list, point_b: list, perpendicular: bool=False) -> float:
    
    if perpendicular == True:
        if point_b[0] - point_a[0] == 0: # AB is vertical 
            m_perp = 0   
        elif point_b[1] - point_a[1] == 0: # AB is horizontal
            m_perp = None 
        else:
            m_ab = (point_b[1] - point_a[1]) / (point_b[0] - point_a[0])
            m_perp = -1 / m_ab

        return m_perp

    elif perpendicular==False:
        if point_b[0] - point_a[0] == 0: # AB is vertical 
            m_ab = None 
        elif point_b[1] - point_a[1] == 0: # AB is horizontal
            m_ab = 0 
        else:
            m_ab = (point_b[1] - point_a[1]) / (point_b[0] - point_a[0])

        return m_ab

def find_parallel_pose(pose_a: Pose, pose_b: Pose, distance: float, towards: bool = True) -> Pose:
    """
    finds a new pose 
    distance from pose_a along the vector pose_a -> pose_b
    towards: true -> orientation facing pose_a
    """
    [x_a, y_a] = point_from_pose(pose_a)
    [x_b, y_b] = point_from_pose(pose_b)

    vector = np.array([x_b - x_a, y_b - y_a])
    magnitude = np.sqrt(np.sum(vector**2))
    if magnitude == 0:
        raise ValueError("pose_a and pose_b are identical")
    
    unit_vector = vector / magnitude
    
    new_point = np.array([x_a, y_a]) + distance * unit_vector    
    new_angle = np.arctan2(y_b - y_a, x_b - x_a)
    
    if towards:
        new_angle = new_angle + np.pi  

    new_quat = quaternion_from_angle(new_angle)

    return pose_from_point(new_point, new_quat)

def find_perpendicular_pose(pose_a: Pose, pose_b: Pose, distance: float, towards: bool = True) -> Pose:
    """
    finds a new pose perpendicular
    distance from pose_a along the vector pose_a -> pose_b
    towards: true -> orientation facing pose_a
    """
    [x_a, y_a] = point_from_pose(pose_a)
    [x_b, y_b] = point_from_pose(pose_b)

    vector = np.array([x_b - x_a, y_b - y_a])
    magnitude = np.sqrt(np.sum(vector**2))
    if magnitude == 0:
        raise ValueError("pose_a and pose_b are identical")
    
    unit_vector = vector / magnitude
    perp_vector = np.array([-unit_vector[1], unit_vector[0]]) # ccw

    if not towards:
        perp_vector = -perp_vector # cw
    
    new_point = np.array([x_a, y_a]) + distance * perp_vector    
    new_angle = np.arctan2(perp_vector[1], perp_vector[0])

    new_quat = quaternion_from_angle(new_angle)

    return pose_from_point(new_point, new_quat)


def pose_from_point(point: list, orientation: list = [0, 0, 0, 1]) -> Pose:
    pose = Pose()
    pose.position.x = point[0]
    pose.position.y = point[1]
    if len(point) > 2:
        pose.position.z = point[2]
    
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]  
    pose.orientation.w = orientation[3]
    
    return pose

def point_from_pose(pose: Pose) -> list:
    return [pose.position.x, pose.position.y]

def quaternion_from_pose(pose: Pose) -> list:
    return [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
